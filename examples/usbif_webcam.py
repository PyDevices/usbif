"""The board *is* the webcam: a USB video device any application can open.

Advertises a UVC video function and streams frames to the host, so the board
appears in Windows' Camera app, in a browser's `getUserMedia`, in a video
call -- anywhere a webcam appears. No driver to install: it is a
class-compliant camera.

This is the mirror of ``uvc_display.py``. That example makes the board a USB
*host* consuming someone else's camera; this one makes it a USB *device*
producing video. The two exercise the same class from opposite ends, which
is a useful thing to have proven both ways round.

**Frames come from the camera when there is one.** On a board with a
MIPI-CSI sensor (the ESP32-P4 panels) this streams what the camera sees:
``cameraif`` captures, converts to YUY2 and decimates to the size the UVC
descriptor advertises, in one pass. With no camera it falls back to
scrolling colour bars -- motion on the host still proves the stream is live
rather than one still repeated, which is what makes the fallback worth
having rather than an error.

**The format is fixed at build time.** A UVC device declares its formats in
descriptors the host reads once at enumeration, so 160x120 YUY2 comes from
``usbif_tusb_ext.h`` and changing it is a reflash. That is a property of the
class, not of this example -- ask the module rather than hard-coding it::

    width, height, frame_bytes = _usbif.uvc_dev_format()

Run it, then open the Camera app on the host::

    mpremote run usbif_webcam.py
"""

import time

import micropython

import _usbif



# Eight bars, as (Y, U, V). The same values the display example uses, so a
# board doing both shows the same colours on its panel as it sends up the
# wire -- white, yellow, cyan, green, magenta, red, blue, black.
BARS = (
    (235, 128, 128), (210, 16, 146), (170, 166, 16), (145, 54, 34),
    (106, 202, 222), (81, 90, 240), (41, 240, 110), (16, 128, 128),
)


@micropython.viper
def render_row(dst: ptr8, width: int, phase: int, ys: ptr8, us: ptr8, vs: ptr8,
               nbars: int):
    """One YUY2 row of vertical colour bars, scrolled by ``phase`` pixels.

    YUY2 packs two pixels into four bytes -- Y0 U Y1 V -- so the pair shares
    one chroma sample. Bars are wide, so both pixels of a pair almost always
    fall in the same bar and taking chroma from the first is exact rather
    than approximate; at a bar edge it is off by one pixel, which is
    invisible and costs nothing to accept.
    """
    x = 0
    o = 0
    while x < width:
        i0 = int((((x + phase) % width) * nbars) // width)
        i1 = int(((((x + 1 + phase) % width)) * nbars) // width)
        dst[o] = ys[i0]
        dst[o + 1] = us[i0]
        dst[o + 2] = ys[i1]
        dst[o + 3] = vs[i0]
        x += 2
        o += 4


def open_camera(width, height):
    """The board's camera, or None.

    Asks ``board_config`` rather than constructing a camera directly: which
    pins the sensor's control bus is on is a board fact, and board_config is
    where this project keeps board facts. That also means this example needs
    no changes to run on a different board with a camera.

    Returns None rather than raising for every reason a board might not have
    one -- no pydevices, no camera module in the firmware, no sensor, or a
    sensor smaller than the advertised frame. The example is worth running
    either way, and a webcam that falls back to a test pattern is more
    useful than one that refuses to start.
    """
    try:
        import board_config
    except ImportError:
        return None
    try:
        cam = board_config.camera
    except (AttributeError, NotImplementedError, OSError) as exc:
        print("no camera on this board:", exc)
        return None
    cam_w, cam_h, _ = cam.size()
    if cam_w < width or cam_h < height:
        print("camera %dx%d is smaller than the advertised %dx%d frame"
              % (cam_w, cam_h, width, height))
        cam.deinit()
        return None
    print("camera:", cam.sensor(), "%dx%d" % (cam_w, cam_h))
    return cam


def main():
    width, height, frame_bytes = _usbif.uvc_dev_format()
    if frame_bytes == 0:
        print("this firmware has no UVC device function built in")
        return
    print("advertising %dx%d YUY2, %d bytes per frame"
          % (width, height, frame_bytes))

    cam = open_camera(width, height)
    frame = bytearray(frame_bytes)

    # The colour-bar fallback, built once. Only used when there is no camera.
    ys = bytearray(len(BARS))
    us = bytearray(len(BARS))
    vs = bytearray(len(BARS))
    for i, (y_, u_, v_) in enumerate(BARS):
        ys[i], us[i], vs[i] = y_, u_, v_
    row_bytes = width * 2
    row = bytearray(row_bytes)
    frame_mv = memoryview(frame)

    _usbif.uvc_dev_reset()
    # A console beside the camera, so the REPL stays reachable on the same
    # cable while the host has the video -- the same courtesy sd_drive.py
    # extends. FN_VIDEO alone works if an application wants only the camera.
    _usbif.dev_functions(_usbif.FN_CDC | _usbif.FN_VIDEO)
    print("costume set;", "streaming the camera" if cam else "streaming colour bars")
    print("open a camera application on the host")

    phase = 0
    sent = 0
    t0 = time.ticks_ms()
    try:
        while True:
            # streaming() and ready() answer different questions and want
            # opposite responses: nobody watching means idle cheaply, while
            # watching-but-busy means come straight back.
            if not _usbif.uvc_dev_streaming():
                time.sleep_ms(50)
                continue
            if not _usbif.uvc_dev_ready():
                time.sleep_ms(2)
                continue

            if cam is not None:
                # Convert and decimate in one pass, straight into the frame
                # the UVC endpoint will send. A short read is the camera not
                # having one ready, which is not an error at 50 fps.
                if cam.capture_yuy2(frame, width, height, 200) == 0:
                    continue
            else:
                render_row(row, width, phase, ys, us, vs, len(BARS))
                for y in range(height):
                    frame_mv[y * row_bytes:(y + 1) * row_bytes] = row
                phase = (phase + 2) % width

            _usbif.uvc_dev_submit(frame)
            sent += 1
            if sent % 50 == 0:
                dt = time.ticks_diff(time.ticks_ms(), t0)
                print("%d frames, %.1f fps, stats %r"
                      % (sent, sent * 1000 / dt, _usbif.uvc_dev_stats()))
    except KeyboardInterrupt:
        print("stopping")
    finally:
        print("final stats (frames,completed,refused,streaming):",
              _usbif.uvc_dev_stats())
        if cam is not None:
            try:
                cam.deinit()
            except Exception:
                pass


if __name__ == "__main__":
    main()
