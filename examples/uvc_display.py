"""Live camera preview: a hosted USB webcam shown on the board's own display.

The board is the USB *host* here. It enumerates a class-compliant UVC camera,
reads its descriptors, negotiates a video mode with it, and streams frames in
over isochronous transfers -- then puts them on the display that
``board_config`` already set up. No PC in the loop at any point.

Run it on a board with a display and a camera on its host port::

    mpremote run uvc_display.py

**Why the uncompressed format and not MJPEG.** A webcam offers far better
resolutions in MJPEG than uncompressed -- on the bench camera, 640x480 against
176x144 -- so MJPEG is the tempting choice. It needs a JPEG decoder, and there
is not one reachable from Python in this firmware today. LVGL is compiled in
and its TJPGD decoder is enabled and registered, but the MicroPython bindings
expose only the decoder *types*, and the widget route (hand an ``lv.image`` a
variable ``image_dsc_t``) does not decode -- verified with the signature
repaired and the binary decoder stood down. Worse for this use, LVGL's
``is_jpg()`` demands a JFIF header in the first ten bytes and a UVC frame does
not have one: it opens straight into a quantisation table and carries its APP0
segment after the Huffman tables.

A ``jpegio`` native module is being added for exactly this path. When it
lands, this example should grow an MJPEG branch and prefer it -- the frames
are already whole JPEGs, complete with Huffman tables. Until then:
uncompressed frames, converted here, blitted straight to ``display_drv``. The
picture is small and the pixels are honest.

**Why it is upscaled by whole numbers.** Nearest-neighbour at an integer
factor is a few instructions per pixel and needs no line buffer beyond one
row. Anything smoother is a real resampler, which is a different example.
"""

import time

import micropython
from board_config import display_drv

import _usbif
from usbif import uvc

# The camera's data endpoint is fed one packet per bus frame, so a mode is
# only reachable if its negotiated payload fits the host's isochronous IN
# limit. That limit is set by the DWC FIFO bias chosen at build time -- see
# usbif#2 -- and is 600 bytes on the current firmware. Modes are tried
# largest-first and the first one the camera and the bus both accept wins.
IN_LIMIT = 600

# 100 ns units, which is how UVC counts frame intervals throughout.
INTERVALS = (2000000, 1333333, 1000000, 666666, 333333)   # 5, 7.5, 10, 15, 30 fps


@micropython.viper
def yuy2_row_to_rgb565(src: ptr8, dst: ptr16, width: int, scale: int):
    """One YUY2 row to RGB565, widened by an integer factor.

    YUY2 packs two pixels into four bytes -- Y0 U Y1 V -- so the two pixels
    share one chroma sample. The conversion is BT.601 in fixed point; the
    shifts are by 8 so the coefficients are the usual ones times 256.
    """
    i = 0       # source pixel
    j = 0       # source byte
    o = 0       # destination pixel
    while i < width:
        y0 = int(src[j])
        u = int(src[j + 1]) - 128
        y1 = int(src[j + 2])
        v = int(src[j + 3]) - 128
        rd = (351 * v) >> 8
        gd = (86 * u + 179 * v) >> 8
        bd = (444 * u) >> 8

        r = y0 + rd
        g = y0 - gd
        b = y0 + bd
        if r < 0:
            r = 0
        if r > 255:
            r = 255
        if g < 0:
            g = 0
        if g > 255:
            g = 255
        if b < 0:
            b = 0
        if b > 255:
            b = 255
        px = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)
        k = 0
        while k < scale:
            dst[o] = px
            o += 1
            k += 1

        r = y1 + rd
        g = y1 - gd
        b = y1 + bd
        if r < 0:
            r = 0
        if r > 255:
            r = 255
        if g < 0:
            g = 0
        if g > 255:
            g = 255
        if b < 0:
            b = 0
        if b > 255:
            b = 255
        px = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)
        k = 0
        while k < scale:
            dst[o] = px
            o += 1
            k += 1

        i += 2
        j += 4


def find_camera(timeout_ms=10000):
    """The first device the enumerator classifies as video."""
    _usbif.host_start(("uvc",))
    deadline = time.ticks_add(time.ticks_ms(), timeout_ms)
    while time.ticks_diff(deadline, time.ticks_ms()) > 0:
        for dev in _usbif.host_devices():
            if "uvc" in dev[5]:
                return dev[0]
        time.sleep_ms(250)
    return None


def pick_mode(dev_id, formats, alts, max_w, max_h):
    """Negotiate the largest uncompressed mode the bus can actually carry.

    Two separate gates, and they have to be asked in this order. Whether a
    mode *exists* is in the descriptors; how much bandwidth it costs is not --
    only the camera can say, and it says it by answering PROBE. So each
    candidate is negotiated for real before it is accepted or rejected.
    """
    candidates = []
    for fmt in formats:
        if fmt.encoding == "mjpeg" or not fmt.frames:
            continue
        for frame in fmt.frames:
            if frame.width > max_w or frame.height > max_h:
                continue
            candidates.append((frame.width * frame.height, fmt, frame))
    candidates.sort(key=lambda c: c[0], reverse=True)

    for _, fmt, frame in candidates:
        for interval in INTERVALS:
            if frame.intervals and interval not in frame.intervals:
                continue
            payload, frame_bytes = _usbif.host_uvc_negotiate(
                dev_id, fmt.interface, fmt.index, frame.index, interval)
            if payload > IN_LIMIT:
                continue
            alt = uvc.alt_for_payload(alts, payload)
            if alt is None:
                continue
            return fmt, frame, interval, payload, frame_bytes, alt
    return None


def main():
    dev_id = find_camera()
    if dev_id is None:
        print("no UVC camera found on the host port")
        return
    print("camera is device", dev_id)

    blob = _usbif.host_desc(dev_id)
    formats = uvc.formats(blob)
    alts = uvc.alt_settings(blob)
    if not formats:
        print("camera declares no video formats")
        return

    picked = pick_mode(dev_id, formats, alts,
                       display_drv.width, display_drv.height)
    if picked is None:
        print("no uncompressed mode fits this host's isochronous IN limit")
        for fmt in formats:
            print("   offered:", uvc.describe(fmt))
        return
    fmt, frame, interval, payload, frame_bytes, alt = picked
    print("streaming", uvc.describe(fmt, frame, interval))
    print("payload %d B/frame on alt %d" % (payload, alt.alt))

    # Whole-number upscale, centred. A 176x144 frame becomes 528x432 on an
    # 800x480 panel; a display smaller than the frame falls back to 1:1.
    scale = min(display_drv.width // frame.width,
                display_drv.height // frame.height) or 1
    out_w = frame.width * scale
    out_h = frame.height * scale
    x0 = (display_drv.width - out_w) // 2
    y0 = (display_drv.height - out_h) // 2
    print("%dx%d upscaled x%d -> %dx%d at (%d, %d)"
          % (frame.width, frame.height, scale, out_w, out_h, x0, y0))

    # One *band* of `scale` identical rows, reused per source row. Two reasons
    # it is a band and not a single row. Building the whole scaled frame would
    # be out_w * out_h * 2 bytes -- close to half a megabyte at 528x432 -- for
    # no benefit. And blit_rect byteswaps its buffer **in place** when the
    # panel needs it, so blitting one row buffer `scale` times would swap it
    # again on every call and leave every repeated row with its colours
    # inverted. One buffer, one blit, one swap.
    band_stride = out_w * 2
    band = bytearray(band_stride * scale)
    band_mv = memoryview(band)
    src = bytearray(frame_bytes)
    src_mv = memoryview(src)
    src_stride = frame.width * 2
    whole_frame = src_stride * frame.height

    _usbif.host_uvc_open(dev_id, fmt.interface, alt.alt, alt.endpoint,
                         alt.max_packet, frame_bytes)
    display_drv.fill(0)
    shown = 0
    t0 = time.ticks_ms()
    try:
        while True:
            n = _usbif.host_uvc_read_frame(src)
            if n <= 0:
                time.sleep_ms(2)
                continue
            # A short frame means the camera sent less than a whole picture.
            # Showing it would tear the bottom of one image across the top of
            # the next, so skip it and leave the last good frame up.
            if n < whole_frame:
                continue
            for sy in range(frame.height):
                yuy2_row_to_rgb565(src_mv[sy * src_stride:], band,
                                   frame.width, scale)
                for k in range(1, scale):
                    band_mv[k * band_stride:(k + 1) * band_stride] = \
                        band_mv[0:band_stride]
                display_drv.blit_rect(band, x0, y0 + sy * scale, out_w, scale)
            # Required, not optional: dotclockframebuffer double-buffers, and
            # FBDisplay leaves needs_refresh False, so nothing else promotes
            # the back buffer. Without this every blit lands and nothing
            # appears.
            display_drv.show()
            shown += 1
            if shown % 25 == 0:
                dt = time.ticks_diff(time.ticks_ms(), t0)
                print("%d frames, %.1f fps, stats %r"
                      % (shown, shown * 1000 / dt, _usbif.host_uvc_stats()))
    except KeyboardInterrupt:
        print("stopping")
    finally:
        _usbif.host_uvc_close()
        print("final stats (frames,packets,bytes,full,torn,big,errors,empty):",
              _usbif.host_uvc_stats())


if __name__ == "__main__":
    main()
