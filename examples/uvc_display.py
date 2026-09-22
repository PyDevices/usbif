"""Live camera preview: a hosted USB webcam shown on the board's own display.

The board is the USB *host* here. It enumerates a class-compliant UVC camera,
reads its descriptors, negotiates a video mode with it, and streams frames in
over isochronous transfers -- then puts them on the display that
``board_config`` already set up. No PC in the loop at any point.

Run it on a board with a display and a camera on its host port::

    mpremote run uvc_display.py

**MJPEG when ``jpegio`` is present.** A webcam offers far better resolutions
in MJPEG than uncompressed -- on the bench camera, 640x480 against 176x144 --
so MJPEG is preferred when the firmware has ``jpegio`` (displayif). Frames
arrive as whole JPEGs; ``jpegio.JpegDecoder`` sniffs SOI only, which is what
UVC needs (a UVC MJPEG frame is not JFIF-first, so LVGL's ``is_jpg()`` rejects
it). Without ``jpegio`` the example falls back to uncompressed YUY2, converted
here and blitted to ``display_drv``.

**Why YUY2 is upscaled by whole numbers.** Nearest-neighbour at an integer
factor is a few instructions per pixel and needs no line buffer beyond one
row. Anything smoother is a real resampler, which is a different example.

**Pairing.** A PyDevices board presenting as a webcam (``usbif_webcam.py`` on
a P4) is a valid camera for this script on an S3, the same way a Logitech is.
P4 high-speed host is blocked (usbif#3), so the host role here is an S3.
"""

import time

import appdev
import board_config
import micropython
from board_config import display_drv

import usbif.auto
from usbif import uvc

app = appdev.App(board_config)

# The camera's data endpoint is fed one packet per bus frame, so a mode is
# only reachable if its negotiated payload fits the host's isochronous IN
# limit. That limit is set by the DWC FIFO bias chosen at build time -- see
# usbif#2 -- and is 600 bytes on the current firmware. Modes are tried
# largest-first and the first one the camera and the bus both accept wins.
IN_LIMIT = 600

# 100 ns units, which is how UVC counts frame intervals throughout.
INTERVALS = (2000000, 1333333, 1000000, 666666, 333333)   # 5, 7.5, 10, 15, 30 fps

try:
    import jpegio
    _HAVE_JPEGIO = True
except ImportError:
    jpegio = None
    _HAVE_JPEGIO = False


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


def find_camera(host, timeout_ms=10000):
    """The first device the enumerator classifies as video."""
    deadline = time.ticks_add(time.ticks_ms(), timeout_ms)
    while time.ticks_diff(deadline, time.ticks_ms()) > 0:
        found = host.find("uvc")
        if found:
            return found[0].id
        time.sleep_ms(250)
    return None


def pick_mode(dev_id, formats, alts, max_w, max_h, prefer_mjpeg):
    """Negotiate the largest mode the bus can actually carry.

    Two separate gates, and they have to be asked in this order. Whether a
    mode *exists* is in the descriptors; how much bandwidth it costs is not --
    only the camera can say, and it says it by answering PROBE. So each
    candidate is negotiated for real before it is accepted or rejected.

    When ``prefer_mjpeg`` is true, MJPEG modes are tried first; otherwise
    only uncompressed encodings are considered.
    """
    candidates = []
    for fmt in formats:
        is_mjpeg = fmt.encoding == "mjpeg"
        if prefer_mjpeg:
            if not is_mjpeg and not fmt.frames:
                continue
            if not is_mjpeg:
                # Prefer MJPEG; keep uncompressed as a fallback pass below.
                continue
        else:
            if is_mjpeg or not fmt.frames:
                continue
        if not fmt.frames:
            continue
        for frame in fmt.frames:
            if frame.width > max_w or frame.height > max_h:
                continue
            # MJPEG first when asked: score by size, then prefer mjpeg in the
            # sort key so equal sizes still land on the compressed path.
            rank = 1 if is_mjpeg else 0
            candidates.append((rank, frame.width * frame.height, fmt, frame))
    candidates.sort(key=lambda c: (c[0], c[1]), reverse=True)

    for _, _, fmt, frame in candidates:
        for interval in INTERVALS:
            if frame.intervals and interval not in frame.intervals:
                continue
            payload, frame_bytes = host.uvc_negotiate(
                dev_id, fmt.interface, fmt.index, frame.index, interval)
            if payload > IN_LIMIT:
                continue
            alt = uvc.alt_for_payload(alts, payload)
            if alt is None:
                continue
            return fmt, frame, interval, payload, frame_bytes, alt
    return None


def _offered(formats):
    for fmt in formats:
        print("   offered:", uvc.describe(fmt))


def _scale_geometry(frame):
    scale = min(display_drv.width // frame.width,
                display_drv.height // frame.height) or 1
    out_w = frame.width * scale
    out_h = frame.height * scale
    x0 = (display_drv.width - out_w) // 2
    y0 = (display_drv.height - out_h) // 2
    return scale, out_w, out_h, x0, y0


host = usbif.auto.host(classes=("uvc",)).start()
dev_id = find_camera(host)
picked = None
if dev_id is None:
    print("no UVC camera found on the host port")
else:
    print("camera is device", dev_id)
    if _HAVE_JPEGIO:
        print("jpegio present -- preferring MJPEG")
    else:
        print("jpegio absent -- uncompressed YUY2 only")
    blob = host.desc(dev_id)
    formats = uvc.formats(blob)
    alts = uvc.alt_settings(blob)
    if not formats:
        print("camera declares no video formats")
    else:
        if _HAVE_JPEGIO:
            picked = pick_mode(dev_id, formats, alts,
                               display_drv.width, display_drv.height, True)
        if picked is None:
            picked = pick_mode(dev_id, formats, alts,
                               display_drv.width, display_drv.height, False)
        if picked is None:
            print("no mode fits this host's isochronous IN limit")
            _offered(formats)

if picked is not None:
    fmt, frame, interval, payload, frame_bytes, alt = picked
    print("streaming", uvc.describe(fmt, frame, interval))
    print("payload %d B/frame on alt %d" % (payload, alt.alt))

    scale, out_w, out_h, x0, y0 = _scale_geometry(frame)
    print("%dx%d upscaled x%d -> %dx%d at (%d, %d)"
          % (frame.width, frame.height, scale, out_w, out_h, x0, y0))

    src = bytearray(frame_bytes)
    host.uvc_open(dev_id, fmt.interface, alt.alt, alt.endpoint,
                  alt.max_packet, frame_bytes)
    display_drv.fill(0)
    _shown = 0
    _t0 = time.ticks_ms()

    if fmt.encoding == "mjpeg":
        decoder = jpegio.JpegDecoder()
        # Native-order RGB565, tight. Sized for the negotiated frame; a camera
        # that sends a larger JPEG than it advertised is refused by decode.
        rgb = bytearray(frame.width * frame.height * 2)
        # Nearest-neighbour upscale into a band, same reason as the YUY2 path:
        # blit_rect byteswaps in place, so one band / one blit / one swap.
        band_stride = out_w * 2
        band = bytearray(band_stride * scale)
        band_mv = memoryview(band)
        rgb_mv = memoryview(rgb)
        src_stride = frame.width * 2

        def _tick(_=None):
            global _shown
            n = host.uvc_read_frame(src)
            if n <= 0:
                return
            try:
                decoder.open(memoryview(src)[:n])
            except Exception:
                return
            if decoder.width != frame.width or decoder.height != frame.height:
                return
            try:
                decoder.decode(rgb, scale=0)
            except Exception:
                return
            # Integer upscale row by row into the band, then blit.
            for sy in range(frame.height):
                row = rgb_mv[sy * src_stride:(sy + 1) * src_stride]
                if scale == 1:
                    display_drv.blit_rect(row, x0, y0 + sy, out_w, 1)
                else:
                    # Expand horizontally into the first band row, then
                    # replicate vertically.
                    o = 0
                    for px in range(0, src_stride, 2):
                        pix = row[px:px + 2]
                        for _k in range(scale):
                            band_mv[o:o + 2] = pix
                            o += 2
                    for k in range(1, scale):
                        band_mv[k * band_stride:(k + 1) * band_stride] = \
                            band_mv[0:band_stride]
                    display_drv.blit_rect(band, x0, y0 + sy * scale, out_w, scale)
            display_drv.show()
            _shown += 1
            if _shown % 25 == 0:
                dt = time.ticks_diff(time.ticks_ms(), _t0)
                print("%d frames, %.1f fps, stats %r"
                      % (_shown, _shown * 1000 / dt, host.uvc_stats()))

    else:
        # Uncompressed YUY2 path.
        band_stride = out_w * 2
        band = bytearray(band_stride * scale)
        band_mv = memoryview(band)
        src_mv = memoryview(src)
        src_stride = frame.width * 2
        whole_frame = src_stride * frame.height

        def _tick(_=None):
            """Blit a frame if one has arrived; cheap when none has.

            Scheduled rather than looped. A ``while True`` here would work and
            would be wrong: it owns the interpreter, so touch, the REPL and
            anything else the app is running never get a turn.
            """
            global _shown
            n = host.uvc_read_frame(src)
            if n <= 0:
                return
            # A short frame means the camera sent less than a whole picture.
            # Showing it would tear; skip and leave the last good frame up.
            if n < whole_frame:
                return
            for sy in range(frame.height):
                yuy2_row_to_rgb565(src_mv[sy * src_stride:], band,
                                   frame.width, scale)
                for k in range(1, scale):
                    band_mv[k * band_stride:(k + 1) * band_stride] = \
                        band_mv[0:band_stride]
                display_drv.blit_rect(band, x0, y0 + sy * scale, out_w, scale)
            display_drv.show()
            _shown += 1
            if _shown % 25 == 0:
                dt = time.ticks_diff(time.ticks_ms(), _t0)
                print("%d frames, %.1f fps, stats %r"
                      % (_shown, _shown * 1000 / dt, host.uvc_stats()))

    # 10 ms. Frames arrive every 200 ms at 5 fps, so nearly every tick returns
    # immediately.
    app.every(_tick, period=10, async_=app.timer_async)
