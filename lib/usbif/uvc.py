"""USB Video Class descriptor parsing, for the host role.

The companion to :mod:`usbif.uac`, and it exists for the same reason: a
class-compliant device announces what it can do in *class-specific*
descriptors interleaved through the configuration blob, and a host has to walk
all of them before it can ask for a single frame. Doing that walk in Python
means a mis-parsed descriptor costs a re-run instead of a reflash, and means
the same parser can be pointed at a blob captured from any machine.

UVC differs from UAC in one structural way that shapes this whole module, and
it is worth stating plainly because it is easy to model wrongly.

In UAC, a format *is* an alternate setting: to pick 48 kHz stereo you claim
alt 1, and the format descriptors live inside that alt. In UVC the formats do
not live in the alts at all. They are declared once, on **alt 0** of the
VideoStreaming interface, as a tree of format descriptors each owning a list
of frame descriptors::

    INTERFACE  class=0x0E subclass=1     VideoControl
      CS_INTERFACE VC_HEADER, terminals, units   -- the camera's topology
    INTERFACE  class=0x0E subclass=2     VideoStreaming, alt 0
      CS_INTERFACE VS_INPUT_HEADER       how many formats, which endpoint
      CS_INTERFACE VS_FORMAT_MJPEG       format 1
        CS_INTERFACE VS_FRAME_MJPEG      1280x720, its frame intervals
        CS_INTERFACE VS_FRAME_MJPEG      640x480, ...
      CS_INTERFACE VS_FORMAT_UNCOMPRESSED  format 2
        CS_INTERFACE VS_FRAME_UNCOMPRESSED ...
    INTERFACE  ... alt 1                 isochronous endpoint, small packets
    INTERFACE  ... alt 2, alt 3          the same endpoint, bigger packets

The alts are pure *bandwidth* tiers: same endpoint, different
``wMaxPacketSize``. Choosing a video mode is therefore two independent
decisions -- which (format, frame) you want, and which alt is big enough to
carry it -- which is why :func:`formats` and :func:`alt_settings` are separate
functions and :func:`choose` takes both.

The actual mode is then negotiated over the control endpoint with
VS_PROBE/VS_COMMIT, which is a transaction rather than a descriptor read and
so does not belong here.

Only MJPEG and uncompressed (YUY2/NV12-style) formats are read. Frame-based
formats use a different frame layout -- ``dwBytesPerLine`` where these have
``dwMaxVideoFrameBufferSize`` -- and are reported by encoding name with no
frames rather than silently mis-parsed.
"""

try:
    from collections import namedtuple
except ImportError:  # pragma: no cover - ucollections on older firmware
    from ucollections import namedtuple

# The configuration-blob walk is generic, not audio-specific; it is shared
# from uac rather than copied, because two parsers that drift apart is a
# worse outcome than one import that reads oddly.
from .uac import descriptors

# Descriptor types
DT_INTERFACE = 0x04
DT_ENDPOINT = 0x05
DT_CS_INTERFACE = 0x24

# Video interface class and subclasses
CLASS_VIDEO = 0x0E
SUBCLASS_VIDEOCONTROL = 0x01
SUBCLASS_VIDEOSTREAMING = 0x02

# VideoStreaming class-specific interface descriptor subtypes
VS_INPUT_HEADER = 0x01
VS_FORMAT_UNCOMPRESSED = 0x04
VS_FRAME_UNCOMPRESSED = 0x05
VS_FORMAT_MJPEG = 0x06
VS_FRAME_MJPEG = 0x07
VS_FORMAT_FRAME_BASED = 0x10
VS_FRAME_FRAME_BASED = 0x11

# Endpoint attribute bits
EP_XFER_MASK = 0x03
EP_XFER_ISOC = 0x01
EP_XFER_BULK = 0x02

# Frame intervals are in 100 ns units throughout UVC.
INTERVAL_UNITS_PER_SECOND = 10000000

# The leading four bytes of an uncompressed format's GUID are its FourCC.
_KNOWN_FOURCC = ("YUY2", "NV12", "UYVY", "I420", "Y800", "BY8 ", "RGBP")

FRAME_FIELDS = ("index", "width", "height", "intervals", "max_frame_bytes",
                "default_interval")
FORMAT_FIELDS = ("interface", "index", "encoding", "bits_per_pixel",
                 "default_frame_index", "frames")
ALT_FIELDS = ("interface", "alt", "endpoint", "max_packet", "per_frame",
              "interval", "transfer")

UvcFrame = namedtuple("UvcFrame", " ".join(FRAME_FIELDS))  # noqa: PYI024
UvcFormat = namedtuple("UvcFormat", " ".join(FORMAT_FIELDS))  # noqa: PYI024
UvcAlt = namedtuple("UvcAlt", " ".join(ALT_FIELDS))  # noqa: PYI024


def _u16(body, offset):
    return body[offset] | (body[offset + 1] << 8)


def _u32(body, offset):
    return (body[offset] | (body[offset + 1] << 8)
            | (body[offset + 2] << 16) | (body[offset + 3] << 24))


def fps(interval):
    """Frames per second for a frame interval in 100 ns units.

    Returned as a float and *not* rounded: 30 fps is stored as 333333, which
    is 30.000030..., and a camera that means 29.97 stores 333667. Rounding
    here would erase exactly the distinction a caller might be looking for.
    """
    if not interval:
        return 0.0
    return INTERVAL_UNITS_PER_SECOND / interval


def _fourcc(body, offset):
    """The FourCC at the head of an uncompressed format's 16-byte GUID."""
    try:
        name = bytes(body[offset:offset + 4]).decode("ascii")
    except (UnicodeError, ValueError):
        return None
    return name if name in _KNOWN_FOURCC else name.strip() or None


def _frame(body, length):
    """One VS_FRAME_{MJPEG,UNCOMPRESSED} descriptor.

    Both subtypes share this layout exactly; only the subtype byte differs,
    which is why one reader serves both.
    """
    if length < 26:
        return None
    count = body[25]
    intervals = []
    if count == 0:
        # Continuous: min, max, step. Reported as (min, max) -- the step is
        # not a rate the camera offers, and presenting three numbers as if
        # they were three rates would be a lie a caller might act on.
        if length >= 38:
            intervals = [_u32(body, 26), _u32(body, 30)]
    else:
        for i in range(count):
            base = 26 + i * 4
            if base + 4 > length:
                break
            intervals.append(_u32(body, base))
    return UvcFrame(
        index=body[3],
        width=_u16(body, 5),
        height=_u16(body, 7),
        intervals=tuple(intervals),
        max_frame_bytes=_u32(body, 17),
        default_interval=_u32(body, 21),
    )


def formats(blob):
    """Every (format, frames) group the device declares, in descriptor order.

    Frames are attached to the format descriptor that precedes them, which is
    how UVC associates them -- there is no back-reference in a frame
    descriptor to its format.
    """
    found = []
    cur_itf = None
    cur_alt = None
    is_vs = False
    pending = None
    for length, dtype, body in descriptors(blob):
        if dtype == DT_INTERFACE and length >= 9:
            cur_itf, cur_alt = body[2], body[3]
            is_vs = (body[5] == CLASS_VIDEO
                     and body[6] == SUBCLASS_VIDEOSTREAMING)
            pending = None
            continue
        if dtype != DT_CS_INTERFACE or length < 3:
            continue
        # VideoStreaming only, and alt 0 only.
        #
        # The subclass check is not belt-and-braces -- the subtype numbers
        # collide outright. In a VideoControl interface, subtype 4 is
        # VC_SELECTOR_UNIT, 5 is VC_PROCESSING_UNIT and 6 is
        # VC_EXTENSION_UNIT, which are exactly the numbers VideoStreaming
        # uses for FORMAT_UNCOMPRESSED, FRAME_UNCOMPRESSED and FORMAT_MJPEG.
        # Without this, a real camera's control topology parses as a handful
        # of phantom formats: the C920e on the bench reported six MJPEG
        # formats with zero frames each, one per extension unit. The
        # synthetic fixture could not catch it -- it had no units to trip on,
        # which is precisely the kind of gap only real hardware fills.
        if not is_vs or cur_alt != 0:
            continue
        subtype = body[2]
        if subtype in (VS_FORMAT_MJPEG, VS_FORMAT_UNCOMPRESSED,
                       VS_FORMAT_FRAME_BASED):
            if subtype == VS_FORMAT_MJPEG:
                encoding, bpp, default_index = "mjpeg", 0, body[6]
            elif subtype == VS_FORMAT_UNCOMPRESSED:
                encoding = _fourcc(body, 5) or "uncompressed"
                bpp = body[21] if length > 21 else 0
                default_index = body[22] if length > 22 else 1
            else:
                encoding, bpp, default_index = "frame-based", 0, 0
            # Fields held loose until the frames that follow are all in.
            # MicroPython's namedtuple has no _replace(), so a record cannot
            # be built now and completed later -- it has to be built once,
            # complete. (CPython's does, which is exactly why the desktop
            # tests were happy with the version that could not run on a board.)
            pending = [cur_itf, body[3], encoding, bpp, default_index, []]
            found.append(pending)
        elif subtype in (VS_FRAME_MJPEG, VS_FRAME_UNCOMPRESSED):
            if pending is None:
                continue
            frame = _frame(body, length)
            if frame is not None:
                pending[5].append(frame)
    # Built now that the frame lists are complete, and with the frames frozen
    # so a returned record cannot be mutated by a later parse of the blob.
    return tuple(
        UvcFormat(interface=f[0], index=f[1], encoding=f[2],
                  bits_per_pixel=f[3], default_frame_index=f[4],
                  frames=tuple(f[5]))
        for f in found
    )


def alt_settings(blob):
    """The VideoStreaming bandwidth tiers: one record per alt with an endpoint.

    Alt 0 is included only if it carries an endpoint, which for a bulk-only
    camera it does. Isochronous alt 0 never does -- that is what makes it the
    zero-bandwidth setting a host parks on when not streaming.
    """
    found = []
    cur_itf = None
    cur_alt = None
    is_vs = False
    for length, dtype, body in descriptors(blob):
        if dtype == DT_INTERFACE and length >= 9:
            cur_itf, cur_alt = body[2], body[3]
            is_vs = (body[5] == CLASS_VIDEO
                     and body[6] == SUBCLASS_VIDEOSTREAMING)
            continue
        if not is_vs or dtype != DT_ENDPOINT or length < 7:
            continue
        raw = _u16(body, 4)
        kind = body[3] & EP_XFER_MASK
        found.append(UvcAlt(
            interface=cur_itf,
            alt=cur_alt,
            endpoint=body[2],
            # Bits 10:0 are the packet size; bits 12:11 are additional
            # transactions per microframe, which is a high-speed concept and
            # reads as 0 (one transaction) on the full-speed bus this host
            # actually drives.
            max_packet=raw & 0x7FF,
            per_frame=((raw >> 11) & 0x03) + 1,
            interval=body[6],
            transfer="isoc" if kind == EP_XFER_ISOC else
                     ("bulk" if kind == EP_XFER_BULK else "other"),
        ))
    return tuple(found)


def has_video(blob):
    """True if the blob declares a VideoStreaming interface at all."""
    for length, dtype, body in descriptors(blob):
        if (dtype == DT_INTERFACE and length >= 9
                and body[5] == CLASS_VIDEO
                and body[6] == SUBCLASS_VIDEOSTREAMING):
            return True
    return False


def required_packet_bytes(frame, interval):
    """Bytes per bus frame needed to carry ``frame`` at ``interval``.

    The full-speed bus schedules one isochronous packet every 1 ms, so this is
    the per-millisecond share of a whole video frame.

    This is an ESTIMATE and an upper bound, useful for reasoning about
    uncompressed formats. Do not select an alternate setting with it -- see
    :func:`alt_for_payload`. ``max_frame_bytes`` is a worst case, and for
    MJPEG a very loose one, so this over-reports by a wide margin on exactly
    the format most cameras actually stream.
    """
    if not interval or not frame.max_frame_bytes:
        return 0
    per_second = frame.max_frame_bytes * fps(interval)
    return int(per_second / 1000 + 0.999)


def alt_for_payload(alts, payload_bytes):
    """The smallest isochronous alt that can carry ``payload_bytes`` per frame.

    This is the second half of setting up a stream, and it is deliberately not
    folded into :func:`choose`. A host does not compute the payload size --
    it asks. VS_PROBE/VS_COMMIT is a negotiation in which the *device* returns
    ``dwMaxPayloadTransferSize`` for the mode being requested, and that number
    is what selects the alt.

    Guessing instead, from ``dwMaxVideoFrameBufferSize`` and a frame rate, does
    not work and cannot be made to: that field is a worst case, and for MJPEG a
    very loose one. Taken literally it says 320x240 at 30 fps needs 4608 bytes
    every millisecond, which no full-speed alternate setting can carry, so a
    chooser built on it would refuse every mode on this bus while real cameras
    stream those modes comfortably.

    Returns ``None`` if nothing fits, which is a real answer: it means the mode
    was negotiated but this interface cannot carry it.
    """
    fits = [a for a in alts
            if a.transfer == "isoc" and a.max_packet * a.per_frame >= payload_bytes]
    if not fits:
        return None
    return min(fits, key=lambda a: a.max_packet * a.per_frame)


def choose(formats_found, width=None, height=None, encoding=None,
           min_fps=None):
    """Pick a ``(format, frame, interval)`` for the requested mode, or ``None``.

    The *mode* only -- see :func:`alt_for_payload` for why the alternate
    setting is a separate, later decision.

    Unspecified criteria do not constrain. Among matching modes the largest
    frame wins, and at equal size the fastest rate.

    Returns ``None`` rather than raising when nothing matches: a device not
    offering a mode is an ordinary outcome for a host that has just met it.
    """
    best = None
    for fmt in formats_found:
        if encoding is not None and fmt.encoding != encoding:
            continue
        for frame in fmt.frames:
            if width is not None and frame.width != width:
                continue
            if height is not None and frame.height != height:
                continue
            intervals = frame.intervals or (frame.default_interval,)
            for interval in intervals:
                if min_fps is not None and fps(interval) < min_fps:
                    continue
                score = (frame.width * frame.height, fps(interval))
                if best is None or score > best[0]:
                    best = (score, fmt, frame, interval)
    if best is None:
        return None
    return best[1], best[2], best[3]


def describe(fmt, frame=None, interval=None):
    """A one-line human description, for a REPL or a log."""
    if frame is None:
        return "%s format %d, %d frame size%s" % (
            fmt.encoding, fmt.index, len(fmt.frames),
            "" if len(fmt.frames) == 1 else "s")
    rate = interval or frame.default_interval
    return "%s %dx%d @ %.4g fps (frame %d, up to %d bytes)" % (
        fmt.encoding, frame.width, frame.height, fps(rate),
        frame.index, frame.max_frame_bytes)
