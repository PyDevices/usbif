"""USB Audio Class descriptor parsing, for the host role.

A UAC device does not announce its formats in its interface descriptors. It
announces them in *class-specific* descriptors interleaved between them, and a
host has to walk the whole configuration blob to learn what the device can do
before it can choose an alternate setting and start streaming. That walk is
this module.

It lives in Python, and in this package rather than in the C module, for the
reason the package docstring already gives: **Python configures and observes;
C moves isochronous bytes.** Choosing a format is configuration. Doing it here
means a mis-parsed descriptor costs a re-run instead of a reflash, and means
the same parser can be pointed at a descriptor captured from any host --
including one from a machine that is not the board.

Only UAC 1.0 is handled, which is what class-compliant devices that work
without a driver actually implement, and what the ESP32's full-speed host can
carry. UAC 2.0's descriptors differ enough to be a separate reader rather than
a flag on this one.

The shape of a configuration blob, for the reader who has not stared at one::

    CONFIGURATION
      INTERFACE  class=1 subclass=1   AudioControl
        CS_INTERFACE ...              topology: terminals and units
      INTERFACE  class=1 subclass=2   AudioStreaming, alt 0  -- always silent
        (no endpoint: alt 0 exists so a device can consume no bandwidth)
      INTERFACE  class=1 subclass=2   AudioStreaming, alt 1
        CS_INTERFACE AS_GENERAL       which terminal, which format tag
        CS_INTERFACE FORMAT_TYPE      channels, bit depth, sample rates
        ENDPOINT     isochronous      address, packet size, interval
      INTERFACE  ... alt 2, alt 3     the same again at other formats

Choosing a format therefore means choosing an *alternate setting*, which is
why :func:`streams` returns one record per alt rather than one per interface.
"""

try:
    from collections import namedtuple
except ImportError:  # pragma: no cover - ucollections on older firmware
    from ucollections import namedtuple

# Descriptor types
DT_INTERFACE = 0x04
DT_ENDPOINT = 0x05
DT_CS_INTERFACE = 0x24
DT_CS_ENDPOINT = 0x25

# Audio interface class and subclasses
CLASS_AUDIO = 0x01
SUBCLASS_AUDIOCONTROL = 0x01
SUBCLASS_AUDIOSTREAMING = 0x02

# AudioStreaming class-specific interface descriptor subtypes
AS_GENERAL = 0x01
AS_FORMAT_TYPE = 0x02

# Endpoint attribute bits
EP_XFER_MASK = 0x03
EP_XFER_ISOC = 0x01
EP_SYNC_MASK = 0x0C
EP_SYNC_ASYNC = 0x04
EP_SYNC_ADAPTIVE = 0x08
EP_SYNC_SYNC = 0x0C
EP_USAGE_MASK = 0x30
EP_USAGE_FEEDBACK = 0x10

IN, OUT = "in", "out"

STREAM_FIELDS = ("interface", "alt", "endpoint", "direction", "rates",
                 "channels", "bits", "frame_bytes", "max_packet", "interval",
                 "sync", "terminal")

UacStream = namedtuple("UacStream", " ".join(STREAM_FIELDS))  # noqa: PYI024


def descriptors(blob):
    """Walk a configuration blob, yielding ``(length, type, memoryview)``.

    Stops at the first zero-length descriptor rather than looping forever: a
    truncated or padded blob is a real thing to receive from real hardware,
    and a parser that hangs on one is worse than a parser that stops early.
    """
    view = memoryview(blob)
    offset = 0
    end = len(view)
    while offset + 2 <= end:
        length = view[offset]
        if length < 2 or offset + length > end:
            return
        yield length, view[offset + 1], view[offset:offset + length]
        offset += length


def _rates(body, offset, count):
    """Sample rates from a FORMAT_TYPE_I descriptor.

    ``bSamFreqType`` is 0 for a continuous min..max pair, or a count of
    discrete rates. Both are three-byte little-endian, which is the detail
    that makes a naive 4-byte read return plausible nonsense.
    """
    out = []
    for i in range(count if count else 2):
        base = offset + i * 3
        if base + 3 > len(body):
            break
        out.append(body[base] | (body[base + 1] << 8) | (body[base + 2] << 16))
    return tuple(out)


def streams(blob):
    """Every AudioStreaming alternate setting that can actually carry audio.

    Alt 0 is deliberately excluded: by specification it has no endpoint and
    exists so a device can be configured while consuming no bus bandwidth.
    Returning it as a choosable stream would offer a format that is silent by
    design, which is the sort of thing that looks like a driver bug later.

    Feedback endpoints are excluded for the same reason -- they carry rate
    corrections, not samples.
    """
    found = []
    itf = alt = None
    pending = None

    for length, dtype, body in descriptors(blob):
        if dtype == DT_INTERFACE and length >= 9:
            if pending is not None:
                found.append(pending)
                pending = None
            itf, alt = body[2], body[3]
            if body[5] == CLASS_AUDIO and body[6] == SUBCLASS_AUDIOSTREAMING and alt != 0:
                pending = {"interface": itf, "alt": alt, "terminal": None,
                           "rates": (), "channels": 0, "bits": 0, "frame_bytes": 0,
                           "endpoint": None, "direction": None,
                           "max_packet": 0, "interval": 0, "sync": None}
            continue

        if pending is None:
            continue

        if dtype == DT_CS_INTERFACE and length >= 3:
            subtype = body[2]
            if subtype == AS_GENERAL and length >= 7:
                pending["terminal"] = body[3]
            elif subtype == AS_FORMAT_TYPE and length >= 8:
                pending["channels"] = body[4]
                pending["frame_bytes"] = body[5]      # bSubframeSize
                pending["bits"] = body[6]             # bBitResolution
                pending["rates"] = _rates(body, 8, body[7])
        elif dtype == DT_ENDPOINT and length >= 7:
            attrs = body[3]
            if (attrs & EP_XFER_MASK) != EP_XFER_ISOC:
                continue
            if (attrs & EP_USAGE_MASK) == EP_USAGE_FEEDBACK:
                continue                              # rate corrections, not audio
            address = body[2]
            pending["endpoint"] = address
            pending["direction"] = IN if address & 0x80 else OUT
            pending["max_packet"] = body[4] | (body[5] << 8)
            pending["interval"] = body[6]
            sync = attrs & EP_SYNC_MASK
            pending["sync"] = {EP_SYNC_ASYNC: "async", EP_SYNC_ADAPTIVE: "adaptive",
                               EP_SYNC_SYNC: "sync"}.get(sync, "none")

    if pending is not None:
        found.append(pending)

    return tuple(UacStream(**s) for s in found if s["endpoint"] is not None)


def has_audio(blob):
    """True if this configuration offers any AudioStreaming interface."""
    for length, dtype, body in descriptors(blob):
        if dtype == DT_INTERFACE and length >= 9 \
                and body[5] == CLASS_AUDIO and body[6] == SUBCLASS_AUDIOSTREAMING:
            return True
    return False


def choose(streams_found, direction, rate=None, channels=None, bits=None):
    """Pick the best stream for a direction, or ``None``.

    Preference order, most specific first: an exact match on everything the
    caller asked for, then the highest rate available, then the widest bit
    depth. A caller that asks for nothing gets the device's best offer, which
    is what "select it like any other output" should mean.
    """
    candidates = [s for s in streams_found if s.direction == direction]
    if rate is not None:
        candidates = [s for s in candidates if rate in s.rates]
    if channels is not None:
        candidates = [s for s in candidates if s.channels == channels]
    if bits is not None:
        candidates = [s for s in candidates if s.bits == bits]
    if not candidates:
        return None
    return max(candidates, key=lambda s: (max(s.rates) if s.rates else 0, s.bits,
                                          s.channels))


def describe(stream):
    """One-line human description of a stream, for logs and REPL use."""
    rates = "/".join(str(r) for r in stream.rates) if stream.rates else "?"
    return "itf {} alt {}: {} {} Hz x {}ch x {}bit, ep {:#04x} {} {}B/{}ms".format(
        stream.interface, stream.alt, stream.direction, rates, stream.channels,
        stream.bits, stream.endpoint, stream.sync, stream.max_packet,
        stream.interval)
