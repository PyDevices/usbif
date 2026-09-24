"""A hosted USB audio device, as an ordinary ``audiodev`` output or input.

Phase 5 item 1. The milestone's wording is the specification: *a board drives
a commercial USB audio interface or headset, selected and played through
``audiodev`` like any other output.* "Like any other output" is the whole
claim -- an application should reach a USB speaker exactly as it reaches an
I2S codec, with the same ``PCMOutput`` surface, the same volume handling, and
no knowledge that a bus is involved.

The division of labour is the one this package states throughout. Choosing the
format happens here, in Python, using :mod:`usbif.uac` to read the device's
descriptor: a UAC device can offer two dozen alternate settings and picking
among them is configuration. Moving the isochronous bytes happens in C, in
``usbif_host_uac.c``, because the bus will not wait for an interpreter.

Volume is deliberately software-side. UAC feature-unit volume control is a
separate control-transfer surface per device and not every device implements
it; ``audiodev``'s own scaling always works, and a device that turns out to
have hardware volume can gain it later without changing what an application
sees.
"""

try:
    from time import sleep_ms, ticks_add, ticks_diff, ticks_ms
except ImportError:  # CPython, where the desktop backends and the tests run
    import time as _time

    def ticks_ms():
        return int(_time.monotonic() * 1000)

    def ticks_add(t, delta):
        return t + delta

    def ticks_diff(a, b):
        return a - b

    def sleep_ms(ms):
        _time.sleep(ms / 1000)

from audiodev import AudioFormat, PCMInput, PCMOutput

from . import uac

try:
    import _usbif
except ImportError:  # pragma: no cover - exercised only off-target
    _usbif = None


def _require():
    if _usbif is None:
        raise ImportError("the usbif native module is not present in this firmware")
    return _usbif


def audio_devices(host_devices=None):
    """Hosted devices offering audio, as ``(dev_id, streams)`` pairs.

    Reads each device's real descriptor rather than trusting the class byte:
    an interface can claim audio and offer no streamable alternate setting at
    all, and a device that cannot actually carry audio should not appear in a
    list of audio outputs.
    """
    _require()
    devices = host_devices if host_devices is not None else _usbif.host_devices()
    found = []
    for dev in devices:
        if "uac" not in (dev[5] or ()):
            continue
        try:
            blob = _usbif.host_desc(dev[0])
        except OSError:
            continue
        streams = _with_clock_rates(dev[0], uac.streams(blob))
        if streams:
            found.append((dev[0], streams))
    return tuple(found)


def _with_clock_rates(dev_id, streams):
    """Fill in the rates of USB Audio 2.0 streams.

    A 2.0 stream's descriptors name a clock source and say nothing about
    rates; the clock answers a RANGE request. One request per clock, and a
    clock that will not answer leaves the stream with no rates, which
    ``choose`` then declines rather than guessing.
    """
    out = []
    asked = {}
    rates_at = uac.STREAM_FIELDS.index("rates")
    for s in streams:
        if s.rates or not s.clock:
            out.append(s)
            continue
        key = (s.control, s.clock)
        if key not in asked:
            try:
                asked[key] = uac.rates_from_ranges(
                    _usbif.host_uac_clock_ranges(dev_id, s.control, s.clock))
            except OSError:
                asked[key] = ()
        fields = list(s)
        fields[rates_at] = asked[key]
        out.append(uac.UacStream(*fields))
    return tuple(out)


def _pick(dev_id, direction, rate, channels, bits):
    for found_id, streams in audio_devices():
        if found_id != dev_id:
            continue
        stream = uac.choose(streams, direction, rate, channels, bits)
        if stream is None:
            raise ValueError(
                "device {} offers no {} stream matching rate={} channels={} bits={}; "
                "it offers: {}".format(dev_id, direction, rate, channels, bits,
                                       ", ".join(uac.describe(s) for s in streams)))
        return stream
    raise ValueError("device {} is not a hosted audio device".format(dev_id))


def _ring_bytes(stream, rate, ring_ms):
    """The ring size ``ring_ms`` asks for, in whole frames; 0 is the default.

    The default is the driver's 8 KB, 43 ms of 48 kHz stereo. That is enough
    for a caller that services the stream from its own tight loop and not
    enough for one on the interpreter thread of an app that also draws a UI
    or talks to the network (usbif#36): ask for the longest stall the app
    has, with room over. Two seconds of 48 kHz stereo is 375 KB, and a ring
    that size lives in PSRAM.
    """
    if ring_ms is None:
        return 0
    frame = stream.channels * stream.frame_bytes
    frames = (int(rate or 48000) * int(ring_ms) + 999) // 1000
    return max(1, frames) * frame


class _UacHostMixin:
    """Shared open/close over the native UAC host driver."""

    def _uac_open(self, stream):
        _usbif.host_uac_open(self._dev_id, stream.interface, stream.alt,
                             stream.endpoint, stream.max_packet, self._rate,
                             stream.clock, stream.control,
                             stream.channels * stream.frame_bytes,
                             _ring_bytes(stream, self._rate, self._ring_ms))

    def _close(self):
        _usbif.host_uac_close()

    @property
    def stream(self):
        """The `usbif.uac.UacStream` this device is running."""
        return self._stream

    def capacity(self):
        """The ring's size in bytes, as opened; 0 while closed."""
        return max(0, _usbif.host_uac_capacity())

    def stats(self):
        """``(packets, bytes, dropped, starved, errors, empty)`` from the driver.

        Separated on purpose. A stream that sounds wrong is nearly always one
        of three things, and lumping them together loses the answer: the ring
        overflowed because Python was late (``dropped``), the ring was empty
        when the bus asked (``starved``), or the bus itself reported a bad
        packet (``errors``).
        """
        return _usbif.host_uac_stats()


class UacHostOutput(_UacHostMixin, PCMOutput):
    """A hosted USB speaker or audio interface, as a ``PCMOutput``.

    ``ring_ms`` sizes the ring between this object and the bus; see
    `output`. `space` says how much a write can take right now, so a caller
    that must not block never has to.
    """

    def __init__(self, dev_id, stream, rate, *, ring_ms=None, **kwargs):
        PCMOutput.__init__(self, AudioFormat(rate, stream.channels, stream.bits),
                           **kwargs)
        self._dev_id = dev_id
        self._stream = stream
        self._rate = rate
        self._ring_ms = ring_ms

    def _open(self):
        self._uac_open(self._stream)

    def _write(self, buf):
        # Short writes are normal and not an error: the ring is finite and the
        # bus drains it in real time, so a caller writing faster than realtime
        # is told how much was taken and comes back. What is not allowed is
        # taking nothing: ``PCMOutput.write`` treats a zero as a stream that
        # has stopped. So a full ring waits here for the bus to drain some of
        # it -- a packet's worth every millisecond -- and only a ring that
        # stays full for far longer than that reports no progress, which then
        # really does mean the transfers are not completing.
        deadline = ticks_add(ticks_ms(), 500)
        while True:
            n = _usbif.host_uac_write(buf)
            if n > 0 or ticks_diff(deadline, ticks_ms()) <= 0:
                return n
            sleep_ms(1)

    def try_write(self, buf):
        """Take what fits now and return at once; never waits for the bus."""
        self.open()
        source = self._prepare(buf)
        return max(0, _usbif.host_uac_write(source))

    def space(self):
        """Bytes a write can take without coming up short, in whole frames."""
        room = _usbif.host_uac_space()
        if room <= 0:
            return 0
        return room - room % self.format.frame_size

    def queued_size(self):
        queued = _usbif.host_uac_queued()
        return max(0, queued)


class UacHostInput(_UacHostMixin, PCMInput):
    """A hosted USB microphone, as a ``PCMInput``."""

    def __init__(self, dev_id, stream, rate, *, ring_ms=None, **kwargs):
        PCMInput.__init__(self, AudioFormat(rate, stream.channels, stream.bits),
                          **kwargs)
        self._dev_id = dev_id
        self._stream = stream
        self._rate = rate
        self._ring_ms = ring_ms

    def _open(self):
        self._uac_open(self._stream)

    def _readinto(self, buf):
        return _usbif.host_uac_read(buf)


def output(dev_id, *, rate=None, channels=None, bits=None, ring_ms=None, **kwargs):
    """Open a hosted USB audio device for playback.

    ``rate``/``channels``/``bits`` filter the device's offered formats; asking
    for something it does not offer raises, listing what it does offer, rather
    than quietly substituting the nearest. Starting a stream at a rate the
    caller did not ask for is how a pitch bug gets shipped.

    ``ring_ms`` is how much audio the driver holds between your writes and
    the bus. Leave it out for the 8 KB default (about 43 ms), which suits a
    loop that does nothing else. An app that feeds the stream from the
    interpreter thread while it also draws or talks to the network should ask
    for longer than its longest stall, for example ``ring_ms=2000``.
    """
    _require()
    stream = _pick(dev_id, uac.OUT, rate, channels, bits)
    chosen = rate if rate is not None else (max(stream.rates) if stream.rates else 0)
    return UacHostOutput(dev_id, stream, chosen, ring_ms=ring_ms, **kwargs)


def input(dev_id, *, rate=None, channels=None, bits=None, ring_ms=None, **kwargs):  # noqa: A001
    """Open a hosted USB audio device for capture. ``ring_ms`` as for `output`."""
    _require()
    stream = _pick(dev_id, uac.IN, rate, channels, bits)
    chosen = rate if rate is not None else (max(stream.rates) if stream.rates else 0)
    return UacHostInput(dev_id, stream, chosen, ring_ms=ring_ms, **kwargs)
