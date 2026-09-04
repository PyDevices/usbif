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
        streams = uac.streams(blob)
        if streams:
            found.append((dev[0], streams))
    return tuple(found)


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


class _UacHostMixin:
    """Shared open/close over the native UAC host driver."""

    def _uac_open(self, stream):
        _usbif.host_uac_open(self._dev_id, stream.interface, stream.alt,
                             stream.endpoint, stream.max_packet, self._rate)

    def _close(self):
        _usbif.host_uac_close()

    @property
    def stream(self):
        """The `usbif.uac.UacStream` this device is running."""
        return self._stream

    def stats(self):
        """``(packets, bytes, dropped, starved, errors)`` from the driver.

        Separated on purpose. A stream that sounds wrong is nearly always one
        of three things, and lumping them together loses the answer: the ring
        overflowed because Python was late (``dropped``), the ring was empty
        when the bus asked (``starved``), or the bus itself reported a bad
        packet (``errors``).
        """
        return _usbif.host_uac_stats()


class UacHostOutput(_UacHostMixin, PCMOutput):
    """A hosted USB speaker or audio interface, as a ``PCMOutput``."""

    def __init__(self, dev_id, stream, rate, **kwargs):
        PCMOutput.__init__(self, AudioFormat(rate, stream.channels, stream.bits),
                           **kwargs)
        self._dev_id = dev_id
        self._stream = stream
        self._rate = rate

    def _open(self):
        self._uac_open(self._stream)

    def _write(self, buf):
        # Short writes are normal and not an error: the ring is finite and the
        # bus drains it in real time, so a caller writing faster than realtime
        # is told how much was taken and comes back. That is the same
        # backpressure contract the I2S adapter provides.
        return _usbif.host_uac_write(buf)

    def queued_size(self):
        queued = _usbif.host_uac_queued()
        return max(0, queued)


class UacHostInput(_UacHostMixin, PCMInput):
    """A hosted USB microphone, as a ``PCMInput``."""

    def __init__(self, dev_id, stream, rate, **kwargs):
        PCMInput.__init__(self, AudioFormat(rate, stream.channels, stream.bits),
                          **kwargs)
        self._dev_id = dev_id
        self._stream = stream
        self._rate = rate

    def _open(self):
        self._uac_open(self._stream)

    def _readinto(self, buf):
        return _usbif.host_uac_read(buf)


def output(dev_id, *, rate=None, channels=None, bits=None, **kwargs):
    """Open a hosted USB audio device for playback.

    ``rate``/``channels``/``bits`` filter the device's offered formats; asking
    for something it does not offer raises, listing what it does offer, rather
    than quietly substituting the nearest. Starting a stream at a rate the
    caller did not ask for is how a pitch bug gets shipped.
    """
    _require()
    stream = _pick(dev_id, uac.OUT, rate, channels, bits)
    chosen = rate if rate is not None else (max(stream.rates) if stream.rates else 0)
    return UacHostOutput(dev_id, stream, chosen, **kwargs)


def input(dev_id, *, rate=None, channels=None, bits=None, **kwargs):  # noqa: A001
    """Open a hosted USB audio device for capture."""
    _require()
    stream = _pick(dev_id, uac.IN, rate, channels, bits)
    chosen = rate if rate is not None else (max(stream.rates) if stream.rates else 0)
    return UacHostInput(dev_id, stream, chosen, **kwargs)
