"""MIDI backend for a board, over the ``_usbif`` native module.

Phase 3 item 2. The board can be either end of a MIDI cable -- a device a DAW
plays, or a host driving a controller -- and until now those were two different
APIs: ``_usbif.midi_read``/``midi_write`` for the device role,
``_usbif.host_midi_*`` for the host role. Both speak plain MIDI bytes, and the
C module made them the same shape on purpose, but a caller still had to know
which one to reach for. That makes role a code change.

Here both are :class:`usbif.MidiPort`, so **role is configuration**. The same
program that harmonises, logs or forwards runs unchanged whether the board is
the instrument or the thing driving one -- and, because it is the same contract
the ``winmm`` and ALSA backends satisfy, unchanged on a workstation too.

Ids name the role: ``"dev:midi"`` is the board's own MIDI function, and
``"host:<device id>"`` is a MIDI device attached to the board acting as host.
Both are bidirectional, because both really are.
"""

from . import INOUT, MidiPort, MidiPortInfo

try:
    import _usbif
except ImportError:  # pragma: no cover - exercised only off-target
    _usbif = None

DEVICE_ID = "dev:midi"
_HOST_PREFIX = "host:"


def _require():
    if _usbif is None:
        raise ImportError(
            "the usbif native module is not present in this firmware; "
            "usbif.auto.midi_ports() reports what this platform offers"
        )
    return _usbif


def _device_function_present():
    """True if this firmware is currently presenting a MIDI device function.

    Asked of ``dev_functions()`` rather than ``dev_functions_built()``: a
    function compiled in but not advertised is not a port anyone can open, and
    reporting it would promise a host that is not there.
    """
    try:
        return bool(_usbif.dev_functions() & _usbif.FN_MIDI)
    except (AttributeError, OSError):
        return False


def ports():
    """The board's own MIDI function, plus any MIDI device it is hosting."""
    if _usbif is None:
        return ()
    found = []
    if _device_function_present():
        found.append(MidiPortInfo(id=DEVICE_ID, name="usbif device MIDI",
                                  direction=INOUT))
    try:
        attached = _usbif.host_devices()
    except (AttributeError, OSError):
        attached = ()
    for dev in attached:
        if "midi" in (dev[5] or ()):
            name = dev[3] or "USB MIDI %04x:%04x" % (dev[1] or 0, dev[2] or 0)
            found.append(MidiPortInfo(id="{}{}".format(_HOST_PREFIX, dev[0]),
                                      name=name, direction=INOUT))
    return tuple(found)


def find(name, direction=None):
    needle = name.lower()
    return tuple(p for p in ports()
                 if needle in (p.name or "").lower()
                 and (direction is None or p.direction in (direction, INOUT)))


def _split_id(port_id):
    text = str(port_id)
    if text == DEVICE_ID:
        return "dev", None
    if text.startswith(_HOST_PREFIX):
        rest = text[len(_HOST_PREFIX):]
        if rest.isdigit():
            return "host", int(rest)
    raise ValueError(
        "bad MIDI port id {!r}; expected {!r} or 'host:<device id>'".format(port_id, DEVICE_ID))


def open_port(port):
    """Open the board's MIDI function, or a hosted MIDI device."""
    _require()
    info = port if isinstance(port, MidiPortInfo) else None
    port_id = info.id if info is not None else port
    role, dev_id = _split_id(port_id)
    if info is None:
        info = MidiPortInfo(id=port_id, direction=INOUT,
                            name="usbif device MIDI" if role == "dev"
                            else "hosted MIDI device {}".format(dev_id))
    return NativeDeviceMidi(info) if role == "dev" else NativeHostMidi(info, dev_id)


class NativeDeviceMidi(MidiPort):
    """The board's own USB MIDI function, as seen by a host."""

    def _read(self, buf):
        return _usbif.midi_read(buf)

    def _write(self, data):
        return _usbif.midi_write(bytes(data))


class NativeHostMidi(MidiPort):
    """A MIDI device attached to the board, with the board as host."""

    def __init__(self, info, dev_id):
        MidiPort.__init__(self, info)
        _usbif.host_midi_open(dev_id)
        self._dev_id = dev_id

    def _read(self, buf):
        return _usbif.host_midi_read(buf)

    def _write(self, data):
        return _usbif.host_midi_write(bytes(data))

    def _close(self):
        _usbif.host_midi_close()

    def dropped(self):
        """``(rx_dropped, release_failed)`` from the host MIDI driver.

        Exposed because a silent drop is the one failure a MIDI stream cannot
        show you: missing notes look like a quiet passage.
        """
        return _usbif.host_midi_dropped()
