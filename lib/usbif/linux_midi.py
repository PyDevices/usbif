"""MIDI backend for Linux, over ALSA rawmidi.

Reads the kernel's own files rather than linking ``libasound``, for the same
reason :mod:`usbif.linux_usb` reads sysfs rather than linking libudev: this
module has to import on a stock MicroPython unix build as readily as on
CPython, and a shared-library dependency would make "the same program runs
here" false on exactly the port that most needs it. ``/proc/asound/devices``
and ``/dev/snd/midiC*D*`` need no privileges, no library and no package.

**Ports are their own namespace and carry the ALSA address.** An id is
``"in:0:0"`` / ``"out:0:0"`` -- direction, card, device -- because a rawmidi
endpoint is identified by a card/device pair, and the same pair can offer both
directions. Direction leads so the id sorts and reads the same way as the
Windows backend's ``"out:1"``, and so a caller mixing directions gets a lookup
failure rather than the wrong endpoint.

**Reads never block.** The device is opened with ``O_NONBLOCK`` and an empty
read is reported as zero bytes, which is what the contract promises: a polling
application calls :meth:`read` constantly and nothing waiting must stay cheap.
Without that flag a rawmidi read blocks until a byte arrives, which would hang
the caller's service loop on a silent instrument -- the exact failure the
contract's "never blocks, never raises on an empty stream" rule exists to
prevent.
"""

import os

from . import IN, INOUT, OUT, MidiPort, MidiPortInfo

PROC_ASOUND = "/proc/asound"
DEV_SND = "/dev/snd"


def _read_text(path):
    try:
        with open(path) as handle:
            return handle.read()
    except OSError:
        return ""


def _rawmidi_addresses(proc=PROC_ASOUND):
    """Card/device pairs offering raw MIDI, from ``/proc/asound/devices``.

    Lines look like ``  4: [ 0- 0]: raw midi``. The bracketed field is the
    card and device; everything else on the line is a human label whose exact
    spacing is not worth depending on.
    """
    found = []
    for line in _read_text(proc + "/devices").splitlines():
        if "raw midi" not in line:
            continue
        start, end = line.find("["), line.find("]")
        if start < 0 or end < start:
            continue
        inner = line[start + 1:end].replace(" ", "")
        card, _, device = inner.partition("-")
        if card.isdigit() and device.isdigit():
            found.append((int(card), int(device)))
    return tuple(found)


def _describe(card, device, proc=PROC_ASOUND):
    """``(name, direction)`` for one rawmidi endpoint.

    The per-device proc file opens with the hardware's name and then lists an
    ``Output`` and/or ``Input`` section depending on what it offers. A device
    that names neither is reported as bidirectional rather than skipped: being
    unable to read the capability is not evidence the capability is absent,
    and an open attempt will say so honestly.
    """
    text = _read_text("{}/card{}/midi{}".format(proc, card, device))
    name = ""
    for line in text.splitlines():
        candidate = line.strip()
        # Skip the section headers: a device whose name line is empty would
        # otherwise be reported as being called "Output 0".
        if not candidate or candidate.startswith(("Output", "Input")):
            continue
        name = candidate
        break
    has_out = "Output" in text
    has_in = "Input" in text
    if has_out and not has_in:
        direction = OUT
    elif has_in and not has_out:
        direction = IN
    else:
        direction = INOUT
    return name or "card {} device {}".format(card, device), direction


def ports(proc=PROC_ASOUND):
    """Every rawmidi endpoint the kernel is offering.

    A bidirectional endpoint is reported once per direction, so a caller can
    open the two halves independently -- ALSA allows that, and a single INOUT
    record would force a caller wanting only input to also hold an output.
    """
    found = []
    for card, device in _rawmidi_addresses(proc):
        name, direction = _describe(card, device, proc)
        for way in (OUT, IN):
            if direction in (way, INOUT):
                found.append(MidiPortInfo(
                    id="{}:{}:{}".format(way, card, device), name=name, direction=way))
    return tuple(found)


def find(name, direction=None, proc=PROC_ASOUND):
    """Ports whose name contains ``name``, case-insensitively."""
    needle = name.lower()
    return tuple(p for p in ports(proc)
                 if needle in (p.name or "").lower()
                 and (direction is None or p.direction == direction))


def _split_id(port_id):
    parts = str(port_id).split(":")
    if len(parts) != 3 or parts[0] not in (IN, OUT) \
            or not parts[1].isdigit() or not parts[2].isdigit():
        raise ValueError(
            "bad MIDI port id {!r}; expected 'in:CARD:DEVICE' or 'out:CARD:DEVICE'".format(port_id))
    return parts[0], int(parts[1]), int(parts[2])


def open_port(port, dev=DEV_SND):
    """Open a port, given a :class:`usbif.MidiPortInfo` or an id string."""
    info = port if isinstance(port, MidiPortInfo) else None
    port_id = info.id if info is not None else port
    way, card, device = _split_id(port_id)
    if info is None:
        name, _ = _describe(card, device)
        info = MidiPortInfo(id=port_id, name=name, direction=way)
    return LinuxMidiPort(info, "{}/midiC{}D{}".format(dev, card, device), way)


class LinuxMidiPort(MidiPort):
    """One direction of an ALSA rawmidi endpoint."""

    def __init__(self, info, path, way):
        MidiPort.__init__(self, info)
        flags = os.O_WRONLY if way == OUT else os.O_RDONLY
        # O_NONBLOCK is not optional: without it a read on a silent instrument
        # blocks the caller's whole service loop. Older MicroPython unix builds
        # may not define it, in which case an output-only port is still usable
        # and an input port is refused rather than allowed to hang.
        nonblock = getattr(os, "O_NONBLOCK", None)
        if nonblock is None and way != OUT:
            raise OSError(
                "os.O_NONBLOCK is unavailable on this build, so a MIDI input "
                "read could block indefinitely; output ports are unaffected")
        self._fd = os.open(path, flags | (nonblock or 0))

    def _read(self, buf):
        try:
            data = os.read(self._fd, len(buf))
        except OSError:
            return 0                      # EAGAIN: nothing waiting, not an error
        if not data:
            return 0
        n = len(data)
        buf[:n] = data
        return n

    def _write(self, data):
        return os.write(self._fd, bytes(data))

    def _close(self):
        os.close(self._fd)
