"""Portable USB host and device contracts for MicroPython and CPython.

Backends subclass :class:`Host` and :class:`Device`. Optional host selection
lives in :mod:`usbif.auto` and is never imported from here, matching
``audiodev`` and ``displaydev``.

Two rules shape everything in this package.

**Python configures and observes; C moves isochronous bytes.** A caller sets a
stream up and watches it run. It never pumps audio or video frames through the
interpreter, because isochronous endpoints must be serviced every USB frame and
the VM cannot promise that (see the transport note below).

**Capabilities are discovered, never assumed.** The same import succeeds on a
board with a full host stack, on a desktop where the OS owns the bus, and on a
port with no USB at all. What differs is what :meth:`Host.capabilities` and
:meth:`Device.capabilities` return, so application code branches on a
frozenset rather than on ``ImportError`` or a chip name.

Transport, and why events arrive by draining rather than by callback: on ESP32
a C-side callback reaches Python through ``mp_sched_schedule``, which is
excellent while the VM runs bytecode and catastrophic inside a long C call.
Measured on an ESP32-S3 at a 1 kHz event rate, a ``sha256`` pass over 120 KB
lost 76% of events and flash writes lost 99% with a single 1537 ms stall.
Backends therefore capture events into a buffer the moment they occur -- in
interrupt context on an MCU, from the OS on a desktop -- and Python collects
them with :meth:`Host.poll`. Delivery latency then equals how often the
application polls, which it controls, instead of what the VM happened to be
doing when the event arrived. Buffer overflow is reported (see
:attr:`Host.overflowed`), never silent, because the mechanism this replaces
failed silently.
"""

try:
    from collections import namedtuple
except ImportError:  # pragma: no cover - ucollections on older firmware
    from ucollections import namedtuple

import events

# --- Event types -----------------------------------------------------------
#
# Registered here rather than in a backend so that ``events.USBATTACH`` exists
# as soon as the contract is imported, whether or not a backend is available.
# Registration is idempotent: ``events.register_event`` raises on a duplicate,
# and this module may legitimately be imported twice under different names
# (``usbif`` and ``pydevices.lib.usbif``) in the same process.
for _name in ("USBATTACH", "USBDETACH"):
    if not hasattr(events, _name):
        events.register_event(_name, fields="type device")
del _name

# --- Device classes --------------------------------------------------------
#
# Spelled as strings so a capability set is printable and comparable without
# importing this module -- a board can report what it supports over a REPL or
# a log line. The values are the USB class names a user would recognise, not
# the numeric bInterfaceClass codes, which stay inside the backends.
HID = "hid"
MSC = "msc"
CDC = "cdc"
MIDI = "midi"
UAC = "uac"
UVC = "uvc"

CLASSES = (HID, MSC, CDC, MIDI, UAC, UVC)

# Speeds, as reported by DeviceInfo.speed. ``None`` means the backend cannot
# tell -- an honest answer the OS sometimes gives on a desktop.
LOW, FULL, HIGH = "low", "full", "high"


# Interface class/subclass -> usbif class name. This mapping is the one place
# the USB wire encoding is interpreted, shared by every backend: Linux reads
# the bytes from sysfs, Windows from a compatible-ID string, and the native
# module from the descriptor itself, but what the bytes *mean* is decided once
# here. Duplicating it per backend is how two implementations of "one API"
# quietly stop agreeing.
_INTERFACE_CLASSES = {
    0x02: CDC,   # Communications
    0x03: HID,
    0x08: MSC,
    0x0A: CDC,   # CDC-Data
    0x0E: UVC,
    0x10: UAC,   # Audio/Video Function
}
_AUDIO_CLASS = 0x01
_MIDISTREAMING_SUBCLASS = 0x03


def class_from_interface(interface_class, interface_subclass=None):
    """usbif class name for a bInterfaceClass/bInterfaceSubClass pair, or None.

    Audio is the one case where the class byte alone gives the wrong answer:
    MIDI is a subclass of audio rather than a class of its own, which is the
    single most common way a USB device gets mislabelled.
    """
    if interface_class == _AUDIO_CLASS:
        return MIDI if interface_subclass == _MIDISTREAMING_SUBCLASS else UAC
    return _INTERFACE_CLASSES.get(interface_class)


def check_class(name):
    """Validate a USB class name, returning it unchanged.

    Raises rather than ignoring an unknown name, so a typo surfaces as an
    error instead of a capability that is silently never satisfied.
    """
    if name not in CLASSES:
        raise ValueError("unknown USB class {!r}; expected one of {}".format(name, CLASSES))
    return name


# --- Device description ----------------------------------------------------
#
# A namedtuple rather than an object with properties: it is cheap to allocate
# on an MCU, it compares by value (which the parity harness relies on to assert
# that both backends describe the same device identically), and it is
# immutable, so a stale copy held by an application cannot misreport a device
# that has since detached.
#
# ``id`` is backend-assigned and opaque: a bus path on Linux, an instance path
# on Windows, an enumeration handle on an MCU. It is the handle every per-class
# call takes. It is stable while the device stays attached and is never reused
# for a different device within a session.
# Field names are spelled out as a constant as well as passed to namedtuple:
# MicroPython's namedtuple has no ``_fields``, so a portable test (or any
# caller wanting to check the shape) has nothing to read otherwise. This is
# the canonical description of the record either way.
DEVICE_FIELDS = ("id", "vid", "pid", "product", "serial", "classes", "speed")

DeviceInfo = namedtuple("DeviceInfo", " ".join(DEVICE_FIELDS))  # noqa: PYI024

# Same reasoning for the event payloads, which carry the event type and the
# device it concerns.
EVENT_FIELDS = ("type", "device")


def describe(info):
    """One-line human description of a device, for logs and REPL use."""
    name = info.product or "USB device"
    ident = "%04x:%04x" % (info.vid or 0, info.pid or 0)
    kinds = ",".join(sorted(info.classes)) if info.classes else "?"
    return "{} [{}] ({})".format(name, ident, kinds)


# --- MIDI ------------------------------------------------------------------
#
# MIDI is the one class where a plain byte stream is the whole API, and that
# is deliberate. The native module already exposes device-side ``midi_read``/
# ``midi_write`` and host-side ``host_midi_read``/``host_midi_write`` with the
# same shape on purpose, so an application that harmonises, logs, or forwards
# does not care which end of the cable it is on. The desktop backends are held
# to the same shape, which is what makes "the same program runs on the board
# and on the workstation" true for MIDI rather than aspirational.
#
# Ports are their own namespace rather than an attribute of DeviceInfo. On a
# board a MIDI port belongs to an enumerated USB device; on a desktop the OS
# has already claimed the hardware and publishes ports by name, with no
# reliable path back to the USB node underneath. Pretending otherwise would
# mean either a Windows-only correlation hack or an id that means something
# different per platform. A separate namespace says the honest thing.

IN = "in"
OUT = "out"
INOUT = "inout"

DIRECTIONS = (IN, OUT, INOUT)

# ``id`` is backend-assigned and opaque, exactly as for DeviceInfo: a winmm
# device index, an ALSA rawmidi address, an enumeration handle on an MCU.
MIDI_PORT_FIELDS = ("id", "name", "direction")

MidiPortInfo = namedtuple("MidiPortInfo", " ".join(MIDI_PORT_FIELDS))  # noqa: PYI024


def check_direction(name):
    """Validate a MIDI direction, returning it unchanged."""
    if name not in DIRECTIONS:
        raise ValueError(
            "unknown MIDI direction {!r}; expected one of {}".format(name, DIRECTIONS)
        )
    return name


def describe_port(info):
    """One-line human description of a MIDI port, for logs and REPL use."""
    return "{} ({})".format(info.name or "MIDI port", info.direction)


class MidiPort:
    """A MIDI 1.0 byte stream, opened on one port.

    Callers speak plain MIDI bytes -- ``b"\x90\x3c\x64"`` is a middle-C
    note-on. USB-MIDI's 4-byte event packing never reaches Python, and neither
    does any OS-specific message encoding; both are the backend's business.

    **Running status is not guaranteed either way.** USB-MIDI and the Windows
    MIDI input API both expand every message to include its status byte, but a
    5-pin stream forwarded by a board may not, so a reader that cannot handle
    running status is a reader with a latent bug. Writers should emit full
    messages and not rely on a backend preserving an omitted status byte.

    Subclasses implement ``_read``, ``_write`` and ``_close``. Direction is
    enforced here so that no backend has to remember to, and so an application
    gets the same error on every platform for the same mistake.
    """

    def __init__(self, info):
        self.info = info
        self.direction = check_direction(info.direction)
        self.is_open = True

    # -- subclass hooks --
    def _read(self, buf):
        raise NotImplementedError

    def _write(self, data):
        raise NotImplementedError

    def _close(self):
        pass

    def _check(self, want):
        if not self.is_open:
            raise OSError("MIDI port is closed")
        if self.direction not in (want, INOUT):
            raise OSError(
                "MIDI port {!r} is {}-only".format(self.info.name, self.direction)
            )

    def read(self, buf):
        """Read waiting MIDI bytes into ``buf``; returns how many.

        Never blocks and never raises on an empty stream: zero is the ordinary
        answer when nothing has arrived, and a polling application relies on
        that being cheap.
        """
        self._check(IN)
        return self._read(buf)

    def write(self, data):
        """Write MIDI bytes; returns how many were accepted."""
        self._check(OUT)
        return self._write(data)

    def close(self):
        if not self.is_open:
            return
        try:
            self._close()
        finally:
            self.is_open = False

    deinit = close

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, traceback):
        self.close()


_VOICE_LEN = {0x8: 2, 0x9: 2, 0xA: 2, 0xB: 2, 0xC: 1, 0xD: 1, 0xE: 2}
# System-common lengths. Realtime (0xF8-0xFF) carries no data and may
# interleave anywhere, including between another message's data bytes.
_COMMON_LEN = {0xF1: 1, 0xF2: 2, 0xF3: 1}


class MidiParser:
    """Plain MIDI 1.0 bytes in, complete messages out.

    :class:`MidiPort` hands out a byte stream and says, correctly, that a
    reader unable to handle running status has a latent bug. This is that
    reader, so no caller has to write it again -- and five did.

    Feed it whatever a read returned; drain complete messages as
    ``(status, data)`` pairs, where ``data`` is a tuple of 0-2 bytes. It
    handles the four things a hand-rolled loop usually gets wrong:

    * **Running status.** A channel-voice status stays armed, so a stream may
      send ``90 3C 64 3E 64`` for two note-ons. System common clears it, as
      the spec requires.
    * **Split reads.** A message spanning two reads is one message, because
      state lives in the parser rather than in the loop.
    * **Realtime interleaving.** Clock and transport bytes may appear between
      any two bytes of another message. They come out in order and disturb
      nothing.
    * **System exclusive.** Swallowed to its terminator, and a status byte
      arriving mid-sysex aborts it rather than corrupting the next message.

    A data byte arriving with no status is counted in :attr:`desync` rather
    than guessed at. A non-zero count means the stream was joined mid-message
    or something upstream is dropping bytes, which is worth seeing.

        >>> p = MidiParser()
        >>> p.feed(b"\x90\x3c\x64\x3e\x64")
        >>> p.drain()
        [(144, (60, 100)), (144, (62, 100))]
    """

    def __init__(self):
        self.status = 0
        self.desync = 0
        self._data = []
        self._want = 0
        self._in_sysex = False
        self._messages = []

    def feed(self, buf, n=None):
        """Absorb ``n`` bytes of ``buf`` (all of it when ``n`` is None)."""
        if n is None:
            n = len(buf)
        for i in range(n):
            self._byte(buf[i])

    def drain(self):
        """Return the messages completed since the last call, and forget them."""
        out = self._messages
        self._messages = []
        return out

    def reset(self):
        """Drop all parser state, as after a port is reopened."""
        self.status = 0
        self._data = []
        self._want = 0
        self._in_sysex = False
        self._messages = []

    def _byte(self, b):
        if b >= 0xF8:                       # realtime: interleaves, no state
            self._messages.append((b, ()))
            return
        if self._in_sysex:
            if b == 0xF7:
                self._in_sysex = False
                self._messages.append((0xF7, ()))
            elif b >= 0x80:                 # a status byte aborts sysex
                self._in_sysex = False
                self._byte(b)
            return
        if b >= 0x80:
            if b == 0xF0:
                self._in_sysex = True
                return
            high = b >> 4
            if high == 0xF:
                self._want = _COMMON_LEN.get(b, 0)
                self.status = b if self._want else 0
                self._data = []
                if not self._want:
                    self._messages.append((b, ()))
            else:
                self.status = b
                self._want = _VOICE_LEN[high]
                self._data = []
            return
        if not self.status:                 # data with no status: desync
            self.desync += 1
            return
        self._data.append(b)
        if len(self._data) == self._want:
            self._messages.append((self.status, tuple(self._data)))
            self._data = []
            if self.status >= 0xF0:         # system common does not stay armed
                self.status = 0


class _Role:
    """Shared capability, lifecycle, and event-buffer housekeeping."""

    role = None

    def __init__(self):
        self.is_open = False
        self._overflowed = False

    def capabilities(self):
        """USB classes this backend can actually work with, as a frozenset.

        An empty set is a valid and common answer: a desktop has no host role
        to offer beyond what the OS already owns, and a port without USB
        offers nothing at all. Callers branch on membership.
        """
        return frozenset()

    def supports(self, name):
        """True if ``name`` (a class constant) is in :meth:`capabilities`."""
        return check_class(name) in self.capabilities()

    @property
    def overflowed(self):
        """True if events were lost since the last :meth:`poll`.

        The buffer is sized for the worst observed VM stall, but a caller that
        stops polling entirely can still outrun it. This flag is how that gets
        said out loud; the transport it replaces dropped events in silence.
        """
        return self._overflowed

    def _start(self):
        pass

    def _stop(self):
        pass

    def start(self):
        if self.is_open:
            return self
        self._start()
        self.is_open = True
        return self

    def stop(self):
        if not self.is_open:
            return
        try:
            self._stop()
        finally:
            self.is_open = False

    deinit = stop

    def __enter__(self):
        return self.start()

    def __exit__(self, exc_type, exc, traceback):
        self.stop()


class Host(_Role):
    """The board (or desktop process) drives attached USB peripherals.

    Subclasses implement :meth:`_devices` and :meth:`_drain`, and set
    :meth:`capabilities`.
    """

    role = "host"

    def _devices(self):
        raise NotImplementedError("Host subclasses must implement _devices")

    def devices(self):
        """Currently attached devices, as a tuple of :class:`DeviceInfo`."""
        self.start()
        return tuple(self._devices())

    def find(self, cls):
        """Attached devices offering a given class, e.g. ``usbif.HID``."""
        check_class(cls)
        return tuple(d for d in self.devices() if cls in d.classes)

    def _drain(self):
        """Return newly buffered events as a list. Subclasses implement."""
        raise NotImplementedError("Host subclasses must implement _drain")

    def poll(self):
        """Collect buffered events and return them, newest last.

        Call this from the application's normal service loop. Events are
        already captured by the time it runs, so a late poll costs latency,
        never data, until the buffer is full.
        """
        self.start()
        self._overflowed = False
        return tuple(self._drain())


class Device(_Role):
    """The board presents itself to a computer as a USB peripheral.

    Configuration only: the classes that move isochronous bytes (UAC, UVC) run
    entirely in C and are observed from Python, never fed by it.
    """

    role = "device"

    # The functions a board can present. Which of them a given firmware can
    # actually offer is reported by ``functions_available()``; what it is
    # presenting right now is ``functions()``.
    FUNCTIONS = ("cdc", "msc", "uac", "midi", "hid")

    def _drain(self):
        return ()

    def poll(self):
        self.start()
        self._overflowed = False
        return tuple(self._drain())

    def functions(self, *names):
        """Report or choose the USB functions this board presents.

        Called with no arguments, returns the frozenset currently
        advertised. Called with names, the board re-enumerates presenting
        exactly those -- USB has no way to change identity in place, so the
        host sees a detach and a fresh attach.

            dev.functions()                  # -> frozenset({'cdc'})
            dev.functions("cdc", "uac")      # a console and a sound card
            dev.functions("midi")            # a bare MIDI instrument

        A backend without runtime selection reports what it was built with
        and raises on any attempt to change it.
        """
        raise NotImplementedError

    def functions_available(self):
        """The functions this firmware could present if asked."""
        raise NotImplementedError


class NullHost(Host):
    """A host that offers nothing, for ports without USB host support.

    Exists so that ``usbif.auto`` can always return an object: application
    code checks ``capabilities()`` once instead of guarding every import.
    """

    def capabilities(self):
        return frozenset()

    def _devices(self):
        return ()

    def _drain(self):
        return ()


__all__ = (
    "CDC",
    "DIRECTIONS",
    "IN",
    "INOUT",
    "MIDI_PORT_FIELDS",
    "MidiParser",
    "MidiPort",
    "MidiPortInfo",
    "OUT",
    "check_direction",
    "describe_port",
    "CLASSES",
    "DEVICE_FIELDS",
    "class_from_interface",
    "EVENT_FIELDS",
    "FULL",
    "HID",
    "HIGH",
    "LOW",
    "MIDI",
    "MSC",
    "UAC",
    "UVC",
    "Device",
    "DeviceInfo",
    "Host",
    "NullHost",
    "check_class",
    "describe",
)
