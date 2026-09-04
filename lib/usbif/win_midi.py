"""MIDI backend for Windows, over ``uwin32``'s winmm bindings.

The OS owns the MIDI hardware here, so this reports and moves bytes rather
than driving a bus -- the same shape as ``win_usb``. What it guarantees is
that a program written against :class:`usbif.MidiPort` on a board behaves the
same on a workstation, which is the whole point of the contract existing.

**Ports are their own namespace, and the two directions are numbered
separately.** winmm has an output list and an input list, each indexed from
zero, so "device 1" means two different pieces of hardware depending on which
you mean. Ids are therefore ``"out:1"`` / ``"in:1"`` rather than bare
integers: a caller that mixes them up gets a lookup failure instead of the
wrong device.

**Input arrives by draining, never by callback.** ``midiInOpen`` is asked for
``CALLBACK_WINDOW``, so Windows posts ``MM_MIM_DATA`` to a message-only window
this module owns, and :meth:`read` drains that queue on the caller's thread.
The alternative, ``CALLBACK_FUNCTION``, runs on winmm's own thread, and
re-entering a Python interpreter from a foreign thread is a crash risk under
MicroPython's ffi. It also happens to be exactly the transport rule the usbif
contract already states: capture at the source, collect by polling.

**Sysex is out of scope, deliberately and loudly.** Short messages are the
whole of ``midiOutShortMsg`` and ``MM_MIM_DATA``; system-exclusive needs
``midiOutLongMsg`` and ``MM_MIM_LONGDATA`` with buffer management on both
sides. Writing a 0xF0 raises rather than silently dropping it, because a
sysex that vanishes looks exactly like a device that ignored it.
"""

import uwin32 as win

from . import IN, OUT, MidiPort, MidiPortInfo

# Data bytes carried by each status, so a byte stream can be split into
# messages. Shared by both directions -- the same table decides how many bytes
# to send and how many a received message contributes.
_VOICE_LEN = {0x8: 2, 0x9: 2, 0xA: 2, 0xB: 2, 0xC: 1, 0xD: 1, 0xE: 2}
_COMMON_LEN = {0xF1: 1, 0xF2: 2, 0xF3: 1}

HWND_MESSAGE = -3

_wndproc_ref = None
_class_registered = False
_CLASS_NAME = "usbif_midi_sink"


def _msg_len(status):
    """Data bytes following ``status``. Realtime and unknown status: none."""
    if status >= 0xF8:
        return 0
    if status >= 0xF0:
        return _COMMON_LEN.get(status, 0)
    return _VOICE_LEN.get(status >> 4, 0)


def ports():
    """Every MIDI port the OS is offering, both directions."""
    found = []
    for i in range(win.midiOutGetNumDevs()):
        found.append(MidiPortInfo(id="out:%d" % i, name=win.midiOutGetDevName(i),
                                  direction=OUT))
    for i in range(win.midiInGetNumDevs()):
        found.append(MidiPortInfo(id="in:%d" % i, name=win.midiInGetDevName(i),
                                  direction=IN))
    return tuple(found)


def find(name, direction=None):
    """Ports whose name contains ``name``, case-insensitively.

    Names are how a human identifies a MIDI port and how they stay stable
    across reboots; indices are not. Matching on a substring means
    ``find("Espressif")`` keeps working when the full product string changes.
    """
    needle = name.lower()
    return tuple(p for p in ports()
                 if needle in (p.name or "").lower()
                 and (direction is None or p.direction == direction))


def _split_id(port_id):
    kind, _, index = str(port_id).partition(":")
    if kind not in ("out", "in") or not index.isdigit():
        raise ValueError("bad MIDI port id {!r}; expected 'out:N' or 'in:N'".format(port_id))
    return kind, int(index)


def open_port(port):
    """Open a port, given a :class:`usbif.MidiPortInfo` or an id string.

    Named ``open_port`` rather than ``open`` so nothing in this module ever
    shadows the builtin -- a trap for whoever later adds file I/O here.
    """
    info = port if isinstance(port, MidiPortInfo) else None
    port_id = info.id if info is not None else port
    kind, index = _split_id(port_id)
    if info is None:
        name = win.midiOutGetDevName(index) if kind == "out" else win.midiInGetDevName(index)
        info = MidiPortInfo(id=port_id, name=name, direction=OUT if kind == "out" else IN)
    return WindowsMidiOut(info, index) if kind == "out" else WindowsMidiIn(info, index)


class WindowsMidiOut(MidiPort):
    """An OS MIDI output. Accepts plain MIDI bytes; packs them for winmm."""

    def __init__(self, info, index):
        MidiPort.__init__(self, info)
        self._handle = win.midiOutOpen(index)
        self._status = 0          # running status carried across writes

    def _write(self, data):
        # Split the stream into whole messages. Running status is honoured
        # because a caller forwarding bytes from a 5-pin stream will have it,
        # and winmm needs each message expanded regardless.
        i, n = 0, len(data)
        sent = 0
        while i < n:
            b = data[i]
            if b == 0xF0:
                raise NotImplementedError(
                    "sysex needs midiOutLongMsg, which this backend does not bind; "
                    "send short messages, or handle sysex above this layer")
            if b >= 0x80:
                self._status = b if b < 0xF8 else self._status
                status, i = b, i + 1
            elif self._status:
                status = self._status
            else:
                i += 1            # data byte with no status: nothing to send
                continue
            want = _msg_len(status)
            if i + want > n:
                break             # partial trailing message; leave it unsent
            d1 = data[i] if want >= 1 else 0
            d2 = data[i + 1] if want >= 2 else 0
            win.midiOutShortMsg(self._handle, status, d1, d2)
            i += want
            sent = i
        return sent

    def _close(self):
        try:
            win.midiOutReset(self._handle)   # silence anything still sounding
        finally:
            win.midiOutClose(self._handle)


class WindowsMidiIn(MidiPort):
    """An OS MIDI input, drained from a message-only window's queue."""

    def __init__(self, info, index):
        MidiPort.__init__(self, info)
        self._hwnd = _make_sink_window()
        self._pending = bytearray()
        self._handle = win.midiInOpen(index, self._hwnd)
        win.midiInStart(self._handle)

    def _drain_queue(self):
        while True:
            msg = win.PeekMessageW(self._hwnd)
            if msg is None:
                return
            if msg.message != win.MM_MIM_DATA:
                continue
            status, d1, d2 = win.midi_unpack(msg.lParam)
            want = _msg_len(status)
            self._pending.append(status)
            if want >= 1:
                self._pending.append(d1)
            if want >= 2:
                self._pending.append(d2)

    def _read(self, buf):
        self._drain_queue()
        n = min(len(buf), len(self._pending))
        if n:
            buf[:n] = self._pending[:n]
            del self._pending[:n]
        return n

    def _close(self):
        try:
            win.midiInStop(self._handle)
            win.midiInReset(self._handle)
        finally:
            win.midiInClose(self._handle)
            if self._hwnd:
                win.DestroyWindow(self._hwnd)
                self._hwnd = None


def _make_sink_window():
    """A message-only window, purely to own a queue winmm can post into.

    HWND_MESSAGE means it is never shown, never painted and never appears in
    the taskbar -- it exists only so ``MM_MIM_DATA`` has somewhere to land.
    The window procedure does nothing but defer, because the messages we care
    about are *posted* and are read straight off the queue by PeekMessageW
    rather than dispatched.
    """
    global _wndproc_ref, _class_registered
    if not _class_registered:
        _wndproc_ref = win.WNDPROC(
            lambda hwnd, msg, wparam, lparam: win.DefWindowProcW(hwnd, msg, wparam, lparam))
        cls = win.WNDCLASSEXW()
        cls.cbSize = win.sizeof(win.WNDCLASSEXW)
        cls.lpfnWndProc = _wndproc_ref
        cls.hInstance = win.GetModuleHandleW()
        cls.lpszClassName = _CLASS_NAME
        win.RegisterClassExW(cls)
        _class_registered = True
    return win.CreateWindowExW(0, _CLASS_NAME, "", 0, 0, 0, 0, 0,
                               parent=HWND_MESSAGE)
