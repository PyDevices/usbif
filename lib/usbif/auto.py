"""Optional host backend selection. Backends never import this module."""

import sys


def _is_micropython():
    return getattr(getattr(sys, "implementation", None), "name", "") == "micropython"


def _module_available(name):
    try:
        __import__(name)
        return True
    except Exception:
        return False


def select_backend():
    """Return the module name of the first usable host backend.

    Unlike ``audiodev.auto``, this never raises: a port with no USB support is
    an ordinary outcome, not a configuration error, and the caller is expected
    to branch on ``capabilities()`` anyway. The fallback backend enumerates
    nothing and reports an empty capability set.
    """
    if _module_available("_usbif"):
        return "native_usb"
    if sys.platform == "win32" and _module_available("uwin32"):
        return "win_usb"
    if sys.platform in ("linux", "linux2") or (
        _is_micropython() and sys.platform == "linux"
    ):
        return "linux_usb"
    return None


def host(**kwargs):
    """Construct the host for this platform, or a :class:`usbif.NullHost`."""
    name = select_backend()
    if name == "native_usb":
        from .native_usb import NativeHost

        return NativeHost(**kwargs)
    if name == "win_usb":
        from .win_usb import WindowsHost

        return WindowsHost(**kwargs)
    if name == "linux_usb":
        from .linux_usb import LinuxHost

        return LinuxHost(**kwargs)
    from . import NullHost

    return NullHost()


def _midi_backend():
    """Module name of the MIDI backend for this platform, or None.

    Deliberately separate from :func:`select_backend`. The USB *host* role and
    the OS *MIDI* service are different questions with different answers: a
    Windows box has no host role to offer but does have MIDI ports, and a board
    with a full host stack may have no OS MIDI service at all.
    """
    if _module_available("_usbif"):
        # A board with the native module is both roles at once: its own MIDI
        # function and anything it is hosting. Checked first because a board
        # running the unix port under Linux would otherwise be handed the ALSA
        # backend, which knows nothing about either.
        return "native_midi"
    if sys.platform == "win32" and _module_available("uwin32"):
        return "win_midi"
    if sys.platform in ("linux", "linux2"):
        return "linux_midi"
    return None


def midi_ports():
    """Every MIDI port this platform offers, as ``MidiPortInfo`` records.

    An empty tuple is a valid and common answer, not an error -- the same
    reasoning as ``capabilities()`` returning an empty frozenset. Callers
    branch on what came back rather than guarding an import.
    """
    name = _midi_backend()
    if name == "native_midi":
        from .native_midi import ports

        return ports()
    if name == "win_midi":
        from .win_midi import ports

        return ports()
    if name == "linux_midi":
        from .linux_midi import ports

        return ports()
    return ()


def open_midi(port):
    """Open a MIDI port by ``MidiPortInfo`` or id, using this platform's backend."""
    name = _midi_backend()
    if name == "native_midi":
        from .native_midi import open_port

        return open_port(port)
    if name == "win_midi":
        from .win_midi import open_port

        return open_port(port)
    if name == "linux_midi":
        from .linux_midi import open_port

        return open_port(port)
    raise OSError(
        "no MIDI backend on this platform; usbif.auto.midi_ports() reports "
        "what is available and returns an empty tuple when nothing is"
    )


__all__ = ("host", "midi_ports", "open_midi", "select_backend")


def device(**kwargs):
    """Construct the USB-device role for this platform, or ``None``.

    Only the native backend can present the board as a peripheral; a desktop
    Python has no such role, and ``None`` is the honest answer rather than a
    stub that pretends.
    """
    if select_backend() == "native_usb":
        from . import native_usb

        return native_usb.NativeDevice(**kwargs)
    return None
