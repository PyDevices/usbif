"""USB host backend for Linux and WSL, reading sysfs.

On a desktop the operating system owns the bus, so this is not a USB stack:
it is the same :mod:`usbif` surface backed by what the kernel already exposes.
Its first job is parity -- identical ``DeviceInfo`` records and identical
attach/detach events -- so an application written against a board keeps
working when run on a workstation. Its second is real capability where the OS
offers it, which arrives per class (raw HID through a ``uhidapi`` shim, MIDI
through ALSA) rather than all at once.

Enumeration reads ``/sys/bus/usb/devices``, which needs no privileges, no
libudev, and no third-party package -- important because this module has to
import on a stock MicroPython unix build as readily as on CPython.

Attach and detach are detected by comparing successive scans rather than by
subscribing to a netlink uevent socket. That costs a directory walk per poll
and gains portability: the same code runs under WSL, inside containers with a
read-only ``/run``, and on MicroPython's unix port, none of which reliably
carry a udev daemon. A netlink source is a later refinement and would slot in
behind the same :meth:`_drain`.
"""

import os

import events

from . import FULL, HIGH, LOW, DeviceInfo, Host, class_from_interface

SYSFS = "/sys/bus/usb/devices"


def _read(path, default=None):
    """Read and strip a sysfs attribute, or return default.

    sysfs reads race with unplug: a device directory can vanish between the
    listing and the read. Every failure mode here means "the device is gone",
    which is not an error -- the next scan reports the detach.
    """
    try:
        with open(path) as f:
            return f.read().strip()
    except (OSError, ValueError):
        return default


def _hex(path):
    value = _read(path)
    try:
        return int(value, 16)
    except (TypeError, ValueError):
        return None


def _speed(entry):
    # sysfs reports megabits as a decimal string; map to the vocabulary in
    # usbif rather than leaking kernel formatting into the portable API.
    raw = _read(entry + "/speed")
    if raw is None:
        return None
    try:
        mbit = float(raw)
    except ValueError:
        return None
    if mbit >= 480:
        return HIGH
    if mbit >= 12:
        return FULL
    return LOW


def _is_device_dir(name):
    # sysfs mixes three kinds of entry in one directory: devices ("1-1"),
    # interfaces ("1-1:1.0"), and root hubs ("usb1"). Only devices carry the
    # descriptors we want, and interfaces are read relative to their parent.
    return ":" not in name and not name.startswith("usb")


def _classes_of(entry, name):
    """USB classes offered by a device, from its interface descriptors."""
    found = set()
    try:
        children = os.listdir(entry)
    except OSError:
        return frozenset()
    prefix = name + ":"
    for child in children:
        if not child.startswith(prefix):
            continue
        itf = entry + "/" + child
        cls = _hex(itf + "/bInterfaceClass")
        if cls is None:
            continue
        name = class_from_interface(cls, _hex(itf + "/bInterfaceSubClass"))
        if name is not None:
            found.add(name)
    return frozenset(found)


def scan(root=SYSFS):
    """Return ``{id: DeviceInfo}`` for every attached USB device."""
    out = {}
    try:
        names = os.listdir(root)
    except OSError:
        return out  # no USB bus visible; an empty result, not an error
    for name in names:
        if not _is_device_dir(name):
            continue
        entry = root + "/" + name
        vid = _hex(entry + "/idVendor")
        pid = _hex(entry + "/idProduct")
        if vid is None or pid is None:
            continue  # not a USB device node, or it detached mid-scan
        out[name] = DeviceInfo(
            id=name,
            vid=vid,
            pid=pid,
            product=_read(entry + "/product"),
            serial=_read(entry + "/serial"),
            classes=_classes_of(entry, name),
            speed=_speed(entry),
        )
    return out


class LinuxHost(Host):
    """Enumeration and attach/detach for Linux, WSL, and containers.

    Enumeration and hot-plug work regardless of :meth:`capabilities`; the
    capability set describes which classes this backend can carry *traffic*
    for, which on a desktop arrives one shim at a time.
    """

    def __init__(self, root=SYSFS):
        super().__init__()
        self.root = root
        self._seen = {}

    def capabilities(self):
        # Phase 1 enumerates and reports; no class traffic is claimed yet, and
        # claiming otherwise would make find()/supports() lie to callers.
        return frozenset()

    def _start(self):
        # Seed the snapshot so the first poll reports change, not the world.
        # An application that starts with a keyboard already plugged in should
        # see it in devices(), not as an attach event it never caused.
        self._seen = scan(self.root)

    def _stop(self):
        self._seen = {}

    def _devices(self):
        return tuple(self._seen.values())

    def _drain(self):
        now = scan(self.root)
        out = []
        for key, info in now.items():
            if key not in self._seen:
                out.append(events.Usbattach(events.USBATTACH, info))
        for key, info in self._seen.items():
            if key not in now:
                out.append(events.Usbdetach(events.USBDETACH, info))
        self._seen = now
        return out


__all__ = ("LinuxHost", "scan")
