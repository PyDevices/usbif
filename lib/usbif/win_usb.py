"""USB host backend for Windows, over ``uwin32``'s cfgmgr32 bindings.

As on Linux, the OS owns the bus, so this reports rather than drives: the same
``DeviceInfo`` records and the same attach/detach events, so an application
written for a board behaves the same on a workstation.

The interesting work here is reconciling two different models of what a device
*is*. Linux publishes one node per device with its interfaces beneath it.
Windows publishes composite devices as one node per interface -- ``MI_00``,
``MI_02`` -- each carrying its own class, with no single node describing the
whole. Reported verbatim, a two-interface board would appear as two devices on
Windows and one on Linux, and "the same API" would be a claim rather than a
fact. So the interface nodes are grouped back into the device they belong to,
by the instance path they share.

Class information comes from the compatible-ID list (``USB\\Class_02&SubClass_02``),
which is the same bInterfaceClass/bInterfaceSubClass pair Linux reads from
sysfs -- interpreted by the shared ``class_from_interface`` so both backends
reach the same answer by construction rather than by coincidence.
"""

import events
import uwin32

from . import DeviceInfo, Host, class_from_interface

ENUMERATOR = "USB"


def _parse_instance_id(instance_id):
    """Split ``USB\\VID_046D&PID_C31C[&MI_00]\\<instance>`` into its parts.

    Returns ``(vid, pid, interface_or_None, instance)``, or None if the ID is
    not a USB device path (the enumerator can also yield hubs and roots).
    """
    parts = instance_id.split("\\")
    if len(parts) < 3:
        return None
    ids, instance = parts[1], parts[2]
    vid = pid = None
    interface = None
    for field in ids.split("&"):
        key, _, value = field.partition("_")
        try:
            if key.upper() == "VID":
                vid = int(value, 16)
            elif key.upper() == "PID":
                pid = int(value, 16)
            elif key.upper() == "MI":
                interface = int(value, 16)
        except ValueError:
            return None
    if vid is None or pid is None:
        return None
    return vid, pid, interface, instance


def _device_key(instance_id, interface):
    """The instance ID of the device an interface node belongs to.

    Windows publishes a composite device both as a parent node and as one node
    per interface. Asking for the parent link is exact; grouping by a shared
    instance-path prefix is not, and reports the parent and its interfaces as
    separate devices -- which would make one board appear twice on Windows and
    once on Linux.
    """
    if interface is None:
        return instance_id
    parent = uwin32.CM_Get_Parent_Device_ID(instance_id)
    if parent and parent.upper().startswith("USB\\"):
        return parent
    return instance_id


def _classes_from_compatible_ids(compatible):
    """usbif class names from a device node's compatible-ID list."""
    found = set()
    for entry in compatible or ():
        interface_class = subclass = None
        for field in entry.split("\\")[-1].split("&"):
            key, _, value = field.partition("_")
            try:
                if key == "Class":
                    interface_class = int(value, 16)
                elif key == "SubClass":
                    subclass = int(value, 16)
            except ValueError:
                interface_class = None
                break
        if interface_class is not None:
            name = class_from_interface(interface_class, subclass)
            if name is not None:
                found.add(name)
    return found


def _serial_of(instance, interface):
    """The device's serial number, when Windows has one to give.

    Windows substitutes a generated instance path (containing ``&``) when a
    device reports no serial, so those are reported as None rather than as a
    serial that would differ on the next port.
    """
    if interface is not None or "&" in instance:
        return None
    return instance


def scan():
    """Return ``{id: DeviceInfo}`` for every present USB device."""
    grouped = {}
    for instance_id in uwin32.CM_Get_Device_ID_List(ENUMERATOR):
        parsed = _parse_instance_id(instance_id)
        if parsed is None:
            continue
        _, _, interface, _ = parsed
        key = _device_key(instance_id, interface)

        entry = grouped.get(key)
        if entry is None:
            # Identity comes from the device node the key names, not from the
            # interface node that led us to it: the parent carries the real
            # serial, and an interface node is named for its function ("USB
            # Serial Device") rather than for the product.
            key_parsed = _parse_instance_id(key) or parsed
            vid, pid, key_interface, key_instance = key_parsed
            entry = grouped[key] = {
                "id": key,
                "vid": vid,
                "pid": pid,
                "classes": set(),
                "serial": _serial_of(key_instance, key_interface),
                "product": uwin32.CM_Get_DevNode_Registry_Property(
                    key, uwin32.CM_DRP_DEVICEDESC),
            }

        entry["classes"].update(
            _classes_from_compatible_ids(
                uwin32.CM_Get_DevNode_Registry_Property(
                    instance_id, uwin32.CM_DRP_COMPATIBLEIDS)
            )
        )
        # A bare "USB Composite Device" says nothing a user would recognise;
        # an interface node's own name usually does.
        if entry["product"] in (None, "USB Composite Device"):
            desc = uwin32.CM_Get_DevNode_Registry_Property(
                instance_id, uwin32.CM_DRP_DEVICEDESC)
            if desc and desc not in ("WinUsb Device", "USB Composite Device"):
                entry["product"] = desc

    return {
        entry["id"]: DeviceInfo(
            id=entry["id"],
            vid=entry["vid"],
            pid=entry["pid"],
            product=entry["product"],
            serial=entry["serial"],
            classes=frozenset(entry["classes"]),
            # cfgmgr32 does not report negotiated speed, and inferring it from
            # the controller would be a fiction. The contract allows None.
            speed=None,
        )
        for entry in grouped.values()
    }


class WindowsHost(Host):
    """Enumeration and attach/detach for Windows.

    Enumeration and hot-plug work regardless of :meth:`capabilities`; the
    capability set describes which classes this backend can carry traffic for,
    which on a desktop arrives one shim at a time.
    """

    def __init__(self):
        super().__init__()
        self._seen = {}

    def capabilities(self):
        return frozenset()

    def _start(self):
        # Seed the snapshot: devices already plugged in belong in devices(),
        # not in a burst of attach events the application never caused.
        self._seen = scan()

    def _stop(self):
        self._seen = {}

    def _devices(self):
        return tuple(self._seen.values())

    def _drain(self):
        now = scan()
        out = []
        for key, info in now.items():
            if key not in self._seen:
                out.append(events.Usbattach(events.USBATTACH, info))
        for key, info in self._seen.items():
            if key not in now:
                out.append(events.Usbdetach(events.USBDETACH, info))
        self._seen = now
        return out


__all__ = ("WindowsHost", "scan")
