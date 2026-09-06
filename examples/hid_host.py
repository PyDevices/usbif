"""A USB keyboard on the host port drives ordinary PyDevices key events.

Milestone M1: the same application code, producing the same ``events.Key``
records, as an SDL keyboard on the desktop. The host stack delivers raw boot
reports; ``usbif.hid_keyboard.KeyboardDecoder`` turns them into presses and
releases by diffing successive reports.

    mpftp run -d COM49 examples/hid_host.py

**Pairing.** A commercial keyboard, or a PyDevices board running
``hid_keyboard.py``. Needs an S3 (or any board whose host mode works); P4
host is blocked (usbif#3).

**Rollover.** When more keys are held than the report can carry, the decoder
ignores the ErrorRollOver report rather than emitting garbage -- see the
module docstring in ``lib/usbif/hid_keyboard.py``.
"""

import time

import _usbif
import events
from usbif.hid_keyboard import KeyboardDecoder


def find_keyboard(timeout_ms=15000):
    _usbif.host_start(("hid",))
    deadline = time.ticks_add(time.ticks_ms(), timeout_ms)
    while time.ticks_diff(deadline, time.ticks_ms()) > 0:
        for dev in _usbif.host_devices():
            if "hid" in dev[5]:
                return dev[0]
        time.sleep_ms(250)
    return None


def main(seconds=30):
    dev_id = find_keyboard()
    if dev_id is None:
        print("no HID device found")
        _usbif.host_stop()
        return

    print("HID device", dev_id, "-- type on it for %d s" % seconds)
    _usbif.host_hid_open(dev_id)
    decoder = KeyboardDecoder()
    buf = bytearray(8)
    t0 = time.ticks_ms()
    try:
        while time.ticks_diff(time.ticks_ms(), t0) < seconds * 1000:
            n = _usbif.host_hid_read(buf)
            if n >= 3:
                for ev in decoder.feed(buf):
                    kind = "DOWN" if ev.type == events.KEYDOWN else "UP"
                    # Field names follow events.Key; fall back to repr if a
                    # firmware builds the record differently.
                    name = getattr(ev, "name", "?")
                    key = getattr(ev, "key", None) or 0
                    mod = getattr(ev, "mod", 0)
                    scan = getattr(ev, "scancode", None)
                    print("  %s %-12s key=0x%02x mod=0x%02x scancode=%s"
                          % (kind, name, key, mod, scan))
            else:
                time.sleep_ms(5)
    finally:
        _usbif.host_hid_close()
        _usbif.host_stop()
        print("done")


if __name__ == "__main__":
    main()
