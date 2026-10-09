"""A USB keyboard on the host port drives ordinary PyDevices key events.

Milestone M1: the same application code, producing the same ``events.Key``
records, as an SDL keyboard on the desktop. The host stack delivers raw boot
reports; ``usbif.hid_keyboard.KeyboardDecoder`` turns them into presses and
releases by diffing successive reports.

    mpftp run -d COM49 examples/hid_host.py

**Pairing.** A commercial keyboard, or a PyDevices board running
``hid_keyboard.py``. Needs an S3, or a P4 with the keyboard plugged straight
into a host port that supplies 5 V: behind a hub the P4 sees only high-speed
devices (usbif#15).

**Rollover.** When more keys are held than the report can carry, the decoder
ignores the ErrorRollOver report rather than emitting garbage -- see the
module docstring in ``lib/usbif/hid_keyboard.py``.
"""

import time

import events
import usbif.auto
from usbif.hid_keyboard import KeyboardDecoder


def find_keyboard(host, timeout_ms=15000):
    deadline = time.ticks_add(time.ticks_ms(), timeout_ms)
    while time.ticks_diff(deadline, time.ticks_ms()) > 0:
        found = host.find("hid")
        if found:
            return found[0].id
        time.sleep_ms(250)
    return None


def keyboard_report(buf, n):
    """The 8-byte boot keyboard report in the first ``n`` bytes, or None.

    A plain keyboard sends 8 bytes. One that shares its interface with a
    mouse (a PyDevices board running ``hid_keyboard.py`` is one) puts a
    report ID in front, so its keyboard reports are 9 bytes; read as boot
    reports unstripped, the ID lands in the modifier byte and every key
    comes with Left Ctrl held. Anything else, such as that interface's
    mouse reports, isn't a keyboard report.
    """
    if n == 8:
        return bytes(buf[:8])
    if n == 9:
        return bytes(buf[1:9])
    return None


def main(seconds=30):
    host = usbif.auto.host(classes=("hid",)).start()
    dev_id = find_keyboard(host)
    if dev_id is None:
        print("no HID device found")
        host.stop()
        return

    print("HID device", dev_id, "-- type on it for %d s" % seconds)
    host.hid_open(dev_id)
    decoder = KeyboardDecoder()
    buf = bytearray(64)
    t0 = time.ticks_ms()
    try:
        while time.ticks_diff(time.ticks_ms(), t0) < seconds * 1000:
            n = host.hid_read(buf)
            report = keyboard_report(buf, n) if n > 0 else None
            if report is not None:
                for ev in decoder.feed(report):
                    kind = "DOWN" if ev.type == events.KEYDOWN else "UP"
                    # Field names follow events.Key; fall back to repr if a
                    # firmware builds the record differently.
                    name = getattr(ev, "name", "?")
                    key = getattr(ev, "key", None) or 0
                    mod = getattr(ev, "mod", 0)
                    scan = getattr(ev, "scancode", None)
                    print("  %s %-12s key=0x%02x mod=0x%02x scancode=%s"
                          % (kind, name, key, mod, scan))
            elif n <= 0:
                time.sleep_ms(5)
    finally:
        host.hid_close()
        host.stop()
        print("done")


if __name__ == "__main__":
    main()
