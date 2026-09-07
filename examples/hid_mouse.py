"""The board is a USB mouse: nudge the host cursor, then put it back.

Presents as a boot mouse (report ID 2) and sends relative motion reports.
Verified originally by watching Windows' cursor move 82 pixels and return --
the host is the instrument, not a counter in this script.

    mpftp run -d COM49 examples/hid_mouse.py

**Report layout.** Boot mouse behind a report ID: buttons, dx, dy, wheel --
four signed bytes after the ID the C side prepends via ``hid_send``.

**Pairing.** Same as ``hid_keyboard.py``: a PC, or an S3 running a HID host.
"""

import time

import _usbif

STEPS = 20
DELTA = 4  # pixels per report


def _send(report, retries=50):
    for _ in range(retries):
        if _usbif.hid_send(_usbif.HID_MOUSE, report):
            return True
        time.sleep_ms(2)
    return False


def move(dx, dy):
    # buttons=0, dx, dy, wheel=0. Values are signed 8-bit.
    def s8(n):
        return n & 0xFF

    return _send(bytes([0, s8(dx), s8(dy), 0]))


def main():
    _usbif.dev_functions(_usbif.FN_CDC | _usbif.FN_HID)
    print("costume: cdc+hid -- watch the host cursor")
    time.sleep_ms(1500)

    print("moving +x")
    for _ in range(STEPS):
        if not move(DELTA, 0):
            print("host did not accept a report")
            return
        time.sleep_ms(20)

    print("moving -x (return)")
    for _ in range(STEPS):
        if not move(-DELTA, 0):
            print("host did not accept a report")
            return
        time.sleep_ms(20)

    print("done; cursor should be back where it started")


if __name__ == "__main__":
    main()
