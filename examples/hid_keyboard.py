"""The board is a USB keyboard: type a short string into the host.

Presents as a boot keyboard (report ID 1) and sends HID reports that any
ordinary OS keyboard stack accepts -- Notepad, a browser address bar, a
terminal. No driver to install.

    mpftp run -d COM49 examples/hid_keyboard.py

**Console.** Costume is CDC+HID so the REPL stays on the same cable. Run the
REPL on the UART bridge if you change the costume to HID alone.

**LEDs.** After typing, the script prints ``hid_leds()`` -- the lock-key state
the host last set (caps / num / scroll). A control surface that wants to show
them reads the same call.

**Pairing.** An S3 hosting HID (``hid_host.py``) can take this board as its
keyboard. A PC works the same way. P4 cannot be the host (usbif#3).
"""

import time

import usbif.auto

# HID usage ids for a-z (0x04..) and a few extras. Boot protocol only.
_ALPHA = {c: 0x04 + i for i, c in enumerate("abcdefghijklmnopqrstuvwxyz")}
_EXTRA = {
    " ": 0x2C,
    "\n": 0x28,
    ".": 0x37,
    ",": 0x36,
    "-": 0x2D,
}
MESSAGE = "hello from usbif\n"


def _send(dev, report, retries=50):
    """Submit one keyboard report; retry while the host has not polled."""
    for _ in range(retries):
        if dev.hid_send(dev.HID_KEYBOARD, report):
            return True
        time.sleep_ms(2)
    return False


def _tap(dev, usage, modifier=0):
    """Press then release one key."""
    down = bytes([modifier, 0, usage, 0, 0, 0, 0, 0])
    up = bytes(8)
    if not _send(dev, down):
        return False
    time.sleep_ms(30)
    return _send(dev, up)


def type_string(dev, text):
    for ch in text:
        lower = ch.lower()
        usage = _ALPHA.get(lower) or _EXTRA.get(ch)
        if usage is None:
            print("skip unsupported char %r" % ch)
            continue
        mod = 0x02 if ch.isalpha() and ch.isupper() else 0  # left shift
        if not _tap(dev, usage, mod):
            print("host did not accept a report; is the keyboard mounted?")
            return False
        time.sleep_ms(20)
    return True


def main():
    dev = usbif.auto.device()
    dev.functions("cdc", "hid")
    print("costume: cdc+hid -- focus a text field on the host")
    # Give the host time to configure us before the first report.
    time.sleep_ms(1500)

    ok = type_string(dev, MESSAGE)
    leds = dev.hid_leds()
    print("typed %r: %s" % (MESSAGE.strip(), "ok" if ok else "failed"))
    print("hid_leds: 0x%02x (bit0=num bit1=caps bit2=scroll)" % leds)


if __name__ == "__main__":
    main()
