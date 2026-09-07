"""Talk to a USB serial device the board is hosting.

Opens the first CDC-ACM device on the host port, writes a short line, and
prints whatever comes back. A USB-serial adapter, another MCU's CDC console,
or a PyDevices board presenting CDC all work -- standard class, no bespoke
protocol.

    mpftp run -d COM49 examples/usb_serial.py

**Pairing.** Device side is ordinary CDC (MicroPython's built-in console, or
``dev_functions(FN_CDC)``). Host side is this script on an S3.
"""

import time

import _usbif


def find_cdc(timeout_ms=15000):
    _usbif.host_start(("cdc",))
    deadline = time.ticks_add(time.ticks_ms(), timeout_ms)
    while time.ticks_diff(deadline, time.ticks_ms()) > 0:
        for dev in _usbif.host_devices():
            if "cdc" in dev[5]:
                return dev[0]
        time.sleep_ms(250)
    return None


def main(seconds=15):
    dev_id = find_cdc()
    if dev_id is None:
        print("no CDC device found")
        _usbif.host_stop()
        return

    print("CDC device", dev_id)
    _usbif.host_cdc_open(dev_id)
    msg = b"hello from usbif host\r\n"
    n = _usbif.host_cdc_write(msg)
    print("wrote", n, "bytes:", msg)

    buf = bytearray(256)
    t0 = time.ticks_ms()
    total = 0
    try:
        while time.ticks_diff(time.ticks_ms(), t0) < seconds * 1000:
            got = _usbif.host_cdc_read(buf)
            if got:
                total += got
                print("rx:", bytes(buf[:got]))
            else:
                time.sleep_ms(20)
    finally:
        print("total bytes read:", total)
        _usbif.host_cdc_close()
        _usbif.host_stop()


if __name__ == "__main__":
    main()
