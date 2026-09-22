"""Talk to a USB serial device the board is hosting.

Opens the first CDC-ACM device on the host port, writes a short line, and
prints whatever comes back. A USB-serial adapter, another MCU's CDC console,
or a PyDevices board presenting CDC all work -- standard class, no bespoke
protocol.

    mpftp run -d COM49 examples/usb_serial.py

**Pairing.** Device side is ordinary CDC (MicroPython's built-in console, or
``dev.functions("cdc")``). Host side is this script on an S3.
"""

import time

import usbif.auto


def find_cdc(host, timeout_ms=15000):
    deadline = time.ticks_add(time.ticks_ms(), timeout_ms)
    while time.ticks_diff(deadline, time.ticks_ms()) > 0:
        found = host.find("cdc")
        if found:
            return found[0].id
        time.sleep_ms(250)
    return None


def main(seconds=15):
    host = usbif.auto.host(classes=("cdc",)).start()
    dev_id = find_cdc(host)
    if dev_id is None:
        print("no CDC device found")
        host.stop()
        return

    print("CDC device", dev_id)
    host.cdc_open(dev_id)
    msg = b"hello from usbif host\r\n"
    n = host.cdc_write(msg)
    print("wrote", n, "bytes:", msg)

    buf = bytearray(256)
    t0 = time.ticks_ms()
    total = 0
    try:
        while time.ticks_diff(time.ticks_ms(), t0) < seconds * 1000:
            got = host.cdc_read(buf)
            if got:
                total += got
                print("rx:", bytes(buf[:got]))
            else:
                time.sleep_ms(20)
    finally:
        print("total bytes read:", total)
        host.cdc_close()
        host.stop()


if __name__ == "__main__":
    main()
