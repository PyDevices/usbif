"""``machine.USBDevice`` beside a costume (usbif#26). Run on the DEVICE board.

MicroPython's runtime USB device (micropython-lib's ``usb-device``) appends a
Python-defined interface after the built-in ones, numbering it from
``machine.USBDevice.BUILTIN_DEFAULT``'s ``itf_max`` and ``ep_max`` and copying
its ``desc_cfg``. On a usbif image those have to describe the costume being
worn, not every function compiled in, or the interface never opens and the
host sees a device in an error state.

Needs ``usb-device`` and ``usb-device-mouse`` from micropython-lib in ``/lib``
(``mpftp mip install usb-device-mouse`` brings both). Run it from the board's
UART console, not its USB REPL: the USB device re-enumerates under it.

    mpftp run --follow -d COM4 tests/hardware/usbdevice_runtime_check.py

Prints ``PASS``/``FAIL`` per check and a final ``GATE PASS``/``GATE FAIL``.
The host's side is worth watching too: the mouse moves the pointer 100 px
right and back, and a host that lists USB devices shows a composite device
with a HID mouse beside the costume's functions, not one in an error state.

The checker can fail, and did: on an image without usbif's patch 0006,
``itf_max`` read 10 and ``desc_cfg`` 533 bytes (every function compiled in)
while the CDC costume has 2 interfaces, the runtime driver was offered
interfaces 2 to 9 and raised ``KeyError`` for each, and Windows showed the
device in its Error state.
"""

import time

import machine
import usb.device
import usbif.auto
from usb.device.mouse import MouseInterface

#: Interface classes each costume function brings (USB class codes).
CLASSES = {
    "cdc": {0x02, 0x0A},
    "msc": {0x08},
    "hid": {0x03},
    "midi": {0x01},
    "uac": {0x01},
    "uvc": {0x0E},
}

FAILS = []


def check(name, ok, detail=""):
    print("%-52s %s %s" % (name, "PASS" if ok else "FAIL", detail))
    if not ok:
        FAILS.append(name)


def walk(cfg):
    """(interface count, interface classes, highest endpoint number)."""
    itfs = set()
    classes = set()
    highest = 0
    o = cfg[0]
    while o + 2 <= len(cfg) and cfg[o] >= 2:
        if cfg[o + 1] == 4:
            itfs.add(cfg[o + 2])
            classes.add(cfg[o + 5])
        elif cfg[o + 1] == 5:
            highest = max(highest, cfg[o + 2] & 0x0F)
        o += cfg[o]
    return len(itfs), classes, highest


def run():
    builtin = machine.USBDevice().BUILTIN_DEFAULT
    cfg = builtin.desc_cfg
    total = cfg[2] | cfg[3] << 8
    itfs, classes, highest = walk(cfg)
    worn = usbif.auto.device().functions()
    want = set()
    for name in worn:
        want |= CLASSES.get(name, set())
    check("desc_cfg describes the worn costume", classes == want,
          "costume %s, classes %s" % (sorted(worn), sorted(classes)))
    check("desc_cfg is as long as its wTotalLength", len(cfg) == total,
          "%d vs %d" % (len(cfg), total))
    check("itf_max is the worn configuration's interface count",
          builtin.itf_max == itfs == cfg[4],
          "itf_max %d, interfaces %d, bNumInterfaces %d"
          % (builtin.itf_max, itfs, cfg[4]))
    check("ep_max is one past its highest endpoint", builtin.ep_max == highest + 1,
          "ep_max %d, highest %d" % (builtin.ep_max, highest))

    mouse = MouseInterface()
    t0 = time.ticks_ms()
    usb.device.get().init(mouse, builtin_driver=True)
    while not mouse.is_open() and time.ticks_diff(time.ticks_ms(), t0) < 8000:
        time.sleep_ms(20)
    check("the runtime mouse opens", mouse.is_open(),
          "after %d ms" % time.ticks_diff(time.ticks_ms(), t0))
    if mouse.is_open():
        time.sleep_ms(1000)
        sent = 0
        for step in (5,) * 20 + (-5,) * 20:
            try:
                mouse.move_by(step, 0)
                sent += 1
            except Exception as exc:                  # noqa: BLE001
                print("move_by:", repr(exc))
                break
            time.sleep_ms(30)
        check("every report is accepted", sent == 40, "%d of 40" % sent)
    print("GATE FAIL" if FAILS else "GATE PASS")


run()
