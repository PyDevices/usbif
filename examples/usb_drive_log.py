"""Append sensor readings to a hosted USB flash drive, read-write.

    mpftp run -d COM49 examples/usb_drive_log.py

The datalogger case: a board writing what it produced onto a stick, then
carrying on. Mounts the stick's FAT filesystem read-write, appends to a
log file, unmounts to flush, and remounts to read the lines back -- so the
proof is that the bytes reached the medium, not a cache.

The one-writer rule applies here exactly as it does on the device side: a
filesystem with two writers is a corrupted filesystem, so nothing else
should hold this stick while the board has it.

The block device is ``host.partition(..., readonly=False)`` -- read-only is
the default, because the safe answer to "may I write to the drive somebody
just plugged in" is no, and this is the case that deliberately says yes.

Verified against a commodity PNY 8 GB stick formatted FAT32 on a PC.
"""
import os
import time

import usbif.auto

host = usbif.auto.host(classes=("msc",)).start()
print("started ->", tuple(sorted(host.started)))

dev = None
for _ in range(20):
    time.sleep_ms(500)
    devs = host.devices()
    if devs:
        dev = devs[0]
        break

if dev is None:
    print("no device attached")
    host.stop()
else:
    print("msc_open ->", host.msc_open(dev.id))
    blocks, bs, inquiry = host.msc_info()
    print("drive:", repr(inquiry))

    mbr = bytearray(bs)
    host.msc_read(0, mbr)
    e = 446
    start = int.from_bytes(mbr[e + 8:e + 12], "little")
    count = int.from_bytes(mbr[e + 12:e + 16], "little")

    part = host.partition(start, count, readonly=False)
    os.mount(part, "/usb")          # read-write this time
    print("mounted /usb read-write")
    print("before:", os.listdir("/usb"))

    # Append, the way a logger would: open in append mode so re-running
    # accumulates rather than replacing.
    with open("/usb/SENSOR.LOG", "a") as f:
        for i in range(5):
            f.write("t={} reading={}\n".format(time.ticks_ms(), 20 + i))
    print("appended 5 lines")

    os.umount("/usb")
    print("unmounted (flushed)")

    # Remount and read it back -- proving the bytes reached the medium and
    # not just a cache.
    os.mount(part, "/usb", readonly=True)
    print("after:", os.listdir("/usb"))
    with open("/usb/SENSOR.LOG") as f:
        data = f.read()
    print("--- SENSOR.LOG ({} bytes) ---".format(len(data)))
    print(data)
    os.umount("/usb")

    host.msc_close()
    host.stop()
    print("done")
