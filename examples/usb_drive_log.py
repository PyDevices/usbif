"""Append sensor readings to a hosted USB flash drive, read-write.

    mpftp run -d COM49 examples/usb_drive_log.py

The datalogger case: a board writing what it produced onto a stick, then
carrying on. Mounts the stick's FAT filesystem read-write, appends to a
log file, unmounts to flush, and remounts to read the lines back -- so the
proof is that the bytes reached the medium, not a cache.

The one-writer rule applies here exactly as it does on the device side: a
filesystem with two writers is a corrupted filesystem, so nothing else
should hold this stick while the board has it.

Verified against a commodity PNY 8 GB stick formatted FAT32 on a PC.
"""
import _usbif
import os
import time


class USBPartition:
    def __init__(self, start_lba, num_blocks, block_size):
        self.start = start_lba
        self.count = num_blocks
        self.bs = block_size
        self._one = bytearray(block_size)

    def readblocks(self, block_num, buf, offset=0):
        want = len(buf)
        done = 0
        lba = self.start + block_num
        if offset:
            _usbif.host_msc_read(lba, self._one)
            take = min(want, self.bs - offset)
            buf[0:take] = self._one[offset:offset + take]
            done = take
            lba += 1
        while done < want:
            take = min(self.bs, want - done)
            if take == self.bs:
                _usbif.host_msc_read(lba, memoryview(buf)[done:done + self.bs])
            else:
                _usbif.host_msc_read(lba, self._one)
                buf[done:done + take] = self._one[0:take]
            done += take
            lba += 1
        return 0

    def writeblocks(self, block_num, buf, offset=0):
        # Whole blocks go straight out. A partial block is read-modify-write:
        # the host driver refuses a short write rather than padding it, and
        # padding is exactly what would corrupt the neighbouring bytes.
        want = len(buf)
        done = 0
        lba = self.start + block_num
        if offset:
            _usbif.host_msc_read(lba, self._one)
            take = min(want, self.bs - offset)
            self._one[offset:offset + take] = buf[0:take]
            _usbif.host_msc_write(lba, self._one)
            done = take
            lba += 1
        while done < want:
            take = min(self.bs, want - done)
            if take == self.bs:
                _usbif.host_msc_write(lba, memoryview(buf)[done:done + self.bs])
            else:
                _usbif.host_msc_read(lba, self._one)
                self._one[0:take] = buf[done:done + take]
                _usbif.host_msc_write(lba, self._one)
            done += take
            lba += 1
        return 0

    def ioctl(self, op, arg):
        if op == 4:
            return self.count
        if op == 5:
            return self.bs
        return 0


print("host_start ->", _usbif.host_start(("msc",)))

dev = None
for _ in range(20):
    time.sleep_ms(500)
    devs = _usbif.host_devices()
    if devs:
        dev = devs[0]
        break

if dev is None:
    print("no device attached")
    _usbif.host_stop()
else:
    print("host_msc_open ->", _usbif.host_msc_open(dev[0]))
    blocks, bs, inquiry = _usbif.host_msc_info()
    print("drive:", repr(inquiry))

    mbr = bytearray(bs)
    _usbif.host_msc_read(0, mbr)
    e = 446
    start = int.from_bytes(mbr[e + 8:e + 12], "little")
    count = int.from_bytes(mbr[e + 12:e + 16], "little")

    part = USBPartition(start, count, bs)
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

    _usbif.host_msc_close()
    _usbif.host_stop()
    print("done")
