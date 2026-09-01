"""Mount a USB flash drive hosted by the board, and list its files.

    mpftp run -d COM49 examples/usb_drive_mount.py

The host side of MSC reads blocks; MicroPython's FAT driver mounts block
devices. All that was missing between them is an object with readblocks /
writeblocks / ioctl -- and the partition offset, since the filesystem does
not start at LBA 0 on a partitioned stick, which is the detail that makes
a naive mount fail with a confusing error rather than an obvious one.

Verified against a commodity PNY 8 GB stick formatted FAT32 on a PC: the
board listed its directory and read back the text of a file written on
the PC, with no PC in the loop at read time.

Mounted **read-only on purpose**. The host driver implements READ(10) and
not WRITE(10), so `writeblocks` raises EROFS rather than quietly doing
nothing -- a drive that silently discards writes is worse than one that
refuses them.
"""
import _usbif
import os
import time


class USBPartition:
    """A partition on a hosted MSC device, as a MicroPython block device."""

    def __init__(self, start_lba, num_blocks, block_size):
        self.start = start_lba
        self.count = num_blocks
        self.bs = block_size
        self._one = bytearray(block_size)

    def readblocks(self, block_num, buf, offset=0):
        # The extended protocol hands us a buffer that may span several
        # blocks; the host driver reads one block per call, so loop.
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
                mv = memoryview(buf)[done:done + self.bs]
                _usbif.host_msc_read(lba, mv)
            else:
                _usbif.host_msc_read(lba, self._one)
                buf[done:done + take] = self._one[0:take]
            done += take
            lba += 1
        return 0

    def writeblocks(self, block_num, buf, offset=0):
        # The host driver has no WRITE(10) yet: reading proves the
        # transport, and writing someone's drive is a decision this layer
        # has not been asked to make. Mount read-only.
        raise OSError(30)   # EROFS

    def ioctl(self, op, arg):
        if op == 4:         # block count
            return self.count
        if op == 5:         # block size
            return self.bs
        if op == 6:         # erase block -- no-op for FAT
            return 0
        return 0


def main():
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
        return

    dev_id, vid, pid = dev[0], dev[1], dev[2]
    print("device {:04x}:{:04x} classes={}".format(vid, pid, dev[5]))
    print("host_msc_open ->", _usbif.host_msc_open(dev_id))

    blocks, bs, inquiry = _usbif.host_msc_info()
    print("drive: {!r}  {} MB".format(inquiry, blocks * bs // (1024 * 1024)))

    mbr = bytearray(bs)
    _usbif.host_msc_read(0, mbr)
    if mbr[510] != 0x55 or mbr[511] != 0xAA:
        print("no MBR signature; not a partitioned disk")
        _usbif.host_msc_close()
        _usbif.host_stop()
        return

    # Partition table: four 16-byte entries starting at 0x1BE. Type byte at
    # +4, first LBA at +8, block count at +12, both little-endian.
    for i in range(4):
        e = 446 + i * 16
        ptype = mbr[e + 4]
        if ptype == 0:
            continue
        start = int.from_bytes(mbr[e + 8:e + 12], "little")
        count = int.from_bytes(mbr[e + 12:e + 16], "little")
        print("partition {}: type 0x{:02x} start {} count {} ({} MB)".format(
            i + 1, ptype, start, count, count * bs // (1024 * 1024)))

        part = USBPartition(start, count, bs)
        try:
            os.mount(part, "/usb", readonly=True)
        except Exception as exc:
            print("  mount failed:", exc)
            continue

        print("  mounted at /usb")
        try:
            for name in os.listdir("/usb"):
                try:
                    st = os.stat("/usb/" + name)
                    kind = "dir " if st[0] & 0x4000 else "file"
                    print("    {} {:>9}  {}".format(kind, st[6], name))
                except Exception:
                    print("    ?           ", name)
            # Read one text file back, to prove real file content crosses.
            for name in os.listdir("/usb"):
                if name.lower().endswith(".txt"):
                    with open("/usb/" + name) as f:
                        data = f.read(200)
                    print("  --- {} ---".format(name))
                    print("  " + repr(data))
                    break
        finally:
            os.umount("/usb")
            print("  unmounted")
        break

    _usbif.host_msc_close()
    _usbif.host_stop()
    print("done")


main()
