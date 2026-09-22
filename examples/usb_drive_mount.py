"""Mount a USB flash drive hosted by the board, and list its files.

    mpftp run -d COM49 examples/usb_drive_mount.py

The host side of MSC reads blocks; MicroPython's FAT driver mounts block
devices. All that was missing between them is an object with readblocks /
writeblocks / ioctl -- and the partition offset, since the filesystem does
not start at LBA 0 on a partitioned stick, which is the detail that makes
a naive mount fail with a confusing error rather than an obvious one.
``host.partition()`` is that object; it used to be copy-pasted into this
file and into ``usb_drive_log.py``, and now lives in ``usbif.native_usb``.

Verified against a commodity PNY 8 GB stick formatted FAT32 on a PC: the
board listed its directory and read back the text of a file written on
the PC, with no PC in the loop at read time.

Mounted **read-only on purpose**. The host driver implements READ(10) and
not WRITE(10), so `writeblocks` raises EROFS rather than quietly doing
nothing -- a drive that silently discards writes is worse than one that
refuses them.
"""
import os
import time

import usbif.auto


def main():
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
        return

    print("device {:04x}:{:04x} classes={}".format(dev.vid, dev.pid, dev.classes))
    print("msc_open ->", host.msc_open(dev.id))

    blocks, bs, inquiry = host.msc_info()
    print("drive: {!r}  {} MB".format(inquiry, blocks * bs // (1024 * 1024)))

    mbr = bytearray(bs)
    host.msc_read(0, mbr)
    if mbr[510] != 0x55 or mbr[511] != 0xAA:
        print("no MBR signature; not a partitioned disk")
        host.msc_close()
        host.stop()
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

        part = host.partition(start, count)          # readonly by default
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

    host.msc_close()
    host.stop()
    print("done")


main()
