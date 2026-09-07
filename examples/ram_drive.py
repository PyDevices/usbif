"""A RAM disk the host sees as a removable drive.

``msc_attach`` serves a buffer the application supplies -- no SD card, no
filesystem on the board, just bytes the host can read and write. Contrast
with ``sd_drive.py``, which hands the host a real block device.

    mpftp run -d COM49 examples/ram_drive.py

**What you get.** A small writable volume (default 64 KiB). Windows / macOS /
Linux will want to format it the first time; that is expected for a blank
buffer. After format, files copy on and off like any thumb drive.

**One writer.** While the host has the drive, do not also mount the same
buffer locally. ``msc_status()`` reports eject; honour it before touching the
bytes yourself.

**Console.** CDC+MSC so the REPL stays on the cable.
"""

import time

import _usbif

# 128 x 512-byte blocks = 64 KiB. Small enough for SRAM on an S3, large enough
# that a host will format and mount it without complaining about capacity.
BLOCKS = 128
BLOCK = 512


def main():
    _usbif.msc_detach()

    buf = bytearray(BLOCKS * BLOCK)
    _usbif.msc_attach(buf, True)

    _usbif.dev_functions(_usbif.FN_CDC | _usbif.FN_MSC)
    attached, n_blocks, _ = _usbif.msc_status()
    print("RAM disk attached:", attached, "blocks:", n_blocks,
          "(%d KiB)" % (n_blocks * BLOCK // 1024))
    print("format it on the host the first time, then copy files")

    while True:
        time.sleep_ms(5)
        _, _, ejected = _usbif.msc_status()
        if ejected:
            print("host ejected; releasing")
            _usbif.msc_detach()
            return


if __name__ == "__main__":
    main()
