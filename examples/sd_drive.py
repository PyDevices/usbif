"""The board's SD card, as an ordinary USB drive on a PC.

Plug the board into a computer, run this, and the card in its slot appears
as a removable drive -- files copy on and off with the file manager, no
recompile and no special tool at either end. That is the whole point of MSC
as a device: fonts and graphics written in, sensor logs read out.

    mpftp run -d COM49 examples/sd_drive.py

Unlike `msc_attach`, which serves a buffer the application supplies (a RAM
disk), this uses `msc_attach_blockdev`, which serves *real storage*: the
host's reads and writes reach the card itself. Any object implementing
MicroPython's standard block protocol works -- `machine.SDCard`, mip's
`sdcard.SDCard` over SPI, a flash partition.

Two things worth knowing before trusting this on a board of your own.

**One writer at a time.** A filesystem with two writers is a corrupted
filesystem. While the host has this drive, the board must not mount the same
card locally -- unmount first, hand it over, and take it back only after the
host ejects (which `msc_status()` reports). This code never mounts it
locally, so there is nothing to unmount here.

**The host waits on your block device.** These callbacks run on the same
thread as the interpreter, so every USB event -- CDC, HID, MIDI -- queues
behind whatever `readblocks` does. Native SDIO (`machine.SDCard`) is quick.
A bit-banged SPI card is not, and a slow enough card can miss the host's own
SCSI timeouts. Measure before trusting a slow backing store under load.
"""

import time

import _usbif


def open_card():
    """Return a block device for the card, or raise with something useful.

    Native SDIO first: on boards that have it (the ESP32-P4 panels, for
    one) it is a hardware peripheral, needs no pin map, and is by far the
    quickest path. Falling back to the board config covers the boards whose
    card is on SPI, where the wiring is a per-board fact that `pydevices`
    already records rather than something this example should guess.
    """
    try:
        from machine import SDCard
    except ImportError:
        SDCard = None

    if SDCard is not None:
        try:
            return SDCard()
        except TypeError:
            return SDCard(slot=0)
        except Exception:
            pass        # no SDIO slot here; try the board config below

    try:
        import board_config  # noqa: F401  (populates the peripheral namespace)
        import boarddev

        return boarddev.sdcard()
    except Exception as exc:
        raise RuntimeError(
            "no SD card found: this board has no native SDIO slot and no "
            "board_config peripheral to describe its wiring. Build the block "
            "device yourself and pass it to _usbif.msc_attach_blockdev(); see "
            "this file's header for what qualifies."
        ) from exc


def main():
    # Anything a previous run left attached: the C side holds the object
    # through a VM root, so it survives a soft reset and would otherwise
    # refuse this attach.
    _usbif.msc_detach()

    card = open_card()
    blocks = card.ioctl(4, 0)
    block_size = card.ioctl(5, 0)
    print("card: {} blocks x {} bytes = {} MB".format(
        blocks, block_size, blocks * block_size // (1024 * 1024)))

    _usbif.msc_attach_blockdev(card, True)

    # Present a console alongside the drive. Advertising both means the REPL
    # stays reachable on the same cable while the host has the card, which
    # is what makes this comfortable to iterate on; MSC alone works equally
    # well if that is what the application wants.
    _usbif.dev_functions(_usbif.FN_CDC | _usbif.FN_MSC)

    attached, n_blocks, _ = _usbif.msc_status()
    print("attached:", attached, "blocks:", n_blocks)
    print("the card should now appear as a drive on the host")

    # Service loop. USB events are delivered through MicroPython's scheduler
    # on this port, so the interpreter only has to stay responsive -- a short
    # sleep is enough, and a long one starves enumeration.
    calls_before = 0
    while True:
        time.sleep_ms(5)

        _, _, ejected = _usbif.msc_status()
        if ejected:
            # "Safely remove" on the host side. Honouring it is what makes
            # the eject mean something, and it is the signal an application
            # waits for before touching the card again itself.
            print("host ejected the drive; releasing the card")
            _usbif.msc_detach()
            return

        calls, errors = _usbif.msc_bd_stats()
        if calls != calls_before:
            # Errors here are block-device exceptions turned into failed SCSI
            # commands. A steadily climbing error count means the host is
            # retrying reads that never succeed -- worth seeing rather than
            # discovering as a drive that mounts but cannot be read.
            print("blocks served:", calls, "errors:", errors)
            calls_before = calls


if __name__ == "__main__":
    main()
