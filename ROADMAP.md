# Roadmap

usbif is heading toward every USB role a board can play, in both directions,
on the boards people actually use.

## Device classes

- A USB microphone: a capture endpoint on the device side, so a board can be a
  USB mic or a full-duplex sound card for a PC or another board.
- `network.USBD_NCM` (Ethernet over USB, new in MicroPython 1.29): either a
  costume for it, or a line in the roster on enabling it without usbif.

## MIDI

- USB MIDI moves onto pydevices' `mididev`: usbif supplies the USB device and
  host ports, and the port contract and desktop MIDI backends move there.

## Closer to MicroPython

- usbif rebuilds on MicroPython's own USB support: device classes become
  packages on `machine.USBDevice`, with what MicroPython lacks added in the
  same style. Code written for usbif's current API will need changing.
- Host mode follows, without losing anything usbif's host does today.

## Host and device together

- An answer to whether a board can be USB host and USB device at once on two
  connectors: a pedal hosting a MIDI controller while a PC sees its REPL,
  sound card or drive.

Bugs, and things you need that don't work yet, go to
[issues](https://github.com/PyDevices/usbif/issues).
