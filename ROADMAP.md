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

- CDC, HID and MIDI device move to MicroPython's own `usb-device` packages on
  `machine.USBDevice`, and usbif's versions retire.
- Audio and video stay in C on TinyUSB's class drivers, since isochronous
  endpoints can't be serviced from Python. Mass storage stays in C for speed.
- Offer MicroPython the small TinyUSB hooks usbif patches in today, so usbif
  builds against an unmodified MicroPython.
- usbif's Python layer moves into pydevices as `usbdev`, beside `displaydev`
  and `audiodev`.
- A portable USB host on TinyUSB's host side, so host mode reaches boards
  beyond the ESP32 (today it uses ESP-IDF's USB Host Library).

## Host and device together

- An answer to whether a board can be USB host and USB device at once on two
  connectors: a pedal hosting a MIDI controller while a PC sees its REPL,
  sound card or drive.

Bugs, and things you need that don't work yet, go to
[issues](https://github.com/PyDevices/usbif/issues).
