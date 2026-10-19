# usbif

**usbif has moved, to two homes.** Its history went with it.

- **The Python packages** (`usb-core` with `usb.core`, `usb.util` and
  `usb_host`; `usb-host-msc`, `-hid`, `-cdc`, `-audio`, `-video`, `-net`;
  `usb-device-audio`, `-video`, `-msc`; `usb-hid`) are in
  [pydevices](https://github.com/PyDevices/pydevices), beside `midiports`.
  Install them from there:
  `mip.install("github:PyDevices/pydevices/usb-host-msc")`.
  [docs/usb.md](https://github.com/PyDevices/pydevices/blob/main/docs/usb.md)
  is their page.
- **The C module** is now **usbhost**, in
  [micropython-pydevices, `modules/usbhost`](https://github.com/PyDevices/micropython-pydevices/tree/main/modules/usbhost), and its binding
  is `_usbhost` (it was `_usbif`). The examples, the proof matrix and the
  host checks moved with it. Name it in a build with
  `build_mp.py ... --modules usbhost`.

Code that uses the packages doesn't change. Code that imported `_usbif`
directly imports `_usbhost`. Open issues against pydevices for the packages
and against micropython-pydevices for the C module. This repository is kept
read-only as a record.
