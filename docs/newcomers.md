# Newcomer's guide to usbif

`usbif` gives a MicroPython board a native USB device and host surface. A board can appear to a computer as a sound card, MIDI device, keyboard, drive, or webcam; it can also host keyboards, drives, MIDI controllers, cameras, and audio devices.

It is a native firmware module. The portable `usbif` API in [pydevices](https://github.com/PyDevices/pydevices) has desktop backends for enumeration and hot-plug observation, while device streaming remains board-native.

## Start by choosing the board's USB identity

```python
import usbif.auto

dev = usbif.auto.device()
dev.functions("cdc", "uac")  # console plus sound card
dev.uac_pump_start(bclk, ws, dout, rate=24000, bits=16, channels=1)
```

Calling `functions()` re-enumerates the board because USB identity cannot change in place. The descriptor is assembled from the selected functions at that point; endpoint limits are checked before the board advertises an impossible costume.

For USB host work, start with [the examples](../examples/README.md), especially `host_enum.py`, then request the host classes your application needs.

## The mental model

```text
Python chooses device functions or host classes
              |
              v
runtime descriptor / host session
              |
              v
TinyUSB callbacks -> C ring buffer -> Python poll()
              |
              v
application events, audio pump, storage, MIDI, or video
```

Events are drained by Python polling rather than delivered directly in callbacks. Long-running Python or native work can starve scheduled callbacks, whereas the C ring preserves events until the application polls and reports overflow explicitly.

## Repository map

| Path | Purpose |
|---|---|
| `src/mod_usbif.c` | Native module entrypoint and Python surface. |
| `src/usbif_*_dev.c` | Device-side MIDI, storage, audio, HID, and video functions. |
| `src/usbif_host*.c` | Host enumeration plus CDC, HID, MSC, MIDI, UAC, and UVC support. |
| `src/shared/` | Event ring and USB-MIDI packet code. |
| `lib/usbif/` | Portable API and desktop backends. |
| `patches/` | Required MicroPython TinyUSB and ESP32 host integration patches. |
| `examples/` | Device and host demonstrations, including costume validation. |
| `tests/` | Native ring, leak, and API tests. |

## Firmware boundary

First apply the repository-owned patches, then configure the board extension header and build with `micropython.cmake` as `USER_C_MODULES`. The root [README](../README.md#building) has the required order. Omitting either integration step can produce a successful build whose USB functions are silently absent.

Build targets have real resource limits. Device functions consume endpoint capacity, and host sessions consume controller channels; ask for a smaller function/class set when the selected combination cannot fit. `examples/costume_selftest.py` verifies every device identity the current firmware can advertise without requiring a host.

## Safe first contributions

Start with a host or device example, a descriptor validation case, or a focused ring-buffer test. Do not add Python callbacks directly from TinyUSB context. USB identity, endpoint allocation, and host channel budgeting are native correctness boundaries; read [the API sketch](api-sketch.md) and the root README before changing them.
