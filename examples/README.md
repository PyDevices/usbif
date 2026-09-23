# usbif examples

One script per use of the shipping API. Each role is class-compliant on its
own: a PC or a commercial peripheral is a valid other end. Board-to-board
pairing is the same scripts on two boards, not a special `pair_*` file.

## Class × role

| Class | Device (board presents as…) | Host (board drives…) |
|---|---|---|
| **UAC** | [`soundcard.py`](soundcard.py) (C pump); [`uac_pump.py`](uac_pump.py) (Python FIFO) | [`usb_speaker.py`](usb_speaker.py), [`usb_mic.py`](usb_mic.py) |
| **UVC** | [`usbif_webcam.py`](usbif_webcam.py) | [`uvc_display.py`](uvc_display.py) (MJPEG via `jpegio` when present) |
| **MIDI** | [`midi_harmonizer.py`](midi_harmonizer.py), [`midi_harmonizer_ui.py`](midi_harmonizer_ui.py), [`midi_device_in.py`](midi_device_in.py), [`midi_latency.py`](midi_latency.py) | [`midi_host.py`](midi_host.py) |
| **HID** | [`hid_keyboard.py`](hid_keyboard.py), [`hid_mouse.py`](hid_mouse.py) | [`hid_host.py`](hid_host.py) |
| **MSC** | [`sd_drive.py`](sd_drive.py), [`ram_drive.py`](ram_drive.py) | [`usb_drive_mount.py`](usb_drive_mount.py), [`usb_drive_log.py`](usb_drive_log.py) |
| **CDC** | built-in MicroPython console / costume bit | [`usb_serial.py`](usb_serial.py) |
| **enum** | — | [`host_enum.py`](host_enum.py) (portable API; board or desktop) |
| **self-test** | [`costume_selftest.py`](costume_selftest.py) | — |

## Board-to-board pairings

Roles follow the hardware each board has, not their size. **P4 is always the
device** in these pairings: its high-speed host detects no device
([usbif#3](https://github.com/PyDevices/usbif/issues/3)).

| Device board | Script | Host board | Script | What is lent |
|---|---|---|---|---|
| **P4** | `soundcard.py` | **S3** | `usb_speaker.py` | Sound output -- the headline offload |
| **P4** | `usbif_webcam.py` | **S3** | `uvc_display.py` | Camera → panel |
| **P4** | `sd_drive.py` | **S3** | `usb_drive_mount.py` | Shared storage |
| **P4** / **S3** | `midi_harmonizer.py` | **S3** | `midi_host.py` | MIDI effect / instrument |
| **S3** | `hid_keyboard.py` | **S3** | `hid_host.py` | Control surface (needs two S3s, or S3-device + PC) |

A PC can replace either end. That is the point of standard classes.

### Headline: P4 as a USB sound card for an S3

1. Console on each board's **UART bridge**, not on the OTG port under test.
2. P4: `mpftp run examples/soundcard.py` -- enumerates as Speakers, C pump
   into the ES8311.
3. S3: OTG adapter or powered hub on the host port (the Waveshare S3 touch
   boards do not switch VBUS; see also [usbif#5](https://github.com/PyDevices/usbif/issues/5)).
4. S3: `mpftp run examples/usb_speaker.py` -- finds the P4, plays a 440 Hz tone.
5. Hear it on the P4's speaker.

### Power

Connecting two self-powered boards can **back-feed** VBUS and leave a board
silent and unflashable. Default link: a cable that omits VBUS, or a powered
hub between them. Unplug both USB cables and use UART alone to recover a
wedged board.

## Firmware holes (do not invent an example)

| Hole | Issue | Consequence for examples |
|---|---|---|
| Device UAC is speaker-only (no mic endpoint) | [usbif#7](https://github.com/PyDevices/usbif/issues/7) | `usb_mic.py` hosts a commercial mic; no device-side capture script |
| P4 high-speed host detects nothing | [usbif#3](https://github.com/PyDevices/usbif/issues/3) | Every pairing puts the P4 in device role |
| Host FIFO split is per session | [usbif#2](https://github.com/PyDevices/usbif/issues/2) | `host(classes=...)` decides it: `uvc` leans IN (600 B), `uac` without `uvc` gives periodic OUT 200 B and IN 528 B; a camera and a 2.0 speaker cannot share one `start()` |

## Conventions

- Headers name the use, the board role, the other end, and whether UART must
  hold the REPL (any costume that drops CDC cuts a native-USB session).
- Reach the hardware through `usbif.auto` and never through `_usbif`. The
  C module is package-internal; `usbif.auto.host()`, `.device()` and
  `.open_midi()` hand back objects that carry the whole surface, streaming
  included. `tests/test_no_usbif_leak.py` enforces it.
- No measured fps or latency numbers are claimed in headers for scripts that
  have not been re-run in this pass -- cite prior evidence or stay quiet.
