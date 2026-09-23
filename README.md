# usbif

Native USB for MicroPython: a board that a computer sees as a sound card, a
MIDI instrument or a webcam, and — on the same board — a USB host that a
keyboard, thumb drive, MIDI controller or camera plugs into.

New here? Read the [newcomer's guide](docs/newcomers.md) for USB identity,
host sessions, event delivery, and the firmware integration boundary.

`usbif` is one of the PyDevices `*if` modules, with
[`displayif`](https://github.com/PyDevices/displayif),
[`audioif`](https://github.com/PyDevices/audioif) and
[`cameraif`](https://github.com/PyDevices/cameraif): a native C module with a
thin, stable Python surface that higher-level packages build on. The portable
API lives here too, in `lib/usbif`, frozen with the C module by `manifest.py`,
and is implemented twice — once by this module and once by desktop backends
over OS services. That
portability covers enumeration, device identity and hot-plug events today, so
an application that observes devices runs unchanged on a workstation; the
streaming surfaces (the audio pump, the video endpoint) are methods on the
object `usbif.auto` hands back, and have no desktop counterpart yet.

## Status: early development

**Nothing is released. Working today: sound card, MIDI instrument, HID
keyboard and mouse, removable drive, and a USB host that enumerates devices
and speaks CDC, HID, and MSC — with the board's USB identity chosen at
runtime from Python, in any of thirty-one combinations.**

The flagship demo, real and audible: one ESP32 board on one cable was
simultaneously the MIDI keyboard driving a DAW **and** the sound card
playing that DAW's output -- first and last device in the signal chain at
once, REPL riding the same connector.

An ESP32-P4 running this enumerates on a PC as a class-compliant USB Audio
device — no driver installed — alongside its CDC REPL on the same connector,
plays audio out of the board's codec, and is **opt-in**: at boot the board is
a plain CDC device, and the audio function appears only when Python asks for
it. A host cannot be wedged by a board nobody is pumping.

The board's USB identity is a Python decision, not a build option: every
function is compiled in, and the application chooses which the host sees.

```python
import usbif.auto

dev = usbif.auto.device()
dev.functions("cdc", "uac")      # a console and a sound card
dev.functions("midi")            # a bare MIDI instrument, interface 0
dev.functions()                  # -> frozenset({'midi'})

dev.uac_pump_start(bclk, ws, dout, rate=24000, bits=16, channels=1)
```

Each call re-enumerates -- USB has no way to change identity in place -- and
the configuration descriptor is assembled at that moment, with interfaces
and endpoints renumbered and the device class set to match what was actually
emitted.

Endpoints are numbered per costume because the controller's budget is small.
The ESP32-S2 and S3 drive IN endpoints 1 to 4 only (their transmit FIFOs stop
there, and TinyUSB gives an IN endpoint the FIFO of its own number), and
every function brings at least one: CDC two, MSC, audio, MIDI, HID and video
one each. A costume that needs more than the chip has is refused by
`functions()` with a `ValueError` rather than worn with an endpoint that
swallows one transfer and never completes another (usbif#23 was that, with
HID at a fixed endpoint 6). `examples/costume_selftest.py` lists which
combinations the board you run it on can wear. The P4's high-speed
controller drives IN endpoints 1 to 7, so every combination fits there.

Also working, and the foundation the rest builds on:

- the portable API and its Linux and Windows desktop backends, in `lib/usbif`,
  with one conformance suite run against every backend
- the event transport in C (`src/shared/usbif_ringbuf.c`), with host-side tests
- a structural validator for the descriptor assembler
  (`examples/costume_selftest.py`), which checks every costume the firmware
  can wear without a host in the loop
- four small patches to MicroPython (`patches/`), each with provenance: a
  `tusb_config.h` hook, a configuration-descriptor hook, three weak hooks that
  let this module vary what it advertises at runtime, and an esp32 helper that
  lets it borrow the OTG controller for host duty

**Working now, and new:** external hubs, and more than one device at once --
two devices behind a powered hub enumerated together, the module's first
multi-device host session, with the hub itself correctly filtered out for
matching no requested class. (The Phase 2 observation that a drive behind a
hub was invisible turned out to be an IDF Kconfig default, not missing
support.) And **MIDI as a host**: `src/usbif_host_midi.c`, ours because
nobody upstream ships an IDF MIDI host driver -- a real keyboard's
MIDIStreaming interface and endpoints discovered, notes packed and sent,
with the USB-MIDI packet codec split into `shared/` and tested on the host.
Incoming notes are proven too: 989 channel messages read from a keyboard --
notes with velocity, pitch bend across its full range, several CCs, and
channel-10 drums -- with zero bytes dropped.

**Video and host audio, both directions, proven:** a USB webcam hosted on an
ESP32-S3 with its picture on the board's own panel (`src/usbif_host_uvc.c`,
`examples/uvc_display.py`), and the board seen as a webcam by Windows
(`src/usbif_uvc_dev.c`, `examples/usbif_webcam.py`) -- sourcing real frames
from a camera where the board has one, colour bars where it does not. Host
audio plays too: the isochronous stall that blocked it was a
`pdMS_TO_TICKS()` rounding to zero ticks at `CONFIG_FREERTOS_HZ=100`, so a
delay that read as 5 ms never yielded at all.

**Not yet working, said precisely:** macOS
desktop support, which sits at the [community-verified
tier](https://github.com/PyDevices/.github/blob/main/docs/platform-support-tiers.md):
no Mac is on this project's bench, `auto.py` returns a null backend there
rather than pretending, and a report from the field is what promotes it.

**Real-storage MSC (`msc_attach_blockdev`), proven on the ESP32-P4.** A device-side drive can now be backed by any
MicroPython block-device object -- an SD card, a flash partition -- not just
a buffer: the read10/write10 callbacks call the object's
`readblocks`/`writeblocks` directly, safe on this port because TinyUSB's
device task runs through MicroPython's own scheduler (the same mechanism
upstream's `machine.USBDevice` runtime already uses to call Python from
inside a TinyUSB callback), so there is no foreign-thread hazard, only a
latency cost stated in the code. A 32 GB card in an ESP32-P4 panel
enumerated on Windows as `MicroPy Mass Storage`, 32,094,814,208 bytes --
exactly its 62685184 blocks x 512 -- with the MBR parsed and a partition
mapped to a drive letter, and **488 block ranges served with zero errors**.
What is *not* yet exercised is the write path against a host, and the card
used carries no filesystem Windows can mount, so Explorer shows no files
on it; both wait on formatting a card, which destroys data and so waits on
a decision rather than being assumed. The P4's `machine.SDCard()` needs
explicit pins, and card initialisation has two rough edges.

And the working host side carries honest limits for now: the controller's
FIFO split is fixed when the host starts, so `host(classes=...)` chooses it
from what was asked for (`uvc` leans IN; `uac` without `uvc` makes room for a
speaker's 196-byte packet and a microphone's; otherwise the board's Kconfig
bias), and a camera and a USB Audio 2.0 speaker cannot share one `start()`;
concurrent sessions are bounded by a
*channel budget*, not by class -- the ESP32-S3 has eight host channels
(`OTG_NUM_HOST_CHAN`), a bulk pair costs two and an interrupt IN costs
one, and every device including a hub costs one for its control pipe.
On the bench that means HID pairs with either MSC or MIDI, while MSC and
MIDI together do not fit. Note the class filter does *not* buy room: it
declines interfaces, not devices, and every enumerated device keeps its
control pipe either way -- so a hub costs two channels before carrying
anything. Host mode is proven at full speed only: the ESP32-P4's
high-speed host mode has an open defect
([#3](https://github.com/PyDevices/usbif/issues/3)). MSC now mounts as a filesystem, read-write
(`examples/usb_drive_mount.py` lists a hosted stick; `usb_drive_log.py`
appends sensor lines to one and reads them back after a remount), with
Bulk-Only Reset Recovery and REQUEST SENSE behind it, both exercised by
a deliberate device-side failure rather than assumed; HID delivers raw reports rather than decoded
events; and `host_stop()` on a device that was genuinely held open is now
reliable at realistic hold times (8 of 8 clean in this session's sample)
after two rounds of fixes -- the teardown now closes each class driver's
session properly instead of skipping straight to the raw device handle,
and does so with the event-pump the library's async endpoint-halt/flush
completions need, which nothing was providing once the host task's own
main loop had already exited for shutdown. A narrower race remains at
very short hold times (call `host_stop()` within a fraction of a second
of opening a session and it can still wedge), root-caused to a single
esp-idf call (`usb_host_endpoint_halt()`) blocking indefinitely -- a layer
below anything in this module's own source, not yet closed. A wedged host
now raises `OSError` instead of silently pretending to keep working
either way. `host_start()`'s class filter **is** now honoured -- verified both
in the intersection arithmetic and against a live device (excluded from a
class tuple, it is invisible to `host_devices()`; included, it attaches) --
and `capabilities()` reports the true built set (`{'cdc', 'hid', 'msc'}` on
the S3 bench build) rather than the empty set it silently returned since
Phase 1. One board-configuration finding worth restating here, now
resolved: the stock `ESP32_GENERIC_S3` board build did not enable
`MICROPY_HW_USB_MSC`, so a first pass of testing found the S3 could only
reach fifteen of the thirty-one costumes (missing MSC) -- a
board-configuration gap, not a module limitation, exactly like the same
distinction already drawn for the P4. The S3 board-header patch in the build
workspace's `patches/usbif-NN-*` series now enables it too (matching the P4's
own patch), and
`examples/costume_selftest.py` confirms **31 of 31** on the S3 as well.
The API design is in [`docs/api-sketch.md`](docs/api-sketch.md).

## Why the events are drained rather than delivered

The design decision most likely to surprise a reader is that USB events are
buffered in C and collected by Python calling `poll()`, instead of arriving as
callbacks. That is a measurement, not a preference.

On ESP32 a C callback reaches Python through `mp_sched_schedule`. Measured on
an ESP32-S3 at a 1 kHz event rate:

| Load during the 3 s window | Events delivered | Lost | Worst stall |
|---|---|---|---|
| Python bytecode | 2999 / 3000 | 0% | 1.0 ms |
| `sha256` over 120 KB | 720 / 3002 | 76% | 11.9 ms |
| Flash file writes | 7 / 4070 | **99%** | **1537 ms** |

While the VM sits inside one long C call, no scheduled callback runs at all —
and MSC block writes, display flushes and audio buffering are exactly the
workloads `usbif` exists to serve. A ring buffer written from interrupt context
cannot be starved that way: a late poll costs latency, which the application
controls, rather than data. Overflow is reported rather than passing in
silence, because the mechanism it replaces failed silently.

## Building

The module follows the standard MicroPython external C module contract, and
`micropython.mk` covers the Make-based ports. Two steps come first, though,
and skipping them builds a module whose USB functions are silently absent
rather than one that fails loudly:

1. **Apply the patches.** The device functions reach the host through hooks
   this module adds to MicroPython's shared TinyUSB glue, and the host side
   needs the esp32 OTG helper:

   ```bash
   ./apply_patches.sh --apply      # --status to check, --revert to undo
   ```

2. **Point the board at the extension header**, by adding this line to your
   board's `mpconfigboard.h` (see `patches/` for the ESP32_GENERIC_P4 and
   ESP32_GENERIC_S3 versions, which are carried as board patches because the
   line is board integration rather than module code):

   ```c
   #define MICROPY_HW_USB_EXT_TUSB_CONFIG "usbif_tusb_ext.h"
   ```

Then build as usual:

```bash
idf.py -D MICROPY_BOARD=ESP32_GENERIC_S3 -D USER_C_MODULES=/path/to/usbif/micropython.cmake build
```

Without both steps the build succeeds and the board simply enumerates without
audio or MIDI -- quiet rather than obviously broken, which is why it is
called out here. One caution learned the hard way
and filed upstream as
[micropython#19667](https://github.com/micropython/micropython/issues/19667):
do not pass `BUILD=` to the esp32 port's `make`. It propagates into the
mpy-cross sub-make, which then plants its own qstr fragments in your build
directory and breaks the link. Use `idf.py -B` for an out-of-tree build
instead, which does not inherit the variable.

## Examples

| File | Role | What it shows |
|---|---|---|
| [`soundcard.py`](examples/soundcard.py) | device | Class-compliant UAC sound card (C pump). Pair with `usb_speaker.py` |
| [`uac_pump.py`](examples/uac_pump.py) | device | Python FIFO pump -- inspectable path, not the shipping card |
| [`hid_keyboard.py`](examples/hid_keyboard.py) | device | Board types into the host |
| [`hid_mouse.py`](examples/hid_mouse.py) | device | Board moves the host cursor |
| [`ram_drive.py`](examples/ram_drive.py) | device | RAM disk via `msc_attach` |
| [`sd_drive.py`](examples/sd_drive.py) | device | SD card as a USB drive via `msc_attach_blockdev` |
| [`usbif_webcam.py`](examples/usbif_webcam.py) | device | Board is a UVC webcam (`cameraif` when present) |
| [`midi_harmonizer.py`](examples/midi_harmonizer.py) | device | MIDI effect: melody in, triads out |
| [`midi_harmonizer_ui.py`](examples/midi_harmonizer_ui.py) | device | Harmonizer with a touchscreen chord picker |
| [`midi_device_in.py`](examples/midi_device_in.py) | device | Prove the board receives MIDI from a host |
| [`midi_latency.py`](examples/midi_latency.py) | device | MIDI-to-audio round-trip timing |
| [`host_enum.py`](examples/host_enum.py) | host | Attach/detach via the portable API (board or desktop) |
| [`hid_host.py`](examples/hid_host.py) | host | USB keyboard → PyDevices key events (M1) |
| [`usb_serial.py`](examples/usb_serial.py) | host | CDC read/write to a USB-serial device |
| [`usb_speaker.py`](examples/usb_speaker.py) | host | Play through a hosted USB speaker / `soundcard.py` |
| [`usb_mic.py`](examples/usb_mic.py) | host | Capture from a hosted USB microphone |
| [`usb_drive_mount.py`](examples/usb_drive_mount.py) | host | Mount a flash drive and list files |
| [`usb_drive_log.py`](examples/usb_drive_log.py) | host | Append sensor lines to a hosted stick |
| [`midi_host.py`](examples/midi_host.py) | host | Host a MIDI keyboard; send a chord back |
| [`uvc_display.py`](examples/uvc_display.py) | host | Hosted webcam on the board's panel (MJPEG via `jpegio` when present) |
| [`costume_selftest.py`](examples/costume_selftest.py) | — | Validate every costume's descriptors without a host |

Board-to-board pairings, VBUS warnings, and firmware holes (by issue number)
live in [`examples/README.md`](examples/README.md).

## Tests

The ring buffer is tested on the host, where a failure is a two-second answer
rather than a reflash:

```bash
cc -std=c11 -Wall -Wextra -Werror -Isrc -o /tmp/test_ringbuf tests/test_ringbuf.c src/shared/usbif_ringbuf.c && /tmp/test_ringbuf
```

The portable API's conformance suite lives with the API, in this repository
(it finds `events` and `audiodev` in a sibling `pydevices` checkout):

```bash
python -m unittest discover -s tests -p "test_usbif.py"
```

## Licence

MIT.
