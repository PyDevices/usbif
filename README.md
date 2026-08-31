# usbif

Native USB for MicroPython: a board that a computer sees as a sound card, a
MIDI instrument or a webcam, and — on the same board — a USB host that a
keyboard, thumb drive, MIDI controller or camera plugs into.

`usbif` is the third PyDevices `*if` module, after
[`displayif`](https://github.com/PyDevices/displayif) and
[`audioif`](https://github.com/PyDevices/audioif): a native C module with a
thin, stable Python surface that higher-level packages build on. The portable
API ships separately, as `lib/usbif` in
[`pydevices`](https://github.com/PyDevices/pydevices), and is implemented twice
— once by this module and once by desktop backends over OS services. That
portability covers enumeration, device identity and hot-plug events today, so
an application that observes devices runs unchanged on a workstation; the
streaming surfaces (MIDI bytes, the audio pump) are reached through `_usbif`
directly and have no desktop counterpart yet.

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
import usbif.auto, _usbif

dev = usbif.auto.device()
dev.functions("cdc", "uac")      # a console and a sound card
dev.functions("midi")            # a bare MIDI instrument, interface 0
dev.functions()                  # -> frozenset({'midi'})

_usbif.uac_pump_start(bclk, ws, dout, rate=24000, bits=16, channels=1)
```

Each call re-enumerates -- USB has no way to change identity in place -- and
the configuration descriptor is assembled at that moment, with interfaces
renumbered and the device class set to match what was actually emitted.

Also working, and the foundation the rest builds on:

- the portable API and its Linux and Windows desktop backends, in `pydevices`
  (`lib/usbif`), with one conformance suite run against every backend
- the event transport in C (`src/shared/usbif_ringbuf.c`), with host-side tests
- a structural validator for the descriptor assembler
  (`examples/costume_selftest.py`), which checks every costume the firmware
  can wear without a host in the loop
- four small patches to MicroPython (`patches/`), each with provenance: a
  `tusb_config.h` hook, a configuration-descriptor hook, three weak hooks that
  let this module vary what it advertises at runtime, and an esp32 helper that
  lets it borrow the OTG controller for host duty

**Not yet working, said precisely:** UVC in either direction; MIDI as a
*host* (the board as a MIDI device is the flagship above); audio as a host;
hubs; a device-side drive backed by anything other than a buffer the
application supplies (an SD card or flash partition would need block
callbacks that reach hardware from the USB task, which is a design step, not
an omission); and macOS
desktop support, which sits at the [community-verified
tier](https://github.com/PyDevices/.github/blob/main/docs/platform-support-tiers.md):
no Mac is on this project's bench, `auto.py` returns a null backend there
rather than pretending, and a report from the field is what promotes it. And the working host side carries honest limits for now: one session per
class at a time; proven at full speed only (the ESP32-P4's high-speed host
mode has an open defect, tracked in the findings); MSC reads blocks but is
not mounted as a filesystem; HID delivers raw reports rather than decoded
events; and `host_stop()` hangs on the S3 for a device that was genuinely
held open, roughly 3 of 4 times in one session's sample -- a fix for the
other case (a device rejected by the class filter and closed immediately)
landed and is reliable, and a wedged host now raises `OSError` instead of
silently pretending to keep working, but the underlying hang is not yet
closed. `host_start()`'s class filter **is** now honoured -- verified both
in the intersection arithmetic and against a live device (excluded from a
class tuple, it is invisible to `host_devices()`; included, it attaches) --
and `capabilities()` reports the true built set (`{'cdc', 'hid', 'msc'}` on
the S3 bench build) rather than the empty set it silently returned since
Phase 1. One board-configuration finding worth restating here: the
"thirty-one costumes" result is P4-specific -- it needs all five device
functions compiled in, and the stock `ESP32_GENERIC_S3` board build used
for this session's host work does not enable `MICROPY_HW_USB_MSC`, so the
device-side costume count on that exact build is fifteen (2^4 - 1, no MSC),
not thirty-one; enabling it is a board-configuration decision, not a module
limitation, per the same distinction already drawn for the P4 case above.
All are recorded in [`docs/phase0-findings.md`](docs/phase0-findings.md).
The plan and the evidence behind every decision are in [`docs/`](docs/).

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

## Tests

The ring buffer is tested on the host, where a failure is a two-second answer
rather than a reflash:

```bash
cc -std=c11 -Wall -Wextra -Werror -Isrc -o /tmp/test_ringbuf tests/test_ringbuf.c src/shared/usbif_ringbuf.c && /tmp/test_ringbuf
```

The portable API's conformance suite lives with the API, in `pydevices`:

```bash
python -m unittest discover -s tests -p "test_usbif.py"
```

## Licence

MIT.
