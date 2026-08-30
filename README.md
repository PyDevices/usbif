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
— once by this module and once by desktop backends over OS services — so an
application written against it runs unchanged on a workstation.

## Status: early development

**Phase 1 of 8. Nothing is released, and no USB class works yet.** What exists
today is the foundation the rest is built on:

- the portable API and its Linux desktop backend, in `pydevices` (`lib/usbif`),
  with a conformance suite that runs against every backend
- the event transport in C (`src/shared/usbif_ringbuf.c`), with host-side tests
- a `tusb_config.h` extension hook (`patches/`) proven to compile TinyUSB's
  UAC driver into MicroPython's own TinyUSB instance
- this module, building into ESP32 firmware and reporting an honest empty
  capability set

Host classes (HID, MSC, CDC) arrive in Phase 2; MIDI in Phase 3; the UAC sound
card — the flagship — in Phase 4. The plan and its evidence are in
[`docs/`](docs/).

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

The module follows the standard MicroPython external C module contract, so it
needs nothing from this workspace:

```bash
idf.py -D MICROPY_BOARD=ESP32_GENERIC_S3 -D USER_C_MODULES=/path/to/usbif/micropython.cmake build
```

`micropython.mk` covers the Make-based ports. One caution learned the hard way
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
