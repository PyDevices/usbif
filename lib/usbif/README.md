# usbif

Portable USB host and device contracts, with one backend per platform — the
same surface on a board and on a workstation, so an application written for
one runs on the other.

```python
from usbif import auto
import usbif, events

host = auto.host().start()
for info in host.devices():
    print(usbif.describe(info))

while True:
    for event in host.poll():
        if event.type == events.USBATTACH:
            print("plugged in:", usbif.describe(event.device))
    ...
```

## What the pieces are

| Module | Role |
|---|---|
| `usbif` | The contracts: `Host`, `Device`, `DeviceInfo`, the class names, and the `USBATTACH`/`USBDETACH` event types |
| `usbif.auto` | Picks a backend. Never imported by a backend, and never raises — a port with no USB returns a `NullHost` |
| `usbif.linux_usb` | Linux, WSL, and containers, reading `/sys/bus/usb/devices` |
| `usbif.win_usb` | Windows, over `uwin32`'s cfgmgr32 bindings |
| `usbif.native_usb` | Hardware, over the `usbif` native C module |

## Two rules worth knowing before you read the code

**Capabilities are discovered, never assumed.** `host.capabilities()` returns a
frozenset of class names, and it is legitimately empty — on a desktop the OS
owns the bus, and on a port without USB there is nothing to own. Where a
backend exists at all, enumeration and hot-plug work; the capability set says
which classes it can carry *traffic* for. Branch on the set, not on
`ImportError`.

Backends exist today for Linux, Windows and the native C module, all
bench-proven. macOS sits at the **community-verified tier, open** (see the
org's [platform support tiers](https://github.com/PyDevices/.github/blob/main/docs/platform-support-tiers.md)):
no Mac is on this project's bench and none is coming, so `auto.select_backend()`
has no `darwin` branch yet and macOS falls through to `NullHost`, which
enumerates nothing and says so. The seam is designed rather than merely
absent — IOKit would carry enumeration, the same way `uwin32` carries it on
Windows — and a report from a Mac user is what promotes the claim.

**Events are drained, not delivered.** `host.poll()` returns what has been
buffered since the last call. This is not a stylistic choice: on ESP32 a
C-side callback reaches Python through `mp_sched_schedule`, which is excellent
while the VM runs bytecode and collapses inside a long C call. Measured on an
ESP32-S3 at 1 kHz, a `sha256` pass over 120 KB lost 76% of events, and flash
writes lost 99% with a single 1537 ms stall. Backends therefore capture events
the moment they happen and hand them over when asked, so a late poll costs
latency — which the application controls — rather than data. Overflow is
reported through `host.overflowed` rather than passing in silence.

## One device, two operating-system models

Linux publishes one node per device with its interfaces beneath it. Windows
publishes a composite device as a parent node *plus* one node per interface,
each with its own class and its own name. Reported verbatim, a two-interface
board would be one device on Linux and three on Windows, and "the same API"
would be a claim rather than a fact — so `win_usb` reassembles them through the
parent link cfgmgr32 provides, and takes identity from the device node rather
than from whichever interface happened to be enumerated first.

Where an OS genuinely cannot answer, the backend says so instead of inventing:
cfgmgr32 does not report negotiated link speed, so `speed` is `None` on
Windows, which the contract allows for exactly this reason.

## Tests

`tests/test_usbif.py` in this repository is a conformance suite, not a
Linux test: the same assertions run against every backend, which is what keeps
the two implementations honest about being one API. The Linux backend is
exercised against a synthetic sysfs tree so the suite behaves the same on a
laptop with devices attached, in CI with none, and on a board.

```bash
python -m unittest discover -s tests -p "test_usbif.py"
```

Backends not available on the running platform skip, so the same command is
correct everywhere: on Linux it exercises the Linux backend against a
synthetic sysfs tree, and on Windows it exercises `win_usb` against the real
bus. Both were also run under MicroPython, since `uwin32` has an `ffi` branch
as well as a `ctypes` one and the two must agree.
