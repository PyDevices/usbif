# usbif API Sketch — v0

**Status:** Gate 1 deliverable for review. Nothing here is stable; every name is an argument, not a promise.

The sketch is written against all backends at once — the ESP native module, the desktop backends, and the hypothetical RP2350 TinyUSB backend of the vision's Appendix A — because the surface must not leak any one stack's assumptions. CircuitPython's `usb_host`/`usb_hid`/`usb_midi` family was studied for shape; names differ where PyDevices conventions differ.

## Design rules carried in from the vision

1. Python configures and observes; C moves isochronous bytes.
2. Events cross the RTOS boundary through a ring buffer, not per-event callbacks.
3. Applications discover capabilities; they never probe by `ImportError` or chip name.
4. The report-to-event and byte-to-message policy is written once, in the provider package, for every backend.

## Package shape

`usbif` is a provider package in `pydevices/lib`, beside `audiodev` and `displaydev`, selecting a backend the same way: the native `_usbif` C module where present, an OS-service backend on desktop. Both halves are always importable; emptiness is expressed through capabilities, not absence:

```python
import usbif

usbif.host.capabilities()    # e.g. frozenset({"hid", "msc", "cdc", "midi"}) — S3 host build
usbif.device.capabilities()  # e.g. frozenset({"uac", "midi", "hid", "cdc", "msc"})
usbif.host.capabilities()    # frozenset() — desktop device role, wasm, unsupported port
```

## Events

Attach and detach are ordinary PyDevices events, delivered through the same polled queue the event system already uses:

```python
# event.type in (usbif.ATTACH, usbif.DETACH); event.device is a DeviceInfo
DeviceInfo: id, vid, pid, product, serial, classes (frozenset), speed
```

HID keyboards and mice are auto-routed through the shared policy into standard key and pointer events — application code cannot tell a USB keyboard on a P4 from an SDL keyboard on the desktop. That routing is the default and can be disabled per device to claim it raw.

## Host surface

```python
usbif.host.start()        # daemon + class tasks; idempotent
usbif.host.stop()
usbif.host.devices()      # tuple[DeviceInfo] currently attached
```

Per class, everything hangs off `usbif.host.<class>` and takes a `DeviceInfo.id`:

| Class | Surface | Notes |
|---|---|---|
| hid | auto-routing by default; `hid.raw(id) -> RawHID` with `read()`/`write()`/report descriptors | RawHID is what `uhidapi` also implements on desktop |
| msc | `msc.mount(id, "/usb")`, `msc.umount(id)`; optional automount policy flag | Exposes a VFS block device; desktop maps to the OS mount point and reports it |
| cdc | `cdc.open(id, baud=115200, ...) -> stream` | `read`/`write`/`any`; COM ports and termios on desktop |
| midi | `midi.ports() -> tuple[MidiPort]`; `port.read()` / `port.write(msg)` | Messages in the `audioif`/`micropython-vst3` model; virtual ports appear here on desktop |
| uac | `uac.devices()` — then selection happens in `audiodev` under shared names | Deliberately not a second audio API |
| uvc | `uvc.open(id, size=(w, h), format="mjpeg") -> Camera`; `camera.read_into(buf)` | Frames arrive display-pipeline-ready; C owns the isochronous side |

## Device surface

Device-role classes compose with `machine.USBDevice`'s runtime model as additional built-in interfaces, so they coexist with CDC, MSC, and Python-defined interfaces. Configuration happens before the device (re)activates; the boot-ordering contract will be documented, not discovered.

```python
snd = usbif.device.uac(sample_rate=48000, channels=2, bits=16, mic=False)
snd.sink          # an audioif-compatible output; audioif plays, C streams
snd.active        # is the host streaming?  (observe, don't pump)

cam = usbif.device.uvc(size=(640, 480), format="mjpeg", source=frame_source)

usbif.device.midi() / .hid(...) / .cdc() / .msc(...)
# thin re-exports of machine.USBDevice + micropython-lib usb-device-* packages,
# under portable names, so host vs device is configuration rather than a
# different library
```

## Desktop truth table

The desktop backend implements the same names; `capabilities()` tells the truth per OS. Host: hid/msc/cdc/midi real, uac enumeration-and-selection only, uvc out of v1 scope. Device: empty except `midi` where virtual ports exist (ALSA sequencer, loopMIDI/IAC). The parity harness asserts event and message equality across backends every phase.

## Open questions for Gate 1 review

1. Event payload shape under MicroPython memory pressure: allocate `DeviceInfo` objects, or interned tuples with accessor helpers?
2. `asyncio` integration: expose streams and awaitables directly, or keep the polled model and let `aiorepl`-style wrappers grow on top?
3. Automount default for MSC: on (friendly) or off (predictable)?
4. Should `uac` host selection live entirely in `audiodev` (as sketched) or mirror a minimal listing under `usbif` for symmetry with the other classes?
5. Error model: plain `OSError` with errno discipline, or a `usbif.Error` hierarchy?
6. Naming of the auto-routing switch (`hid.claim(id)` vs `hid.raw(id)` implying the claim)?
