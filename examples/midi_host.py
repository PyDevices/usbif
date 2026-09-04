"""Host a USB MIDI instrument: read what it plays, play notes back to it.

    mpftp run -d COM49 examples/midi_host.py

The board becomes the USB host and a commercial MIDI device plugs into it
-- a keyboard, a drum machine, a sound module. No PC anywhere in the chain.

This completes the other half of the MIDI story. `midi_harmonizer.py` has
the board as a MIDI *device* being driven by a host; here the roles are
reversed. Both directions use the same plain MIDI byte stream, so code
that parses or generates MIDI does not care which end it is on -- the
USB-MIDI 32-bit packet framing lives in C and never reaches Python.

**Status: written against the driver, not yet run against hardware.** The
driver (`src/usbif_host_midi.c`) is new and compile-verified only; nobody
upstream ships a MIDI host driver for the IDF, so it is ours. Expect to
find things. What it does when it meets a real instrument is exactly what
wants recording in docs/phase0-findings.md.
"""

import time

import _usbif
import usbif

# Two-note chord sent to the device, to prove the OUT pipe. Middle C and the
# fifth above it, loud enough to hear on a sound module.
_NOTE_ON = 0x90
_NOTE_OFF = 0x80
_CHORD = (60, 67)


def describe(status, d1, d2):
    """Human-readable line for one MIDI message, for the console."""
    kind = status & 0xF0
    ch = (status & 0x0F) + 1
    if kind == 0x90 and d2:
        return "note on  ch{} note {} vel {}".format(ch, d1, d2)
    if kind == 0x80 or (kind == 0x90 and not d2):
        return "note off ch{} note {}".format(ch, d1)
    if kind == 0xB0:
        return "control  ch{} cc {} = {}".format(ch, d1, d2)
    if kind == 0xE0:
        return "bend     ch{} {}".format(ch, (d2 << 7 | d1) - 8192)
    if kind == 0xC0:
        return "program  ch{} {}".format(ch, d1)
    return "status {:02x} {} {}".format(status, d1, d2)


def main():
    print("host_start ->", _usbif.host_start(("midi",)))

    dev = None
    for _ in range(20):
        time.sleep_ms(500)
        devs = _usbif.host_devices()
        if devs:
            dev = devs[0]
            break

    if dev is None:
        print("no MIDI device attached")
        print("stats", _usbif.host_stats())
        _usbif.host_stop()
        return

    dev_id, vid, pid = dev[0], dev[1], dev[2]
    print("device {:04x}:{:04x} classes={} speed={}".format(vid, pid, dev[5], dev[6]))
    if "midi" not in dev[5]:
        print("not a MIDI device; nothing to do")
        _usbif.host_stop()
        return

    _usbif.host_midi_open(dev_id)
    print("opened -- play something (30 s)")

    # Send a chord first: on a sound module this is audible proof the OUT
    # pipe works before a single key is pressed.
    try:
        n = _usbif.host_midi_write(bytes([_NOTE_ON, _CHORD[0], 100,
                                          _NOTE_ON, _CHORD[1], 100]))
        print("sent a chord,", n, "bytes accepted")
    except OSError as exc:
        # A keyboard with no OUT pipe is a legitimate device, not a failure.
        print("no OUT pipe on this device (receive-only):", exc)

    buf = bytearray(64)
    # usbif.MidiParser keeps a partial message across reads rather than
    # guessing at it, and counts any byte it cannot place.
    parser = usbif.MidiParser()
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < 30000:
        n = _usbif.host_midi_read(buf)
        if n:
            parser.feed(buf, n)
            for status, data in parser.drain():
                if status >= 0xF8:
                    continue                # realtime clock/sense: not worth printing
                d1 = data[0] if data else 0
                d2 = data[1] if len(data) > 1 else 0
                print(" ", describe(status, d1, d2))
        else:
            time.sleep_ms(5)

    try:
        _usbif.host_midi_write(bytes([_NOTE_OFF, _CHORD[0], 0,
                                      _NOTE_OFF, _CHORD[1], 0]))
    except OSError:
        pass

    if parser.desync:
        print("desync:", parser.desync, "byte(s) arrived with no status")
    dropped = _usbif.host_midi_dropped()
    if dropped:
        print("rx dropped:", dropped, "bytes (ring overflowed)")
    _usbif.host_midi_close()
    _usbif.host_stop()
    print("done")


main()
