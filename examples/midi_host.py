"""Host a USB MIDI instrument: read what it plays, play notes back to it.

    mpftp run -d COM49 examples/midi_host.py

The board becomes the USB host and a commercial MIDI device plugs into it
-- a keyboard, a drum machine, a sound module. No PC anywhere in the chain.

This completes the other half of the MIDI story. `midi_harmonizer.py` has
the board as a MIDI *device* being driven by a host; here the roles are
reversed. Both directions use the same plain MIDI byte stream, so code
that parses or generates MIDI does not care which end it is on -- the
USB-MIDI 32-bit packet framing lives in C and never reaches Python.

**Proven on hardware.** A Donner keyboard delivered 989 channel messages
(notes with velocity, pitch bend across its range, CCs, channel-10 drums)
with zero bytes dropped. Host MIDI OUT was closed on a DIN loopback through
an M-Audio interface: all eight sent messages returned byte-exact, median
5 ms round trip. Nobody upstream ships an IDF MIDI host driver, so this one
is ours (`src/usbif_host_midi.c`); the numbers above are why the status line
no longer says "compile-verified only".
"""

import time

import usbif
import usbif.auto

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
    host = usbif.auto.host(classes=("midi",)).start()
    print("started ->", tuple(sorted(host.started)))

    dev = None
    for _ in range(20):
        time.sleep_ms(500)
        devs = host.devices()
        if devs:
            dev = devs[0]
            break

    if dev is None:
        print("no MIDI device attached")
        print("stats", host.stats())
        host.stop()
        return

    print("device {:04x}:{:04x} classes={} speed={}".format(
        dev.vid, dev.pid, dev.classes, dev.speed))
    if "midi" not in dev.classes:
        print("not a MIDI device; nothing to do")
        host.stop()
        return

    # The hosted device as a MidiPort: the same object the board's own MIDI
    # function gives, so the parsing and generating below is role-agnostic.
    port = usbif.auto.open_midi("host:{}".format(dev.id))
    print("opened -- play something (30 s)")

    # Send a chord first: on a sound module this is audible proof the OUT
    # pipe works before a single key is pressed.
    try:
        n = port.write(bytes([_NOTE_ON, _CHORD[0], 100,
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
        n = port.read(buf)
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
        port.write(bytes([_NOTE_OFF, _CHORD[0], 0,
                          _NOTE_OFF, _CHORD[1], 0]))
    except OSError:
        pass

    if parser.desync:
        print("desync:", parser.desync, "byte(s) arrived with no status")
    # (rx_dropped, release_failed), not a single count -- and a 2-tuple is
    # always truthy, so testing the tuple itself reported a drop on every
    # clean run.
    rx_dropped, release_failed = port.dropped()
    if rx_dropped:
        print("rx dropped:", rx_dropped, "bytes (ring overflowed)")
    if release_failed:
        print("transfer releases failed:", release_failed)
    port.close()
    host.stop()
    print("done")


main()
