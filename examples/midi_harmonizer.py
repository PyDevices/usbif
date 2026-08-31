# A MIDI effect, not an audio effect: harmonize whatever this board is
# handed. Every note that arrives on the USB MIDI function goes back out
# with a major third and a perfect fifth stacked on top -- melody in,
# triads out. Everything that is not a note (controllers, pitch bend,
# program changes, clock) passes through untouched, which is what being a
# well-mannered MIDI effect means.
#
# The example neither knows nor cares who is on the other end. Tonight
# that is a DAW talking both directions down one cable (USB MIDI is full
# duplex); the day this board grows a second USB voice, the same loop
# ships its output to a hardware synth instead by pointing `send` at the
# other port. The music logic would not change.
#
# Deploy standalone by copying to /main.py: it starts at boot, needs no
# console, and runs until reset -- the pattern for a board that lives its
# life as an instrument. Interactively: `mpftp run examples/midi_harmonizer.py`
# (detached) and interrupt to stop.

import time

import _usbif

# The chord stack, in semitones above each played note. Change these and
# the board becomes a different instrument: (12,) octaves, (3, 7) minor,
# (4, 7, 10) dominant sevenths, (7,) power chords.
HARMONY = (4, 7)

_rx = bytearray(64)


def send(msg):
    _usbif.midi_write(msg)


def harmonize(status, note, vel):
    # The played note first, then its harmonies, clamped to the MIDI
    # ceiling. Offsets are constant, so the note-off for every harmony is
    # regenerated identically from the note-on's math -- no bookkeeping,
    # no stuck notes.
    send(bytes([status, note, vel]))
    for iv in HARMONY:
        h = note + iv
        if h <= 127:
            send(bytes([status, h, vel]))


def run():
    # Minimal running-status MIDI parser: hosts may split messages across
    # reads or omit repeated status bytes; both are normal on a wire.
    status = 0
    data = bytearray(2)
    need = 0
    have = 0
    while True:
        n = _usbif.midi_read(_rx)
        if n <= 0:
            time.sleep_ms(2)
            continue
        for i in range(n):
            b = _rx[i]
            if b >= 0xF8:
                send(bytes([b]))        # real-time: pass through, mid-anything
            elif b >= 0x80:
                status = b
                have = 0
                need = 1 if 0xC0 <= b <= 0xDF else 2
                if b >= 0xF0:
                    status = 0          # system common: swallowed, keeps this small
            elif status:
                data[have] = b
                have += 1
                if have == need:
                    have = 0            # running status: stay armed
                    kind = status & 0xF0
                    if kind == 0x90 or kind == 0x80:
                        harmonize(status, data[0], data[1])
                    elif need == 2:
                        send(bytes([status, data[0], data[1]]))
                    else:
                        send(bytes([status, data[0]]))


run()
