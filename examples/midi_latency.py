# Measure total MIDI-to-audio round-trip latency using one board as both
# ends of the chain.
#
# The board sends a MIDI note-on out its USB MIDI function and timestamps
# it; a DAW on the host (input-monitoring a synth, its output routed to
# this board's sound-card function) renders the note; the board then polls
# its own audio-OUT endpoint until the first non-silent samples arrive and
# timestamps that. Because the departure and the arrival happen on the
# same clock, one ticks_diff spans the entire system:
#
#   midi_write -> host MIDI stack -> DAW input -> synthesis ->
#   host audio engine -> USB audio -> this FIFO.
#
# What the number means: the whole system's latency, dominated by the
# host's audio buffering (DAW block size, shared-mode engine periods),
# not by usbif itself. Run several trials and read the minimum as the
# floor; the first trial often pays one-time costs.
#
# Prerequisites: uac_enable(True) done and the host's DAW armed as above;
# the C pump must NOT be running (it would consume the samples this
# script inspects) -- uac_pump_stop() first, and bring your board's
# normal audio back up afterwards (on the P4 dev board: re-run
# /soundcard.py).

# First measured run (2026-08-31, ESP32-P4 -> Windows 11 -> REAPER with
# ReaSynth on default audio settings): 8/8 trials, min 200.7 ms,
# avg 204.4 ms, max 207.8 ms. The six-millisecond spread says buffering,
# not jitter -- that floor belongs to the DAW's audio device
# configuration, and shrinks with it.

import time

import _usbif

TRIALS = 8
NOTE = 0x3C            # middle C
SILENCE_MS = 250       # how long the line must be quiet between trials
TIMEOUT_MS = 2000
# Amplitude, not nonzero-ness: a shared-mode host ships +/-1 LSB dither
# even when "silent", so any-nonzero-byte detection never sees quiet.
# A real note at full velocity is thousands of LSB.
LOUD = 300

buf = bytearray(512)


def block_is_loud():
    n = _usbif.uac_read(buf)
    if n <= 0:
        return False
    # Every 4th sample is plenty: an onset spans many blocks.
    for i in range(0, n - 1, 8):
        v = buf[i] | (buf[i + 1] << 8)
        if v >= 32768:
            v -= 65536
        if v > LOUD or v < -LOUD:
            return True
    return False


def drain_until_quiet():
    quiet_since = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), quiet_since) < SILENCE_MS:
        if block_is_loud():
            quiet_since = time.ticks_ms()


# Clear anything a previous session left ringing: a note-on whose off was
# lost sustains forever, and "wait for silence" then waits with it.
_usbif.midi_write(bytes([0xB0, 123, 0]))   # CC 123: all notes off
_usbif.midi_write(bytes([0x80, NOTE, 0]))
time.sleep(0.3)

results = []
for trial in range(TRIALS):
    drain_until_quiet()
    t0 = time.ticks_us()
    _usbif.midi_write(bytes([0x90, NOTE, 0x7F]))
    latency_ms = None
    while time.ticks_diff(time.ticks_us(), t0) < TIMEOUT_MS * 1000:
        if block_is_loud():
            latency_ms = time.ticks_diff(time.ticks_us(), t0) / 1000
            break
    _usbif.midi_write(bytes([0x80, NOTE, 0]))
    print('trial', trial, 'latency_ms', latency_ms)
    if latency_ms is not None:
        results.append(latency_ms)

if results:
    print('trials', len(results),
        'min %.1f ms  avg %.1f ms  max %.1f ms'
        % (min(results), sum(results) / len(results), max(results)))
else:
    print('no audio detected: is the DAW monitoring MIDI into a synth,'
        ' with output routed to this board?')
