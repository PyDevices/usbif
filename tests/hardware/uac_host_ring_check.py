"""UAC host ring check (usbif#36). Run on the HOST board.

The host board plays to a hosted USB speaker (a P4 running
``examples/soundcard.py`` on the bench) from the interpreter thread, the way an
app with a UI does, while stalls are injected into that thread:

- **long Python work**: a busy loop of ``WORK_MS`` every ``WORK_EVERY_MS``;
- **scheduled ticks**: a ``machine.Timer`` whose callback runs ``TICK_MS`` of
  Python every ``TICK_EVERY_MS``. The callback is scheduled, so it runs between
  the feeder's bytecodes on the same thread and stalls it exactly as a UI
  redraw or a network poll would.

The two periods are coprime, so the stalls sometimes stack (up to
``WORK_MS + TICK_MS``).

The feeder never blocks: it asks ``space()`` and writes that much with
``try_write()``. Each case opens the stream, fills the ring, then counts the
driver's ``starved`` packets (a bus interval that found the ring short and sent
silence) across ``SECONDS`` of stalls. The first fill is not counted: a ring
starts empty, and so does every stream.

Gate: a case with ``expect="pass"`` must starve 0 packets; a case with
``expect="fail"`` must starve at least ``FAIL_MIN``. The second kind is the
proof that the checker can fail -- the old 8 KB ring under the same stalls.
A ``control`` case, 8 KB with no stalls, shows the small ring is not simply
broken on its own.

The signal is a 400 Hz sine at -40 dBFS: quiet enough to leave running near
people, loud enough to confirm by ear if you want to.

    mpftp run --follow -d COM17 tests/hardware/uac_host_ring_check.py

Prints one ``CASE`` line per case and a final ``GATE PASS``/``GATE FAIL``.
"""

import gc
import math
import time

import machine
import usbif.auto
from usbif import uac, uac_audio

RATE = 48000
SECONDS = 20
WORK_MS = 300
WORK_EVERY_MS = 1500
TICK_MS = 150
TICK_EVERY_MS = 1100
FAIL_MIN = 100          # starved packets; a 300 ms stall on 8 KB alone is ~250

# (name, ring_ms or None for the 8 KB default, stalls on?, expect)
CASES = (
    ("control-8KB-quiet", None, False, "pass"),
    ("8KB-stalls", None, True, "fail"),
    ("2s-stalls", 2000, True, "pass"),
)


def busy(ms):
    t0 = time.ticks_ms()
    x = 0
    while time.ticks_diff(time.ticks_ms(), t0) < ms:
        x += 1
    return x


def tone(rate, channels, freq=400, level=0.01):
    # One period tiled to a second: 400 Hz is exactly 120 frames at 48 kHz,
    # so the buffer loops without a click and builds in milliseconds.
    period = rate // freq
    amp = int(32767 * level)
    one = bytearray(period * 2 * channels)
    for i in range(period):
        s = int(amp * math.sin(2 * math.pi * i / period)) & 0xFFFF
        for ch in range(channels):
            k = (i * channels + ch) * 2
            one[k] = s & 0xFF
            one[k + 1] = s >> 8
    return memoryview(bytes(one) * freq)


def find(timeout_ms=15000):
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < timeout_ms:
        for dev_id, streams in uac_audio.audio_devices():
            if any(s.direction == uac.OUT for s in streams):
                return dev_id
        time.sleep_ms(250)
    return None


class Feeder:
    def __init__(self, out, pcm):
        self.out = out
        self.pcm = pcm
        self.pos = 0

    def top_up(self):
        room = self.out.space()
        while room > 0:
            take = min(room, len(self.pcm) - self.pos)
            n = self.out.try_write(self.pcm[self.pos:self.pos + take])
            if n <= 0:
                return
            self.pos = (self.pos + n) % len(self.pcm)
            room -= n


def run_case(dev_id, pcm, name, ring_ms, stalls, expect):
    out = uac_audio.output(dev_id, rate=RATE, ring_ms=ring_ms)
    out.open()
    feeder = Feeder(out, pcm)
    cap = out.capacity()
    # Fill, and let the bus take a few packets so the count starts clean.
    feeder.top_up()
    time.sleep_ms(20)
    feeder.top_up()
    gc.collect()
    base = out.stats()

    ticks = [0, 0]      # (fired, ms spent)

    def tick(_t):
        t0 = time.ticks_ms()
        busy(TICK_MS)
        ticks[0] += 1
        ticks[1] += time.ticks_diff(time.ticks_ms(), t0)

    timer = None
    if stalls:
        timer = machine.Timer(0)
        timer.init(period=TICK_EVERY_MS, mode=machine.Timer.PERIODIC, callback=tick)

    t0 = time.ticks_ms()
    next_work = time.ticks_add(t0, WORK_EVERY_MS)
    next_beat = time.ticks_add(t0, 1000)
    longest = 0         # longest gap between two feeder passes, ms
    last = t0
    works = 0
    min_fill = cap
    try:
        while time.ticks_diff(time.ticks_ms(), t0) < SECONDS * 1000:
            now = time.ticks_ms()
            gap = time.ticks_diff(now, last)
            if gap > longest:
                longest = gap
            last = now
            fill = out.queued_size()
            if fill < min_fill:
                min_fill = fill
            feeder.top_up()
            if stalls and time.ticks_diff(now, next_work) >= 0:
                busy(WORK_MS)
                works += 1
                next_work = time.ticks_add(next_work, WORK_EVERY_MS)
            if time.ticks_diff(now, next_beat) >= 0:
                s = out.stats()
                print("  .. %s t=%ds starved=%d" % (
                    name, time.ticks_diff(now, t0) // 1000, s[3] - base[3]))
                next_beat = time.ticks_add(next_beat, 1000)
            time.sleep_ms(2)
    finally:
        if timer:
            timer.deinit()
        end = out.stats()
        out.close()

    starved = end[3] - base[3]
    packets = end[0] - base[0]
    dropped = end[2] - base[2]
    errors = end[4] - base[4]
    if expect == "pass":
        ok = starved == 0 and errors == 0
    else:
        ok = starved >= FAIL_MIN
    print("CASE %s ring=%dB (%s) stalls=%s work=%dx%dms ticks=%dx%dms "
          "longest_gap=%dms min_fill=%dB packets=%d starved=%d dropped=%d errors=%d "
          "expect=%s -> %s" % (
              name, cap, "default" if ring_ms is None else "%dms" % ring_ms,
              "on" if stalls else "off", works, WORK_MS, ticks[0], TICK_MS,
              longest, min_fill, packets, starved, dropped, errors,
              expect, "ok" if ok else "WRONG"))
    return ok


def main():
    host = usbif.auto.host(classes=("uac",)).start()
    try:
        dev_id = find()
        if dev_id is None:
            print("GATE FAIL: no USB audio output found")
            return
        pcm = tone(RATE, 2)
        results = []
        for case in CASES:
            results.append(run_case(dev_id, pcm, *case))
            time.sleep_ms(300)
        print("GATE %s" % ("PASS" if all(results) else "FAIL"))
    finally:
        host.stop()


main()
