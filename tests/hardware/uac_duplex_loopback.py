"""UAC host duplex check: play to a USB sound card and record from it at once.

Run on a board that hosts a USB audio device whose output is looped back into
its input: a USB-C DAC dongle with a headset splitter, its headphone plug in
its own mic jack. Nothing else should be connected to that output, and the
level stays moderate (``TONE_DBFS``).

    mpftp run --follow -d COM4 tests/hardware/uac_duplex_loopback.py

Both streams stay open for the whole run, through four phases:

1. **tone**: a quiet window, then 1 kHz and 1.5 kHz at ``TONE_DBFS``. For
   each, the returned waveform's period (by autocorrelation) gives its
   fundamental; the FFT peak, the zero-crossing rate and the level are
   reported beside it. The quiet window is the control: it must show no
   period at all.
2. **latency**: ``BURSTS`` short bursts, each written into a stream of
   silence, timed from the write to the first captured sample above the
   noise, in milliseconds. ``ring`` says how much of that was the output
   ring ahead of the burst.
3. **soak**: ``SOAK_S`` seconds with the tone playing and capture read,
   then each direction's counters over that window. The gate is zero
   dropped, starved, errored and empty packets either way, and a packet
   every millisecond.
4. **planted stall**: the same feed with a ``STALL_MS`` stall, which must
   starve the output. It is the proof that phase 3's zero can fail.

A headset dongle's capture path is its own business, and the one this was
written on processes its mic heavily: a 1 kHz sine loops back with a 1 ms
period but its 3rd harmonic on top, 1.5 kHz comes back repeating every 2 ms,
quiet steady tones are suppressed, and it sends no capture at all unless
playback is running. So the tone gate asks for the period and for spectral
peaks on harmonics of the tone, not for a clean sine.

Logs stay in RAM until the end: a flash write stalls USB interrupts and costs
isochronous packets (usbif#49). Prints ``RESULT`` lines and a final
``GATE PASS`` or ``GATE FAIL``.
"""

import array
import gc
import math
import time

import usbif.auto
from usbif import uac_audio
from ulab import numpy as np
from ulab import utils

RATE = 48000
FRAME = 4                   # stereo 16-bit
TONES = (1000, 1500)        # each a whole number of samples per period
TONE_DBFS = -20
BURST_DBFS = -12
BURSTS = 10
BURST_GAP_MS = 400
SOAK_S = 30
STALL_MS = 300
OUT_RING_MS = 200
IN_RING_MS = 2000
DEEP = 150 * 192            # bytes kept queued when only continuity matters
SHALLOW = 40 * 192          # ... and when the ring's delay is being measured

out_log = []
_beat = [0]


def say(*args):
    out_log.append(" ".join(str(a) for a in args))


def beat():
    """A dot every 2 s: ``mpftp run --follow`` gives up on a quiet board.

    The UART carries it, never flash, so it costs the USB streams nothing.
    """
    now = time.ticks_ms()
    if time.ticks_diff(now, _beat[0]) >= 2000:
        _beat[0] = now
        print(".", end="")


def tone(hz, dbfs, frames):
    """``frames`` of a stereo sine at ``hz``, as bytes."""
    amp = int(32767 * 10 ** (dbfs / 20))
    a = array.array("h", bytes(frames * FRAME))
    for i in range(frames):
        v = int(amp * math.sin(2 * math.pi * hz * i / RATE))
        a[2 * i] = v
        a[2 * i + 1] = v
    return bytes(a)


class Rig:
    def __init__(self, dev):
        self.out = uac_audio.output(dev, rate=RATE, channels=2, bits=16, ring_ms=OUT_RING_MS)
        self.mic = uac_audio.input(dev, rate=RATE, channels=2, bits=16, ring_ms=IN_RING_MS)
        self.chunk = bytearray(8192)
        self.read_frames = 0
        self.src = None
        self.pos = 0

    def play(self, src):
        self.src = memoryview(src)
        self.pos = 0

    def feed(self, keep):
        """Keep ``keep`` bytes queued from the current source, cyclically."""
        beat()
        room = keep - self.out.queued_size()
        while room >= FRAME:
            take = min(room, len(self.src) - self.pos)
            take -= take % FRAME
            n = self.out.try_write(self.src[self.pos:self.pos + take])
            if n <= 0:
                break
            self.pos = (self.pos + n) % len(self.src)
            room -= n

    def capture(self, into, got):
        """Read into ``into`` from ``got``; past its end, read and discard."""
        if got < len(into):
            n = self.mic.readinto(memoryview(into)[got:])
            self.read_frames += n // FRAME
            return got + n
        n = self.mic.readinto(self.chunk)
        while n:
            self.read_frames += n // FRAME
            n = self.mic.readinto(self.chunk)
        return got

    def run(self, ms, keep, into=None, got=0):
        t0 = time.ticks_ms()
        scratch = into if into is not None else bytearray(0)
        while time.ticks_diff(time.ticks_ms(), t0) < ms:
            self.feed(keep)
            got = self.capture(scratch, got)
            time.sleep_ms(1)
        return got


def left(buf, nbytes):
    full = np.frombuffer(buf, dtype=np.int16, count=nbytes // 2)
    x = np.array(full[0::2], dtype=np.float)
    return x - np.mean(x)


def level(x):
    rms = math.sqrt(np.sum(x * x) / len(x))
    return (20 * math.log10(rms / 32768) if rms else -200.0), float(np.max(abs(x)))


def period(x, lo=12, hi=200):
    """(fundamental Hz, correlation) from the waveform's period.

    The smallest lag whose normalised autocorrelation comes within 0.02 of the
    best one, refined by a parabola through its neighbours. A harmonic-rich
    waveform still repeats at its fundamental's period; noise repeats at none.
    """
    m = len(x) - hi
    base = x[:m]
    e0 = float(np.sum(base * base)) or 1.0
    r = [float(np.sum(base * x[lag:lag + m])) / e0 for lag in range(lo, hi)]
    best = max(r)
    k = next(i for i, v in enumerate(r) if v >= best - 0.02)
    while k + 1 < len(r) and r[k + 1] > r[k]:
        k += 1
    d = 0.0
    if 0 < k < len(r) - 1:
        a, b, c = r[k - 1], r[k], r[k + 1]
        if a - 2 * b + c:
            d = (a - c) / (2 * (a - 2 * b + c))
    return RATE / (lo + k + d), r[k]


def zero_cross_hz(x):
    n = 0
    prev = x[0]
    for i in range(1, len(x)):
        v = x[i]
        if prev < 0 <= v:
            n += 1
        prev = v
    return n * RATE / len(x)


def fft_peak(x):
    size = 16384
    seg = x[:size]
    win = np.array([0.5 - 0.5 * math.cos(2 * math.pi * i / size) for i in range(size)])
    mag = utils.spectrogram(seg * win)
    k = int(np.argmax(mag[1:size // 2])) + 1
    return k * RATE / size


def analyse(name, buf, nbytes):
    x = left(buf, nbytes)
    db, pk = level(x)
    print(".", end="")
    f0, corr = period(x[:9600])
    print(".", end="")
    zc = zero_cross_hz(x[:9600])
    fp = fft_peak(x)
    print(".", end="")
    del x
    gc.collect()
    return db, pk, f0, corr, zc, fp


def phase_tone(rig):
    cap = bytearray(RATE * FRAME // 2)      # 500 ms
    rig.play(bytes(48 * FRAME * 10))
    rig.run(1000, DEEP)                     # past the open's spike
    got = rig.run(600, DEEP, cap)
    db, pk, f0, corr, zc, fp = analyse("quiet", cap, got)
    say("RESULT quiet: level %.1f dBFS, peak %d; period correlation %.2f" % (db, pk, corr))
    noise_peak = pk
    control_found = corr > 0.9
    ok = not control_found
    for hz in TONES:
        rig.play(tone(hz, TONE_DBFS, RATE // 10))      # 100 ms, whole periods
        rig.run(500, DEEP)
        o0 = rig.out.stats()[3]
        got = rig.run(600, DEEP, cap)
        starved = rig.out.stats()[3] - o0
        db, pk, f0, corr, zc, fp = analyse(str(hz), cap, got)
        harmonic = fp / hz
        say("RESULT tone %d Hz at %d dBFS: back at %.1f Hz fundamental (period correlation %.2f),"
            " level %.1f dBFS, peak %d; FFT peak %.1f Hz = %.2f x sent; zero crossings %.0f Hz;"
            " starved %d" % (hz, TONE_DBFS, f0, corr, db, pk, fp, harmonic, zc, starved))
        # The period found is the tone's own or a whole multiple of it (the
        # dongle's processing can add a slower repeat), and the spectrum's
        # peak sits on a harmonic of what was sent: a stream at the wrong
        # rate moves both.
        ratio = hz / f0
        ok = ok and corr > 0.9 and abs(ratio - round(ratio)) < 0.005 * ratio and starved == 0 \
            and abs(harmonic - round(harmonic)) < 0.01 and db > -40
    say("RESULT tone gate", "PASS" if ok else "FAIL", "| control (silence) shows a period:",
        control_found)
    return ok, noise_peak


def phase_latency(rig, noise_peak):
    """Each burst: write it, then find its onset in what comes back."""
    silence = bytes(48 * FRAME * 10)
    burst = tone(1000, BURST_DBFS, 96)      # 2 ms
    threshold = int(max(1000, 20 * noise_peak))
    window = bytearray(RATE * FRAME // 5)   # 200 ms after each burst
    lat, ring = [], []
    rig.play(silence)
    s0 = rig.out.stats()[3]
    for _ in range(BURSTS):
        rig.run(BURST_GAP_MS, SHALLOW)
        rig.capture(bytearray(0), 0)        # empty the capture ring
        # The capture position at the moment of writing: frames read so far
        # plus those captured and waiting.
        mark = rig.read_frames + rig.mic.available() // FRAME
        q = rig.out.queued_size()
        n = rig.out.try_write(burst)
        first = rig.read_frames
        got = rig.run(200, SHALLOW, window)
        if n != len(burst):
            say("RESULT latency: burst write short", n)
            continue
        a = array.array("h", window[:got])
        for i in range(0, len(a), 2):
            v = a[i]
            if v > threshold or v < -threshold:
                lat.append((first + i // 2 - mark) * 1000 / RATE)
                ring.append(q * 1000 / (FRAME * RATE))
                break
    starved = rig.out.stats()[3] - s0
    if not lat:
        say("RESULT latency: no burst came back over", threshold)
        return False
    beyond = sorted(a - b for a, b in zip(lat, ring))
    lat.sort()
    ring.sort()
    say("RESULT latency, write to read-back: %d of %d bursts, min %.1f, median %.1f, max %.1f ms;"
        " output ring ahead of the burst %.1f ms median; threshold %d; starved %d"
        % (len(lat), BURSTS, lat[0], lat[len(lat) // 2], lat[-1], ring[len(ring) // 2],
           threshold, starved))
    # Beyond the ring: the driver's scheduled OUT transfers (3 x 8 ms), the
    # device's DAC, analogue loop, ADC and processing, and up to one 8 ms
    # capture transfer before the bytes are readable.
    say("RESULT latency beyond the output ring: min %.1f, median %.1f, max %.1f ms"
        % (beyond[0], beyond[len(beyond) // 2], beyond[-1]))
    return len(lat) == BURSTS and starved == 0


def phase_soak(rig, seconds, stall_ms=0):
    rig.play(tone(1000, TONE_DBFS, RATE // 10))
    rig.run(300, DEEP)                      # prime
    o0, i0 = rig.out.stats(), rig.mic.stats()
    t0 = time.ticks_ms()
    stalled = False
    while time.ticks_diff(time.ticks_ms(), t0) < seconds * 1000:
        rig.feed(DEEP)
        rig.capture(bytearray(0), 0)
        if stall_ms and not stalled and time.ticks_diff(time.ticks_ms(), t0) > 1000:
            time.sleep_ms(stall_ms)          # the planted fault: the feeder stops
            stalled = True
        time.sleep_ms(5)
    o1, i1 = rig.out.stats(), rig.mic.stats()
    d_out = [b - a for a, b in zip(o0, o1)]
    d_in = [b - a for a, b in zip(i0, i1)]
    names = ("packets", "bytes", "dropped", "starved", "errors", "empty")
    tag = " (planted %d ms stall)" % stall_ms if stall_ms else ""
    say("RESULT soak%s %d s out:" % (tag, seconds),
        ", ".join("%s %d" % (k, v) for k, v in zip(names, d_out)))
    say("RESULT soak%s %d s in: " % (tag, seconds),
        ", ".join("%s %d" % (k, v) for k, v in zip(names, d_in)))
    clean = (d_out[2] == 0 and d_out[3] == 0 and d_out[4] == 0
             and d_in[2] == 0 and d_in[4] == 0 and d_in[5] == 0
             and d_out[0] >= seconds * 990 and d_in[0] >= seconds * 990)
    return clean, d_out


def main():
    host = usbif.auto.host(classes=("uac",))
    host.start()
    devs = ()
    for _ in range(60):
        devs = uac_audio.audio_devices()
        if devs:
            break
        time.sleep_ms(250)
    if not devs:
        print("no USB audio device")
        return
    rig = Rig(devs[0][0])
    say("streams: %d B out on ep 0x%02x, %d B in on ep 0x%02x" % (
        rig.out.stream.max_packet, rig.out.stream.endpoint,
        rig.mic.stream.max_packet, rig.mic.stream.endpoint))
    gates = {}
    rig.out.open()
    rig.mic.open()
    try:
        gates["tone"], noise_peak = phase_tone(rig)
        gates["latency"] = phase_latency(rig, noise_peak)
        gates["soak"], _ = phase_soak(rig, SOAK_S)
        stall_clean, d_out = phase_soak(rig, 5, STALL_MS)
        gates["planted stall is caught"] = (not stall_clean) and d_out[3] > 0
        say("RESULT totals out", rig.out.stats(), "in", rig.mic.stats())
    finally:
        rig.out.close()
        rig.mic.close()
        host.stop()
    print()
    for line in out_log:
        print(line)
    for k, v in gates.items():
        print("GATE", k, "PASS" if v else "FAIL")
    print("GATE", "PASS" if all(gates.values()) else "FAIL")


main()
