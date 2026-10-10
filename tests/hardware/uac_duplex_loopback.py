"""UAC host duplex check: play to a USB sound card and record from it at once.

Run on a board that hosts a USB audio device whose output is looped back into
its input: a USB-C DAC dongle with a headset splitter, its headphone plug in
its own mic jack. Nothing else should be connected to that output, and the
level stays moderate (``TONE_DBFS``).

    mpftp run --follow -d COM4 tests/hardware/uac_duplex_loopback.py

Both streams stay open for the whole run, through four phases:

1. **tone**: a quiet window (the control, which must *not* find a 1 kHz
   tone), then 1 kHz at ``TONE_DBFS``. The captured tone's frequency is read
   two ways, by FFT (ulab) and by counting rising zero crossings, with its
   level in dBFS.
2. **latency**: ``BURSTS`` short bursts, each written into a stream of
   silence. The round trip is from the moment a burst is written to the
   moment it is found in what was captured, in milliseconds; ``queued``
   says how much of that was the output ring ahead of it.
3. **soak**: ``SOAK_S`` seconds with the tone playing and capture read,
   then each direction's counters over that window. The gate is zero
   dropped, starved and errored packets either way.
4. **planted stall**: the same feed with a ``STALL_MS`` stall, which must
   starve the output. It is the proof that phase 3's zero can fail.

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

try:
    from ulab import numpy as np
except ImportError:
    np = None

RATE = 48000
FRAME = 4                   # stereo 16-bit
TONE_HZ = 1000
TONE_DBFS = -20
BURST_DBFS = -12
BURSTS = 10
BURST_GAP_MS = 400
SOAK_S = 30
STALL_MS = 300
OUT_RING_MS = 200
IN_RING_MS = 2000

out_log = []


def say(*args):
    out_log.append(" ".join(str(a) for a in args))


def cycle(dbfs):
    """One cycle of TONE_HZ at RATE, stereo, as bytes (48 frames at 48 kHz)."""
    amp = int(32767 * 10 ** (dbfs / 20))
    per = RATE // TONE_HZ
    buf = bytearray(per * FRAME)
    a = array.array("h", buf)       # a copy; written back below
    for i in range(per):
        v = int(amp * math.sin(2 * math.pi * i / per))
        a[2 * i] = v
        a[2 * i + 1] = v
    return bytes(a)


class Rig:
    def __init__(self, dev):
        self.out = uac_audio.output(dev, rate=RATE, channels=2, bits=16, ring_ms=OUT_RING_MS)
        self.mic = uac_audio.input(dev, rate=RATE, channels=2, bits=16, ring_ms=IN_RING_MS)
        self.chunk = bytearray(8192)
        self.read_frames = 0

    def open(self):
        self.out.open()
        self.mic.open()

    def close(self):
        self.out.close()
        self.mic.close()

    def feed(self, src, pos, keep_bytes):
        """Write `src` cyclically from `pos`, until `keep_bytes` are queued."""
        room = keep_bytes - self.out.queued_size()
        while room >= FRAME:
            take = min(room, len(src) - pos)
            take -= take % FRAME
            n = self.out.try_write(memoryview(src)[pos:pos + take])
            if n <= 0:
                break
            pos = (pos + n) % len(src)
            room -= n
        return pos

    def drain(self, sink=None):
        """Read everything captured; append it to `sink` (a list) if given."""
        while True:
            n = self.mic.readinto(self.chunk)
            if not n:
                return
            self.read_frames += n // FRAME
            if sink is not None:
                sink.append(bytes(self.chunk[:n]))


def left(blobs):
    data = b"".join(blobs)
    a = array.array("h", data)
    n = len(a) // 2
    if np is not None:
        full = np.frombuffer(data, dtype=np.int16)
        return np.array(full[0::2], dtype=np.float), n
    return array.array("h", (a[2 * i] for i in range(n))), n


def level_dbfs(x, n):
    if np is not None:
        rms = math.sqrt(np.sum(x * x) / n)
        peak = np.max(abs(x))
    else:
        rms = math.sqrt(sum(v * v for v in x) / n)
        peak = max(abs(v) for v in x)
    db = 20 * math.log10(rms / 32768) if rms else -200
    return db, peak


def zero_cross_hz(x, n):
    rising = []
    prev = x[0]
    for i in range(1, n):
        v = x[i]
        if prev < 0 <= v:
            rising.append(i)
        prev = v
    if len(rising) < 3:
        return 0.0, len(rising)
    return (len(rising) - 1) * RATE / (rising[-1] - rising[0]), len(rising)


def fft_peak(x, n):
    """(Hz, peak bin magnitude over median) from an N=2**k window, or None."""
    if np is None:
        return None
    size = 1
    while size * 2 <= n:
        size *= 2
    seg = x[:size]
    win = np.array([0.5 - 0.5 * math.cos(2 * math.pi * i / size) for i in range(size)])
    seg = seg * win
    try:
        from ulab import utils
        mag = utils.spectrogram(seg)
    except (ImportError, AttributeError):
        spec = np.fft.fft(seg)
        if isinstance(spec, tuple):
            re, im = spec
            mag = np.sqrt(re * re + im * im)
        else:
            mag = abs(spec)
    half = mag[1:size // 2]
    k = int(np.argmax(half)) + 1
    # Parabolic interpolation between the bins either side.
    a, b, c = float(mag[k - 1]), float(mag[k]), float(mag[k + 1])
    d = (a - c) / (2 * (a - 2 * b + c)) if (a - 2 * b + c) else 0.0
    med = float(np.median(half)) or 1.0
    return (k + d) * RATE / size, b / med


def phase_tone(rig, silence, tone):
    pos = 0
    rig.drain()
    # Settle past the open's spike, then the control window: silence.
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < 1000:
        pos = rig.feed(silence, pos, 40 * 192)
        rig.drain()
        time.sleep_ms(2)
    quiet = []
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < 500:
        pos = rig.feed(silence, pos, 40 * 192)
        rig.drain(quiet)
        time.sleep_ms(2)
    # The tone: settle 500 ms, then 1 s.
    pos = 0
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < 500:
        pos = rig.feed(tone, pos, 40 * 192)
        rig.drain()
        time.sleep_ms(2)
    loud = []
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < 1000:
        pos = rig.feed(tone, pos, 40 * 192)
        rig.drain(loud)
        time.sleep_ms(2)

    results = {}
    for name, blobs in (("quiet", quiet), ("tone", loud)):
        x, n = left(blobs)
        db, peak = level_dbfs(x, n)
        zc, crossings = zero_cross_hz(x, n)
        fp = fft_peak(x, n)
        results[name] = (db, fp, peak)
        say("RESULT tone", name, "frames", n, "level %.1f dBFS" % db, "peak", int(peak),
            "zero-cross %.1f Hz" % zc,
            "fft %.1f Hz (peak/median %.0f)" % fp if fp else "fft n/a")
        del x
        gc.collect()
    tone_db, tone_fp, _ = results["tone"]
    quiet_db, quiet_fp, quiet_peak = results["quiet"]

    def found(fp):
        return bool(fp) and abs(fp[0] - TONE_HZ) < 5 and fp[1] > 100

    tone_ok = found(tone_fp) and tone_db > quiet_db + 20
    # The control: the same test on the silent window must fail.
    control_found = found(quiet_fp)
    say("RESULT tone gate", "PASS" if tone_ok else "FAIL",
        "| control (silence) finds a tone:", control_found,
        "| loopback gain %.1f dB" % (tone_db - (TONE_DBFS - 3.0)))
    # Sent RMS of a sine is 3 dB under its peak level.
    return tone_ok and not control_found, tone_db - (TONE_DBFS - 3.0), quiet_peak


def phase_latency(rig, silence, burst, gain_db, noise_peak):
    """Each burst: write it, then find its onset in what comes back.

    The onset is the first left sample over a threshold set between the
    quiet window's noise peak and the burst's expected peak, from the gain
    the tone phase measured.
    """
    pos = 0
    keep = 10 * 192         # keep the output ring short: 10 ms ahead
    lat = []
    queued_ms = []
    expected = 32768 * 10 ** ((BURST_DBFS + gain_db) / 20)
    threshold = int(max(3 * noise_peak, expected / 3))
    say("RESULT latency threshold", threshold, "expected burst peak", int(expected),
        "noise peak", int(noise_peak))
    for _ in range(BURSTS):
        # Silence for the gap, read and discarded.
        t0 = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), t0) < BURST_GAP_MS:
            pos = rig.feed(silence, pos, keep)
            rig.drain()
            time.sleep_ms(1)
        # Capture position at the moment of writing: frames read so far plus
        # those already captured and waiting.
        rig.drain()
        mark = rig.read_frames + rig.mic.available() // FRAME
        q = rig.out.queued_size()
        n = rig.out.try_write(burst)
        if n != len(burst):
            say("RESULT latency: burst write short", n)
        queued_ms.append(q / (FRAME * RATE / 1000))
        got = []
        t0 = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), t0) < 200:
            pos = rig.feed(silence, pos, keep)
            rig.drain(got)
            time.sleep_ms(1)
        data = b"".join(got)
        a = array.array("h", data)
        first = rig.read_frames - len(a) // 2      # frame index of a[0]
        onset = None
        for i in range(0, len(a), 2):
            v = a[i]
            if v > threshold or v < -threshold:
                onset = first + i // 2
                break
        if onset is not None:
            lat.append((onset - mark) * 1000 / RATE)
    if not lat:
        say("RESULT latency: no burst came back")
        return False
    lat.sort()
    say("RESULT latency write->read ms: n", len(lat), "min %.1f median %.1f max %.1f"
        % (lat[0], lat[len(lat) // 2], lat[-1]),
        "| output ring ahead of the burst %.1f ms median" % sorted(queued_ms)[len(queued_ms) // 2])
    return len(lat) == BURSTS


def phase_soak(rig, tone, seconds, stall_ms=0):
    pos = 0
    keep = 100 * 192
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < 300:      # prime
        pos = rig.feed(tone, pos, keep)
        rig.drain()
        time.sleep_ms(2)
    o0, i0 = rig.out.stats(), rig.mic.stats()
    t0 = time.ticks_ms()
    stalled = False
    while time.ticks_diff(time.ticks_ms(), t0) < seconds * 1000:
        pos = rig.feed(tone, pos, keep)
        n = rig.mic.readinto(rig.chunk)
        while n:
            rig.read_frames += n // FRAME
            n = rig.mic.readinto(rig.chunk)
        if stall_ms and not stalled and time.ticks_diff(time.ticks_ms(), t0) > 1000:
            time.sleep_ms(stall_ms)          # the planted fault: the feeder stops
            stalled = True
        time.sleep_ms(5)
    o1, i1 = rig.out.stats(), rig.mic.stats()
    d_out = [b - a for a, b in zip(o0, o1)]
    d_in = [b - a for a, b in zip(i0, i1)]
    names = ("packets", "bytes", "dropped", "starved", "errors", "empty")
    say("RESULT soak%s %ds out:" % (" (planted %d ms stall)" % stall_ms if stall_ms else "", seconds),
        ", ".join("%s %d" % (k, v) for k, v in zip(names, d_out)))
    say("RESULT soak%s %ds in: " % (" (planted %d ms stall)" % stall_ms if stall_ms else "", seconds),
        ", ".join("%s %d" % (k, v) for k, v in zip(names, d_in)))
    clean = (d_out[2] == 0 and d_out[3] == 0 and d_out[4] == 0
             and d_in[2] == 0 and d_in[4] == 0 and d_in[5] == 0
             and d_out[0] >= seconds * 990 and d_in[0] >= seconds * 990)
    return clean, d_out, d_in


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
    dev = devs[0][0]
    rig = Rig(dev)
    say("streams:", rig.out.stream.max_packet, "B out on ep 0x%02x," % rig.out.stream.endpoint,
        rig.mic.stream.max_packet, "B in on ep 0x%02x" % rig.mic.stream.endpoint)
    silence = bytes(48 * FRAME * 10)
    tone = cycle(TONE_DBFS) * 10
    burst = cycle(BURST_DBFS) * 2        # 2 ms
    gates = {}
    rig.open()
    try:
        gates["tone"], gain_db, noise_peak = phase_tone(rig, silence, tone)
        gates["latency"] = phase_latency(rig, silence, burst, gain_db, noise_peak)
        gates["soak"], _, _ = phase_soak(rig, tone, SOAK_S)
        stall_clean, d_out, _ = phase_soak(rig, tone, 5, STALL_MS)
        gates["planted stall is caught"] = (not stall_clean) and d_out[3] > 0
        say("RESULT totals out", rig.out.stats(), "in", rig.mic.stats())
    finally:
        rig.close()
        host.stop()
    for line in out_log:
        print(line)
    for k, v in gates.items():
        print("GATE", k, "PASS" if v else "FAIL")
    print("GATE", "PASS" if all(gates.values()) else "FAIL")


main()
