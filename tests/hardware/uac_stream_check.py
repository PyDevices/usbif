"""Sound card stream check, board half (usbif#37). Run on the board.

Starts the sound card as ``examples/soundcard.py`` does, then prints one
``MEASURE`` line every 10 s:

- ``pkts/s``: isochronous packets TinyUSB handed over. A high-speed host sends
  8000 a second on a firmware built with TinyUSB 0.19 or later, and 1000 on
  an older one (the endpoint's interval differs, see usbif_desc.c); a
  full-speed host sends 1000.
- ``pump%``: bytes the pump gave I2S, against the wire rate. The gate is
  99.5 % or more.
- ``dma_rate``: the rate the I2S DMA actually consumed, in frames a second:
  the clock the codec really gets.
- ``pitch``, every third window: the pitch of the samples handed to I2S,
  from the median period between rising zero crossings. A splice disturbs
  one cycle, not the median. ``heard`` scales that by the DMA rate against
  the wire rate. Play a 440 Hz sine and it should read 440.

Counts in a pitch window include the board's own work on the tap. Set
``STRESS = 1`` to keep the interpreter busy allocating instead of sleeping,
which is how usbif#39 was measured: 91-95 % on the old TinyUSB, 100 % on
0.21.

Run it with ``mpftp run -d COM4 tests/hardware/uac_stream_check.py``, play a
tone with ``uac_play_tone.py`` from Windows, and read the lines with
``mpftp monitor COM4 --seconds 80``.
"""

import time

import _usbif
import board_peripherals as bp
import usbif.auto

PITCH_EVERY = 3
STRESS = 0


def wait10():
    if not STRESS:
        time.sleep_ms(10000)
        return
    t0 = time.ticks_ms()
    junk = []
    while time.ticks_diff(time.ticks_ms(), t0) < 10000:
        junk.append(bytearray(256))
        if len(junk) > 200:
            junk = []


def median_pitch(buf, rate):
    import array
    a = array.array("h", buf)
    t = []
    prev = a[0]
    for i in range(1, len(a)):
        cur = a[i]
        if prev < 0 <= cur:
            t.append(i - 1 + (-prev) / (cur - prev))
        prev = cur
    p = sorted(t[i + 1] - t[i] for i in range(len(t) - 1))
    if not p:
        return 0.0
    return rate / p[len(p) // 2]


def main():
    dev = usbif.auto.device()
    w = bp.AUDIO_OUT.wire
    f = bp.AUDIO_OUT.default
    bp.audio_power(True, volume=85)
    dev.functions("cdc", "uac")
    kw = dict(rate=f.rate, bits=f.bits, channels=f.channels)
    if w.mck is not None and w.mck >= 0:
        kw["mclk"] = w.mck
        kw["mclk_multiple"] = w.mck_fs
    dev.uac_pump_start(w.sck, w.ws, w.sd, **kw)
    print("MEASURE start stress=%d" % STRESS)
    prev = None
    k = 0
    while True:
        wait10()
        s = dev.uac_pump_stats()
        c = _usbif.uac_pump_clock()
        host, wire, _ = dev.uac_pump_rate()
        cur = (s[1], c[0], c[1], c[2], c[3], c[4], s[3])
        if prev:
            k += 1
            dt = (cur[5] - prev[5]) / 1e6
            pump = (cur[0] - prev[0]) / dt
            pk = (cur[1] - prev[1]) / dt
            dma = (cur[3] - prev[3]) / dt
            pitch = ""
            if k % PITCH_EVERY == 0:
                tap = _usbif.uac_pump_tap()
                fr = median_pitch(tap, wire) if (tap and wire) else 0.0
                pitch = " pitch=%.1f heard=%.1f" % (fr, fr * (dma / 2) / wire)
            if k % PITCH_EVERY == PITCH_EVERY - 1:
                _usbif.uac_pump_tap(True)
            print("MEASURE host=%d wire=%d pkts/s=%.1f pump%%=%.3f dma_rate=%.1f "
                  "starved=%d timeouts=%d%s"
                  % (host, wire, pk, 100 * pump / (2 * wire) if wire else 0,
                     dma / 2, cur[4] - prev[4], cur[6] - prev[6], pitch))
        prev = cur


main()
