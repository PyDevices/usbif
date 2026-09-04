# usbif Spike C — event transport measurement (ESP32-S3)
# A: mp_sched_schedule delivery (what USB callbacks use today)
# B: poll-loop granularity (bounds ring-buffer delivery latency)
# C: scheduler burst capacity (what a burst of USB events costs)
import machine, micropython, time, array, gc

micropython.alloc_emergency_exception_buf(128)
try:
    import uasyncio as asyncio
except ImportError:
    import asyncio

LOG = open("/spike_c.txt", "w")


def log(s):
    LOG.write(s + "\n")
    LOG.flush()
    print(s)


def stats(name, deltas, ideal):
    n = len(deltas)
    if not n:
        log("%s: no samples" % name)
        return
    s = sorted(deltas)
    tot = 0
    for v in s:
        tot += v
    p = lambda q: s[min(n - 1, int(n * q))]
    late = 0
    for v in s:
        if v > ideal * 3 // 2:
            late += 1
    log(
        "%s n=%d min=%d med=%d p95=%d p99=%d max=%d mean=%d late(>1.5x)=%d (%d%%)"
        % (name, n, s[0], p(0.5), p(0.95), p(0.99), s[-1], tot // n, late, late * 100 // n)
    )


# ---------------------------------------------------------------- load
def busy_ms(ms):
    # Stand-in for an LVGL render frame: pure-Python CPU work.
    t0 = time.ticks_ms()
    x = 0
    while time.ticks_diff(time.ticks_ms(), t0) < ms:
        x += 1
    return x


# ------------------------------------------------- A: scheduler path
N = 1200
stamps = array.array("i", [0] * N)
idx = 0


def _sched_cb(_):
    global idx
    if idx < N:
        stamps[idx] = time.ticks_us()
        idx += 1


def _tick(t):
    # ESP32 machine.Timer already delivers via mp_sched; this callback
    # therefore measures exactly the path a USB C-callback would take.
    _sched_cb(0)


def run_a(freq, load_ms, label):
    global idx
    idx = 0
    gc.collect()
    t = machine.Timer(0)
    t.init(mode=machine.Timer.PERIODIC, freq=freq, callback=_tick)
    t0 = time.ticks_ms()
    # Busy-load the VM while events arrive, in frame-sized chunks.
    while idx < N and time.ticks_diff(time.ticks_ms(), t0) < 5000:
        busy_ms(load_ms)
    t.deinit()
    d = []
    for i in range(1, idx):
        d.append(time.ticks_diff(stamps[i], stamps[i - 1]))
    ideal = 1000000 // freq
    log("--- A %s: %d Hz events, %d ms load chunks, ideal gap %d us" % (label, freq, load_ms, ideal))
    stats("   sched-delivery gap(us)", d, ideal)
    return d


# ------------------------------------------- B: poll-loop granularity
async def _poller(out, n, stop):
    last = time.ticks_us()
    while len(out) < n:
        await asyncio.sleep_ms(0)
        now = time.ticks_us()
        out.append(time.ticks_diff(now, last))
        last = now
    stop[0] = True


async def _loader(load_ms, stop):
    while not stop[0]:
        busy_ms(load_ms)
        await asyncio.sleep_ms(0)


def run_b(load_ms, label):
    gc.collect()
    out = []
    stop = [False]

    async def main():
        await asyncio.gather(_poller(out, 300, stop), _loader(load_ms, stop))

    try:
        asyncio.run(main())
    except AttributeError:  # no gather on this build
        async def main2():
            asyncio.create_task(_loader(load_ms, stop))
            await _poller(out, 300, stop)
            await asyncio.sleep_ms(5)
        asyncio.run(main2())
    log("--- B %s: poll-loop granularity under %d ms load chunks" % (label, load_ms))
    stats("   drain-interval(us)", out, 1000)
    return out


# ---------------------------------------- C: scheduler burst capacity
def run_c():
    log("--- C scheduler burst capacity (depth is compile-time)")
    for burst in (4, 8, 16, 32):
        got = 0
        fail = 0
        for _ in range(burst):
            try:
                if micropython.schedule(_sched_cb, 0) is False:
                    fail += 1
                else:
                    got += 1
            except RuntimeError:
                fail += 1
        time.sleep_ms(20)  # let the queue drain
        log("   burst=%d accepted=%d rejected=%d" % (burst, got, fail))


log("=== usbif Spike C  %s" % str(time.localtime()))
import sys

log("build=%s" % str(sys.implementation._build))
run_c()
run_a(1000, 2, "1kHz/light")
run_a(1000, 12, "1kHz/heavy-frame")
run_a(250, 12, "250Hz/heavy-frame")
run_b(2, "light")
run_b(12, "heavy-frame")
log("=== done")
LOG.close()
