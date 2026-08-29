# usbif Spike C, part 2 — the case run 1 did not cover.
#
# Run 1 loaded the VM with pure-Python bytecode, where the scheduler
# preempts between every bytecode and looked flawless. The real hazard
# for USB events is a long-running *C* call, during which no scheduled
# callback can run at all and periodic events are silently coalesced.
# Here we measure event LOSS (expected fires vs delivered) and worst-case
# delivery gap under three realistic blocking loads.
import machine, micropython, time, gc, hashlib

micropython.alloc_emergency_exception_buf(128)

LOG = open("/spike_c2.txt", "w")


def log(s):
    LOG.write(s + "\n")
    LOG.flush()
    print(s)


count = 0
last = 0
worst = 0


def _tick(t):
    global count, last, worst
    now = time.ticks_us()
    if last:
        d = time.ticks_diff(now, last)
        if d > worst:
            worst = d
    last = now
    count += 1


BIG = bytearray(120000)


def load_flash_write(ms_budget):
    # Realistic: logging / MSC block writes. Flash write blocks in C.
    t0 = time.ticks_ms()
    f = open("/_load.bin", "wb")
    while time.ticks_diff(time.ticks_ms(), t0) < ms_budget:
        f.write(BIG)
        f.flush()
    f.close()


def load_hash(ms_budget):
    # Realistic: a C-level scan over a large buffer (checksum, decode).
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < ms_budget:
        h = hashlib.sha256()
        h.update(BIG)
        h.digest()


def load_bytecode(ms_budget):
    # Control: the run-1 load, pure Python.
    t0 = time.ticks_ms()
    x = 0
    while time.ticks_diff(time.ticks_ms(), t0) < ms_budget:
        x += 1


def measure(loadfn, label, freq=1000, ms=3000):
    global count, last, worst
    gc.collect()
    count = 0
    last = 0
    worst = 0
    t = machine.Timer(0)
    t0 = time.ticks_ms()
    t.init(mode=machine.Timer.PERIODIC, freq=freq, callback=_tick)
    loadfn(ms)
    elapsed = time.ticks_diff(time.ticks_ms(), t0)
    t.deinit()
    expected = elapsed * freq // 1000
    lost = expected - count
    pct = (lost * 100 // expected) if expected else 0
    log(
        "%-16s freq=%dHz elapsed=%dms expected=%d delivered=%d LOST=%d (%d%%) worst_gap=%dus"
        % (label, freq, elapsed, expected, count, lost, pct, worst)
    )


log("=== usbif Spike C part 2 — blocking-C loads")
import sys

log("build=%s" % str(sys.implementation._build))
measure(load_bytecode, "bytecode(ctrl)")
measure(load_hash, "sha256-120KB")
measure(load_flash_write, "flash-write")
try:
    import os

    os.remove("/_load.bin")
except Exception:
    pass
log("=== done")
LOG.close()
