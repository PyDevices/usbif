"""Soft reset check: usbif shuts its hardware down on a soft reset.

Run on the PC, with CPython and ``mpftp`` on the PATH (or named by ``MPFTP``),
against a board running usbif with a USB microphone on its host port: the
ESP32-P4 DEV-KIT with a mic in its USB-A hub, say.

    python tests/hardware/soft_reset_check.py -d COM55 --cycles 5

Each cycle runs a short program on the board through the raw REPL, the way an
app would, and leaves everything running for the soft reset to deal with:

- the USB host, started at full speed for the UAC class;
- the microphone's input stream, open and delivering;
- a 4 KB RAM drive attached as the board's USB mass storage;
- with ``--pump``, the sound card's I2S pump, on the board's ``AUDIO_OUT``
  wire (``board_peripherals``). Nothing brings the codec up, so it's silent.

Then it soft-resets the board and goes again. From the second cycle on, each
check expects a fresh start:

- ``host stopped``: the host isn't running before the program starts it;
- ``attach event``: starting the host queues an attach for the mic, so the
  host really started again rather than finding the old one;
- ``mic opens``: the mic opens and delivers audio within a second;
- ``drive detached``: the drive the last program attached is gone;
- ``pump stopped`` (``--pump``): the pump isn't running, and starts again.

Prints ``PASS``/``FAIL`` per check and a final ``GATE PASS``/``GATE FAIL``.

The checker can fail, and did. On firmware from before the fix, the second
cycle found the host still running (``host stopped`` FAIL), no attach event,
the mic's stream still open from the first cycle (``host_uac_open failed
(-1)``), and the drive still attached to a buffer the soft reset had freed.
Calling ``host_stop()`` from Python at that point didn't help: the stream's
transfers never retired and the host came back wedged until a hard reset.
"""

import argparse
import json
import os
import shlex
import subprocess
import sys

BOARD_STEP = r"""
import time, _usbif
def say(*a):
    print("@@", *a)
say("host_running_before", _usbif.host_stats()[0])
say("drive_before", _usbif.msc_status()[0] if _usbif.msc_status() else False)
say("pump_before", _usbif.uac_pump_stats()[0] if _usbif.uac_pump_stats() else False)
_usbif.host_start(("uac",), full_speed=True)
t0 = time.ticks_ms()
attached = False
while time.ticks_diff(time.ticks_ms(), t0) < 10000 and not attached:
    events, _ = _usbif.host_drain(16)
    attached = any(kind == 0 and "uac" in row[5] for kind, row in events)
    time.sleep_ms(50)
say("attach_event", attached)
from usbif import uac, uac_audio
dev = None
for dev_id, streams in uac_audio.audio_devices():
    if any(s.direction == uac.IN for s in streams):
        dev = dev_id
got = 0
try:
    mic = uac_audio.input(dev, ring_ms=500)
    mic.open()
    buf = bytearray(960)
    t1 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t1) < 1000:
        got += mic.readinto(buf) or 0
        time.sleep_ms(10)
except Exception as e:
    say("mic_error", repr(e))
say("mic_bytes", got)
drive = bytearray(4096)
_usbif.msc_attach(drive)
say("drive_attached", _usbif.msc_status()[0])
"""

PUMP_STEP = r"""
import _usbif
import board_peripherals as bp
w = bp.AUDIO_OUT.wire
f = bp.AUDIO_OUT.default
kw = {"rate": 48000, "bits": f.bits, "channels": f.channels}
if w.mck is not None and w.mck >= 0:
    kw["mclk"] = w.mck
    kw["mclk_multiple"] = w.mck_fs
try:
    _usbif.uac_pump_start(w.sck, w.ws, w.sd, **kw)
    print("@@ pump_started", True)
except Exception as e:
    print("@@ pump_started", repr(e))
"""


def mpftp(args, device):
    cmd = shlex.split(os.environ.get("MPFTP", "mpftp"))
    result = subprocess.run(cmd + [args[0], "-d", device] + args[1:],
                            capture_output=True, text=True, timeout=120)
    try:
        return json.loads(result.stdout)
    except ValueError:
        return {"ok": False, "error": (result.stdout + result.stderr).strip()}


def run_step(device, code):
    reply = mpftp(["exec", code], device)
    text = reply.get("output") or reply.get("error") or ""
    values = {}
    for line in text.splitlines():
        if line.startswith("@@ "):
            key, _, value = line[3:].partition(" ")
            values[key] = value.strip()
    return values, text


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("-d", "--device", required=True, help="the board's REPL, e.g. COM55")
    ap.add_argument("--cycles", type=int, default=5, help="soft resets to check (default 5)")
    ap.add_argument("--pump", action="store_true", help="also start the I2S pump each cycle")
    args = ap.parse_args()

    failed = 0

    def check(name, ok, detail):
        nonlocal failed
        print("%s  %-15s %s" % ("PASS" if ok else "FAIL", name, detail))
        failed += not ok

    for cycle in range(args.cycles + 1):
        if cycle:
            mpftp(["soft-reset"], args.device)
        values, text = run_step(args.device, BOARD_STEP)
        if args.pump:
            pump_values, pump_text = run_step(args.device, PUMP_STEP)
            values.update(pump_values)
            text += pump_text
        print("-- cycle %d%s" % (cycle, "" if cycle else " (the first program)"))
        if not values:
            check("board answered", False, text.strip()[-300:])
            continue
        fresh = cycle > 0
        if fresh:
            check("host stopped", values.get("host_running_before") == "False",
                  "running=" + str(values.get("host_running_before")))
            check("drive detached", values.get("drive_before") == "False",
                  "attached=" + str(values.get("drive_before")))
            if args.pump:
                check("pump stopped", values.get("pump_before") == "False",
                      "running=" + str(values.get("pump_before")))
        check("attach event", values.get("attach_event") == "True",
              "seen=" + str(values.get("attach_event")))
        mic_bytes = int(values.get("mic_bytes", "0") or 0)
        check("mic opens", mic_bytes > 0,
              "%d bytes in 1 s%s" % (mic_bytes, "; " + values["mic_error"] if "mic_error" in values else ""))
        if args.pump:
            check("pump starts", values.get("pump_started") == "True",
                  str(values.get("pump_started")))

    print("GATE %s" % ("FAIL" if failed else "PASS"))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
