"""Sound card stream check, host half (usbif#37). Run with Windows Python.

Plays a 440 Hz sine to "Speakers (Espressif Device)" through WASAPI, then
prints the frame rate the host actually delivered and any underflows:

    python.exe uac_play_tone.py RATE SECONDS [excl|shared]

Exclusive mode sets the card to RATE (44100 or 48000). Shared mode uses the
rate Windows has configured for the device, which is how a browser plays.
Needs numpy and sounddevice.
"""

import sys
import time

import numpy as np
import sounddevice as sd

rate = int(sys.argv[1])
secs = float(sys.argv[2])
excl = (sys.argv[3] if len(sys.argv) > 3 else "excl") == "excl"
FREQ = 440.0

dev = [i for i, d in enumerate(sd.query_devices())
       if "Espressif" in d["name"] and d["max_output_channels"]
       and sd.query_hostapis(d["hostapi"])["name"] == "Windows WASAPI"][0]
state = {"phase": 0, "under": 0}


def cb(out, frames, t, status):
    if status.output_underflow:
        state["under"] += 1
    n = np.arange(frames) + state["phase"]
    s = (0.5 * np.sin(2 * np.pi * FREQ * n / rate) * 32767).astype(np.int16)
    out[:, 0] = s
    out[:, 1] = s
    state["phase"] += frames


with sd.OutputStream(device=dev, samplerate=rate, channels=2, dtype="int16",
                     callback=cb, extra_settings=sd.WasapiSettings(exclusive=excl),
                     latency="high"):
    t0 = time.perf_counter()
    time.sleep(secs)
    el = time.perf_counter() - t0
print("HOST rate=%d exclusive=%s frames/s=%.1f underflows=%d"
      % (rate, excl, state["phase"] / el, state["under"]))
