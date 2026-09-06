"""Play a tone through a hosted USB speaker -- the S3 half of the sound card.

A commercial USB audio interface, a headset, or a PyDevices board running
``soundcard.py`` all look the same here: ``usbif.uac_audio.output`` returns an
ordinary ``audiodev.PCMOutput``. The application never knows a bus is involved.

    mpftp run -d COM49 examples/usb_speaker.py

**Headline pairing.** P4 runs ``soundcard.py`` (device, C pump into its ES8311).
This script on an S3 hosts that P4 and plays through it. A PC hosting the P4
works too -- standard classes are the protocol.

**Power.** S3 host needs a powered hub or OTG adapter. Two self-powered boards
can back-feed unless the cable omits VBUS; see ``examples/README.md``.

**FIFO bias.** Hosted stereo playback competes with hosted video for DWC FIFO
space (usbif#2). On a Bias-IN build, mono may be what survives.
"""

import math
import time

import _usbif
from usbif import uac
from usbif import uac_audio


def find_speaker(timeout_ms=15000):
    _usbif.host_start(("uac",))
    deadline = time.ticks_add(time.ticks_ms(), timeout_ms)
    while time.ticks_diff(deadline, time.ticks_ms()) > 0:
        for dev_id, streams in uac_audio.audio_devices():
            out = [s for s in streams if s.direction == uac.OUT]
            if out:
                return dev_id, out
        time.sleep_ms(250)
    return None, ()


def tone_frames(rate, channels, bits, freq=440.0, seconds=2.0, volume=0.2):
    """Generate a sine as int16 little-endian frames."""
    n = int(rate * seconds)
    amp = int(32767 * volume)
    # One frame = channels samples.
    raw = bytearray(n * channels * (bits // 8))
    for i in range(n):
        sample = int(amp * math.sin(2 * math.pi * freq * i / rate))
        # int16 LE
        lo = sample & 0xFF
        hi = (sample >> 8) & 0xFF
        base = i * channels * 2
        for ch in range(channels):
            raw[base + ch * 2] = lo
            raw[base + ch * 2 + 1] = hi
    return raw


def main():
    dev_id, outs = find_speaker()
    if dev_id is None:
        print("no USB audio output found")
        print("attach a speaker, headset, or a board running soundcard.py")
        _usbif.host_stop()
        return

    print("audio device", dev_id)
    for s in outs:
        print(" ", uac.describe(s))

    out = uac_audio.output(dev_id)
    out.open()
    fmt = getattr(out, "format", None) or getattr(out, "_format", None)
    if fmt is None:
        # Fall back to the stream the adapter negotiated.
        stream = out.stream
        rate = max(stream.rates) if stream.rates else 48000
        channels, bits = stream.channels, stream.bits
    else:
        rate, channels, bits = fmt.rate, fmt.channels, fmt.bits
    print("playing 440 Hz for 2 s at %d Hz %d ch %d-bit" % (rate, channels, bits))

    pcm = tone_frames(rate, channels, bits)
    view = memoryview(pcm)
    sent = 0
    try:
        while sent < len(pcm):
            n = out.write(view[sent:])
            if n <= 0:
                time.sleep_ms(5)
                continue
            sent += n
        # Let the ring drain so the tail of the tone is audible.
        time.sleep_ms(500)
    finally:
        print("sent", sent, "bytes; stats", out.stats())
        out.close()
        _usbif.host_stop()


if __name__ == "__main__":
    main()
