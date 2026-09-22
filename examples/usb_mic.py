"""Capture a few seconds from a hosted USB microphone.

The host-side mirror of what a device-side mic would be -- except the device
side cannot present a microphone yet (speaker-only UAC; usbif#7). This script
drives a commercial USB mic (or any UAC capture device) through
``usbif.uac_audio.input``, the same ``PCMInput`` surface as an on-board ADC.

    mpftp run -d COM49 examples/usb_mic.py

Prints peak and RMS over the captured buffer so you can see that real audio
arrived, not silence. Verified originally against a C-Media USB mic and
Brad's voice (3,014 packets, zero dropped).
"""

import time

import usbif.auto
from usbif import uac
from usbif import uac_audio

SECONDS = 3


def find_mic(host, timeout_ms=15000):
    deadline = time.ticks_add(time.ticks_ms(), timeout_ms)
    while time.ticks_diff(deadline, time.ticks_ms()) > 0:
        for dev_id, streams in uac_audio.audio_devices():
            ins = [s for s in streams if s.direction == uac.IN]
            if ins:
                return dev_id, ins
        time.sleep_ms(250)
    return None, ()


def _peak_rms(buf, sample_bytes=2):
    """Peak absolute sample and RMS over int16 LE mono/stereo interleaved."""
    n = len(buf) // sample_bytes
    if n == 0:
        return 0, 0.0
    peak = 0
    acc = 0
    for i in range(0, len(buf), sample_bytes):
        sample = buf[i] | (buf[i + 1] << 8)
        if sample >= 0x8000:
            sample -= 0x10000
        a = sample if sample >= 0 else -sample
        if a > peak:
            peak = a
        acc += sample * sample
    rms = (acc / n) ** 0.5
    return peak, rms


def main():
    host = usbif.auto.host(classes=("uac",)).start()
    dev_id, ins = find_mic(host)
    if dev_id is None:
        print("no USB audio input found")
        print("attach a USB microphone")
        print("note: a PyDevices board cannot present as a mic yet (usbif#7)")
        host.stop()
        return

    print("audio device", dev_id)
    for s in ins:
        print(" ", uac.describe(s))

    mic = uac_audio.input(dev_id)
    mic.open()
    fmt = getattr(mic, "format", None) or getattr(mic, "_format", None)
    if fmt is None:
        stream = mic.stream
        rate = max(stream.rates) if stream.rates else 48000
        channels, bits = stream.channels, stream.bits
    else:
        rate, channels, bits = fmt.rate, fmt.channels, fmt.bits
    print("capturing %d s at %d Hz %d ch %d-bit" % (SECONDS, rate, channels, bits))

    frame = channels * (bits // 8)
    want = int(rate * SECONDS) * frame
    buf = bytearray(want)
    view = memoryview(buf)
    got = 0
    t0 = time.ticks_ms()
    try:
        while got < want and time.ticks_diff(time.ticks_ms(), t0) < (SECONDS + 2) * 1000:
            n = mic.readinto(view[got:])
            if n <= 0:
                time.sleep_ms(5)
                continue
            got += n
    finally:
        peak, rms = _peak_rms(buf[:got])
        print("got %d / %d bytes; peak=%d rms=%.1f; stats %r"
              % (got, want, peak, rms, mic.stats()))
        mic.close()
        host.stop()


if __name__ == "__main__":
    main()
