"""The board is a class-compliant USB sound card -- the shipping path.

A PC (or another PyDevices board hosting UAC) sees Speakers (Espressif Device)
and plays through it. Audio moves from the isochronous endpoint to the board's
I2S codec in a C FreeRTOS task: Python configures and observes, C moves the
bytes. That is the vision's rule, and this is the example that follows it.

    mpftp run -d COM49 examples/soundcard.py

**Pairing.** On a P4 this is the device half of the sound-card offload: an S3
without its own codec runs ``usb_speaker.py`` as host and plays through this
board. A laptop works identically -- standard classes are the protocol.

**Not this file.** ``uac_pump.py`` is the Python FIFO pump -- useful for
watching the buffer and for latency tools that need ``uac_read``. Do not use
it as the sound card; use this.

**Console.** Costume is CDC+UAC so the REPL stays on the same cable. UART is
still the safer place for the REPL when iterating (a costume change that drops
CDC cuts a native-USB session mid-run).

**Pins and the amp.** I2S pin numbers and codec bring-up come from
``board_peripherals``: the pump owns the I2S channel, not the ES8311. Without
enabling the speaker amp you get perfect byte counters and silence -- the
failure mode that ate an afternoon in Phase 4.
"""

import time

import board_peripherals as bp
import _usbif

# Host advertises 48 kHz stereo; the board codec is typically 24 kHz mono.
# The C pump decimates. Match the board's own rate so pitch is right.
DEFAULT_VOLUME = 85  # digital gain on this hardware; see phase0 findings


def _i2s_pins():
    """(bclk, ws, dout, mclk) from board_peripherals, or raise usefully."""
    # Boards publish these under a few historical names; try them in order.
    for names in (
        ("I2S_BCLK", "I2S_WS", "I2S_DOUT", "I2S_MCLK"),
        ("bclk", "ws", "dout", "mclk"),
        ("BCLK", "WS", "DOUT", "MCLK"),
    ):
        vals = []
        ok = True
        for n in names:
            if not hasattr(bp, n):
                ok = False
                break
            vals.append(getattr(bp, n))
        if ok:
            return tuple(vals)
    pins = getattr(bp, "I2S_OUT_PINS", None)
    if pins is not None and len(pins) >= 3:
        bclk, ws, dout = pins[0], pins[1], pins[2]
        mclk = pins[3] if len(pins) > 3 else -1
        return bclk, ws, dout, mclk
    raise RuntimeError(
        "board_peripherals does not publish I2S output pins "
        "(looked for I2S_BCLK/I2S_WS/I2S_DOUT/I2S_MCLK and I2S_OUT_PINS). "
        "Pass them to _usbif.uac_pump_start yourself, or extend the board "
        "package."
    )


def _bring_up_codec():
    """Power the amp and set a sane volume before the pump starts."""
    # Prefer the board helper that wires power + volume correctly.
    audio_out = getattr(bp, "audio_out", None)
    if callable(audio_out):
        out = audio_out()
        try:
            out.open()
        except Exception:
            pass
        try:
            out.set_volume(DEFAULT_VOLUME)
            out.mute(False)
        except Exception:
            pass
        return out
    # Fall back: poke the codec through the private hooks uac_pump.py uses.
    power = getattr(bp, "_output_power", None)
    if callable(power):
        try:
            power(True)
        except Exception:
            pass
    set_vol = getattr(bp, "_codec_call", None)
    if callable(set_vol):
        try:
            set_vol("set_dac_volume", DEFAULT_VOLUME)
            set_vol("dac_mute", False)
        except Exception:
            pass
    return None


def main():
    bclk, ws, dout, mclk = _i2s_pins()
    fmt = getattr(bp, "_FORMAT", None)
    rate = getattr(fmt, "rate", 24000) if fmt is not None else 24000
    bits = getattr(fmt, "bits", 16) if fmt is not None else 16
    channels = getattr(fmt, "channels", 1) if fmt is not None else 1

    out = _bring_up_codec()

    # Costume first so the host sees the sound card before we start the pump.
    # CDC stays so the REPL survives on the same connector.
    _usbif.dev_functions(_usbif.FN_CDC | _usbif.FN_AUDIO)
    print("costume: cdc+uac -- look for Speakers on the host")

    kwargs = {"rate": rate, "bits": bits, "channels": channels}
    if mclk is not None and mclk >= 0:
        kwargs["mclk"] = mclk
    _usbif.uac_pump_start(bclk, ws, dout, **kwargs)
    print("C pump started: I2S bclk=%d ws=%d dout=%d rate=%d ch=%d"
          % (bclk, ws, dout, rate, channels))
    print("play audio to this board from a PC, or from usb_speaker.py on an S3")

    try:
        while True:
            time.sleep_ms(1000)
            running, moved, idle, timeouts, shed = _usbif.uac_pump_stats()
            print("pump running=%s bytes=%d idle=%d timeouts=%d shed=%d"
                  % (running, moved, idle, timeouts, shed))
    except KeyboardInterrupt:
        print("stopping")
    finally:
        _usbif.uac_pump_stop()
        if out is not None:
            try:
                out.close()
            except Exception:
                pass


if __name__ == "__main__":
    main()
