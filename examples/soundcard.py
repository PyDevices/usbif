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
import usbif.auto

# Host advertises 48 kHz stereo; the board codec is typically 24 kHz mono.
# The C pump decimates. Match the board's own rate so pitch is right.
DEFAULT_VOLUME = 85  # digital gain on this hardware; see phase0 findings


def _wire():
    """The board's I2S output wire: port and pin numbers, nothing opened.

    This file is the third kind of audio consumer. It does not want a sample
    player and it does not even want a PCM sink -- the C FreeRTOS pump owns
    the I2S channel, so all Python needs to hand over is where the wires go.
    ``AudioCapability.wire`` exists for exactly this.

    What used to be here: three historical pin-naming conventions tried in
    turn, then an ``I2S_OUT_PINS`` tuple, then a RuntimeError saying
    "board_peripherals does not publish I2S output pins". On the ESP32-P4 it
    reached that error every time, because the pins were private names
    (``_SCLK``/``_LRCK``/``_DSDIN``) that no contract promised.
    """
    wire = getattr(bp.AUDIO_OUT, "wire", None)
    if wire is None:
        raise RuntimeError(
            "this board's AUDIO_OUT capability publishes no I2S wire, so the "
            "C pump has nothing to open. Boards with a software-only or "
            "non-I2S audio path cannot host the C sound card."
        )
    return wire


def _bring_up_codec():
    """Power the amp and set a sane volume, WITHOUT opening an I2S stream.

    The ordering here is the whole difficulty. The C pump opens the I2S
    channel itself, so Python must not: two owners of one peripheral is a
    silent failure, not an error. But the codec still has to be powered and
    unmuted first or every byte moves and nothing is audible.

    ``audio_power`` is the board role for exactly that -- analog path on,
    no stream. A board without one leaves the codec alone and the pump may
    still be silent; that is reported rather than papered over, because the
    old code caught every exception here and printed nothing.
    """
    power = getattr(bp, "audio_power", None)
    if not callable(power):
        print("note: this board publishes no audio_power role; the codec is "
              "not being brought up, and the pump may run silently")
        return False
    power(True, volume=DEFAULT_VOLUME)
    return True


def main():
    dev = usbif.auto.device()
    wire = _wire()
    bclk, ws, dout, mclk = wire.sck, wire.ws, wire.sd, wire.mck
    fmt = bp.AUDIO_OUT.default
    rate, bits, channels = fmt.rate, fmt.bits, fmt.channels

    powered = _bring_up_codec()

    # Costume first so the host sees the sound card before we start the pump.
    # CDC stays so the REPL survives on the same connector.
    dev.functions("cdc", "uac")
    print("costume: cdc+uac -- look for Speakers on the host")

    kwargs = {"rate": rate, "bits": bits, "channels": channels}
    if mclk is not None and mclk >= 0:
        kwargs["mclk"] = mclk
        # The board publishes the MCLK ratio its codec is configured against;
        # the pump used to hard-code 512, which contradicted the board and made
        # anything above 32 kHz fail outright. See usbif#12.
        kwargs["mclk_multiple"] = wire.mck_fs
    dev.uac_pump_start(bclk, ws, dout, **kwargs)
    print("C pump started: I2S bclk=%d ws=%d dout=%d rate=%d ch=%d codec=%s"
          % (bclk, ws, dout, rate, channels, "up" if powered else "UNTOUCHED"))
    print("play audio to this board from a PC, or from usb_speaker.py on an S3")

    try:
        while True:
            time.sleep_ms(1000)
            running, moved, idle, timeouts, shed = dev.uac_pump_stats()
            print("pump running=%s bytes=%d idle=%d timeouts=%d shed=%d"
                  % (running, moved, idle, timeouts, shed))
    except KeyboardInterrupt:
        print("stopping")
    finally:
        dev.uac_pump_stop()
        if powered:
            # Amp off after the pump releases I2S, not before: dropping the
            # analog path while DMA is still clocking pops the speaker.
            bp.audio_power(False)


if __name__ == "__main__":
    main()
