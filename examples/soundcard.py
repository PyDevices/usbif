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

**REPL.** ``main()`` starts the card and returns: the pump runs in C, so the
REPL stays usable while it plays. ``stats()`` prints the pump's counters,
``watch()`` prints them every second until Ctrl-C, and ``stop()`` stops the
card.

**Console.** Costume is CDC+UAC so the REPL stays on the same cable. UART is
still the safer place for the REPL when iterating (a costume change that drops
CDC cuts a native-USB session mid-run).

**Pins and the amp.** I2S pin numbers and codec bring-up come from
``board_peripherals``: the pump owns the I2S channel, not the ES8311. Without
enabling the speaker amp you get perfect byte counters and silence -- the
failure mode that ate an afternoon in Phase 4.

**Spectrum.** On a board with a display (a ``board_config`` with
``display_drv``), the panel shows a spectrum of what the computer plays, if
pydevices-examples' ``spectrum`` analyzer is on the board: its
``lib/examples/spectrum`` folder copied to ``/lib/spectrum`` (or installed with
``mip``). A headless board, or one without the analyzer, plays the same and
says why there's no spectrum.
"""

import os
import sys
import time

import board_peripherals as bp
import usbif.auto

# The host sees 48 kHz stereo by default and may choose 44.1 kHz instead.
# The I2S wire runs at the host's own rate when the board can clock it there
# (see _wire_rate), so the pump passes every frame and follows a 44.1 kHz host
# at 1:1. A board with fixed clocks gets its own rate, and the pump decimates
# by the ratio -- one frame in N, unfiltered, so everything above half the
# board rate aliases. Channels stay the board's: a mono codec gets the host's
# left and right averaged.
HOST_RATE = 48000
DEFAULT_VOLUME = 85  # digital gain; 100 overdrives the P4 panel amp, 50 is barely audible


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


def _wire_rate(capability):
    """The I2S rate: the host's, if the board can clock its codec there.

    ``rates`` is None for a PLL that takes any rate (the ESP32 family), or
    the list a fixed crystal can make. Where the host's rate is out of reach
    the board's default stands and the pump decimates to it: on the
    ESP32-P4 panel that was 24 kHz -- every other frame dropped, unfiltered,
    so nothing above 12 kHz and aliasing below it.
    """
    rates = capability.rates
    if rates is None or HOST_RATE in rates:
        return HOST_RATE
    return capability.default.rate


def _find_spectrum():
    """The folder holding the spectrum analyzer, or None."""
    here = __file__.replace("\\", "/").rsplit("/", 1)[0] if "/" in __file__ else "."
    for folder in (here + "/spectrum", here + "/examples/spectrum", "/lib/spectrum", "/lib/examples/spectrum"):
        try:
            os.stat(folder + "/analyzer.py")
        except OSError:
            continue
        return folder
    return None


def _spectrum():
    """Start the spectrum on the board's display; return its app, or None.

    The analyzer's modules import each other by plain name, so its folder goes
    on ``sys.path`` and ``analyzer`` is imported directly (importing the
    ``spectrum`` package would start its demo instead).
    """
    try:
        import board_config
    except ImportError:
        return None  # a headless board: board_peripherals only
    display_drv = getattr(board_config, "display_drv", None)
    if display_drv is None:
        return None
    folder = _find_spectrum()
    if folder is None:
        print("no spectrum: copy pydevices-examples' lib/examples/spectrum to /lib/spectrum")
        return None
    if folder not in sys.path:
        sys.path.append(folder)
    try:
        import appdev
        from analyzer import Spectrum, levels_for_soundcard

        meter = Spectrum(display_drv, levels_for_soundcard())
    except (ImportError, ValueError, OSError) as error:
        # OSError: another meter already holds the sound card.
        print("no spectrum:", error)
        return None
    # A panel that presents just the rows the meter changed has the app's
    # whole-frame refresh turned off.
    app = appdev.App(board_config, refresh_period=0 if meter.present_rows else None)
    meter.start(app)
    print("spectrum: on the display, from", folder)
    global _meter
    _meter = meter
    return app


_dev = None
_powered = False
_meter = None


def main():
    """Start the sound card and return, so the REPL stays usable.

    The pump moves the audio in C, so nothing in Python has to keep looping
    for it: ``stats()`` reports the pump once, ``watch()`` reports it every
    second until Ctrl-C, and ``stop()`` stops the card.
    """
    global _dev, _powered
    if _dev is not None:
        print("the sound card is already running; stop() first")
        return
    dev = usbif.auto.device()
    if dev.uac_pump_stats()[0]:
        # A soft reset leaves the C pump running (its buffers aren't on the
        # Python heap), with nothing in Python left to stop it, and holding
        # I2S: uac_pump_start() would fail. Stop it, then start afresh.
        dev.uac_pump_stop()
        print("stopped the pump left running by a soft reset")
    _spectrum()  # before the card, as the meter attaches to its pump
    wire = _wire()
    bclk, ws, dout, mclk = wire.sck, wire.ws, wire.sd, wire.mck
    fmt = bp.AUDIO_OUT.default
    rate, bits, channels = _wire_rate(bp.AUDIO_OUT), fmt.bits, fmt.channels

    _powered = _bring_up_codec()

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
    _dev = dev
    print("C pump started: I2S bclk=%d ws=%d dout=%d rate=%d ch=%d codec=%s"
          % (bclk, ws, dout, rate, channels, "up" if _powered else "UNTOUCHED"))
    print("play audio to this board from a PC, or from usb_speaker.py on an S3")
    print("at the REPL: soundcard.stats(), soundcard.watch(), soundcard.stop()")


def stats():
    """Print the pump's counters once."""
    if _dev is None:
        print("the sound card isn't running: soundcard.main() starts it")
        return
    running, moved, idle, timeouts, shed = _dev.uac_pump_stats()
    host_rate, wire_rate, _ = _dev.uac_pump_rate()
    print("pump running=%s host=%d wire=%d bytes=%d idle=%d "
          "timeouts=%d shed=%d"
          % (running, host_rate, wire_rate, moved, idle, timeouts, shed))


def watch():
    """Print the pump's counters every second until Ctrl-C; the card plays on."""
    try:
        while True:
            stats()
            time.sleep_ms(1000)
    except KeyboardInterrupt:
        pass


def stop():
    """Stop the sound card: the pump, then the amp."""
    global _dev, _powered, _meter
    if _meter is not None:
        # Stop drawing, and let go of the sound card, so a later main() can
        # attach a meter of its own.
        _meter.stop()
        close = getattr(_meter.source, "close", None)
        if close is not None:
            close()
        _meter = None
    if _dev is None:
        return
    _dev.uac_pump_stop()
    _dev = None
    if _powered:
        # Amp off after the pump releases I2S, not before: dropping the
        # analog path while DMA is still clocking pops the speaker.
        bp.audio_power(False)
        _powered = False

if __name__ == "__main__":
    main()
