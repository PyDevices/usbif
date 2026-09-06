# usbif: Python FIFO pump -- the inspectable path from USB Audio to the codec.
#
# This is NOT the shipping sound card. The production path is the C pump in
# `soundcard.py` (`_usbif.uac_pump_start`), which moves isochronous bytes
# without the interpreter. Keep this script for two jobs it still does better
# than the C pump: watching the FIFO fill from Python (latency tooling such as
# `midi_latency.py` also depends on the FIFO being readable), and proving the
# path end to end when you need every byte count in a log file.
#
# History, kept so the numbers make sense: TinyUSB's example FIFO sizing is a
# multiple of the endpoint packet; at 24 kHz mono that was ~5 ms and a third
# of the stream was lost under a Python consumer. usbif sizes the FIFO in
# milliseconds instead. Measured at 96-99% of the offered stream on an
# ESP32-P4 with this loop -- which is why the first pass left the pump in
# Python. The C pump later took over for the flagship; this file stayed as
# the transparent half.
import time

import _usbif
import board_peripherals as bp
from audiodev.i2s_audio import I2SPCMOutput

# 20 ms at 24 kHz mono 16-bit. Batching matters: at this size one run moved the
# same audio in 578 reads that an unbatched loop needed 27,560 reads for.
CHUNK = (bp._FORMAT.rate // 1000) * bp._FORMAT.frame_size * 20

# The host has a volume control (the UAC feature unit) but does not have to use
# it -- Windows drives its slider in software and leaves ours untouched, so the
# codec sits wherever we put it. Full scale on this board's amplifier is
# overdriven, so start where the board itself starts.
DEFAULT_VOLUME = 50


def host_volume_percent(db256, fallback):
    """UAC2's signed 1/256 dB onto the codec's 0-100 scale.

    The descriptor advertises -100 dB .. 0 dB. A host that has never touched
    the control reports 0 dB, which is indistinguishable from a deliberate full
    scale -- so that case keeps the board's own default rather than shouting.
    """
    if db256 == 0:
        return fallback
    percent = 100 + db256 // 256
    return 0 if percent < 0 else (100 if percent > 100 else percent)


def main(seconds=30, log_path="/uac_pump.txt"):
    log_file = open(log_path, "w")

    def log(s):
        log_file.write(s + "\n")
        log_file.flush()

    out = I2SPCMOutput(
        lambda: bp._output_stream(),
        bp._FORMAT,
        session=bp._SESSION,
        set_hardware_volume=lambda v: bp._codec_call("set_dac_volume", v),
        set_hardware_mute=lambda v: bp._codec_call("dac_mute", v),
        # Not optional: this drives the speaker amplifier's control pin and
        # calls enable_output() on the codec. Without it every byte still
        # moves and nothing is audible.
        power=bp._output_power,
    )
    out.open()
    out.mute(False)

    muted, db256 = _usbif.uac_volume()
    volume = host_volume_percent(db256, DEFAULT_VOLUME)
    out.set_volume(volume)
    out.mute(muted)
    log("codec %r, volume %d, host %d/256 dB" % (bp._FORMAT, volume, db256))

    buf = bytearray(CHUNK)
    view = memoryview(buf)
    avail, read, write = _usbif.uac_available, _usbif.uac_read, out.write
    total = reads = 0
    last = (muted, db256)
    t0 = time.ticks_ms()

    while time.ticks_diff(time.ticks_ms(), t0) < seconds * 1000:
        if avail() >= CHUNK:
            n = read(buf)
            if n:
                write(view[:n])
                total += n
                reads += 1
                if reads % 100 == 0:
                    now = _usbif.uac_volume()
                    if now != last:
                        out.set_volume(host_volume_percent(now[1], DEFAULT_VOLUME))
                        out.mute(now[0])
                        last = now
        else:
            time.sleep_ms(2)

    log("done: %d bytes in %d reads (avg %d)"
        % (total, reads, total // max(reads, 1)))
    out.close()
    log_file.close()
    return total


if __name__ == "__main__":
    main()
