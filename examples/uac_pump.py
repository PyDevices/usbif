# usbif: play what the host sends over USB Audio out of the board's codec.
#
# The isochronous endpoint is serviced in C on TinyUSB's task; this loop only
# moves already-buffered blocks from that FIFO to the I2S sink, which is a soft
# deadline set by FIFO depth rather than a per-frame one. It is a proof that
# the path is continuous, not the shipping design -- see the findings on why
# the pump belongs in C.
import time

import _usbif
import board_peripherals as bp
from audiodev.i2s_audio import I2SPCMOutput

LOG = open("/uac_pump.txt", "w")


def log(s):
    LOG.write(s + "\n")
    LOG.flush()
    print(s)


# power= is not optional on this board: it drives the speaker amplifier's
# control pin and calls enable_output() on the ES8311. Without it everything
# else works, bytes flow, and nothing is audible -- which is exactly the way
# this was first got wrong.
out = I2SPCMOutput(
    lambda: bp._output_stream(),
    bp._FORMAT,
    session=bp._SESSION,
    set_hardware_volume=lambda v: bp._codec_call("set_dac_volume", v),
    set_hardware_mute=lambda v: bp._codec_call("dac_mute", v),
    power=bp._output_power,
)
out.open()


def host_volume_percent(db256):
    """UAC2's signed 1/256 dB onto the codec's 0-100 scale.

    The descriptor advertises -100 dB .. 0 dB, so 0 maps to full scale and the
    bottom of the range to silence.
    """
    db = db256 / 256.0
    percent = int(100 + db)
    return 0 if percent < 0 else (100 if percent > 100 else percent)


muted, db256 = _usbif.uac_volume()
out.set_volume(host_volume_percent(db256))
out.mute(muted)
log("codec open: %r  host volume %d/256 dB, muted %s" % (bp._FORMAT, db256, muted))

buf = bytearray(1024)
view = memoryview(buf)
total = 0
blocks = 0
last_vol = (muted, db256)
t0 = time.ticks_ms()

while time.ticks_diff(time.ticks_ms(), t0) < 30000:
    n = _usbif.uac_read(buf)
    if n:
        out.write(view[:n])
        total += n
        blocks += 1
        if blocks % 400 == 0:
            # Follow the host's slider while streaming; Windows sends these as
            # feature-unit set requests whenever the user moves it.
            now = _usbif.uac_volume()
            if now != last_vol:
                out.set_volume(host_volume_percent(now[1]))
                out.mute(now[0])
                last_vol = now
                log("host volume -> %d/256 dB, muted %s" % (now[1], now[0]))
            log("streamed %d bytes (%d blocks)" % (total, blocks))
    else:
        time.sleep_ms(1)

g, s, itf, unh, streaming, rate = _usbif.uac_stats()
log("done: %d bytes in %d blocks" % (total, blocks))
log("uac: get=%d set=%d set_itf=%d unhandled=%s streaming=%s rate=%d"
    % (g, s, itf, hex(unh), streaming, rate))
out.close()
LOG.close()
