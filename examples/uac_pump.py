# usbif: play what the host sends over USB Audio out of the board's codec.
#
# The isochronous endpoint is serviced in C on TinyUSB's task; this loop only
# moves already-buffered blocks from that FIFO to the I2S sink, which is a soft
# deadline set by FIFO depth rather than a per-frame one.
import time

import _usbif
import board_peripherals as bp
from audiodev.i2s_audio import I2SPCMOutput

LOG = open("/uac_pump.txt", "w")


def log(s):
    LOG.write(s + "\n")
    LOG.flush()
    print(s)


out = I2SPCMOutput(
    lambda: bp._output_stream(),
    bp._FORMAT,
    session=bp._SESSION,
    set_hardware_volume=lambda v: bp._codec_call("set_dac_volume", v),
    set_hardware_mute=lambda v: bp._codec_call("dac_mute", v),
)
out.open()
out.set_volume(100)          # on-device audio demos run at full volume
out.mute(False)
log("codec open: %r" % (bp._FORMAT,))

buf = bytearray(1024)
view = memoryview(buf)
total = 0
blocks = 0
silent_polls = 0
t0 = time.ticks_ms()

while time.ticks_diff(time.ticks_ms(), t0) < 30000:
    n = _usbif.uac_read(buf)
    if n:
        out.write(view[:n])
        total += n
        blocks += 1
        if blocks % 200 == 0:
            log("streamed %d bytes (%d blocks)" % (total, blocks))
    else:
        silent_polls += 1
        time.sleep_ms(1)

g, s, itf, unh, streaming, rate = _usbif.uac_stats()
log("done: %d bytes in %d blocks; idle polls %d" % (total, blocks, silent_polls))
log("uac: get=%d set=%d set_itf=%d unhandled=%s streaming=%s rate=%d"
    % (g, s, itf, hex(unh), streaming, rate))
out.close()
LOG.close()
