"""UacHostOutput.c_sink() on MicroPython's unix port (usbif#43).

Run by CI against a unix build that includes the _usbif module; not a
unittest module, because CPython has no _usbif to import. Two things are
checked on the real interpreter rather than CPython:

- the native binding exists and, with no USB host on this port, raises
  OSError(EOPNOTSUPP) rather than returning something a usermod would trust;
- the Python method opens the stream and passes the chosen format through to
  the driver, over a stand-in driver.

The C struct and its lifetime rules are proven by tests/test_pcm_sink.c.
"""

import errno

import _usbif
from usbif import uac_audio

assert hasattr(_usbif, "host_uac_c_sink")
try:
    _usbif.host_uac_c_sink(2, 16)
    raise AssertionError("host_uac_c_sink returned on a port with no USB host")
except OSError as e:
    assert e.errno == errno.EOPNOTSUPP, e


class Stream:
    interface = 1
    alt = 1
    endpoint = 0x01
    max_packet = 196
    clock = 0
    control = 0
    channels = 2
    bits = 16
    frame_bytes = 2
    rates = (48000,)


class Driver:
    def __init__(self):
        self.opened = 0
        self.closed = 0
        self.asked = None

    def host_uac_open(self, *args):
        self.opened += 1

    def host_uac_close(self):
        self.closed += 1

    def host_uac_c_sink(self, channels, bits):
        self.asked = (channels, bits)
        return b"PCMS" + bytes(52)


driver = Driver()
uac_audio._usbif = driver
out = uac_audio.UacHostOutput(4, Stream(), 48000)
blob = out.c_sink()
assert driver.opened == 1, driver.opened
assert driver.asked == (2, 16), driver.asked
assert isinstance(blob, bytes) and blob[:4] == b"PCMS"
out.close()
assert driver.closed == 1
print("unix c_sink: PASS")
