# usbif Spike B, part 3 — who claims an AUDIO-class interface?
#
# Code reading says MicroPython's runtime driver is registered as a TinyUSB
# *application* driver, which gets first refusal on every interface and
# declines only those below USBD_ITF_BUILTIN_MAX. If that is right, an
# audio-class interface declared from Python is claimed by Python and
# TinyUSB's compiled-in audiod_open() never sees it — which would make the
# "declare UAC at runtime" shortcut impossible and force usbif's isochronous
# classes to be built-in interfaces instead.
#
# Driven over the UART console so the CDC port is free to re-enumerate.
import machine, time

LOG = open("/spike_b_itf.txt", "w")


def log(s):
    LOG.write(s + "\n")
    LOG.flush()


claimed = []


def _open_itf(desc):
    # Fires from runtime_dev_open() when the Python driver claims interfaces.
    b = bytes(desc)
    claimed.append(b)
    try:
        log("open_itf_cb FIRED len=%d class=0x%02x subclass=0x%02x itf=%d" % (len(b), b[5], b[6], b[2]))
    except Exception as e:
        log("open_itf_cb fired, parse error %s raw=%s" % (e, b))


# --- descriptors -------------------------------------------------------
# Device: vendor-neutral, one configuration.
desc_dev = bytes(
    [
        18, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 64,
        0x3A, 0x03, 0x42, 0x40, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01,
    ]
)

# Config: one AUDIO / AUDIOCONTROL interface, no endpoints (valid USB:
# an audio control interface needs none).
ITF = bytes([9, 0x04, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00])
CFG = bytes([9, 0x02, (9 + len(ITF)) & 0xFF, 0x00, 0x01, 0x01, 0x00, 0x80, 50])
desc_cfg = CFG + ITF

log("=== usbif Spike B part 3: audio-class interface ownership")
usbd = machine.USBDevice()
try:
    usbd.active(False)
    time.sleep_ms(1500)
    usbd.builtin_driver = False
    usbd.config(
        desc_dev=desc_dev,
        desc_cfg=desc_cfg,
        desc_strs=[],
        open_itf_cb=_open_itf,
    )
    usbd.active(True)
    log("activated; waiting for host SET_CONFIGURATION")
except Exception as e:
    log("init/activate failed: %r" % (e,))

for _ in range(60):
    time.sleep_ms(250)
    if claimed:
        break

log("claims=%d" % len(claimed))
if claimed:
    log("VERDICT: Python runtime driver claimed the AUDIO interface -> "
        "audiod_open never sees it; runtime UAC declaration is impossible.")
else:
    log("VERDICT: no Python claim observed (host may not have configured; "
        "inconclusive rather than contrary).")

# Hand the port back to the normal built-in CDC.
try:
    usbd.active(False)
except Exception as e:
    log("deactivate: %r" % (e,))
log("=== done")
LOG.close()
