"""USB host backend over the ``usbif`` native C module.

This is the Python half of the MCU backend: it configures, observes, and
drains, while the C module owns enumeration, the class drivers, and -- for
UAC and UVC -- the isochronous byte movement Python must never be asked to do.

The division is deliberate and measured. See the transport note in
``usbif/__init__.py``: on ESP32 a scheduler-delivered callback loses most of a
1 kHz event stream during a blocking C call, so ``_usbif`` captures events into
a lock-free ring buffer in interrupt context and this module drains it. The
buffer is sized for the worst observed VM stall rather than a round number, and
reports overflow instead of dropping in silence.

The C module surface this expects, which is the contract the native side must
satisfy:

``_usbif.host_start(classes)``  start the daemon and the class drivers for the
                               requested class names; returns the set actually
                               started.
``_usbif.host_stop()``          stop them and release the controller.
``_usbif.host_devices()``       tuple of ``(id, vid, pid, product, serial,
                                classes, speed)`` tuples for attached devices.
``_usbif.host_drain(limit)``    pop up to ``limit`` buffered events, each
                                ``(kind, device_tuple)``; returns
                                ``(events, overflowed)``.
``_usbif.capabilities()``       class names this firmware was built with.
``_usbif.dev_functions([mask])`` report or choose the device functions this
                               board presents; setting re-enumerates.
``_usbif.dev_functions_built()`` the functions this firmware could present.
"""

import events

from . import Device, DeviceInfo, Host

try:
    import _usbif
except ImportError:  # pragma: no cover - exercised only off-target
    _usbif = None

# Event kinds as the C side numbers them. Kept small and integral so the ring
# buffer entry stays a fixed-size record that can be written from an ISR.
_ATTACH = 0
_DETACH = 1

# Events drained per poll. Bounded so a burst cannot turn one poll into an
# unbounded allocation storm on a device with 8 KB of free heap; whatever is
# left stays buffered for the next call.
DRAIN_LIMIT = 64


def _require():
    if _usbif is None:
        raise ImportError(
            "the usbif native module is not present in this firmware; "
            "build it as a user C module (see the usbif repository) or use "
            "usbif.auto.host() to get a backend appropriate to this port"
        )
    return _usbif


class NativeHost(Host):
    """USB host on hardware, backed by the native module."""

    def __init__(self, classes=None, drain_limit=DRAIN_LIMIT):
        super().__init__()
        self.classes = tuple(classes) if classes else None
        self.drain_limit = int(drain_limit)
        self._started = frozenset()

    def capabilities(self):
        if _usbif is None:
            return frozenset()
        return frozenset(_usbif.capabilities())

    @property
    def started(self):
        """The classes whose drivers actually came up, as a frozenset.

        Not the same question as :meth:`capabilities`, which is what the
        firmware was *built* with. A class that was asked for and did not
        start is missing from here, and that difference is the only thing
        that says so.
        """
        return self._started

    def _start(self):
        wanted = self.classes if self.classes is not None else tuple(self.capabilities())
        self._started = frozenset(_require().host_start(wanted))

    def _stop(self):
        _require().host_stop()
        self._started = frozenset()

    def _devices(self):
        return tuple(DeviceInfo(*row) for row in _require().host_devices())

    def _drain(self):
        raw, overflowed = _require().host_drain(self.drain_limit)
        self._overflowed = bool(overflowed)
        out = []
        for kind, row in raw:
            info = DeviceInfo(*row)
            if kind == _ATTACH:
                out.append(events.Usbattach(events.USBATTACH, info))
            elif kind == _DETACH:
                out.append(events.Usbdetach(events.USBDETACH, info))
        return out

    # -- The bus itself -----------------------------------------------------
    #
    # Everything below forwards to the C module unchanged. They are here so
    # that an application never has to name ``_usbif``: the object it already
    # holds is the whole host surface. Arguments are forwarded rather than
    # re-declared, so a wrapper cannot drift out of step with the C signature
    # it stands in front of, and the C module's own errors reach the caller.

    def desc(self, *args):
        """Raw configuration descriptor of an attached device, as bytes."""
        return _require().host_desc(*args)

    def stats(self, *args):
        """Counters from the host daemon: enumerations, transfers, errors."""
        return _require().host_stats(*args)

    def port_cycle(self, *args):
        """Power-cycle the root port, forcing every device to re-enumerate."""
        return _require().host_port_cycle(*args)

    def intr_dump(self, *args):
        """Snapshot of the controller's interrupt registers, for bring-up."""
        return _require().host_intr_dump(*args)

    # -- MSC: a hosted block device ----------------------------------------
    def msc_open(self, *args):
        return _require().host_msc_open(*args)

    def msc_info(self, *args):
        """``(block_count, block_size, inquiry_string)`` of the open drive."""
        return _require().host_msc_info(*args)

    def msc_read(self, *args):
        return _require().host_msc_read(*args)

    def msc_write(self, *args):
        return _require().host_msc_write(*args)

    def msc_close(self, *args):
        return _require().host_msc_close(*args)

    def msc_diag(self, *args):
        return _require().host_msc_diag(*args)

    def msc_provoke(self, *args):
        return _require().host_msc_provoke(*args)

    def partition(self, start_lba, num_blocks, readonly=True):
        """A :class:`MscPartition` on the open drive, ready for ``os.mount``.

        The block size comes from :meth:`msc_info` rather than the caller,
        because a partition told the wrong one mounts and then reads the
        wrong sectors -- a failure that looks like a corrupt filesystem.
        """
        return MscPartition(self, start_lba, num_blocks,
                            self.msc_info()[1], readonly=readonly)

    # -- UVC: a hosted camera ----------------------------------------------
    def uvc_negotiate(self, *args):
        return _require().host_uvc_negotiate(*args)

    def uvc_open(self, *args):
        return _require().host_uvc_open(*args)

    def uvc_read_frame(self, *args):
        return _require().host_uvc_read_frame(*args)

    def uvc_frame_ready(self, *args):
        return _require().host_uvc_frame_ready(*args)

    def uvc_stats(self, *args):
        return _require().host_uvc_stats(*args)

    def uvc_close(self, *args):
        return _require().host_uvc_close(*args)

    # -- CDC: a hosted serial port -----------------------------------------
    def cdc_open(self, *args):
        return _require().host_cdc_open(*args)

    def cdc_read(self, *args):
        return _require().host_cdc_read(*args)

    def cdc_write(self, *args):
        return _require().host_cdc_write(*args)

    def cdc_close(self, *args):
        return _require().host_cdc_close(*args)

    # -- HID: a hosted keyboard, mouse or gamepad --------------------------
    def hid_open(self, *args):
        return _require().host_hid_open(*args)

    def hid_read(self, *args):
        return _require().host_hid_read(*args)

    def hid_close(self, *args):
        return _require().host_hid_close(*args)


class NativeDevice(Device):
    """The board as a USB peripheral, with its identity chosen at runtime.

    Every function this firmware was built with is present in the binary;
    which ones the host is shown is a Python decision, taken here.
    """

    # Portable names to the C module's bitmask. Kept here rather than in the
    # C module so the names stay the portable API's, not the firmware's --
    # hence "uac" for FN_AUDIO and "uvc" for FN_VIDEO.
    _BITS = {"cdc": 1, "msc": 2, "uac": 4, "midi": 8, "hid": 16, "uvc": 32}

    def _mask_to_names(self, mask):
        return frozenset(n for n, b in self._BITS.items() if mask & b)

    def _names_to_mask(self, names):
        mask = 0
        for n in names:
            try:
                mask |= self._BITS[n]
            except KeyError:
                raise ValueError("unknown USB function: {}".format(n))
        return mask

    def functions(self, *names):
        usbif = _require()
        if not names:
            return self._mask_to_names(usbif.dev_functions())
        wanted = self._names_to_mask(names)
        if not wanted:
            # Presenting nothing is a legitimate wish, but USB expresses it
            # by detaching, not by an empty configuration -- a descriptor
            # with no interfaces is malformed and hosts flag the device.
            raise ValueError("a device must present at least one function")
        missing = self._mask_to_names(wanted & ~usbif.dev_functions_built())
        if missing:
            # Explicit rather than silent: a firmware that cannot present a
            # function should say so, not enumerate without it.
            raise ValueError(
                "not built into this firmware: {}".format(", ".join(sorted(missing)))
            )
        usbif.dev_functions(wanted)
        return self._mask_to_names(usbif.dev_functions())

    def functions_available(self):
        return self._mask_to_names(_require().dev_functions_built())

    # -- The board as a peripheral -----------------------------------------
    #
    # As on NativeHost, thin forwards so that an application holds one object
    # and never names ``_usbif``. The C names carry a ``dev_`` or ``uvc_dev_``
    # prefix that distinguishes the device role from the host role inside a
    # flat C namespace; on an object that *is* the device, that prefix says
    # nothing, so it is dropped here.

    def state(self, *args):
        """``(connected, mounted, suspended)`` as the host currently sees us."""
        return _require().dev_state(*args)

    def reinit(self, *args):
        """Tear the USB stack down and bring it back up on the same costume."""
        return _require().dev_reinit(*args)

    def pid(self, *args):
        """The product id this costume enumerates with."""
        return _require().dev_pid(*args)

    def desc_check(self, *args):
        """Validate the descriptor set this costume would present."""
        return _require().dev_desc_check(*args)

    def builtin_desc_cfg(self, *args):
        """The configuration descriptor the firmware would send, as bytes."""
        return _require().builtin_desc_cfg(*args)

    # -- MSC: the board as a flash drive -----------------------------------
    def msc_attach(self, *args):
        return _require().msc_attach(*args)

    def msc_attach_blockdev(self, *args):
        return _require().msc_attach_blockdev(*args)

    def msc_detach(self, *args):
        return _require().msc_detach(*args)

    def msc_status(self, *args):
        return _require().msc_status(*args)

    def msc_buffer(self, *args):
        return _require().msc_buffer(*args)

    def msc_bd_stats(self, *args):
        return _require().msc_bd_stats(*args)

    # -- HID: the board as a keyboard or mouse -----------------------------
    def hid_send(self, *args):
        return _require().hid_send(*args)

    def hid_leds(self, *args):
        """The host's keyboard LED state -- caps lock, num lock, scroll lock."""
        return _require().hid_leds(*args)

    # -- UAC: the board as a sound card -------------------------------------
    def uac_enable(self, *args):
        return _require().uac_enable(*args)

    def uac_pump_start(self, *args, **kwargs):
        return _require().uac_pump_start(*args, **kwargs)

    def uac_pump_stop(self, *args):
        return _require().uac_pump_stop(*args)

    def uac_pump_stats(self, *args):
        return _require().uac_pump_stats(*args)

    def uac_pump_rate(self, *args):
        """``(host_rate, wire_rate, retunes)``: the rate the host chose, the
        rate the pump's I2S wire is clocked at, and how often it followed."""
        return _require().uac_pump_rate(*args)

    def uac_available(self, *args):
        return _require().uac_available(*args)

    def uac_volume(self, *args):
        """The host's playback volume and mute, as the host set them."""
        return _require().uac_volume(*args)

    def uac_read(self, *args):
        return _require().uac_read(*args)

    def uac_stats(self, *args):
        return _require().uac_stats(*args)

    # -- UVC: the board as a webcam ----------------------------------------
    def uvc_format(self, *args):
        """``(width, height, frame_bytes)`` the host negotiated."""
        return _require().uvc_dev_format(*args)

    def uvc_reset(self, *args):
        return _require().uvc_dev_reset(*args)

    def uvc_streaming(self, *args):
        """True while the host has the video interface on an active alt setting."""
        return _require().uvc_dev_streaming(*args)

    def uvc_ready(self, *args):
        return _require().uvc_dev_ready(*args)

    def uvc_submit(self, *args):
        return _require().uvc_dev_submit(*args)

    def uvc_stats(self, *args):
        return _require().uvc_dev_stats(*args)


# HID report ids, as class attributes so a caller reaches them through the
# object it already holds rather than through the C module. Assigned after the
# class body because they are read from the C module at import time when it is
# present, and a firmware without HID simply has neither.
NativeDevice.HID_KEYBOARD = getattr(_usbif, "HID_KEYBOARD", 1)
NativeDevice.HID_MOUSE = getattr(_usbif, "HID_MOUSE", 2)


class MscPartition:
    """One partition of a hosted MSC device, as a MicroPython block device.

    Lifted out of the two examples that each carried a copy: MicroPython's FAT
    driver wants ``readblocks``/``writeblocks``/``ioctl``, the host driver
    reads and writes one block per call, and the filesystem does not begin at
    LBA 0 on a partitioned stick -- which is the detail that makes a naive
    mount fail with a confusing error rather than an obvious one.

        host = usbif.auto.host(classes=("msc",)).start()
        host.msc_open(dev_id)
        part = host.partition(start_lba, num_blocks, readonly=False)
        os.mount(part, "/usb")

    ``readonly`` is the default because the safe answer to "may I write to the
    drive somebody just plugged in" is no, and because a read-only mount that
    refuses a write with EROFS is far better than one that appears to accept
    it. Pass ``readonly=False`` for the datalogger case.
    """

    def __init__(self, host, start_lba, num_blocks, block_size, readonly=True):
        self.host = host
        self.start = start_lba
        self.count = num_blocks
        self.bs = block_size
        self.readonly = readonly
        self._one = bytearray(block_size)

    def readblocks(self, block_num, buf, offset=0):
        # The extended protocol hands us a buffer that may span several
        # blocks; the host driver reads one block per call, so loop.
        want = len(buf)
        done = 0
        lba = self.start + block_num
        if offset:
            self.host.msc_read(lba, self._one)
            take = min(want, self.bs - offset)
            buf[0:take] = self._one[offset:offset + take]
            done = take
            lba += 1
        while done < want:
            take = min(self.bs, want - done)
            if take == self.bs:
                self.host.msc_read(lba, memoryview(buf)[done:done + self.bs])
            else:
                self.host.msc_read(lba, self._one)
                buf[done:done + take] = self._one[0:take]
            done += take
            lba += 1
        return 0

    def writeblocks(self, block_num, buf, offset=0):
        # Whole blocks go straight out. A partial block is read-modify-write:
        # the host driver refuses a short write rather than padding it, and
        # padding is exactly what would corrupt the neighbouring bytes.
        if self.readonly:
            raise OSError(30)   # EROFS
        want = len(buf)
        done = 0
        lba = self.start + block_num
        if offset:
            self.host.msc_read(lba, self._one)
            take = min(want, self.bs - offset)
            self._one[offset:offset + take] = buf[0:take]
            self.host.msc_write(lba, self._one)
            done = take
            lba += 1
        while done < want:
            take = min(self.bs, want - done)
            if take == self.bs:
                self.host.msc_write(lba, memoryview(buf)[done:done + self.bs])
            else:
                self.host.msc_read(lba, self._one)
                self._one[0:take] = buf[done:done + take]
                self.host.msc_write(lba, self._one)
            done += take
            lba += 1
        return 0

    def ioctl(self, op, arg):
        if op == 4:         # block count
            return self.count
        if op == 5:         # block size
            return self.bs
        if op == 6:         # erase block -- no-op for FAT
            return 0
        return 0


__all__ = ("NativeHost", "NativeDevice", "MscPartition")
