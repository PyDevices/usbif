# SPDX-FileCopyrightText: 2026 Brad Barnett
#
# SPDX-License-Identifier: MIT
"""examples/sd_drive.py must refuse a card that never answered (usbif#4).

A card that is absent or timing out answers ``ioctl(4)``/``ioctl(5)`` with -1
instead of raising. The example used to hand that to the host as a drive of
no blocks and then sit in its service loop, silent. These tests run the
example's ``main()`` against a fake card and a fake device, so the refusal is
checked on the path a user actually takes, with a healthy card as the control
that must still attach.
"""

import importlib.util
import pathlib
import types
import unittest

import _env  # noqa: F401

import usbif.auto

ROOT = pathlib.Path(__file__).resolve().parents[1]


def _load_example():
    spec = importlib.util.spec_from_file_location(
        "sd_drive_example", ROOT / "examples" / "sd_drive.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    # CPython's time has no sleep_ms; the service loop only needs it to exist.
    module.time = types.SimpleNamespace(sleep_ms=lambda ms: None)
    return module


class _Card:
    def __init__(self, blocks, block_size):
        self._answers = {4: blocks, 5: block_size}

    def ioctl(self, op, arg):
        return self._answers.get(op)


class _Device:
    """Records what the example asked of the USB device."""

    def __init__(self):
        self.attached = None

    def msc_detach(self):
        pass

    def msc_attach_blockdev(self, card, writable):
        self.attached = card

    def functions(self, *names):
        pass

    def msc_status(self):
        # Attached, and already ejected, so the service loop ends at once.
        return (self.attached is not None, 0, True)

    def msc_bd_stats(self):
        return (0, 0)


class SdDriveRefusesDeadCard(unittest.TestCase):
    def setUp(self):
        self.example = _load_example()
        self.dev = _Device()
        self._saved_device = usbif.auto.device
        usbif.auto.device = lambda: self.dev

    def tearDown(self):
        usbif.auto.device = self._saved_device

    def _run_with(self, card):
        self.example.open_card = lambda: card
        self.example.main()

    def test_card_that_never_answered_is_refused_before_attach(self):
        for blocks, size in ((-1, -1), (0, 512), (62685184, -1), (None, 512)):
            with self.subTest(blocks=blocks, size=size):
                self.dev.attached = None
                with self.assertRaises(RuntimeError) as ctx:
                    self._run_with(_Card(blocks, size))
                self.assertIn("did not answer", str(ctx.exception))
                self.assertIsNone(
                    self.dev.attached,
                    "a card with no geometry was served to the host")

    def test_healthy_card_still_attaches(self):
        card = _Card(62685184, 512)
        self._run_with(card)
        self.assertIs(self.dev.attached, card)


if __name__ == "__main__":
    unittest.main()
