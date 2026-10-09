# SPDX-FileCopyrightText: 2026 Brad Barnett
#
# SPDX-License-Identifier: MIT
"""examples/hid_host.py must read a keyboard that prefixes a report ID.

A keyboard sharing its interface with a mouse (a board running
``hid_keyboard.py``) sends ``[report id, modifiers, 0, key, ...]``. Fed to the
boot-report decoder as it came, the report ID 1 read as the Left Ctrl bit, so
every key arrived with Ctrl held and Ctrl was never released.
"""

import importlib.util
import pathlib
import unittest

import _env  # noqa: F401

import events
from usbif.hid_keyboard import KeyboardDecoder

ROOT = pathlib.Path(__file__).resolve().parents[1]


def _load_example():
    spec = importlib.util.spec_from_file_location(
        "hid_host_example", ROOT / "examples" / "hid_host.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


H = 0x0B


class HidHostReports(unittest.TestCase):
    def setUp(self):
        self.example = _load_example()

    def _keys(self, raw_reports):
        decoder = KeyboardDecoder()
        out = []
        for raw in raw_reports:
            buf = bytearray(64)
            buf[:len(raw)] = raw
            report = self.example.keyboard_report(buf, len(raw))
            if report is not None:
                out.extend((e.type, e.mod) for e in decoder.feed(report))
        return out

    def test_report_id_is_stripped(self):
        got = self._keys([bytes([1, 0, 0, H, 0, 0, 0, 0, 0]), bytes([1] + [0] * 8)])
        self.assertEqual([t for t, _ in got], [events.KEYDOWN, events.KEYUP])
        self.assertTrue(all(mod == 0 for _, mod in got), got)

    def test_plain_boot_report_still_reads(self):
        got = self._keys([bytes([0, 0, H, 0, 0, 0, 0, 0]), bytes(8)])
        self.assertEqual([t for t, _ in got], [events.KEYDOWN, events.KEYUP])

    def test_mouse_report_on_the_same_interface_is_skipped(self):
        self.assertEqual(self._keys([bytes([2, 1, 5, 0xFB, 0, 0])]), [])


if __name__ == "__main__":
    unittest.main()
