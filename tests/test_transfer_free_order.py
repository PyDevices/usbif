# SPDX-FileCopyrightText: 2026 Brad Barnett
#
# SPDX-License-Identifier: MIT
"""A host class driver frees its transfers only after the interface is released.

A halted and flushed transfer stays queued in the ESP-IDF host library until
the client pump retires it, and the pump writes to it as it does.
``usb_host_interface_release()`` succeeding is the library's signal that none
is left in flight, so a driver's release function must call it before any
``usb_host_transfer_free()``. The MIDI host freed first and panicked in the
allocator at ``host_stop()`` (usbif#74); the HID host had the same order
(usbif#75). This reads each driver's release function and checks the order.
"""

import pathlib
import re
import unittest

SRC = pathlib.Path(__file__).resolve().parents[1] / "src"

# The function in each driver that releases the interface on close().
RELEASES = {
    "usbif_host_hid.c": "usbif_hid_release",
    "usbif_host_midi.c": "usbif_midih_release",
}


def _function(text, name):
    """The body of ``static void name(void) { ... }`` in ``text``."""
    m = re.search(r"static\s+void\s+" + name + r"\s*\(\s*void\s*\)\s*\{", text)
    if m is None:
        return None
    depth = 0
    for j in range(m.end() - 1, len(text)):
        if text[j] == "{":
            depth += 1
        elif text[j] == "}":
            depth -= 1
            if depth == 0:
                return text[m.end():j]
    return None


def free_before_release(body):
    """Why ``body`` frees a transfer the library may still hold, or None."""
    release = body.find("usb_host_interface_release(")
    if release < 0:
        return "never releases the interface"
    free = body.find("usb_host_transfer_free(")
    if free < 0:
        return "never frees its transfers"
    if free < release:
        return "frees a transfer before interface_release()"
    tail = body[release:]
    if not re.search(r"if\s*\(\s*err\s*==\s*ESP_OK\s*\)\s*\{[^}]*usb_host_transfer_free\(", tail):
        return "frees its transfers without checking that the release succeeded"
    return None


class TransferFreeOrder(unittest.TestCase):
    def test_each_driver_frees_after_a_successful_release(self):
        problems = []
        for filename, name in RELEASES.items():
            body = _function((SRC / filename).read_text(encoding="utf-8"), name)
            if body is None:
                problems.append("{}: no {}()".format(filename, name))
                continue
            why = free_before_release(body)
            if why:
                problems.append("{}: {}() {}".format(filename, name, why))
        self.assertEqual(problems, [])

    def test_the_check_finds_the_old_order(self):
        # The check above passes when nothing is found, so prove it can fail:
        # the HID release as it was before usbif#75.
        old = ("\n    usb_host_transfer_free(usbif_hid.xfer_in);\n"
               "    usbif_hid.xfer_in = NULL;\n"
               "    esp_err_t err = ESP_FAIL;\n"
               "    for (TickType_t i = 0; i < release_limit; i++) {\n"
               "        err = usb_host_interface_release(client, dev, itf);\n"
               "        if (err == ESP_OK) {\n"
               "            break;\n"
               "        }\n"
               "    }\n")
        self.assertEqual(free_before_release(old),
                         "frees a transfer before interface_release()")

    def test_the_check_wants_the_release_to_have_succeeded(self):
        unchecked = ("\n    esp_err_t err = usb_host_interface_release(client, dev, itf);\n"
                     "    usb_host_transfer_free(usbif_hid.xfer_in);\n")
        self.assertEqual(free_before_release(unchecked),
                         "frees its transfers without checking that the release succeeded")


if __name__ == "__main__":
    unittest.main()
