# SPDX-FileCopyrightText: 2026 Brad Barnett
#
# SPDX-License-Identifier: MIT
"""Waits in the C sources must be bounded in milliseconds, not ticks (usbif#29).

A loop that calls ``vTaskDelay(1)`` a fixed number of times waits that many
ticks, and a tick is 10 ms only at ``CONFIG_FREERTOS_HZ=100``. On a board
built at 1000 Hz every such wait shrinks to a tenth: ``host_stop()`` gave
teardown 200 ms instead of 2 s and reported a clean stop as a wedge. This
scans ``src/`` for a ``for`` loop whose bound is a bare number and whose body
sleeps a tick, which is the shape of that bug. Bound the loop with
``USBIF_MS_TICKS(ms)`` from ``src/usbif_ticks.h`` instead.
"""

import pathlib
import re
import unittest

SRC = pathlib.Path(__file__).resolve().parents[1] / "src"

# "for (int i = 0; i < 200 ..." or "... i < SOME_TICKS ..." -- a count, not a
# duration. A bound that goes through USBIF_MS_TICKS() (or another *_TICKS(ms)
# macro taking milliseconds) is a duration and isn't matched.
_COUNTED = re.compile(r"for\s*\(\s*(?:int|unsigned|uint32_t|TickType_t)\s+\w+\s*=\s*0\s*;"
                      r"\s*\w+\s*<\s*(\d+|[A-Z_]+_TICKS)\b(?!\s*\()")


def _body(text, start):
    """The text of the block that opens at the first '{' after ``start``."""
    i = text.index("{", start)
    depth = 0
    for j in range(i, len(text)):
        if text[j] == "{":
            depth += 1
        elif text[j] == "}":
            depth -= 1
            if depth == 0:
                return text[i:j + 1]
    return text[i:]


def counted_tick_waits():
    found = []
    for path in sorted(SRC.rglob("*.c")):
        text = path.read_text(encoding="utf-8")
        for m in _COUNTED.finditer(text):
            if "vTaskDelay(1)" in _body(text, m.end()):
                line = text.count("\n", 0, m.start()) + 1
                found.append("{}:{}: {}".format(path.name, line, m.group(0)))
    return found


class TickRateWaits(unittest.TestCase):
    def test_no_wait_is_a_count_of_ticks(self):
        self.assertEqual(counted_tick_waits(), [])

    def test_the_scan_finds_the_old_shape(self):
        # The check above passes on an empty match, so prove it can fail: the
        # host_stop() wait as it was before usbif#29.
        old = ("for (int i = 0; i < 200 && usbif_host_task_handle != NULL; i++) {\n"
               "    vTaskDelay(1);\n}\n")
        m = _COUNTED.search(old)
        self.assertIsNotNone(m)
        self.assertIn("vTaskDelay(1)", _body(old, m.end()))

    def test_a_millisecond_bound_is_not_flagged(self):
        new = ("const TickType_t stop_limit = USBIF_MS_TICKS(USBIF_HOST_STOP_WAIT_MS);\n"
               "for (TickType_t i = 0; i < stop_limit && usbif_host_task_handle != NULL; i++) {\n"
               "    vTaskDelay(1);\n}\n")
        self.assertIsNone(_COUNTED.search(new))


if __name__ == "__main__":
    unittest.main()
