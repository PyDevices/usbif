# SPDX-FileCopyrightText: 2026 Brad Barnett
#
# SPDX-License-Identifier: MIT
"""Put usbif's Python half, and the pydevices trees it leans on, on sys.path.

usbif ships both halves of itself: the native module built from ``src/`` and
the portable API in ``lib/usbif`` that fronts it. The tests exercise the
Python half, so this puts it first.

``pydevices`` is added after it because the audio-facing parts of the API
implement contracts that live there -- ``usbif.uac_audio`` returns objects a
caller uses as ``audiodev.PCMOutput`` / ``PCMInput``, and asserting that is
the point of those tests rather than an incidental import. The path is
resolved as a sibling checkout and simply skipped when it is absent, so the
descriptor tests still run in a clone of this repository alone.
"""

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]

_paths = [ROOT / "lib"]

_pydevices = ROOT.parent / "pydevices"
if _pydevices.is_dir():
    _paths.extend([_pydevices / "lib", _pydevices / "utils"])

for _p in _paths:
    _s = str(_p)
    if _p.is_dir() and _s not in sys.path:
        sys.path.insert(0, _s)
