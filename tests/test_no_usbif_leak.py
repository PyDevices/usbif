# SPDX-FileCopyrightText: 2026 Brad Barnett
#
# SPDX-License-Identifier: MIT
"""`_usbif` is package-internal, and this is what keeps it that way.

The C usermod is called `_usbif` and always will be: MicroPython cannot
register it as `usbif` while a frozen Python package also occupies that name.
That is a build constraint, not an invitation -- an application reaching past
`usbif.auto` into the C module is writing code that runs on exactly one
firmware, and it was how every example in this repository was written.

So the rule is drawn here rather than left to reviewers: three files in
`lib/usbif/` may import the C module, and nothing an application reads --
examples, the root README's snippets, the package README -- may name it at
all. The test walks the files instead of trusting a convention, because the
twenty examples that broke this rule each looked perfectly reasonable alone.

Companion assertions:

* every wrapped method raises a *useful* ImportError off-target, rather than
  AttributeError on ``None``, which is what an ordinary desktop Python would
  otherwise get from the facade;
* the HID report ids the facade falls back to still match the C module's,
  because a fallback that silently disagrees with its source is the shape of
  bug this whole exercise exists to stop.

PyDevices/usbif#10.
"""

import pathlib
import re
import unittest

import _env  # noqa: F401

from usbif import native_usb


ROOT = pathlib.Path(__file__).resolve().parents[1]

# The three files that front the C module, and the only ones allowed to name
# it. `auto.py` is deliberately not on this list: it asks whether the module
# can be imported, by name, and never imports it.
ALLOWED = {
    ROOT / "lib" / "usbif" / "native_usb.py",
    ROOT / "lib" / "usbif" / "native_midi.py",
    ROOT / "lib" / "usbif" / "uac_audio.py",
}

_IMPORT = re.compile(r"^\s*(?:import\s+_usbif|from\s+_usbif\s+import)", re.M)
_ANY_USE = re.compile(r"\b_usbif\s*\.")


def _fenced_python(text):
    """The ```python blocks of a markdown file, joined."""
    return "\n".join(re.findall(r"```python\n(.*?)```", text, re.S))


class NoLeakTests(unittest.TestCase):
    def test_no_example_imports_the_c_module(self):
        offenders = []
        for path in sorted((ROOT / "examples").rglob("*.py")):
            text = path.read_text()
            if _IMPORT.search(text) or _ANY_USE.search(text):
                offenders.append(path.relative_to(ROOT).as_posix())
        self.assertEqual(
            offenders, [],
            "these examples reach into the C module; go through usbif.auto: "
            + ", ".join(offenders))

    def test_no_application_facing_doc_shows_the_c_module(self):
        offenders = []
        for rel in ("README.md", "examples/README.md", "lib/usbif/README.md"):
            path = ROOT / rel
            if not path.is_file():
                continue
            code = _fenced_python(path.read_text())
            if _IMPORT.search(code) or _ANY_USE.search(code):
                offenders.append(rel)
        self.assertEqual(
            offenders, [],
            "these docs teach applications to import the C module: "
            + ", ".join(offenders))

    def test_only_three_package_files_import_the_c_module(self):
        found = set()
        for path in sorted((ROOT / "lib" / "usbif").rglob("*.py")):
            if _IMPORT.search(path.read_text()):
                found.add(path)
        unexpected = sorted(p.relative_to(ROOT).as_posix() for p in found - ALLOWED)
        self.assertEqual(
            unexpected, [],
            "only native_usb, native_midi and uac_audio may import _usbif: "
            + ", ".join(unexpected))


class FacadeWithoutTheModuleTests(unittest.TestCase):
    """Off-target, every wrapped call must explain itself, not blow up.

    ``native_usb._usbif`` is ``None`` in a desktop Python, so an unguarded
    passthrough would raise ``AttributeError: 'NoneType' object has no
    attribute ...`` -- true, and useless. Each wrapper goes through
    ``_require()`` instead, and this asserts it for every one rather than for
    a sample, because a wrapper added later without the guard is exactly the
    regression that would slip through a sample.
    """

    HOST_METHODS = (
        "desc", "stats", "port_cycle", "intr_dump",
        "msc_open", "msc_info", "msc_read", "msc_write", "msc_close",
        "msc_diag", "msc_provoke",
        "uvc_negotiate", "uvc_open", "uvc_read_frame", "uvc_frame_ready",
        "uvc_stats", "uvc_close",
        "cdc_open", "cdc_read", "cdc_write", "cdc_close",
        "hid_open", "hid_read", "hid_close",
    )

    DEVICE_METHODS = (
        "state", "reinit", "pid", "desc_check", "builtin_desc_cfg",
        "functions", "functions_available",
        "msc_attach", "msc_attach_blockdev", "msc_detach", "msc_status",
        "msc_buffer", "msc_bd_stats",
        "hid_send", "hid_leds",
        "uac_enable", "uac_pump_start", "uac_pump_stop", "uac_pump_stats",
        "uac_pump_rate",
        "uac_available", "uac_volume", "uac_read", "uac_stats",
        "uvc_format", "uvc_reset", "uvc_streaming", "uvc_ready",
        "uvc_submit", "uvc_stats",
    )

    def setUp(self):
        if native_usb._usbif is not None:  # pragma: no cover - on-target only
            self.skipTest("the C module is present; this asserts its absence")

    def _assert_raises_importerror(self, obj, names):
        for name in names:
            with self.subTest(method=name):
                with self.assertRaises(ImportError):
                    getattr(obj, name)()

    def test_every_host_method_raises_importerror(self):
        self._assert_raises_importerror(native_usb.NativeHost(), self.HOST_METHODS)

    def test_every_device_method_raises_importerror(self):
        self._assert_raises_importerror(native_usb.NativeDevice(), self.DEVICE_METHODS)

    def test_the_wrapper_lists_cover_the_classes(self):
        """The two lists above must not drift behind the classes they check.

        Without this, adding an unguarded wrapper and forgetting to list it
        leaves the suite green -- the absence-reads-as-agreement shape.
        """
        for cls, listed, base in (
            # capabilities() answers off-target by design; partition() takes
            # arguments and has its own case below.
            (native_usb.NativeHost, self.HOST_METHODS,
             ("capabilities", "started", "partition")),
            (native_usb.NativeDevice, self.DEVICE_METHODS, ()),
        ):
            public = {
                n for n in vars(cls)
                if not n.startswith("_") and callable(vars(cls)[n])
            }
            missing = sorted(public - set(listed) - set(base))
            self.assertEqual(
                missing, [],
                "{}: wrappers not covered by this test: {}".format(
                    cls.__name__, ", ".join(missing)))

    def test_partition_raises_importerror(self):
        """``partition()`` asks the drive for its block size, so it needs one."""
        with self.assertRaises(ImportError):
            native_usb.NativeHost().partition(2048, 1024)

    def test_capabilities_is_empty_rather_than_raising(self):
        """The one honest exception: asking what is supported always answers.

        ``usbif.auto`` promises a caller can branch on ``capabilities()``
        without guarding an import, so this call must not be behind
        ``_require()``.
        """
        self.assertEqual(native_usb.NativeHost().capabilities(), frozenset())


class HidReportIdTests(unittest.TestCase):
    """The facade's fallback report ids against the C module's own."""

    def test_fallbacks_match_the_c_module(self):
        source = (ROOT / "src" / "mod_usbif.c").read_text()
        found = dict(
            (name, int(value))
            for name, value in re.findall(
                r"MP_ROM_QSTR\(MP_QSTR_(HID_KEYBOARD|HID_MOUSE)\),"
                r"\s*MP_ROM_INT\((\d+)\)", source)
        )
        self.assertEqual(
            sorted(found), ["HID_KEYBOARD", "HID_MOUSE"],
            "no HID report ids found in src/mod_usbif.c")
        self.assertEqual(native_usb.NativeDevice.HID_KEYBOARD, found["HID_KEYBOARD"])
        self.assertEqual(native_usb.NativeDevice.HID_MOUSE, found["HID_MOUSE"])


if __name__ == "__main__":
    unittest.main()
