"""USB HID boot-keyboard reports to PyDevices key events.

This is the policy layer M1 needs. The host stack delivers raw HID reports --
correctly, and deliberately: a class driver should not decide what a keypress
*means*. But a raw report is not a keypress, and M1's promise is that a USB
keyboard on a board drives an application through the ordinary event system,
producing the same ``events.Key`` records an SDL keyboard produces on the
desktop. Without this translation "the same application code" is not true, so
the milestone is not met by delivering reports.

Boot-protocol report layout, the one every keyboard supports without parsing a
report descriptor::

    byte 0   modifier bitmask (L/R of ctrl, shift, alt, gui)
    byte 1   reserved
    bytes 2-7 up to six concurrently-held HID usage ids, unordered

**Keys are reported as a set, not as events.** The keyboard says what is held
*now*; it never says "pressed" or "released". Events come from diffing
successive reports, which is why this class holds state and why one instance
must follow one keyboard.

**Rollover is reported, not silently dropped.** When more keys are held than
the report can carry, a keyboard fills every slot with ``0x01``
(ErrorRollOver). Treating that as six real keypresses would emit garbage; the
report is ignored and the previously-held set left untouched, so the eventual
release still balances.
"""

import events
import keys

# HID usage id -> SDL-style keycode. Boot-protocol range only: everything a
# keyboard can send in bytes 2-7 of a boot report. Values follow keys.py,
# which follows SDL, so an application comparing against keys.K_a works
# identically whether the key came from USB or from SDL.
_USAGE = {}


def _fill():
    # a-z: usages 0x04..0x1D map to lowercase ASCII, exactly as SDL reports
    # unshifted letters.
    for i in range(26):
        _USAGE[0x04 + i] = keys.K_a + i
    # 1-9 then 0: usages 0x1E..0x27. The zero is last in HID and first in
    # ASCII, which is the off-by-one this loop exists to get right.
    for i in range(9):
        _USAGE[0x1E + i] = ord("1") + i
    _USAGE[0x27] = ord("0")
    _USAGE.update({
        0x28: keys.K_RETURN,
        0x29: keys.K_ESCAPE,
        0x2A: keys.K_BACKSPACE,
        0x2B: keys.K_TAB,
        0x2C: keys.K_SPACE,
        0x2D: ord("-"), 0x2E: ord("="), 0x2F: ord("["), 0x30: ord("]"),
        0x31: ord("\\"), 0x33: ord(";"), 0x34: ord("'"), 0x35: ord("`"),
        0x36: ord(","), 0x37: ord("."), 0x38: ord("/"),
    })
    # F1-F12 are contiguous in both encodings.
    for i in range(12):
        _USAGE[0x3A + i] = keys.K_F1 + i
    # Arrows: HID orders them right, left, down, up.
    _USAGE.update({
        0x4F: keys.K_RIGHT, 0x50: keys.K_LEFT,
        0x51: keys.K_DOWN, 0x52: keys.K_UP,
    })
    for name, usage in (("K_INSERT", 0x49), ("K_HOME", 0x4A), ("K_PAGEUP", 0x4B),
                        ("K_DELETE", 0x4C), ("K_END", 0x4D), ("K_PAGEDOWN", 0x4E),
                        ("K_CAPSLOCK", 0x39), ("K_PRINTSCREEN", 0x46),
                        ("K_SCROLLLOCK", 0x47), ("K_PAUSE", 0x48)):
        code = getattr(keys, name, None)
        if code is not None:
            _USAGE[usage] = code


_fill()
del _fill

# Modifier bit -> (KMOD mask, keycode, HID usage), in report bit order.
#
# Modifiers need all three because HID reports them only as a bitmask, never
# in the usage array -- so pressing Shift alone produces a report with no
# usages at all. SDL emits a KEYDOWN for the modifier key itself, and M1's
# promise is the same events as SDL, so these must be diffed and emitted too.
# Their usage ids (0xE0-0xE7) are what a keyboard would send if modifiers
# appeared in the array, and are the honest scancode to report.
_MODIFIER_KEYS = tuple(
    (bit, kmod, getattr(keys, name, None), usage)
    for bit, kmod, name, usage in (
        (0x01, keys.KMOD_LCTRL, "K_LCTRL", 0xE0),
        (0x02, keys.KMOD_LSHIFT, "K_LSHIFT", 0xE1),
        (0x04, keys.KMOD_LALT, "K_LALT", 0xE2),
        (0x08, getattr(keys, "KMOD_LGUI", 0), "K_LGUI", 0xE3),
        (0x10, keys.KMOD_RCTRL, "K_RCTRL", 0xE4),
        (0x20, keys.KMOD_RSHIFT, "K_RSHIFT", 0xE5),
        (0x40, keys.KMOD_RALT, "K_RALT", 0xE6),
        (0x80, getattr(keys, "KMOD_RGUI", 0), "K_RGUI", 0xE7),
    )
)

# Modifier bit -> keys.KMOD_* mask, in report bit order.
_MODIFIERS = (
    (0x01, keys.KMOD_LCTRL), (0x02, keys.KMOD_LSHIFT),
    (0x04, keys.KMOD_LALT), (0x08, getattr(keys, "KMOD_LGUI", 0)),
    (0x10, keys.KMOD_RCTRL), (0x20, keys.KMOD_RSHIFT),
    (0x40, keys.KMOD_RALT), (0x80, getattr(keys, "KMOD_RGUI", 0)),
)

ERROR_ROLLOVER = 0x01


def modifier_mask(report_byte):
    """The ``keys.KMOD_*`` mask for a boot report's modifier byte."""
    mask = keys.KMOD_NONE
    for bit, kmod in _MODIFIERS:
        if report_byte & bit:
            mask |= kmod
    return mask


def keycode(usage):
    """SDL-style keycode for a HID usage id, or ``None`` if unmapped."""
    return _USAGE.get(usage)


class KeyboardDecoder:
    """Diffs successive boot reports into ``events.Key`` records.

    One instance per keyboard: the state it holds is that keyboard's currently
    pressed set, and feeding two keyboards through one decoder would report
    each other's keys as released.
    """

    def __init__(self):
        self._held = frozenset()
        self._mod = keys.KMOD_NONE
        self._mod_bits = 0

    def feed(self, report):
        """Translate one report, returning a tuple of ``events.Key``.

        Order is releases first, then presses. A key swapped between two
        reports is then seen up-then-down, which is the order an application
        holding "what is pressed" state needs to stay consistent.
        """
        if len(report) < 3:
            return ()
        usages = report[2:8]
        if all(u == ERROR_ROLLOVER for u in usages if u):
            if usages and usages[0] == ERROR_ROLLOVER:
                # More keys held than the report can carry. Emitting six
                # ErrorRollOver "keys" would be garbage, and treating it as
                # "nothing held" would emit a release for every real key still
                # physically down. Leave the held set alone.
                return ()
        now = frozenset(u for u in usages if u and u != ERROR_ROLLOVER)
        mod = modifier_mask(report[0])

        out = []
        # Modifier releases first, then ordinary releases, then ordinary
        # presses, then modifier presses. That ordering means a chord always
        # reads as modifier-down before the key it modifies, and
        # key-up before modifier-up -- the sequence an application
        # reconstructing chords expects, and the one SDL produces.
        raw_mod = report[0]
        for bit, _kmod, code, usage in _MODIFIER_KEYS:
            was, is_now = self._mod_bits & bit, raw_mod & bit
            if was and not is_now and code is not None:
                out.append(events.Key(events.KEYUP, keys.keyname(code), code, mod, usage, None))
        for usage in sorted(self._held - now):
            out.append(self._event(events.KEYUP, usage, mod))
        for usage in sorted(now - self._held):
            out.append(self._event(events.KEYDOWN, usage, mod))
        for bit, _kmod, code, usage in _MODIFIER_KEYS:
            was, is_now = self._mod_bits & bit, raw_mod & bit
            if is_now and not was and code is not None:
                out.append(events.Key(events.KEYDOWN, keys.keyname(code), code, mod, usage, None))
        self._held = now
        self._mod = mod
        self._mod_bits = raw_mod
        return tuple(out)

    def _event(self, kind, usage, mod):
        code = keycode(usage)
        return events.Key(
            kind,
            keys.keyname(code) if code is not None else "Unknown",
            code,
            mod,
            usage,          # the HID usage is the hardware scancode
            None,           # no window: this did not come from a display
        )

    @property
    def held(self):
        """HID usage ids currently pressed."""
        return self._held

    @property
    def modifiers(self):
        return self._mod
