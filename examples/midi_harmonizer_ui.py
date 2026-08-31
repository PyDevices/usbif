# deps: lvgl
"""The MIDI harmonizer with a control surface: the board's touchscreen
picks the chord and the direction while notes flow.

Same contract as midi_harmonizer.py -- takes whatever MIDI it is handed,
embellishes, sends it back out -- but HARMONY is now live-editable from
two button rows:

    chord:      Major | Minor | 7th | Power | Octave
    direction:  Up | Down | Both

Changing either sends All Notes Off first, so no harmony ever outlives
the setting that created it. The MIDI pump rides an LVGL timer inside
the display loop; deploy standalone as /main.py, or `mpftp run` it.
"""

import display_driver  # noqa: F401 -- wires LVGL flush + input + event_loop
import lvgl as lv

import _usbif

CHORDS = (
    ("Major", (4, 7)),
    ("Minor", (3, 7)),
    ("7th", (4, 7, 10)),
    ("Power", (7,)),
    ("Octave", (12,)),
)
DIRECTIONS = ("Up", "Down", "Both")

_state = {"chord": 0, "direction": 0}
_harmony = ()
_styles = []


def _rebuild_harmony():
    global _harmony
    base = CHORDS[_state["chord"]][1]
    d = DIRECTIONS[_state["direction"]]
    ivs = []
    if d in ("Up", "Both"):
        ivs.extend(base)
    if d in ("Down", "Both"):
        ivs.extend(-iv for iv in base)
    _harmony = tuple(ivs)
    # No harmony may outlive the setting that created it.
    _usbif.midi_write(bytes([0xB0, 123, 0]))


_rebuild_harmony()

_rx = bytearray(64)
_parse = {"status": 0, "need": 0, "have": 0, "d0": 0}


def _emit(status, note, vel):
    _usbif.midi_write(bytes([status, note, vel]))
    for iv in _harmony:
        h = note + iv
        if 0 <= h <= 127:
            _usbif.midi_write(bytes([status, h, vel]))


def _pump(_t):
    # Runs from LVGL's timer: drain everything waiting, parse with
    # running-status tolerance, harmonize notes, pass the rest through.
    while True:
        n = _usbif.midi_read(_rx)
        if n <= 0:
            return
        for i in range(n):
            b = _rx[i]
            if b >= 0xF8:
                _usbif.midi_write(bytes([b]))
            elif b >= 0x80:
                _parse["status"] = 0 if b >= 0xF0 else b
                _parse["need"] = 1 if 0xC0 <= b <= 0xDF else 2
                _parse["have"] = 0
            elif _parse["status"]:
                if _parse["have"] == 0 and _parse["need"] == 2:
                    _parse["d0"] = b
                    _parse["have"] = 1
                else:
                    _parse["have"] = 0
                    st = _parse["status"]
                    kind = st & 0xF0
                    if _parse["need"] == 1:
                        _usbif.midi_write(bytes([st, b]))
                    elif kind in (0x80, 0x90):
                        _emit(st, _parse["d0"], b)
                    else:
                        _usbif.midi_write(bytes([st, _parse["d0"], b]))


def _matrix(parent, options, key, y_ofs, status_lbl):
    m = lv.buttonmatrix(parent)
    m.set_map(list(options) + [""])
    m.set_button_ctrl_all(lv.buttonmatrix.CTRL.CHECKABLE)
    m.set_one_checked(True)
    m.set_button_ctrl(0, lv.buttonmatrix.CTRL.CHECKED)
    m.set_size(lv.pct(94), 130)
    m.align(lv.ALIGN.TOP_MID, 0, y_ofs)

    def on_change(_e):
        sel = m.get_selected_button()
        if 0 <= sel < len(options):
            _state[key] = sel
            _rebuild_harmony()
            status_lbl.set_text("intervals: " + (" ".join(
                "%+d" % iv for iv in _harmony) or "(none)"))

    m.add_event_cb(on_change, lv.EVENT.VALUE_CHANGED, None)
    return m


def build_ui():
    inst = display_driver.event_loop.current_instance()
    if inst is not None:
        inst.disable()
    try:
        scr = lv.screen_active()
        title = lv.label(scr)
        title.set_text("MIDI Harmonizer")
        title.align(lv.ALIGN.TOP_MID, 0, 18)

        status = lv.label(scr)
        status.align(lv.ALIGN.BOTTOM_MID, 0, -24)
        status.set_text("intervals: +4 +7")

        _matrix(scr, tuple(c[0] for c in CHORDS), "chord", 70, status)
        _matrix(scr, DIRECTIONS, "direction", 230, status)

        lv.timer_create(_pump, 5, None)
    finally:
        if inst is not None:
            inst.enable()


build_ui()
print("harmonizer ui up")
