#!/usr/bin/env python3
"""Generate USBIF_MIDI_Test.RPP: Canon in D (after Pachelbel) driving
hardware MIDI out for usbif board testing. Prints a verification report
to stdout and writes the .RPP to the path given on argv[1].
"""
import sys
import uuid

PPQ = 960              # ticks per quarter note
BPM = 66.0
TICKS_PER_BEAT = PPQ
TICKS_PER_BAR = PPQ * 4        # 4/4
SEC_PER_BEAT = 60.0 / BPM

DEVICE_INDEX = 1        # "Espressif Device" (confirmed via Preferences screenshot)
PASSTHROUGH_CHANNEL = 0  # 0 = all channels / pass-through unchanged
HWOUT_VALUE = PASSTHROUGH_CHANNEL | (DEVICE_INDEX << 5)   # 32

# ---------------------------------------------------------------------------
# Musical material
# ---------------------------------------------------------------------------

# Authentic Pachelbel ground bass (D-A-B-F#-G-D-G-A), one half note each,
# 8 half notes = 4 bars of 4/4. Octave 2 (D2=38).
BASS_PATTERN = [38, 45, 47, 42, 43, 38, 43, 45]
BASS_NOTE_DUR_TICKS = TICKS_PER_BEAT * 2   # half note

# Canon subject: 16 quarter notes (4 bars), every note a chord tone of the
# ground bass harmony sounding under it at that beat. Chords, 2 beats each:
#   bar1: D(D F# A) | A(A C# E)
#   bar2: Bm(B D F#) | F#m(F# A C#)
#   bar3: G(G B D)  | D(D F# A)
#   bar4: G(G B D)  | A(A C# E)
SUBJECT = [74, 69, 73, 76, 78, 74, 73, 69, 67, 71, 69, 66, 67, 74, 73, 76]
SUBJECT_CHORDS = [
    "D", "D", "A", "A", "Bm", "Bm", "F#m", "F#m",
    "G", "G", "D", "D", "G", "G", "A", "A",
]
CHORD_TONES = {
    "D": {74 % 12, 69 % 12, 78 % 12},   # D F# A pitch classes (any octave)
    "A": {69 % 12, 73 % 12, 76 % 12},   # A C# E
    "Bm": {71 % 12, 74 % 12, 78 % 12},  # B D F#
    "F#m": {78 % 12, 69 % 12, 73 % 12}, # F# A C#
    "G": {67 % 12, 71 % 12, 74 % 12},   # G B D
}
SUBJECT_NOTE_DUR_TICKS = TICKS_PER_BEAT   # quarter note
SUBJECT_LEN_TICKS = len(SUBJECT) * TICKS_PER_BEAT   # 15360 = 4 bars

assert SUBJECT_LEN_TICKS == 4 * TICKS_PER_BAR
assert len(BASS_PATTERN) * BASS_NOTE_DUR_TICKS == 4 * TICKS_PER_BAR

# verify every subject note actually IS a chord tone of its stated chord
for i, (n, ch) in enumerate(zip(SUBJECT, SUBJECT_CHORDS)):
    assert n % 12 in CHORD_TONES[ch], (i, n, ch)

N_BASS_CYCLES = 8          # bars 1-32
N_SUBJECT_CYCLES = 8       # violin I: bars 1-32 (matches bass cycles)
BARS_PER_CYCLE = 4
CODA_BAR = 32              # 0-indexed bar at which the coda starts (bar 33, 1-indexed)
PROJECT_END_TICKS = (CODA_BAR + 1) * TICKS_PER_BAR   # through end of coda bar
CODA_START_TICKS = CODA_BAR * TICKS_PER_BAR
CODA_LEN_TICKS = TICKS_PER_BAR   # one whole bar, whole-note chord

VIOLIN_ENTRY_BAR = {1: 0, 2: 4, 3: 8}     # violin I/II/III entry bars (0-indexed)
CODA_CHORD = {1: 74, 2: 78, 3: 81}         # D5 F#5 A5 -- close-position D major

PERCUSSION_START_BAR = 12   # bar 13 (1-indexed)
PERCUSSION_END_TICKS = CODA_START_TICKS   # silent through the coda
MARACAS = 70
TRIANGLE_OPEN = 81

PROJECT_TOTAL_SECONDS = PROJECT_END_TICKS / TICKS_PER_BEAT * SEC_PER_BEAT

GAP_TICKS = 12   # tiny separation between a note-off and the following note-on


def guid():
    return "{%s}" % str(uuid.uuid4()).upper()


def ticks_to_seconds(ticks):
    return ticks / TICKS_PER_BEAT * SEC_PER_BEAT


class Events:
    """Collects (abs_tick, order, bytes_hex) and renders as delta-tick E lines.
    order: note-offs (0) sort before note-ons (1) at the same tick."""

    def __init__(self):
        self.items = []

    def add(self, abs_tick, order, hexbytes):
        self.items.append((abs_tick, order, hexbytes))

    def note_on(self, tick, status, note, vel):
        self.add(tick, 1, "%02x %02x %02x" % (status, note, vel))

    def note_off(self, tick, status_on, note):
        # note-off uses 0x80-nibble status (not running-status note-on-vel0),
        # matching house style seen in Patch_Test.RPP.
        status_off = (status_on & 0x0F) | 0x80
        self.add(tick, 0, "%02x %02x 00" % (status_off, note))

    def program_change(self, tick, status_pc, program):
        self.add(tick, 1, "%02x %02x" % (status_pc, program))

    def cc(self, tick, status_cc, cc_num, val):
        self.add(tick, 1, "%02x %02x %02x" % (status_cc, cc_num, val))

    def render_lines(self, end_tick_for_final_cc, status_for_final_cc):
        self.items.sort(key=lambda t: (t[0], t[1]))
        lines = []
        cursor = 0
        for abs_tick, _order, hexbytes in self.items:
            lines.append("        E %d %s" % (abs_tick - cursor, hexbytes))
            cursor = abs_tick
        # trailing all-sound-off, matching house style
        final_tick = max(cursor, end_tick_for_final_cc)
        lines.append("        E %d %02x 7b 00" % (final_tick - cursor, 0xB0 | (status_for_final_cc & 0x0F)))
        return lines


def build_violin_track(voice_num, human_channel, track_name):
    """voice_num in {1,2,3}; human_channel 1-based MIDI channel for the
    status byte (this is the channel BAKED INTO the event bytes -- the
    track's own MIDIOUT is pass-through so this is what reaches the wire)."""
    status_nibble = (human_channel - 1) & 0x0F
    status_on = 0x90 | status_nibble
    status_pc = 0xC0 | status_nibble

    entry_bar = VIOLIN_ENTRY_BAR[voice_num]
    entry_tick_abs = entry_bar * TICKS_PER_BAR
    n_cycles = N_SUBJECT_CYCLES - (entry_bar // BARS_PER_CYCLE)

    ev = Events()
    ev.program_change(0, status_pc, 40)   # GM 40 = Violin

    t = 0  # relative to item start (== entry_tick_abs)
    for cyc in range(n_cycles):
        for note in SUBJECT:
            ev.note_on(t, status_on, note, 85)
            ev.note_off(t + SUBJECT_NOTE_DUR_TICKS - GAP_TICKS, status_on, note)
            t += SUBJECT_NOTE_DUR_TICKS
    subject_end_rel = t
    assert entry_tick_abs + subject_end_rel == CODA_START_TICKS, (
        voice_num, entry_tick_abs, subject_end_rel)

    # coda: sustained whole-note chord tone
    coda_note = CODA_CHORD[voice_num]
    coda_start_rel = subject_end_rel
    ev.note_on(coda_start_rel, status_on, coda_note, 75)
    coda_end_rel = coda_start_rel + CODA_LEN_TICKS
    ev.note_off(coda_end_rel - GAP_TICKS, status_on, coda_note)

    item_len_ticks = coda_end_rel
    lines = ev.render_lines(item_len_ticks, status_nibble)
    return {
        "name": track_name,
        "position_sec": ticks_to_seconds(entry_tick_abs),
        "length_sec": ticks_to_seconds(item_len_ticks),
        "lines": lines,
        "channel_1based": human_channel,
        "n_notes": n_cycles * len(SUBJECT) + 1,
    }


def build_bass_track():
    status_nibble = (4 - 1) & 0x0F   # channel 4
    status_on = 0x90 | status_nibble
    status_pc = 0xC0 | status_nibble

    ev = Events()
    ev.program_change(0, status_pc, 42)   # GM 42 = Cello (continuo)

    t = 0
    for cyc in range(N_BASS_CYCLES):
        for note in BASS_PATTERN:
            ev.note_on(t, status_on, note, 80)
            ev.note_off(t + BASS_NOTE_DUR_TICKS - GAP_TICKS, status_on, note)
            t += BASS_NOTE_DUR_TICKS
    assert t == CODA_START_TICKS

    coda_note = 38   # D2, tonic
    ev.note_on(t, status_on, coda_note, 70)
    coda_end = t + CODA_LEN_TICKS
    ev.note_off(coda_end - GAP_TICKS, status_on, coda_note)

    lines = ev.render_lines(coda_end, status_nibble)
    return {
        "name": "Basso Continuo",
        "position_sec": 0.0,
        "length_sec": ticks_to_seconds(coda_end),
        "lines": lines,
        "channel_1based": 4,
        "n_notes": N_BASS_CYCLES * len(BASS_PATTERN) + 1,
    }


def build_percussion_track():
    human_channel = 10
    status_nibble = (human_channel - 1) & 0x0F   # 9
    assert status_nibble == 9
    status_on = 0x90 | status_nibble
    status_pc = 0xC0 | status_nibble

    start_tick_abs = PERCUSSION_START_BAR * TICKS_PER_BAR
    n_cycles = (PERCUSSION_END_TICKS - start_tick_abs) // (BARS_PER_CYCLE * TICKS_PER_BAR)

    ev = Events()
    ev.program_change(0, status_pc, 0)   # GM percussion kit 0 = Standard Kit

    hit_dur = 90
    t = 0
    n_hits = 0
    for cyc in range(n_cycles):
        for i in range(8):   # 8 half-note pulses per 4-bar cycle, matching bass
            off = i * (TICKS_PER_BEAT * 2)
            if i == 0:
                note, vel = TRIANGLE_OPEN, 70
            else:
                note, vel = MARACAS, 55
            ev.note_on(t + off, status_on, note, vel)
            ev.note_off(t + off + hit_dur, status_on, note)
            n_hits += 1
        t += BARS_PER_CYCLE * TICKS_PER_BAR

    item_len_ticks = t
    assert start_tick_abs + item_len_ticks == PERCUSSION_END_TICKS
    lines = ev.render_lines(item_len_ticks, status_nibble)
    return {
        "name": "Percussion (ch10)",
        "position_sec": ticks_to_seconds(start_tick_abs),
        "length_sec": ticks_to_seconds(item_len_ticks),
        "lines": lines,
        "channel_1based": human_channel,
        "n_notes": n_hits,
    }


def track_chunk(track_guid, track_id_guid, name, midi_status_nibble_hint, track_data):
    lines = []
    lines.append("  <TRACK %s" % track_guid)
    lines.append('    NAME "%s"' % name)
    lines.append("    TRACKHEIGHT 0 0 0 0 0 0")
    lines.append("    VOLPAN 1 0 1 -1 1")
    lines.append("    MUTESOLO 0 0 0")
    lines.append("    NCHAN 2")
    lines.append("    FX 0")
    lines.append("    TRACKID %s" % track_id_guid)
    lines.append("    PERF 0")
    lines.append("    MIDIOUT %d -1" % HWOUT_VALUE)
    lines.append("    MAINSEND 1 0")
    lines.append("    <ITEM")
    lines.append("      POSITION %.9f" % track_data["position_sec"])
    lines.append("      SNAPOFFS 0")
    lines.append("      LENGTH %.9f" % track_data["length_sec"])
    lines.append("      LOOP 0")
    lines.append("      ALLTAKES 0")
    lines.append("      FADEIN 0 0 0 1 0 0 0")
    lines.append("      FADEOUT 0 0 0 1 0 0 0")
    lines.append("      MUTE 0 0")
    lines.append("      SEL 0")
    lines.append("      IGUID %s" % guid())
    lines.append("      IID %d" % 1)
    lines.append('      NAME "%s"' % name)
    lines.append("      VOLPAN 1 0 1 -1")
    lines.append("      SOFFS 0 0")
    lines.append("      PLAYRATE 1 1 0 -1 0 0.0025")
    lines.append("      CHANMODE 0")
    lines.append("      GUID %s" % guid())
    lines.append("      <SOURCE MIDI")
    lines.append("        HASDATA 1 %d QN" % PPQ)
    lines.append("        CCINTERP 32")
    lines.append("        POOLEDEVTS %s" % guid())
    lines.extend(track_data["lines"])
    lines.append("        CCINTERP 32")
    lines.append("        CHASE_CC_TAKEOFFS 1")
    lines.append("        GUID %s" % guid())
    lines.append("        IGNTEMPO 0 120 4 4")
    lines.append("        SRCCOLOR 2")
    lines.append("        EVTFILTER 0 -1 -1 -1 -1 0 0 0 0 -1 -1 -1 -1 0 -1 0 -1 -1")
    lines.append("      >")
    lines.append("    >")
    lines.append("  >")
    return lines


def build_project():
    tracks_data = [
        build_violin_track(1, 1, "Violin I (canon voice 1)"),
        build_violin_track(2, 2, "Violin II (canon voice 2)"),
        build_violin_track(3, 3, "Violin III (canon voice 3)"),
        build_bass_track(),
        build_percussion_track(),
    ]

    lines = []
    lines.append('<REAPER_PROJECT 0.1 "7.79" 0')
    lines.append("  RIPPLE 0")
    lines.append("  GROUPOVERRIDE 0 0 0")
    lines.append("  AUTOXFADE 129")
    lines.append("  TEMPO %.6f 4 4" % BPM)
    lines.append("  SAMPLERATE 48000 0 0")
    lines.append("  MASTER_VOLUME 1 0 -1 -1 1")
    lines.append("  MASTER_NCH 2 2")
    lines.append("  <TEMPOENVEX")
    lines.append("    EGUID %s" % guid())
    lines.append("    ACT 1 -1")
    lines.append("    VIS 1 0 1")
    lines.append("    LANEHEIGHT 0 0")
    lines.append("    ARM 0")
    lines.append("    DEFSHAPE 1 -1 -1")
    lines.append("    PT 0.000000000 %.6f 1" % BPM)
    lines.append("  >")

    for td in tracks_data:
        tg = guid()
        tid = guid()
        lines.extend(track_chunk(tg, tid, td["name"], td["channel_1based"], td))

    lines.append(">")
    text = "\n".join(lines) + "\n"
    return text, tracks_data


def main():
    out_path = sys.argv[1] if len(sys.argv) > 1 else None
    text, tracks_data = build_project()
    if out_path:
        with open(out_path, "w") as f:
            f.write(text)
        print("wrote %s (%d bytes)" % (out_path, len(text)))

    print()
    print("TEMPO=%.3f BPM, PPQ=%d, HWOUT_VALUE=%d (device=%d, channel=%d pass-through)"
          % (BPM, PPQ, HWOUT_VALUE, DEVICE_INDEX, PASSTHROUGH_CHANNEL))
    print("Project total length: %.6f s (bar count = %d, incl. 1-bar coda)"
          % (PROJECT_TOTAL_SECONDS, CODA_BAR + 1))
    print()
    print("%-28s %-8s %10s %10s %8s" % ("track", "ch(1b)", "start(s)", "len(s)", "n_notes"))
    for td in tracks_data:
        print("%-28s %-8d %10.3f %10.3f %8d"
              % (td["name"], td["channel_1based"], td["position_sec"], td["length_sec"], td["n_notes"]))


if __name__ == "__main__":
    raise SystemExit(main())
