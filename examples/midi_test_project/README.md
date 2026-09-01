# USBIF MIDI test project

A REAPER project for MIDI-out hardware testing against a usbif dev board.
No VST3 plugins, no rendering, no audio in this project — it exists purely
to push real MIDI traffic out a hardware port for ~2 minutes so the board's
receive side has something musical (not test tones) to chew on.

## What it is

`USBIF_MIDI_Test.RPP` — Canon in D (after Pachelbel): the authentic 1680
ground bass (D-A-B-F#-G-D-G-A) under a 3-voice strict canon, plus a soft
percussion pulse on the GM percussion channel. 66 BPM, 4/4, 33 bars, ends
on a sustained D major chord — a clean 120.000 second stop, not a cutoff
mid-phrase. Five tracks, five independent MIDI channels:

| Track | MIDI channel | Enters at | Role |
|---|---|---|---|
| Violin I (canon voice 1) | 1 | 0:00 | canon subject |
| Violin II (canon voice 2) | 2 | 0:14.5 | same subject, 4 bars behind |
| Violin III (canon voice 3) | 3 | 0:29.1 | same subject, 8 bars behind |
| Basso Continuo | 4 | 0:00 | the ground bass ostinato |
| Percussion (ch10) | **10** | 0:43.6 | soft triangle + maracas pulse |

The percussion track is on **channel 10** (GM percussion — `0x99`/`0x89`
status bytes, i.e. zero-indexed channel 9), required and verified: every
event on that track carries channel 10 in its status byte, and no other
track ever does.

The melodic subject (all three violins, identical, offset in time — a
true canon/round) is chord-tone-anchored against the ground bass at every
beat, so it isn't a note-for-note transcription of Pachelbel's original
violin part — it's an original canonic treatment over Pachelbel's
authentic, unambiguously-public-domain bass line. Public domain either
way; said here so nobody mistakes it for a scholarly transcription.

## How to open it

Just open `USBIF_MIDI_Test.RPP` in REAPER — double-click it, or File ->
Open Project. All routing is already configured; nothing else to set up.
Press Play. Every track's hardware MIDI output is enabled and pointed at
the **Espressif Device** MIDI output, on channel 0 (pass-through), so each
track's own channel (including the percussion track's channel 10) reaches
the wire unchanged — REAPER does not remap it.

## How to tell it's really reaching the board, not the GS synth

Open a track's routing (right-click the track -> "MIDI hardware output" or
the small MIDI-out icon in the track panel) and confirm it reads
**"Espressif Device"**. All five tracks should show the same device.

As of this project's construction, the Microsoft GS Wavetable Synth is
**not enabled** as a MIDI output in this REAPER install (Preferences ->
MIDI -> MIDI Outputs), so there is currently no path by which this
project could silently make audible PC sound while the board receives
nothing — if you hear it playing through your speakers instead of seeing
activity on the board, something has changed on this machine (the GS
synth got enabled, or the routing got knocked loose) and is worth
checking directly in Preferences before anything else.

## If the routing looks wrong

Device indices are assigned by driver enumeration order and shift
whenever a MIDI device is added or removed — this project will outlive
today's enumeration. If playback doesn't seem to reach the board:

1. Actions -> Show action list -> ReaScript: Load... and pick
   `USBIF_MIDI_Test_fix_routing.lua` in this directory.
2. Run it. It finds the MIDI output device whose name contains
   "Espressif" and repoints every track's hardware MIDI output there
   (channel 0 / pass-through, matching how this project is wired). If no
   such device is found, it changes nothing and lists what MIDI outputs
   REAPER does see, so you can tell a missing/powered-off board from a
   genuine routing bug.

## Regenerating it

`build_canon.py` (plain Python, no dependencies) generated
`USBIF_MIDI_Test.RPP` and is included here for provenance and so a
different tempo, device index, or musical shape doesn't require
hand-editing MIDI event bytes:

```
python3 build_canon.py USBIF_MIDI_Test.RPP
```

It also prints a track/channel/timing table and asserts, at generation
time, that every subject note is a genuine chord tone of the bass
harmony sounding under it — the composition is correct by construction,
not by ear. Re-running it assigns fresh GUIDs to every track and item
(REAPER requires them unique; a diff against the checked-in file will
show GUID churn even with no musical change — that's expected, not a
sign anything broke).
