-- USBIF_MIDI_Test_fix_routing.lua
--
-- Portability net, not a doubt-correction: MIDI output device indices are
-- assigned by driver enumeration order and shift whenever a USB MIDI
-- device is plugged, unplugged, or re-enumerated on this machine. This
-- project was built assuming "Espressif Device" enumerates at output
-- index 1 (confirmed at build time via REAPER Preferences -> MIDI
-- Outputs). That mapping is a snapshot of one moment, and this project
-- will outlive it -- if a later session's enumeration differs, every
-- track's hardware MIDI output would silently point at the wrong device.
--
-- Run this once from Actions -> Show action list -> ReaScript: Load...
-- (pick this file), then run it, any time playback doesn't seem to be
-- reaching the board. It resolves "Espressif Device" BY NAME across
-- every track in the project and repoints I_MIDIHWOUT there, on channel
-- 0 (pass-through) -- exactly how every track in this project is meant
-- to be wired, so each item's own MIDI channel (including the
-- percussion track's channel 10) survives to the wire unchanged.

local TARGET_NAME_SUBSTR = "espressif"   -- matched case-insensitively

local function find_device_index()
  local n = reaper.GetNumMIDIOutputs()
  for i = 0, n - 1 do
    local present, name = reaper.GetMIDIOutputName(i, "")
    if present and name ~= "" and name:lower():find(TARGET_NAME_SUBSTR, 1, true) then
      return i, name
    end
  end
  return nil
end

local function list_devices()
  local n = reaper.GetNumMIDIOutputs()
  local lines = {}
  for i = 0, n - 1 do
    local present, name = reaper.GetMIDIOutputName(i, "")
    if present and name ~= "" then
      lines[#lines + 1] = string.format("  [%d] %s", i, name)
    end
  end
  if #lines == 0 then
    return "  (none reported)"
  end
  return table.concat(lines, "\n")
end

local dev_index, dev_name = find_device_index()

if not dev_index then
  reaper.ShowMessageBox(
    "No MIDI output device matching \"" .. TARGET_NAME_SUBSTR .. "\" was found.\n\n" ..
    "Available MIDI outputs:\n" .. list_devices() .. "\n\n" ..
    "Routing was NOT changed. Plug in / power up the board (and enable\n" ..
    "its output in Preferences -> MIDI -> MIDI Outputs if needed), then\n" ..
    "re-run this script.",
    "USBIF MIDI Test: fix routing", 0)
  return
end

reaper.Undo_BeginBlock()

local PASSTHROUGH_CHANNEL = 0                          -- 0 = all channels, unmodified
local new_value = PASSTHROUGH_CHANNEL + (dev_index * 32) -- low 5 bits=channel, next 5=device

local n_tracks = reaper.CountTracks(0)
for i = 0, n_tracks - 1 do
  local track = reaper.GetTrack(0, i)
  reaper.SetMediaTrackInfo_Value(track, "I_MIDIHWOUT", new_value)
end

reaper.Undo_EndBlock("Fix MIDI hardware output routing to " .. dev_name, -1)
reaper.UpdateArrange()

reaper.ShowMessageBox(
  string.format(
    "Routed %d track(s) to MIDI output [%d] %s\n(channel 0 / pass-through, I_MIDIHWOUT=%d).",
    n_tracks, dev_index, dev_name, new_value),
  "USBIF MIDI Test: fix routing", 0)
