# Device MIDI IN: prove the board receives MIDI from a host.
#
# This is the last open corner of M3's four-way MIDI matrix. The other
# three are closed: the board plays a DAW and a hardware GM box (device
# MIDI OUT), reads a keyboard (host MIDI IN), and drives an instrument
# (host MIDI OUT). What has never been exercised is the direction a
# software instrument needs most -- a host sequencer playing *into* the
# board, so `audioif` can render what a DAW sends it.
#
# The board wears a pure-MIDI costume, so on the host it is a MIDI
# device and nothing else. Anything that can send MIDI drives it: a DAW
# track whose output is this board, or a keyboard routed through one.
#
# Run it with the REPL on the UART bridge, not on native USB -- the
# costume change drops CDC, and a REPL living on the port under test
# would be cut off mid-run.
#
#   mpftp run usbif/examples/midi_device_in.py -d COM49 --follow --timeout 90
#
# The tally at the end is the evidence: which message types arrived,
# how many of each, and whether the parser ever fell out of sync. A
# clean run shows note-ons with a spread of velocities (not a column of
# 64s, which would mean the source is sending a fixed velocity and the
# velocity byte is never really being tested), note-offs matching them,
# and zero desync bytes.

import time

import _usbif

DURATION_MS = 20000       # how long to listen once the host has mounted us
MOUNT_TIMEOUT_MS = 10000  # how long to wait for the host to configure us
BUF = bytearray(256)

# Data bytes carried by each channel-voice status, indexed by high nibble.
VOICE_LEN = {0x8: 2, 0x9: 2, 0xA: 2, 0xB: 2, 0xC: 1, 0xD: 1, 0xE: 2}
# System-common lengths; realtime (0xF8-0xFF) is zero and may interleave
# anywhere, including inside another message's data bytes.
COMMON_LEN = {0xF1: 1, 0xF2: 2, 0xF3: 1}

NAMES = {
    0x8: "note-off", 0x9: "note-on", 0xA: "aftertouch",
    0xB: "cc", 0xC: "program", 0xD: "pressure", 0xE: "pitch-bend",
}


class Parser:
    """Plain MIDI 1.0 byte stream in, complete messages out.

    TinyUSB hands us unpacked USB-MIDI events, so every message should
    arrive whole and running status should never appear. The parser
    supports it anyway: if the assumption is ever wrong, the evidence
    should be a correct tally rather than a pile of desync bytes hiding
    a framing bug.
    """

    def __init__(self):
        self.status = 0
        self.data = []
        self.want = 0
        self.in_sysex = False
        self.desync = 0
        self.messages = []

    def feed(self, buf, n):
        for i in range(n):
            self._byte(buf[i])

    def _byte(self, b):
        if b >= 0xF8:                      # realtime: interleaves, no state
            self.messages.append((b, ()))
            return
        if self.in_sysex:
            if b == 0xF7:
                self.in_sysex = False
                self.messages.append((0xF7, ()))
            elif b >= 0x80:                # a status byte aborts sysex
                self.in_sysex = False
                self._byte(b)
            return
        if b >= 0x80:
            if b == 0xF0:
                self.in_sysex = True
                return
            high = b >> 4
            if high == 0xF:
                self.want = COMMON_LEN.get(b, 0)
                self.status = b if self.want else 0
                self.data = []
                if not self.want:
                    self.messages.append((b, ()))
            else:
                self.status = b
                self.want = VOICE_LEN[high]
                self.data = []
            return
        if not self.status:                # data with no status: desync
            self.desync += 1
            return
        self.data.append(b)
        if len(self.data) == self.want:
            self.messages.append((self.status, tuple(self.data)))
            self.data = []
            # Running status: a channel-voice status stays armed.
            if self.status >= 0xF0:
                self.status = 0

    def drain(self):
        out = self.messages
        self.messages = []
        return out


def wait_for_mount():
    deadline = time.ticks_add(time.ticks_ms(), MOUNT_TIMEOUT_MS)
    while time.ticks_diff(deadline, time.ticks_ms()) > 0:
        connected, mounted, _ = _usbif.dev_state()
        if mounted:
            return True
        time.sleep_ms(50)
    connected, mounted, suspended = _usbif.dev_state()
    print("not mounted after {} ms: connected={} mounted={} suspended={}".format(
        MOUNT_TIMEOUT_MS, connected, mounted, suspended))
    if not connected:
        # No bus reset ever seen. Nothing above this layer can help.
        print("  connected=False means no host is on the native USB port at all --")
        print("  check the cable is a data cable and is in the native USB jack,")
        print("  not only the UART bridge.")
    return False


def main():
    restore = _usbif.dev_functions()
    built = _usbif.dev_functions_built()
    if not (built & _usbif.FN_MIDI):
        print("this firmware has no MIDI device function built in")
        return False

    counts = {}
    velocities = set()
    channels = set()
    notes_on = 0
    notes_off = 0
    bend_lo = 8192
    bend_hi = 8192
    parser = Parser()
    total_bytes = 0

    try:
        _usbif.dev_functions(_usbif.FN_MIDI)
        print("costume: midi only -- look for the board as a MIDI device on the host")
        if not wait_for_mount():
            return False
        print("mounted. play into it for {} s ...".format(DURATION_MS // 1000))

        deadline = time.ticks_add(time.ticks_ms(), DURATION_MS)
        while time.ticks_diff(deadline, time.ticks_ms()) > 0:
            n = _usbif.midi_read(BUF)
            if not n:
                time.sleep_ms(2)
                continue
            total_bytes += n
            parser.feed(BUF, n)
            for status, data in parser.drain():
                high = status >> 4
                name = NAMES.get(high, "system")
                counts[name] = counts.get(name, 0) + 1
                if high < 0xF:
                    channels.add(status & 0x0F)
                if high == 0x9 and len(data) == 2 and data[1]:
                    notes_on += 1
                    velocities.add(data[1])
                    print("  note-on  ch{:<2} note {:<3} vel {}".format(
                        (status & 0x0F) + 1, data[0], data[1]))
                elif high == 0x8 or (high == 0x9 and len(data) == 2 and not data[1]):
                    # A note-on with velocity 0 is a note-off; count it as one.
                    notes_off += 1
                elif high == 0xE and len(data) == 2:
                    value = data[0] | (data[1] << 7)
                    bend_lo = min(bend_lo, value)
                    bend_hi = max(bend_hi, value)
    finally:
        _usbif.dev_functions(restore)

    print()
    print("--- device MIDI IN ---")
    print("bytes read      {}".format(total_bytes))
    for name in sorted(counts):
        print("{:<15} {}".format(name, counts[name]))
    print("notes on/off    {} / {}".format(notes_on, notes_off))
    print("channels seen   {}".format(
        sorted(c + 1 for c in channels) if channels else "none"))
    print("velocities      {} distinct{}".format(
        len(velocities),
        " (min {} max {})".format(min(velocities), max(velocities))
        if velocities else ""))
    if bend_hi != bend_lo:
        print("pitch bend      {} .. {}".format(bend_lo, bend_hi))
    print("desync bytes    {}".format(parser.desync))
    ok = total_bytes > 0 and parser.desync == 0
    print("result          {}".format("PASS" if ok else "no traffic" if not total_bytes else "FAIL"))
    return ok


main()
