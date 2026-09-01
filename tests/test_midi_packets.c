// SPDX-License-Identifier: MIT
//
// Host-side tests for the USB-MIDI packet codec. No MicroPython, no IDF, no
// board -- and written before the driver ever met an instrument, because the
// packing rules are the part most likely to be wrong and the part hardest to
// see on-target: a mis-packed note-on is a stuck note on someone's
// synthesiser, not an error message.
#include "shared/usbif_midi_packet.h"

#include <stdio.h>
#include <string.h>

static int failures = 0;

#define CHECK(cond) do { \
    if (!(cond)) { \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
        failures++; \
    } \
} while (0)

static void check_bytes(const char *what, const uint8_t *got, size_t got_len,
    const uint8_t *want, size_t want_len) {
    if (got_len != want_len || memcmp(got, want, want_len) != 0) {
        printf("FAIL %s: got", what);
        for (size_t i = 0; i < got_len; i++) {
            printf(" %02x", got[i]);
        }
        printf(", want");
        for (size_t i = 0; i < want_len; i++) {
            printf(" %02x", want[i]);
        }
        printf("\n");
        failures++;
    }
}

int main(void) {
    uint8_t out[64];
    size_t n, consumed;

    // --- CIN lengths: the specification's table, spot-checked ------------
    CHECK(usbif_midi_cin_len(0x0) == 0);   // reserved: not forwarded
    CHECK(usbif_midi_cin_len(0x1) == 0);
    CHECK(usbif_midi_cin_len(0x9) == 3);   // note on
    CHECK(usbif_midi_cin_len(0xC) == 2);   // program change
    CHECK(usbif_midi_cin_len(0xF) == 1);   // single-byte realtime
    CHECK(usbif_midi_cin_len(0x19) == 3);  // high nibble ignored (cable no.)

    // --- pack: a three-byte channel message -----------------------------
    {
        const uint8_t midi[] = { 0x90, 60, 100 };
        const uint8_t want[] = { 0x09, 0x90, 60, 100 };
        n = usbif_midi_pack(midi, sizeof(midi), out, sizeof(out), &consumed);
        check_bytes("note on", out, n, want, sizeof(want));
        CHECK(consumed == 3);
    }

    // --- pack: two-byte messages get a zero pad, not a stray byte -------
    {
        const uint8_t midi[] = { 0xC0, 7 };
        const uint8_t want[] = { 0x0C, 0xC0, 7, 0 };
        n = usbif_midi_pack(midi, sizeof(midi), out, sizeof(out), &consumed);
        check_bytes("program change", out, n, want, sizeof(want));
        CHECK(consumed == 2);
    }

    // --- pack: realtime is one byte and may appear anywhere -------------
    {
        const uint8_t midi[] = { 0xF8 };
        const uint8_t want[] = { 0x0F, 0xF8, 0, 0 };
        n = usbif_midi_pack(midi, sizeof(midi), out, sizeof(out), &consumed);
        check_bytes("clock", out, n, want, sizeof(want));
        CHECK(consumed == 1);
    }

    // --- pack: several messages in one call -----------------------------
    {
        const uint8_t midi[] = { 0x90, 60, 100, 0x80, 60, 0, 0xB0, 7, 127 };
        const uint8_t want[] = {
            0x09, 0x90, 60, 100,
            0x08, 0x80, 60, 0,
            0x0B, 0xB0, 7, 127,
        };
        n = usbif_midi_pack(midi, sizeof(midi), out, sizeof(out), &consumed);
        check_bytes("three messages", out, n, want, sizeof(want));
        CHECK(consumed == 9);
    }

    // --- pack: a partial trailing message is NOT consumed ----------------
    // The point of the whole design: the caller can present it again with
    // the rest appended, instead of a synthesiser receiving half a note.
    {
        const uint8_t midi[] = { 0x90, 60, 100, 0x90, 62 };   // second is short
        const uint8_t want[] = { 0x09, 0x90, 60, 100 };
        n = usbif_midi_pack(midi, sizeof(midi), out, sizeof(out), &consumed);
        check_bytes("partial tail", out, n, want, sizeof(want));
        CHECK(consumed == 3);       // only the complete message
    }

    // --- pack: a stray data byte is skipped, not guessed at --------------
    {
        const uint8_t midi[] = { 60, 100, 0x90, 60, 100 };
        const uint8_t want[] = { 0x09, 0x90, 60, 100 };
        n = usbif_midi_pack(midi, sizeof(midi), out, sizeof(out), &consumed);
        check_bytes("leading data bytes", out, n, want, sizeof(want));
        CHECK(consumed == 5);
    }

    // --- pack: SysEx stops packing and consumes only what preceded it ----
    {
        const uint8_t midi[] = { 0x90, 60, 100, 0xF0, 0x7E, 0xF7 };
        const uint8_t want[] = { 0x09, 0x90, 60, 100 };
        n = usbif_midi_pack(midi, sizeof(midi), out, sizeof(out), &consumed);
        check_bytes("sysex refused", out, n, want, sizeof(want));
        CHECK(consumed == 3);
    }

    // --- pack: respects a small destination ------------------------------
    {
        const uint8_t midi[] = { 0x90, 60, 100, 0x90, 62, 100 };
        n = usbif_midi_pack(midi, sizeof(midi), out, 4, &consumed);
        CHECK(n == 4);              // one packet only
        CHECK(consumed == 3);       // and only the message it packed
    }

    // --- unpack: the inverse, including a short packet -------------------
    {
        const uint8_t pkts[] = {
            0x09, 0x90, 60, 100,
            0x0F, 0xF8, 0, 0,
            0x0C, 0xC0, 7, 0,
        };
        const uint8_t want[] = { 0x90, 60, 100, 0xF8, 0xC0, 7 };
        n = usbif_midi_unpack(pkts, sizeof(pkts), out, sizeof(out));
        check_bytes("unpack mixed", out, n, want, sizeof(want));
    }

    // --- unpack: reserved CINs contribute nothing ------------------------
    {
        const uint8_t pkts[] = {
            0x00, 0x11, 0x22, 0x33,     // CIN 0: reserved
            0x09, 0x90, 60, 100,
        };
        const uint8_t want[] = { 0x90, 60, 100 };
        n = usbif_midi_unpack(pkts, sizeof(pkts), out, sizeof(out));
        check_bytes("unpack skips reserved", out, n, want, sizeof(want));
    }

    // --- unpack: a trailing fragment is ignored, not half-read -----------
    {
        const uint8_t pkts[] = { 0x09, 0x90, 60, 100, 0x09, 0x90 };
        const uint8_t want[] = { 0x90, 60, 100 };
        n = usbif_midi_unpack(pkts, sizeof(pkts), out, sizeof(out));
        check_bytes("unpack fragment", out, n, want, sizeof(want));
    }

    // --- round trip -------------------------------------------------------
    {
        const uint8_t midi[] = { 0x90, 60, 100, 0xE0, 0, 64, 0xD0, 90 };
        uint8_t pkts[64], back[64];
        size_t np = usbif_midi_pack(midi, sizeof(midi), pkts, sizeof(pkts), &consumed);
        CHECK(consumed == sizeof(midi));
        size_t nb = usbif_midi_unpack(pkts, np, back, sizeof(back));
        check_bytes("round trip", back, nb, midi, sizeof(midi));
    }

    if (failures == 0) {
        printf("test_midi_packets: all checks passed\n");
    }
    return failures != 0;
}
