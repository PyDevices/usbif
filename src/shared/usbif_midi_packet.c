// SPDX-License-Identifier: MIT
//
// USB-MIDI 1.0 packet codec. See the header for the format and for why the
// two refusals below are refusals rather than best guesses.

#include "shared/usbif_midi_packet.h"

// USB-MIDI 1.0 table 4-1, as a table rather than a switch because the
// mapping is the specification's and not a policy of ours.
static const uint8_t usbif_midi_cin_len_tbl[16] = {
    0,  // 0x0 misc / reserved
    0,  // 0x1 cable event / reserved
    2,  // 0x2 two-byte system common
    3,  // 0x3 three-byte system common
    3,  // 0x4 SysEx starts or continues
    1,  // 0x5 SysEx ends with one byte, or single-byte system common
    2,  // 0x6 SysEx ends with two bytes
    3,  // 0x7 SysEx ends with three bytes
    3,  // 0x8 note off
    3,  // 0x9 note on
    3,  // 0xA poly key pressure
    3,  // 0xB control change
    2,  // 0xC program change
    2,  // 0xD channel pressure
    3,  // 0xE pitch bend
    1,  // 0xF single byte (realtime)
};

uint8_t usbif_midi_cin_len(uint8_t cin) {
    return usbif_midi_cin_len_tbl[cin & 0x0F];
}

size_t usbif_midi_unpack(const uint8_t *src, size_t src_len,
    uint8_t *dst, size_t dst_max) {
    size_t out = 0;
    for (size_t i = 0; i + 4 <= src_len; i += 4) {
        uint8_t n = usbif_midi_cin_len(src[i]);
        for (uint8_t j = 0; j < n && out < dst_max; j++) {
            dst[out++] = src[i + 1 + j];
        }
    }
    return out;
}

size_t usbif_midi_pack(const uint8_t *src, size_t src_len,
    uint8_t *dst, size_t dst_max, size_t *consumed) {
    size_t in = 0;
    size_t out = 0;

    while (in < src_len && out + 4 <= dst_max) {
        uint8_t status = src[in];

        if (status < 0x80) {
            // A data byte where a status byte belongs. Running status is not
            // reconstructed: guessing would emit messages the caller never
            // asked to send.
            in++;
            continue;
        }

        if (status >= 0xF8) {
            // System realtime: one byte, CIN 0xF, and legal *between* the
            // bytes of another message -- which is why it is handled before
            // any length arithmetic.
            dst[out + 0] = 0x0F;
            dst[out + 1] = status;
            dst[out + 2] = 0;
            dst[out + 3] = 0;
            out += 4;
            in++;
            continue;
        }

        if (status == 0xF0) {
            break;      // SysEx: needs state across calls; not handled here
        }

        uint8_t hi = (uint8_t)(status >> 4);
        // Program change and channel pressure carry one data byte; every
        // other channel message carries two.
        size_t need = (hi == 0x0C || hi == 0x0D) ? 2 : 3;
        if (in + need > src_len) {
            break;      // partial message: leave it for the next call
        }
        dst[out + 0] = hi;      // cable 0; for channel messages CIN == the
                                // status nibble, which is why this is not a
                                // lookup
        dst[out + 1] = status;
        dst[out + 2] = src[in + 1];
        dst[out + 3] = (need == 3) ? src[in + 2] : 0;
        out += 4;
        in += need;
    }

    if (consumed) {
        *consumed = in;
    }
    return out;
}
