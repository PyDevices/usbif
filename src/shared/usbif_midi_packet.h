// SPDX-License-Identifier: MIT
//
// USB-MIDI 1.0 packet codec, kept free of IDF and MicroPython so it can be
// tested on a host where a failure is a two-second answer -- the same
// reasoning as the event ring beside it.
//
// USB-MIDI does not carry a MIDI byte stream. It carries fixed 32-bit event
// packets: [cable<<4 | CIN][byte0][byte1][byte2], where the Code Index
// Number names the kind of message and therefore how many of the three data
// bytes are real. A three-byte note-on and a one-byte clock tick occupy the
// same four bytes on the wire. Everything above this layer -- the host MIDI
// driver, and Python above it -- deals in plain MIDI bytes.
#ifndef USBIF_MIDI_PACKET_H
#define USBIF_MIDI_PACKET_H

#include <stddef.h>
#include <stdint.h>

// How many of a packet's three data bytes are real, by Code Index Number.
// Zero means "not a message to forward": CIN 0 and 1 are reserved for
// vendor-specific and cable events. USB-MIDI 1.0 table 4-1.
uint8_t usbif_midi_cin_len(uint8_t cin);

// Unpack one IN transfer into a plain MIDI byte stream.
//
// `src_len` need not be a multiple of four; a trailing fragment is ignored
// rather than guessed at. Returns bytes written to `dst`, never more than
// `dst_max`.
size_t usbif_midi_unpack(const uint8_t *src, size_t src_len,
    uint8_t *dst, size_t dst_max);

// Pack a plain MIDI byte stream into event packets.
//
// Writes whole packets to `dst` (four bytes each, at most `dst_max` bytes)
// and reports through `consumed` how many input bytes were used. A trailing
// *partial* message is deliberately not consumed and not packed, so the
// caller can present it again with the rest appended: sending half a
// message leaves a synthesiser in a state no later byte explains, which is
// far worse than a short write.
//
// Returns bytes written to `dst`.
//
// Two deliberate refusals, both because the alternative is inventing data:
//   - a data byte where a status byte belongs is skipped (running status is
//     not reconstructed here);
//   - SysEx (0xF0) stops the packing, consuming only what came before it.
size_t usbif_midi_pack(const uint8_t *src, size_t src_len,
    uint8_t *dst, size_t dst_max, size_t *consumed);

#endif // USBIF_MIDI_PACKET_H
