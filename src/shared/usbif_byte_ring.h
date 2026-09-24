// SPDX-License-Identifier: MIT
//
// Single-producer / single-consumer byte ring over caller-sized storage.
//
// The UAC host streams through one of these: on playback Python writes and the
// isochronous completion callback reads, on capture the reverse. Its size is
// the caller's choice (usbif#36): 8 KB covers 43 ms of 48 kHz stereo, which a
// busy interpreter outlasts, while a couple of seconds covers a UI redraw or a
// Web API call. So the storage is supplied from outside, and the ring works at
// any size, not only a power of two.
//
// head and tail run over two laps of the storage, [0, 2 * size), so a full
// ring (head one lap ahead of tail) differs from an empty one (equal) and
// every byte of the storage is usable, at any size up to 2 GB. Free-running
// uint32 counters would not do: reduced modulo a size that is not a power of
// two they jump when the counter wraps, about six hours into a 48 kHz stereo
// stream. Each index is written by one side only, published with release
// ordering after the bytes it covers are copied, and read with acquire
// ordering by the other side: the producer and consumer can run on different
// cores on the S3 and P4.
//
// Copies are memcpy in at most two spans (to the end of the storage, then
// from its start), not a loop over bytes.
#ifndef USBIF_BYTE_RING_H
#define USBIF_BYTE_RING_H

#include <stdint.h>

typedef struct {
    uint8_t *buf;
    uint32_t size;
    volatile uint32_t head;     // in [0, 2 * size); producer writes
    volatile uint32_t tail;     // in [0, 2 * size); consumer writes
} usbif_byte_ring_t;

// Initialise over caller-provided storage of `size` bytes. No allocation.
void usbif_byte_ring_init(usbif_byte_ring_t *r, uint8_t *buf, uint32_t size);

// Bytes queued. Safe from either side.
uint32_t usbif_byte_ring_used(const usbif_byte_ring_t *r);

// Bytes that can be pushed now. Safe from either side.
uint32_t usbif_byte_ring_free(const usbif_byte_ring_t *r);

// Producer: copy in up to `len` bytes, return how many were taken (short when
// the ring fills; the caller decides whether that is a drop).
uint32_t usbif_byte_ring_push(usbif_byte_ring_t *r, const uint8_t *data, uint32_t len);

// Consumer: copy out up to `max` bytes, return how many.
uint32_t usbif_byte_ring_pop(usbif_byte_ring_t *r, uint8_t *out, uint32_t max);

#endif // USBIF_BYTE_RING_H
