// SPDX-License-Identifier: MIT
//
// The lifetime gate behind usbif's pcm_c_sink_t (usbif#43): a byte ring that
// producers on any task may write to, and that its owner may close and free
// while one of them is mid-write.
//
// The guarantee, stated once and tested in tests/test_pcm_sink.c:
//
//   After usbif_pcm_sink_close() returns, no usbif_pcm_sink_write() or
//   usbif_pcm_sink_space() call is inside the ring, and none will enter it
//   under the generation that was closed. The owner may free the ring's
//   storage the moment close() returns.
//
// How. `live` holds the open stream's generation, 0 while closed; `users`
// counts calls in progress. A call increments `users`, THEN reads `live`, and
// leaves at once unless `live` still names its generation. close() stores 0
// to `live`, THEN waits for `users` to reach zero. Both pairs are sequentially
// consistent, which is the whole argument: in the single order all four
// operations share, either the call's increment comes before close()'s read
// of `users` (so close() waits for it), or close()'s store of 0 comes before
// the call's read of `live` (so the call never touches the ring). A call's
// decrement is a release and close()'s read of `users` an acquire, so the
// bytes it wrote are behind it before the storage goes.
//
// Generations make a sink handed out for one stream stay dead after that
// stream closes, even once another stream opens in its place: a consumer that
// held on to an old sink gets -1, not a slot in somebody else's audio.
//
// `busy` is a try-lock between producers. The ring is single-producer; with
// the interpreter and a usermod's task both able to write, the loser of a
// race takes 0 bytes rather than corrupting the ring's head.
//
// Portable C11 plus GCC/Clang __atomic builtins, so the same code runs on the
// S3, the P4 and a CI runner. The only platform piece is how close() waits,
// which the owner supplies (vTaskDelay on FreeRTOS, sched_yield in the test).
#ifndef USBIF_PCM_SINK_H
#define USBIF_PCM_SINK_H

#include <stddef.h>
#include <stdint.h>

#include "shared/usbif_byte_ring.h"

typedef struct {
    usbif_byte_ring_t *ring;    // the ring this feeds; the owner's storage
    uint32_t frame;             // writes are cut to whole frames of this
    uint32_t live;              // open generation, 0 while closed
    uint32_t users;             // write/space calls in progress
    uint32_t busy;              // 1 while a producer is pushing
    uint32_t last_gen;          // the most recent generation handed out
    void (*wait)(void);         // how close() lets a caller finish
} usbif_pcm_sink_t;

// Once, before anything else. `wait` is called in close()'s loop; it must let
// a task on the same core run (a tick's sleep, a yield).
void usbif_pcm_sink_init(usbif_pcm_sink_t *s, usbif_byte_ring_t *ring, void (*wait)(void));

// The ring is initialised and ready: start accepting writes, and return the
// new stream's generation (never 0). The owner hands it out as the sink's ctx.
uint32_t usbif_pcm_sink_open(usbif_pcm_sink_t *s, uint32_t frame);

// Stop accepting writes and wait out the ones in progress; see the guarantee
// above. Idempotent. Not to be called concurrently with open().
void usbif_pcm_sink_close(usbif_pcm_sink_t *s);

// The open generation, or 0. For an owner's own writes (the interpreter's),
// which are meant for whatever stream is open now.
uint32_t usbif_pcm_sink_current(const usbif_pcm_sink_t *s);

// Bytes taken, in whole frames (0 when full or another producer is pushing);
// -1 unless `gen` is the open generation.
int usbif_pcm_sink_write(usbif_pcm_sink_t *s, uint32_t gen, const uint8_t *data, size_t len);

// Bytes a write could take now, in whole frames; -1 unless `gen` is open.
int usbif_pcm_sink_space(usbif_pcm_sink_t *s, uint32_t gen);

#endif // USBIF_PCM_SINK_H
