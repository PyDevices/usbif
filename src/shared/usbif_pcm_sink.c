// SPDX-License-Identifier: MIT
//
// The lifetime gate behind usbif's pcm_c_sink_t. The reasoning is in the
// header; this file is only the four atomic operations it names.
#include "shared/usbif_pcm_sink.h"

// A test widens the race window here, inside the gate, to prove that close()
// really waits (tests/test_pcm_sink.c). Empty in firmware.
#ifndef USBIF_PCM_SINK_INSIDE_HOOK
#define USBIF_PCM_SINK_INSIDE_HOOK() do { } while (0)
#endif

void usbif_pcm_sink_init(usbif_pcm_sink_t *s, usbif_byte_ring_t *ring, void (*wait)(void)) {
    s->ring = ring;
    s->frame = 0;
    s->live = 0;
    s->users = 0;
    s->busy = 0;
    s->last_gen = 0;
    s->wait = wait;
}

uint32_t usbif_pcm_sink_open(usbif_pcm_sink_t *s, uint32_t frame) {
    uint32_t gen = s->last_gen + 1;
    if (gen == 0) {
        gen = 1;                // 0 means closed; four billion opens later
    }
    s->last_gen = gen;
    s->frame = frame;
    // Everything the owner set up -- the ring, `frame` -- is published by
    // this store: a caller that reads `gen` from `live` sees all of it.
    __atomic_store_n(&s->live, gen, __ATOMIC_SEQ_CST);
    return gen;
}

void usbif_pcm_sink_close(usbif_pcm_sink_t *s) {
    __atomic_store_n(&s->live, 0, __ATOMIC_SEQ_CST);
    #ifndef USBIF_PCM_SINK_PLANT_NO_WAIT
    // Unbounded on purpose. A caller inside is doing one memcpy of at most
    // the ring's size; giving up and freeing anyway would trade a short
    // stall for a write into freed memory.
    while (__atomic_load_n(&s->users, __ATOMIC_SEQ_CST) != 0) {
        s->wait();
    }
    #endif
}

uint32_t usbif_pcm_sink_current(const usbif_pcm_sink_t *s) {
    return __atomic_load_n(&s->live, __ATOMIC_ACQUIRE);
}

static inline int usbif_pcm_sink_enter(usbif_pcm_sink_t *s, uint32_t gen) {
    __atomic_fetch_add(&s->users, 1, __ATOMIC_SEQ_CST);
    if (gen == 0 || __atomic_load_n(&s->live, __ATOMIC_SEQ_CST) != gen) {
        __atomic_fetch_sub(&s->users, 1, __ATOMIC_RELEASE);
        return 0;
    }
    return 1;
}

static inline void usbif_pcm_sink_leave(usbif_pcm_sink_t *s) {
    __atomic_fetch_sub(&s->users, 1, __ATOMIC_RELEASE);
}

int usbif_pcm_sink_write(usbif_pcm_sink_t *s, uint32_t gen, const uint8_t *data, size_t len) {
    if (!usbif_pcm_sink_enter(s, gen)) {
        return -1;
    }
    USBIF_PCM_SINK_INSIDE_HOOK();
    int taken = 0;
    #ifndef USBIF_PCM_SINK_PLANT_NO_TRYLOCK
    if (__atomic_exchange_n(&s->busy, 1, __ATOMIC_ACQUIRE) == 0) {
    #else
    {
    #endif
        uint32_t room = usbif_byte_ring_free(s->ring);
        uint32_t n = len < room ? (uint32_t)len : room;
        if (s->frame) {
            n -= n % s->frame;
        }
        if (n) {
            taken = (int)usbif_byte_ring_push(s->ring, data, n);
        }
        __atomic_store_n(&s->busy, 0, __ATOMIC_RELEASE);
    }
    USBIF_PCM_SINK_INSIDE_HOOK();
    usbif_pcm_sink_leave(s);
    return taken;
}

int usbif_pcm_sink_space(usbif_pcm_sink_t *s, uint32_t gen) {
    if (!usbif_pcm_sink_enter(s, gen)) {
        return -1;
    }
    uint32_t room = usbif_byte_ring_free(s->ring);
    if (s->frame) {
        room -= room % s->frame;
    }
    usbif_pcm_sink_leave(s);
    return (int)room;
}
