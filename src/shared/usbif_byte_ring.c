// SPDX-License-Identifier: MIT
//
// See usbif_byte_ring.h for the contract.
#include "shared/usbif_byte_ring.h"

#include <string.h>

void usbif_byte_ring_init(usbif_byte_ring_t *r, uint8_t *buf, uint32_t size) {
    r->buf = buf;
    r->size = size;
    r->head = 0;
    r->tail = 0;
}

// head and tail run over [0, 2 * size): the extra lap is what tells a full
// ring from an empty one without a spare byte or a shared count.
static inline uint32_t usbif_byte_ring_span(const usbif_byte_ring_t *r,
    uint32_t head, uint32_t tail) {
    return head >= tail ? head - tail : head + 2 * r->size - tail;
}

static inline uint32_t usbif_byte_ring_advance(const usbif_byte_ring_t *r,
    uint32_t idx, uint32_t n) {
    idx += n;
    return idx >= 2 * r->size ? idx - 2 * r->size : idx;
}

uint32_t usbif_byte_ring_used(const usbif_byte_ring_t *r) {
    uint32_t head = __atomic_load_n(&r->head, __ATOMIC_ACQUIRE);
    uint32_t tail = __atomic_load_n(&r->tail, __ATOMIC_ACQUIRE);
    return usbif_byte_ring_span(r, head, tail);
}

uint32_t usbif_byte_ring_free(const usbif_byte_ring_t *r) {
    return r->size - usbif_byte_ring_used(r);
}

uint32_t usbif_byte_ring_push(usbif_byte_ring_t *r, const uint8_t *data, uint32_t len) {
    const uint32_t head = r->head;     // ours: no ordering needed to read it
    const uint32_t tail = __atomic_load_n(&r->tail, __ATOMIC_ACQUIRE);
    const uint32_t room = r->size - usbif_byte_ring_span(r, head, tail);
    const uint32_t n = len < room ? len : room;
    if (n == 0) {
        return 0;
    }
    const uint32_t at = head < r->size ? head : head - r->size;
    const uint32_t first = (r->size - at) < n ? (r->size - at) : n;
    memcpy(r->buf + at, data, first);
    if (n > first) {
        memcpy(r->buf, data + first, n - first);
    }
    __atomic_store_n(&r->head, usbif_byte_ring_advance(r, head, n), __ATOMIC_RELEASE);
    return n;
}

uint32_t usbif_byte_ring_pop(usbif_byte_ring_t *r, uint8_t *out, uint32_t max) {
    const uint32_t tail = r->tail;     // ours
    const uint32_t head = __atomic_load_n(&r->head, __ATOMIC_ACQUIRE);
    const uint32_t have = usbif_byte_ring_span(r, head, tail);
    const uint32_t n = max < have ? max : have;
    if (n == 0) {
        return 0;
    }
    const uint32_t at = tail < r->size ? tail : tail - r->size;
    const uint32_t first = (r->size - at) < n ? (r->size - at) : n;
    memcpy(out, r->buf + at, first);
    if (n > first) {
        memcpy(out + first, r->buf, n - first);
    }
    __atomic_store_n(&r->tail, usbif_byte_ring_advance(r, tail, n), __ATOMIC_RELEASE);
    return n;
}
