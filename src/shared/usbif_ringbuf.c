// SPDX-License-Identifier: MIT

#include "shared/usbif_ringbuf.h"

#include <string.h>

static inline uint16_t next_index(const usbif_ringbuf_t *rb, uint16_t index) {
    index++;
    return index == rb->capacity ? 0 : index;
}

void usbif_rb_init(usbif_ringbuf_t *rb, usbif_event_t *slots, uint16_t capacity) {
    rb->slots = slots;
    rb->capacity = capacity;
    rb->head = 0;
    rb->tail = 0;
    rb->dropped = 0;
}

bool usbif_rb_push(usbif_ringbuf_t *rb, const usbif_event_t *event) {
    // Read tail once: the consumer may advance it while we are here, which can
    // only turn a full ring into a non-full one. Acting on the older value is
    // therefore safe -- it can cost a spurious drop, never a corrupted slot.
    const uint16_t head = rb->head;
    const uint16_t advanced = next_index(rb, head);
    if (advanced == rb->tail) {
        rb->dropped++;
        return false;
    }
    rb->slots[head] = *event;
    // Publish only after the slot is written, so a consumer that observes the
    // new head always sees a complete record.
    rb->head = advanced;
    return true;
}

bool usbif_rb_pop(usbif_ringbuf_t *rb, usbif_event_t *out) {
    const uint16_t tail = rb->tail;
    if (tail == rb->head) {
        return false;
    }
    *out = rb->slots[tail];
    rb->tail = next_index(rb, tail);
    return true;
}

uint16_t usbif_rb_count(const usbif_ringbuf_t *rb) {
    const uint16_t head = rb->head;
    const uint16_t tail = rb->tail;
    if (head >= tail) {
        return (uint16_t)(head - tail);
    }
    return (uint16_t)(rb->capacity - tail + head);
}

uint32_t usbif_rb_take_dropped(usbif_ringbuf_t *rb) {
    const uint32_t count = rb->dropped;
    rb->dropped = 0;
    return count;
}
