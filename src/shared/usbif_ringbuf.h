// SPDX-License-Identifier: MIT
//
// Single-producer / single-consumer event ring buffer.
//
// This is the piece the whole transport decision rests on. Measured on an
// ESP32-S3, delivering USB events to Python through mp_sched_schedule loses
// 76% of a 1 kHz stream during a sha256 pass and 99% during flash writes, with
// a single 1537 ms stall -- because no scheduled callback runs while the VM is
// inside one long C call. Those are not exotic workloads for usbif: they are
// MSC block writes, display flushes and audio buffering.
//
// So the producer never waits for the VM. A USB callback (interrupt or task
// context, depending on the port) writes one fixed-size record here and
// returns; Python collects records later with usbif_rb_pop(). Delivery latency
// becomes how often the application polls, which it controls, instead of what
// the VM happened to be doing when the event arrived.
//
// Concurrency contract: exactly one producer and one consumer. Head is written
// only by the producer, tail only by the consumer, both are volatile and
// word-sized (naturally atomic on every target port), and the buffer holds
// capacity-1 records so a full ring is distinguishable from an empty one
// without a separate count that both sides would have to update. That is what
// removes the need for a lock in the producer -- and taking a lock in an ISR
// is what we are avoiding.
//
// Overflow is recorded, never silent. The mechanism this replaces failed
// silently, which is precisely what made it dangerous.
#ifndef USBIF_RINGBUF_H
#define USBIF_RINGBUF_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Event kinds. Kept small and integral so a record stays a fixed-size POD that
// is safe to write from an ISR.
#define USBIF_EV_ATTACH (0)
#define USBIF_EV_DETACH (1)

typedef struct {
    uint8_t kind;
    uint8_t speed;      // usbif_speed_t, 0 when unknown
    uint16_t vid;
    uint16_t pid;
    uint16_t classes;   // bitmask of USBIF_CLASS_*
    uint32_t dev_id;    // backend handle, stable while attached
} usbif_event_t;

typedef struct {
    usbif_event_t *slots;
    uint16_t capacity;          // slot count; usable depth is capacity - 1
    volatile uint16_t head;     // producer writes
    volatile uint16_t tail;     // consumer writes
    volatile uint32_t dropped;  // events lost to a full ring, cumulative
} usbif_ringbuf_t;

// Initialise over caller-provided storage. No allocation happens here: the
// buffer must exist before the first USB callback can fire, and allocating on
// an MCU heap from that path is exactly what must not happen.
void usbif_rb_init(usbif_ringbuf_t *rb, usbif_event_t *slots, uint16_t capacity);

// Producer side. Returns false if the ring is full, having counted the drop.
// Safe to call from interrupt context; never blocks and never allocates.
bool usbif_rb_push(usbif_ringbuf_t *rb, const usbif_event_t *event);

// Consumer side. Returns false if empty.
bool usbif_rb_pop(usbif_ringbuf_t *rb, usbif_event_t *out);

// Records currently waiting.
uint16_t usbif_rb_count(const usbif_ringbuf_t *rb);

// Cumulative drops, cleared to zero as it is read, so a caller learns of
// overflow exactly once.
uint32_t usbif_rb_take_dropped(usbif_ringbuf_t *rb);

#endif // USBIF_RINGBUF_H
