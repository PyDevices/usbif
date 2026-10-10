// SPDX-License-Identifier: MIT
//
// How much isochronous schedule the UVC host keeps queued with the controller.
//
// Completions reach the driver only when the host task pumps the USB library,
// and it pumps once per FreeRTOS tick: every 10 ms at 100 Hz. A transfer that
// completes just after a pump waits a whole tick to be resubmitted, so the
// queue has to hold more than a tick of bus time or the endpoint goes unpolled
// until the next pump. A camera doesn't show that as a gap. It keeps the
// frame's bytes in its own FIFO, overflows it, and goes on sending frames of
// the right size whose rows come from the wrong places. That was the scrambled
// YUY2 on the ESP32-P4's high-speed host (usbif#67), where three transfers of
// eight microframes covered 3 ms of every 10 ms tick.
//
// So the queue is sized in time: at least USBIF_UVC_QUEUE_TICKS ticks of bus
// time, in transfers of at most USBIF_UVC_HS_PKTS_PER_XFER packets on a
// high-speed bus (4 ms at one packet a microframe) or
// USBIF_UVC_FS_PKTS_PER_XFER on a full-speed one, where a packet is a whole
// millisecond.
//
// Header-only and free of ESP-IDF so the host-side tests can check it.
#ifndef USBIF_UVC_QUEUE_H
#define USBIF_UVC_QUEUE_H

#include <stdbool.h>
#include <stdint.h>

#define USBIF_UVC_HS_PKTS_PER_XFER (32)
#define USBIF_UVC_FS_PKTS_PER_XFER (8)
#define USBIF_UVC_QUEUE_TICKS      (3)
#define USBIF_UVC_MIN_XFER         (3)
#define USBIF_UVC_MAX_XFER         (12)

// ESP-IDF's host controller driver keeps one transfer's packets in a list of
// 64 (micro)frame slots, three of them held back as timing margin, so a
// transfer may span at most 61 slots, counting the ones its interval skips.
#define USBIF_UVC_MAX_XFER_SLOTS   (61)

typedef struct {
    uint8_t pkts_per_xfer;
    uint8_t num_xfer;
} usbif_uvc_queue_t;

// `interval` is the endpoint's period in bus slots (microframes on a
// high-speed bus, frames on a full-speed one); `tick_ms` is the FreeRTOS tick
// period, which is how often completions are serviced.
static inline usbif_uvc_queue_t usbif_uvc_queue_size(bool high_speed,
    uint32_t interval, uint32_t tick_ms) {
    if (interval == 0) {
        interval = 1;
    }
    if (tick_ms == 0) {
        tick_ms = 1;
    }
    uint32_t pkts = high_speed ? USBIF_UVC_HS_PKTS_PER_XFER : USBIF_UVC_FS_PKTS_PER_XFER;
    if (pkts * interval > USBIF_UVC_MAX_XFER_SLOTS) {
        pkts = interval >= USBIF_UVC_MAX_XFER_SLOTS ? 1 : USBIF_UVC_MAX_XFER_SLOTS / interval;
    }
    const uint32_t slot_us = high_speed ? 125 : 1000;
    const uint32_t xfer_us = pkts * interval * slot_us;
    const uint32_t queue_us = (uint32_t)USBIF_UVC_QUEUE_TICKS * tick_ms * 1000;
    uint32_t nxfer = (queue_us + xfer_us - 1) / xfer_us;
    if (nxfer < USBIF_UVC_MIN_XFER) {
        nxfer = USBIF_UVC_MIN_XFER;
    } else if (nxfer > USBIF_UVC_MAX_XFER) {
        nxfer = USBIF_UVC_MAX_XFER;
    }
    usbif_uvc_queue_t q = { (uint8_t)pkts, (uint8_t)nxfer };
    return q;
}

// Bus time the queue covers, in microseconds.
static inline uint32_t usbif_uvc_queue_us(usbif_uvc_queue_t q, bool high_speed,
    uint32_t interval) {
    if (interval == 0) {
        interval = 1;
    }
    return (uint32_t)q.pkts_per_xfer * q.num_xfer * interval * (high_speed ? 125u : 1000u);
}

#endif // USBIF_UVC_QUEUE_H
