// SPDX-License-Identifier: MIT
//
// Host-side tests for the UVC host's transfer queue size (usbif#67): the
// queue must cover more than one FreeRTOS tick of bus time, because the host
// task services completions only once a tick. With less, a camera on a
// high-speed host goes unpolled for most of each tick, overflows its FIFO and
// sends scrambled frames.
#include "shared/usbif_uvc_queue.h"

#include <stdio.h>

static int failures = 0;

#define CHECK(cond) do { \
    if (!(cond)) { \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
        failures++; \
    } \
} while (0)

// The invariant: at least USBIF_UVC_QUEUE_TICKS ticks queued, or the most the
// transfer cap allows, and never a transfer longer than the controller's list.
static int covers(usbif_uvc_queue_t q, bool hs, uint32_t interval, uint32_t tick_ms) {
    uint32_t us = usbif_uvc_queue_us(q, hs, interval);
    bool capped = q.num_xfer == USBIF_UVC_MAX_XFER;
    return (us >= USBIF_UVC_QUEUE_TICKS * tick_ms * 1000u || capped)
           && us > tick_ms * 1000u
           && q.pkts_per_xfer * interval <= USBIF_UVC_MAX_XFER_SLOTS
           && q.num_xfer >= USBIF_UVC_MIN_XFER;
}

int main(void) {
    // The bench case: ESP32-P4, high speed, bInterval 1, 100 Hz tick.
    usbif_uvc_queue_t q = usbif_uvc_queue_size(true, 1, 10);
    CHECK(q.pkts_per_xfer == 32);
    CHECK(q.num_xfer == 8);
    CHECK(usbif_uvc_queue_us(q, true, 1) == 32000);
    CHECK(covers(q, true, 1, 10));

    // The queue this driver had before: 3 transfers of 8 microframes is 3 ms,
    // less than one 10 ms tick. The invariant has to reject it, or the check
    // above proves nothing.
    usbif_uvc_queue_t old = { 8, 3 };
    CHECK(!covers(old, true, 1, 10));

    // A 1000 Hz board on a full-speed bus (the ESP32-S3) keeps what it had.
    q = usbif_uvc_queue_size(false, 1, 1);
    CHECK(q.pkts_per_xfer == 8);
    CHECK(q.num_xfer == 3);
    CHECK(covers(q, false, 1, 1));

    // Full speed at 100 Hz: 8 ms transfers, four of them for 30 ms.
    q = usbif_uvc_queue_size(false, 1, 10);
    CHECK(q.pkts_per_xfer == 8);
    CHECK(q.num_xfer == 4);
    CHECK(covers(q, false, 1, 10));

    // High speed at 1000 Hz needs no more than the minimum.
    q = usbif_uvc_queue_size(true, 1, 1);
    CHECK(q.num_xfer == USBIF_UVC_MIN_XFER);
    CHECK(covers(q, true, 1, 1));

    // Every interval on both buses and both tick rates, including ones whose
    // packets don't fit 32 to a transfer.
    for (uint32_t iv = 1; iv <= 1024; iv <<= 1) {
        for (int hs = 0; hs <= 1; hs++) {
            for (uint32_t tick = 1; tick <= 10; tick *= 10) {
                q = usbif_uvc_queue_size(hs, iv, tick);
                CHECK(q.pkts_per_xfer >= 1);
                CHECK(q.pkts_per_xfer * iv <= USBIF_UVC_MAX_XFER_SLOTS || q.pkts_per_xfer == 1);
                CHECK(q.num_xfer >= USBIF_UVC_MIN_XFER && q.num_xfer <= USBIF_UVC_MAX_XFER);
            }
        }
    }

    // Zero inputs don't divide by zero.
    q = usbif_uvc_queue_size(true, 0, 0);
    CHECK(q.pkts_per_xfer == 32 && q.num_xfer == USBIF_UVC_MIN_XFER);

    if (failures) {
        printf("%d failure(s)\n", failures);
        return 1;
    }
    printf("uvc queue: all checks passed\n");
    return 0;
}
