// SPDX-License-Identifier: MIT
//
// Host-side tests for the UAC host's byte ring (usbif#36): any size, every
// byte usable, two-span copies across the wrap, and a long stream through an
// odd size to catch index arithmetic that only fails after many laps.
#include "shared/usbif_byte_ring.h"

#include <stdio.h>
#include <string.h>

static int failures = 0;

#define CHECK(cond) do { \
    if (!(cond)) { \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
        failures++; \
    } \
} while (0)

int main(void) {
    uint8_t store[7];
    uint8_t out[16];
    usbif_byte_ring_t r;

    // Empty
    usbif_byte_ring_init(&r, store, sizeof(store));
    CHECK(usbif_byte_ring_used(&r) == 0);
    CHECK(usbif_byte_ring_free(&r) == 7);
    CHECK(usbif_byte_ring_pop(&r, out, sizeof(out)) == 0);

    // Every byte usable: 7 of 7, then a short push
    const uint8_t a[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    CHECK(usbif_byte_ring_push(&r, a, 10) == 7);
    CHECK(usbif_byte_ring_used(&r) == 7);
    CHECK(usbif_byte_ring_free(&r) == 0);
    CHECK(usbif_byte_ring_push(&r, a, 1) == 0);

    // Partial pop, then a push that wraps (two spans)
    CHECK(usbif_byte_ring_pop(&r, out, 5) == 5);
    CHECK(memcmp(out, a, 5) == 0);
    const uint8_t b[5] = {20, 21, 22, 23, 24};
    CHECK(usbif_byte_ring_push(&r, b, 5) == 5);
    CHECK(usbif_byte_ring_used(&r) == 7);
    // A pop that wraps: 6, 7, then 20..24
    CHECK(usbif_byte_ring_pop(&r, out, 16) == 7);
    const uint8_t want[7] = {6, 7, 20, 21, 22, 23, 24};
    CHECK(memcmp(out, want, 7) == 0);
    CHECK(usbif_byte_ring_used(&r) == 0);

    // A long stream through an odd size, in odd chunk sizes: every byte comes
    // out in order and the fill level never exceeds the size. Many laps, so
    // the two-lap index arithmetic crosses its own wrap thousands of times.
    static uint8_t big[1021];
    usbif_byte_ring_init(&r, big, sizeof(big));
    uint32_t wr = 0, rd = 0, bad = 0;
    uint8_t chunk[97];
    for (int step = 0; step < 200000; step++) {
        uint32_t pn = (uint32_t)(step * 7 % 97) + 1;
        for (uint32_t i = 0; i < pn; i++) {
            chunk[i] = (uint8_t)(wr + i);
        }
        wr += usbif_byte_ring_push(&r, chunk, pn);
        if (usbif_byte_ring_used(&r) > sizeof(big)) {
            bad++;
        }
        uint32_t qn = (uint32_t)(step * 13 % 89) + 1;
        uint32_t got = usbif_byte_ring_pop(&r, chunk, qn);
        for (uint32_t i = 0; i < got; i++) {
            if (chunk[i] != (uint8_t)(rd + i)) {
                bad++;
            }
        }
        rd += got;
    }
    CHECK(bad == 0);
    CHECK(wr - rd == usbif_byte_ring_used(&r));
    CHECK(rd > 100u * sizeof(big));

    if (failures) {
        printf("%d failure(s)\n", failures);
        return 1;
    }
    printf("byte ring: all tests passed\n");
    return 0;
}
