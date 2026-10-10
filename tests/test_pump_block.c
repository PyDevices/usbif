// SPDX-License-Identifier: MIT
//
// Host-side tests for the sound card pump's block shaping: the spectrum meter
// must see what the wire plays, after the host's volume and mute. It used to
// see the host's frames before the volume multiply, so a host that sets the
// card's own volume (Windows) kept the meter at full scale even when muted.
//
// Built with -DUSBIF_PUMP_PLANT_METER_BEFORE_GAIN, the header meters in the
// old order and this test has to fail (CI checks that it does).
#include "shared/usbif_pump_block.h"

#include <stdio.h>
#include <string.h>

static int failures = 0;

#define CHECK(cond) do { \
    if (!(cond)) { \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
        failures++; \
    } \
} while (0)

// What the meter was handed, copied: the pump reuses its block.
static int16_t seen[1024];
static uint32_t seen_n, seen_channels, seen_rate, seen_calls;

static void meter(const int16_t *frames, uint32_t n, uint32_t channels, uint32_t rate) {
    seen_calls++;
    seen_n = n;
    seen_channels = channels;
    seen_rate = rate;
    memcpy(seen, frames, n * channels * sizeof(int16_t));
}

static void reset_meter(void) {
    memset(seen, 0x55, sizeof(seen));
    seen_n = seen_channels = seen_rate = seen_calls = 0;
}

// A loud stereo block: left and right differ, so a downmix shows.
static uint16_t fill(int16_t *block, uint16_t frames) {
    for (uint16_t f = 0; f < frames; f++) {
        block[f * 2] = (int16_t)(f % 2 ? 30000 : -30000);
        block[f * 2 + 1] = (int16_t)(f % 2 ? 20000 : -20000);
    }
    return (uint16_t)(frames * 4);
}

// The meter got one call holding exactly the bytes the pump will write.
static void check_meter_is_wire(const int16_t *block, uint16_t out_bytes,
    uint32_t channels, uint32_t rate) {
    CHECK(seen_calls == 1);
    CHECK(seen_channels == channels);
    CHECK(seen_rate == rate);
    CHECK(seen_n * channels * 2 == out_bytes);
    CHECK(memcmp(seen, block, out_bytes) == 0);
}

static void test_muted_meters_silence(void) {
    int16_t block[96 * 2];
    reset_meter();
    uint16_t usable = fill(block, 96);
    uint16_t out = usbif_pump_shape_block((uint8_t *)block, usable, 2, 2, 1, 0, 48000, meter);
    CHECK(out == usable);
    check_meter_is_wire(block, out, 2, 48000);
    int peak = 0;
    for (uint32_t i = 0; i < seen_n * 2; i++) {
        int v = seen[i] < 0 ? -seen[i] : seen[i];
        peak = v > peak ? v : peak;
    }
    CHECK(peak == 0);
}

static void test_half_volume_meters_half(void) {
    int16_t block[96 * 2];
    reset_meter();
    uint16_t usable = fill(block, 96);
    uint16_t out = usbif_pump_shape_block((uint8_t *)block, usable, 2, 2, 1, 32768u, 48000, meter);
    CHECK(out == usable);
    check_meter_is_wire(block, out, 2, 48000);
    CHECK(seen[0] == -15000 && seen[1] == -10000);
    CHECK(seen[2] == 15000 && seen[3] == 10000);
}

static void test_mono_decimated_meters_the_wire(void) {
    int16_t block[96 * 2];
    reset_meter();
    uint16_t usable = fill(block, 96);
    // Quarter volume, mixed to mono, one frame in two: the wire is 24 kHz mono.
    uint16_t out = usbif_pump_shape_block((uint8_t *)block, usable, 2, 1, 2, 16384u, 48000, meter);
    CHECK(out == 48 * 2);
    check_meter_is_wire(block, out, 1, 24000);
    // Frames 0, 2, 4 ... are the negative ones: (-30000 + -20000) / 2 / 4.
    CHECK(seen[0] == -6250 && seen[47] == -6250);
}

static void test_unity_passthrough_meters_input(void) {
    int16_t block[96 * 2], copy[96 * 2];
    reset_meter();
    uint16_t usable = fill(block, 96);
    memcpy(copy, block, usable);
    uint16_t out = usbif_pump_shape_block((uint8_t *)block, usable, 2, 2, 1, 65536u, 44100, meter);
    CHECK(out == usable);
    CHECK(memcmp(block, copy, usable) == 0);
    check_meter_is_wire(block, out, 2, 44100);
}

static void test_no_meter(void) {
    int16_t block[96 * 2];
    reset_meter();
    uint16_t usable = fill(block, 96);
    uint16_t out = usbif_pump_shape_block((uint8_t *)block, usable, 2, 2, 1, 0, 48000, NULL);
    CHECK(out == usable);
    CHECK(seen_calls == 0);
    CHECK(block[0] == 0 && block[1] == 0);
}

int main(void) {
    test_muted_meters_silence();
    test_half_volume_meters_half();
    test_mono_decimated_meters_the_wire();
    test_unity_passthrough_meters_input();
    test_no_meter();
    if (failures) {
        printf("%d failure(s)\n", failures);
        return 1;
    }
    printf("pump block: all passed\n");
    return 0;
}
