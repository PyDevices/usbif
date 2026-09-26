// SPDX-License-Identifier: MIT
//
// Band levels for a spectrum meter, computed in the sound card's C pump
// (usbif_meter.c). Spike code.

#ifndef USBIF_METER_H
#define USBIF_METER_H

#include <stdbool.h>
#include <stdint.h>

#define USBIF_METER_MAX_BANDS (96)

typedef struct {
    bool enabled;
    uint32_t bands;
    uint32_t analyses;
    uint64_t feed_us;
    uint64_t fft_us;
    uint32_t max_fft_us;
    int64_t elapsed_us;
    uint32_t cpu_mhz;
} usbif_meter_stats_t;

bool usbif_meter_alloc(void);
void usbif_meter_configure(int bands, float lo_hz, float hi_hz);
void usbif_meter_feed(const int16_t *frames, uint32_t n, uint32_t channels, uint32_t rate);
uint32_t usbif_meter_read(uint8_t *levels, uint32_t max, uint8_t *peak, uint8_t *rms, uint32_t *nbands);
void usbif_meter_stats(usbif_meter_stats_t *st);

#endif
