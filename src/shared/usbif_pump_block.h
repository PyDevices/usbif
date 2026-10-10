// SPDX-License-Identifier: MIT
//
// One block of the sound card's I2S pump, shaped for the wire: the host's
// volume and mute applied, stereo mixed down to a mono sink, one frame kept in
// every `decimate`. Then the spectrum meter, if one is attached, sees the
// result.
//
// The meter reads what actually plays. It used to read the host's frames
// before the volume multiply, so a host that sets the card's own volume
// control (Windows) left the meter at full scale at any slider position, even
// muted, while a host that scales its PCM itself (Android) moved it. Metering
// after the multiply makes every host look the same.
//
// Header-only so the pump compiles it inline and the host-side test
// (tests/test_pump_block.c) runs the same code.
#ifndef USBIF_PUMP_BLOCK_H
#define USBIF_PUMP_BLOCK_H

#include <stdbool.h>
#include <stdint.h>

// audiodsp's audiometer_uac_feed(), or NULL in a firmware without it.
typedef void (*usbif_pump_meter_fn)(const int16_t *frames, uint32_t n, uint32_t channels, uint32_t rate);

// Shapes `usable` bytes of whole host frames in `block`, in place, and returns
// the bytes to write to the wire. `gain` is Q16 (65536 is unity) and never
// above unity. `host_rate` is the host's rate; the meter gets the wire's.
static inline uint16_t usbif_pump_shape_block(uint8_t *block, uint16_t usable,
    uint8_t src_channels, uint8_t sink_channels, uint8_t decimate,
    uint32_t gain, uint32_t host_rate, usbif_pump_meter_fn meter) {
    const uint16_t frame_bytes = (uint16_t)(2 * src_channels);
    #ifdef USBIF_PUMP_PLANT_METER_BEFORE_GAIN
    // A planted fault for the test: the old order, metering the host's frames.
    if (meter) {
        meter((const int16_t *)(void *)block, usable / frame_bytes, src_channels, host_rate);
    }
    #endif
    uint16_t out_bytes = usable;
    const bool passthrough = (src_channels == sink_channels)
        && (decimate == 1) && (gain == 65536u);
    if (!passthrough) {
        const int16_t *in = (const int16_t *)(void *)block;
        int16_t *out = (int16_t *)(void *)block;
        const uint16_t frames = usable / frame_bytes;
        uint16_t kept = 0;
        // In-place is safe here because the write index never overtakes
        // the read index: stereo-to-stereo at decimate 1 writes exactly
        // where it read, and downmixing produces fewer samples than it
        // consumes. EXPANDING a mono source onto a stereo wire would NOT
        // be safe in place -- it emits two samples per frame consumed --
        // which is why pump_start rejects that combination outright
        // rather than leaving a trap here for whoever makes the USB
        // descriptor configurable.
        for (uint16_t f = 0; f < frames; f += decimate) {
            if (src_channels == 2 && sink_channels == 2) {
                // Stereo through to a stereo sink: keep both sides. The
                // old code averaged here regardless of the sink and fed
                // half the samples the hardware was clocking for.
                int32_t l = (int32_t)(((int64_t)in[f * 2] * gain) >> 16);
                int32_t r = (int32_t)(((int64_t)in[f * 2 + 1] * gain) >> 16);
                out[kept++] = (int16_t)l;
                out[kept++] = (int16_t)r;
                continue;
            }
            int32_t sample;
            if (src_channels == 2) {
                // Average rather than take one side: a mono sink fed only
                // the left channel loses anything panned right, which on
                // real music is most of it.
                sample = ((int32_t)in[f * 2] + (int32_t)in[f * 2 + 1]) / 2;
            } else {
                sample = in[f];
            }
            // Gain is <= unity (the advertised range tops out at 0 dB),
            // so the product cannot exceed int16 and needs no clamp.
            sample = (int32_t)(((int64_t)sample * gain) >> 16);
            out[kept++] = (int16_t)sample;
        }
        out_bytes = (uint16_t)(kept * 2);
    }
    #ifndef USBIF_PUMP_PLANT_METER_BEFORE_GAIN
    // With no Meter attached to audiometer.UAC this costs a call and a load.
    if (meter && out_bytes) {
        meter((const int16_t *)(void *)block, out_bytes / (2u * sink_channels),
            sink_channels, host_rate / decimate);
    }
    #endif
    return out_bytes;
}

#endif // USBIF_PUMP_BLOCK_H
