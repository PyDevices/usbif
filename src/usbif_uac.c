// SPDX-License-Identifier: MIT
//
// UAC2 control-request handling: what makes the host actually start the
// audio function rather than merely recognise it.
//
// Descriptors alone get a device as far as "Windows loaded usbaudio2.sys and
// then reported Code 10". A UAC2 host interrogates the clock source before it
// will stream -- current sample rate, the supported range, whether the clock
// is valid -- and TinyUSB routes those to weak callbacks that default to
// refusing. A refused clock query is a stalled control transfer, which the
// host reports as a device that cannot start.
//
// Entity IDs below are fixed by TUD_AUDIO_SPEAKER_MONO_FB_DESCRIPTOR, which
// numbers its clock source 4, feature unit 2, input terminal 1 and output
// terminal 3. They are spelled out here because the descriptor macro and this
// file must agree, and nothing in the compiler will notice if they stop.

#include <math.h>

#include "py/mpconfig.h"

// tusb.h must come first: CFG_TUD_AUDIO is defined by MicroPython's
// tusb_config.h (via usbif's extension header), not by mpconfig.h. Testing it
// before including this is silent -- the macro is simply undefined, the whole
// file compiles to nothing, TinyUSB falls back to its weak callbacks which
// refuse every control request, and the host reports a device that enumerates
// and then cannot start. That cost two wrong hypotheses to find.
#if MICROPY_HW_ENABLE_USBDEV
#include "tusb.h"
#include "mp_usbd.h"
#include "py/mphal.h"
#include <string.h>
#endif

#if defined(CFG_TUD_AUDIO) && CFG_TUD_AUDIO

extern bool usbif_pump_is_running(void);
extern void usbif_pump_notify(void);

#define USBIF_UAC_ENTITY_CLOCK (0x04)
#define USBIF_UAC_ENTITY_FEATURE_UNIT (0x02)

// The one rate the descriptor advertises. Kept as state rather than a constant
// because the host still performs a set-current on it, and answering a later
// get with a different value than the host set is a way to fail slowly.
static uint32_t usbif_uac_sample_rate = CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE;
static bool usbif_uac_mute;
static uint16_t usbif_uac_volume;   // 1/256 dB, as UAC2 specifies
// The volume and mute above, folded into one linear Q16 multiplier for the
// stream path. Recomputed here, in control context where a float pow is
// cheap and rare, so the pump only ever multiplies. 65536 is unity; the
// advertised range tops out at 0 dB, so gain never exceeds unity and the
// per-sample multiply cannot clip.
static volatile uint32_t usbif_uac_gain_q16 = 65536;

static void usbif_uac_update_gain(void) {
    if (usbif_uac_mute) {
        usbif_uac_gain_q16 = 0;
        return;
    }
    int16_t db256 = (int16_t)usbif_uac_volume;
    if (db256 >= 0) {
        usbif_uac_gain_q16 = 65536;
        return;
    }
    usbif_uac_gain_q16 = (uint32_t)(powf(10.0f, (float)db256 / (256.0f * 20.0f)) * 65536.0f + 0.5f);
}

uint32_t usbif_uac_gain(void) {
    return usbif_uac_gain_q16;
}
static bool usbif_uac_streaming;

// Diagnostic counters. The first question when a host refuses to start an
// audio function is whether the class driver is being asked anything at all:
// a descriptor the host dislikes and a driver that never bound look identical
// from outside, and they need opposite fixes.
// Whether the audio function is advertised at all. Default off: a board that
// always enumerates as a sound card is not merely untidy. A host will happily
// make it the default output, route system audio to it, and then stall its
// audio engine when nothing aboard is draining the stream -- observed on
// Windows as the whole desktop backing up, keyboard included.
extern uint16_t usbif_fn_get(void);   // owned by usbif_desc.c

uint32_t usbif_uac_get_reqs;
uint32_t usbif_uac_set_reqs;
uint32_t usbif_uac_itf_sets;
uint32_t usbif_uac_unhandled;
uint32_t usbif_uac_overflows;

bool usbif_uac_is_streaming(void) {
    return usbif_uac_streaming;
}

bool usbif_uac_is_enabled(void) {
    return (usbif_fn_get() & (1u << 2)) != 0;   // USBIF_FN_AUDIO
}

// Called by usbif_desc.c on an advertise/withdraw toggle (overrides its weak).
void usbif_uac_on_ext_toggled(void) {
    usbif_uac_streaming = false;
}

void usbif_uac_note_read(void) {
}

uint32_t usbif_uac_current_rate(void) {
    return usbif_uac_sample_rate;
}

// Host-side volume and mute, as the feature unit last received them. UAC2
// carries volume as a signed 1/256 dB value. Both are applied as digital gain
// in the stream path (usbif owns the stream, so the host's slider works on
// every board with no codec involvement); these getters stay for diagnostics
// and for boards that want to mirror the values onto a codec register --
// which would apply volume twice, so such a board should expect the pump's
// gain and account for it.
bool usbif_uac_is_muted(void) {
    return usbif_uac_mute;
}

int usbif_uac_volume_db256(void) {
    return (int16_t)usbif_uac_volume;
}

static bool clock_get(uint8_t rhport, audio_control_request_t const *request) {
    if (request->bControlSelector == AUDIO_CS_CTRL_SAM_FREQ) {
        if (request->bRequest == AUDIO_CS_REQ_CUR) {
            audio_control_cur_4_t cur = { .bCur = (int32_t)tu_htole32(usbif_uac_sample_rate) };
            return tud_audio_buffer_and_schedule_control_xfer(
                rhport, (tusb_control_request_t const *)request, &cur, sizeof(cur));
        }
        if (request->bRequest == AUDIO_CS_REQ_RANGE) {
            // A single discrete rate. Advertising a range we cannot actually
            // clock would move the failure from enumeration to playback,
            // where it is far harder to diagnose.
            audio_control_range_4_n_t(1) range = {
                .wNumSubRanges = tu_htole16(1),
                .subrange[0] = {
                    .bMin = (int32_t)tu_htole32(CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE),
                    .bMax = (int32_t)tu_htole32(CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE),
                    .bRes = (int32_t)tu_htole32(0),
                },
            };
            return tud_audio_buffer_and_schedule_control_xfer(
                rhport, (tusb_control_request_t const *)request, &range, sizeof(range));
        }
        return false;
    }
    if (request->bControlSelector == AUDIO_CS_CTRL_CLK_VALID
        && request->bRequest == AUDIO_CS_REQ_CUR) {
        audio_control_cur_1_t cur = { .bCur = 1 };
        return tud_audio_buffer_and_schedule_control_xfer(
            rhport, (tusb_control_request_t const *)request, &cur, sizeof(cur));
    }
    return false;
}

static bool feature_unit_get(uint8_t rhport, audio_control_request_t const *request) {
    if (request->bRequest != AUDIO_CS_REQ_CUR && request->bRequest != AUDIO_CS_REQ_RANGE) {
        return false;
    }
    if (request->bControlSelector == AUDIO_FU_CTRL_MUTE
        && request->bRequest == AUDIO_CS_REQ_CUR) {
        audio_control_cur_1_t cur = { .bCur = usbif_uac_mute ? 1 : 0 };
        return tud_audio_buffer_and_schedule_control_xfer(
            rhport, (tusb_control_request_t const *)request, &cur, sizeof(cur));
    }
    if (request->bControlSelector == AUDIO_FU_CTRL_VOLUME) {
        if (request->bRequest == AUDIO_CS_REQ_RANGE) {
            // -100 dB .. 0 dB in 1 dB steps, in UAC2's 1/256 dB units.
            audio_control_range_2_n_t(1) range = {
                .wNumSubRanges = tu_htole16(1),
                .subrange[0] = {
                    .bMin = tu_htole16((uint16_t)(-100 * 256)),
                    .bMax = tu_htole16(0),
                    .bRes = tu_htole16(256),
                },
            };
            return tud_audio_buffer_and_schedule_control_xfer(
                rhport, (tusb_control_request_t const *)request, &range, sizeof(range));
        }
        audio_control_cur_2_t cur = { .bCur = (int16_t)tu_htole16(usbif_uac_volume) };
        return tud_audio_buffer_and_schedule_control_xfer(
            rhport, (tusb_control_request_t const *)request, &cur, sizeof(cur));
    }
    return false;
}

bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
    audio_control_request_t const *request = (audio_control_request_t const *)p_request;
    usbif_uac_get_reqs++;
    if (request->bEntityID == USBIF_UAC_ENTITY_CLOCK) {
        if (clock_get(rhport, request)) {
            return true;
        }
    } else if (request->bEntityID == USBIF_UAC_ENTITY_FEATURE_UNIT) {
        if (feature_unit_get(rhport, request)) {
            return true;
        }
    }
    // Record the shape of what we refused: entity, selector, request, channel.
    usbif_uac_unhandled = 0x80000000u
        | ((uint32_t)request->bEntityID << 16)
        | ((uint32_t)request->bControlSelector << 8)
        | (uint32_t)request->bRequest;
    return false;
}

bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request,
    uint8_t *buf) {
    (void)rhport;
    audio_control_request_t const *request = (audio_control_request_t const *)p_request;
    usbif_uac_set_reqs++;

    if (request->bEntityID == USBIF_UAC_ENTITY_CLOCK
        && request->bControlSelector == AUDIO_CS_CTRL_SAM_FREQ) {
        usbif_uac_sample_rate = tu_le32toh(((audio_control_cur_4_t *)buf)->bCur);
        return true;
    }
    if (request->bEntityID == USBIF_UAC_ENTITY_FEATURE_UNIT) {
        if (request->bControlSelector == AUDIO_FU_CTRL_MUTE) {
            usbif_uac_mute = ((audio_control_cur_1_t *)buf)->bCur != 0;
            usbif_uac_update_gain();
            return true;
        }
        if (request->bControlSelector == AUDIO_FU_CTRL_VOLUME) {
            usbif_uac_volume = tu_le16toh(((audio_control_cur_2_t *)buf)->bCur);
            usbif_uac_update_gain();
            return true;
        }
    }
    return false;
}

// The host switches to alternate setting 1 to start streaming and back to 0 to
// stop; that transition is the only reliable "is audio running" signal, and it
// is what usbif.device reports to Python as `active`.
bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
    (void)rhport;
    usbif_uac_itf_sets++;
    usbif_uac_streaming = tu_u16_low(tu_le16toh(p_request->wValue)) != 0;
    return true;
}

bool tud_audio_set_itf_close_EP_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
    (void)rhport;
    (void)p_request;
    usbif_uac_streaming = false;
    return true;
}

// Explicit feedback: the host's clock and ours are independent, so the device
// reports how fast it is actually consuming samples and the host adjusts. FIFO
// counting is TinyUSB's simplest method and needs no timer capture hardware.
// Called after each received packet. If nothing is draining the FIFO it fills,
// the FIFO-count feedback tells the host to slow down, and the host ends up
// waiting on a device that will never catch up -- which on Windows stalled the
// audio engine badly enough to back up the whole desktop.
//
// The trigger is a high-water mark rather than a "no consumer for N ms" timer.
// A timer deadlocks against any consumer that batches: draining on a timeout
// keeps the FIFO empty, a consumer waiting for a 20 ms block never sees one,
// so it never reads, so the timeout never clears. (Observed: the pump moved
// exactly zero bytes.) A high-water mark cannot deadlock -- it does nothing
// at all while a consumer keeps up, whatever its block size, and only sheds
// samples that were never going to be played.
#define USBIF_UAC_HIGH_WATER (CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ * 3 / 4)

bool tud_audio_rx_done_post_read_cb(uint8_t rhport, uint16_t n_bytes_received,
    uint8_t func_id, uint8_t ep_out, uint8_t cur_alt_setting) {
    (void)rhport;
    (void)n_bytes_received;
    (void)func_id;
    (void)ep_out;
    (void)cur_alt_setting;

    // Only when no C pump exists. tud_audio_read() has one consumer by design,
    // and this guard reading the same FIFO makes a second one -- it shed 371
    // blocks out from under the pump and halved throughput. The pump therefore
    // carries its own overflow guard, in its own loop, and this one covers the
    // case where the only consumer is Python or there is none at all.
    // Wake the C pump the instant audio lands, so it never has to sleep
    // through a FIFO smaller than a scheduler tick.
    if (usbif_pump_is_running()) {
        usbif_pump_notify();
        return true;
    }

    if (tud_audio_available() > USBIF_UAC_HIGH_WATER) {
        static uint8_t sink[64];
        usbif_uac_overflows++;
        while (tud_audio_available() > USBIF_UAC_HIGH_WATER / 2) {
            if (!tud_audio_read(sink, sizeof(sink))) {
                break;
            }
        }
    }
    return true;
}

void tud_audio_feedback_params_cb(uint8_t func_id, uint8_t alt_itf,
    audio_feedback_params_t *feedback_param) {
    (void)func_id;
    (void)alt_itf;
    // No feedback data: the host transmits at nominal rate. This is deliberate
    // and measured, not an omission. AUDIO_FEEDBACK_METHOD_FIFO_COUNT regulates
    // the FIFO toward half-full, but the C pump drains the FIFO the moment
    // anything lands, so the level the regulator samples is set by sampling
    // phase and pump latency, not by the host's rate. Hunting for an
    // unreachable equilibrium, it asked the host for ~4% less than nominal
    // (23,030 Hz of 24,000 at the codec -- heard as drag and skipping). With
    // feedback disabled: 23,960 Hz, zero overflows, zero sink timeouts over an
    // 88.9 s window. The feedback EP stays advertised and silent -- TinyUSB's
    // own default when this callback is not defined, and hosts fall back to
    // nominal rate. Honest device-computed feedback (a slow integrator on the
    // FIFO trend, via tud_audio_fb_set) is future work; see
    // docs/phase0-findings.md for the full account.
    feedback_param->method = AUDIO_FEEDBACK_METHOD_DISABLED;
    feedback_param->sample_freq = usbif_uac_sample_rate;
}

#endif // CFG_TUD_AUDIO
