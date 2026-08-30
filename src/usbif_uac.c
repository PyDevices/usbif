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

#define USBIF_UAC_ENTITY_CLOCK (0x04)
#define USBIF_UAC_ENTITY_FEATURE_UNIT (0x02)

// The one rate the descriptor advertises. Kept as state rather than a constant
// because the host still performs a set-current on it, and answering a later
// get with a different value than the host set is a way to fail slowly.
static uint32_t usbif_uac_sample_rate = CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE;
static bool usbif_uac_mute;
static uint16_t usbif_uac_volume;   // 1/256 dB, as UAC2 specifies
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
static bool usbif_uac_enabled;

uint32_t usbif_uac_get_reqs;
uint32_t usbif_uac_set_reqs;
uint32_t usbif_uac_itf_sets;
uint32_t usbif_uac_unhandled;
uint32_t usbif_uac_overflows;

bool usbif_uac_is_streaming(void) {
    return usbif_uac_streaming;
}

bool usbif_uac_is_enabled(void) {
    return usbif_uac_enabled;
}

void usbif_uac_note_read(void) {
}

// The built-in configuration descriptor, with the audio function present or
// absent. The enabled form is the compile-time one; the disabled form is its
// prefix with two fields corrected, which avoids keeping a second descriptor
// in step with the first by hand.
static uint8_t usbif_desc_buf[MP_USBD_BUILTIN_DESC_CFG_LEN];

#define USBIF_DESC_LEN_WITHOUT_AUDIO \
    (MP_USBD_BUILTIN_DESC_CFG_LEN - MICROPY_HW_USB_EXT_DESC_CFG_LEN)

const uint8_t *mp_usbd_builtin_desc_cfg_get(void) {
    if (usbif_uac_enabled) {
        return mp_usbd_builtin_desc_cfg;
    }
    memcpy(usbif_desc_buf, mp_usbd_builtin_desc_cfg, USBIF_DESC_LEN_WITHOUT_AUDIO);
    // wTotalLength and bNumInterfaces, at their fixed offsets in the
    // configuration descriptor (USB 2.0 table 9-10).
    usbif_desc_buf[2] = (uint8_t)(USBIF_DESC_LEN_WITHOUT_AUDIO & 0xFF);
    usbif_desc_buf[3] = (uint8_t)(USBIF_DESC_LEN_WITHOUT_AUDIO >> 8);
    usbif_desc_buf[4] = (uint8_t)USBD_ITF_AUDIO;
    return usbif_desc_buf;
}

// Must track the descriptor: runtime_dev_open() uses this to decide which
// interfaces belong to built-in drivers, and a bound that disagrees with the
// descriptor leaves an interface claimed by nobody.
uint8_t mp_usbd_builtin_itf_max(void) {
    return usbif_uac_enabled ? USBD_ITF_BUILTIN_MAX : (uint8_t)USBD_ITF_AUDIO;
}

// Re-enumerate so the host re-reads the configuration. USB has no way to
// change identity in place; a detach and re-attach is the mechanism.
void usbif_uac_set_enabled(bool enable) {
    if (enable == usbif_uac_enabled) {
        return;
    }
    usbif_uac_enabled = enable;
    usbif_uac_streaming = false;
    tud_disconnect();
    mp_hal_delay_ms(120);
    tud_connect();
}

uint32_t usbif_uac_current_rate(void) {
    return usbif_uac_sample_rate;
}

// Host-side volume and mute, as the feature unit last received them. UAC2
// carries volume as a signed 1/256 dB value; converting that to whatever a
// codec wants is policy, so it happens in Python beside every other codec
// decision rather than here.
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
            return true;
        }
        if (request->bControlSelector == AUDIO_FU_CTRL_VOLUME) {
            usbif_uac_volume = tu_le16toh(((audio_control_cur_2_t *)buf)->bCur);
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
    feedback_param->method = AUDIO_FEEDBACK_METHOD_FIFO_COUNT;
    feedback_param->sample_freq = usbif_uac_sample_rate;
}

#endif // CFG_TUD_AUDIO
