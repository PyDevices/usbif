// SPDX-License-Identifier: MIT
//
// Runtime service of the built-in configuration descriptor: the opt-in
// toggle (advertise the extension functions or only the plain built-ins)
// and, learned at high speed's expense, per-speed correction of the MIDI
// bulk endpoint sizes. A bulk endpoint must be 512 bytes at high speed and
// at most 64 at full speed; a static descriptor can only be right for one
// of them, and Windows enforces the rule -- it saw our 64-byte endpoints at
// high speed and refused the whole configuration, silently. The descriptor
// is therefore served from a RAM copy, patched at request time, when the
// negotiated speed is already known.
//
// This lives outside usbif_uac.c so that audio-less (slim) builds keep the
// same opt-in behaviour and the same speed correctness.

#include "py/mpconfig.h"
#include "py/mphal.h"

#if MICROPY_HW_ENABLE_USBDEV && defined(MICROPY_HW_USB_EXT_TUSB_CONFIG)

#include <string.h>

#include "tusb.h"
#include "mp_usbd.h"

// Whether the extension functions are advertised. Off at boot: a board that
// always enumerates as a sound card or instrument is not merely untidy (see
// usbif_uac.c's history with wedged hosts).
bool usbif_ext_enabled;

// Audio needs to reset its streaming state on a toggle; defined weakly so
// slim builds link without it.
void usbif_uac_on_ext_toggled(void) __attribute__((weak));

static uint8_t usbif_desc_buf[MP_USBD_BUILTIN_DESC_CFG_LEN];

#define USBIF_DESC_LEN_WITHOUT_EXT \
    (MP_USBD_BUILTIN_DESC_CFG_LEN - MICROPY_HW_USB_EXT_DESC_CFG_LEN)

const uint8_t *mp_usbd_builtin_desc_cfg_get(void) {
    if (!usbif_ext_enabled) {
        memcpy(usbif_desc_buf, mp_usbd_builtin_desc_cfg, USBIF_DESC_LEN_WITHOUT_EXT);
        // wTotalLength and bNumInterfaces, at their fixed offsets in the
        // configuration descriptor (USB 2.0 table 9-10).
        usbif_desc_buf[2] = (uint8_t)(USBIF_DESC_LEN_WITHOUT_EXT & 0xFF);
        usbif_desc_buf[3] = (uint8_t)(USBIF_DESC_LEN_WITHOUT_EXT >> 8);
        usbif_desc_buf[4] = (uint8_t)USBD_ITF_AUDIO;
        return usbif_desc_buf;
    }
    memcpy(usbif_desc_buf, mp_usbd_builtin_desc_cfg, MP_USBD_BUILTIN_DESC_CFG_LEN);
    #ifdef USBD_EP_MIDI_OUT
    // MIDI bulk endpoint sizes for the speed actually negotiated.
    const uint16_t mps = (tud_speed_get() == TUSB_SPEED_HIGH) ? 512 : 64;
    uint16_t off = 0;
    while (off + 2u <= MP_USBD_BUILTIN_DESC_CFG_LEN && usbif_desc_buf[off] >= 2) {
        const uint8_t dlen = usbif_desc_buf[off];
        if (usbif_desc_buf[off + 1] == TUSB_DESC_ENDPOINT && dlen >= 7) {
            const uint8_t addr = usbif_desc_buf[off + 2];
            if (addr == USBD_EP_MIDI_OUT || addr == USBD_EP_MIDI_IN) {
                usbif_desc_buf[off + 4] = (uint8_t)(mps & 0xFF);
                usbif_desc_buf[off + 5] = (uint8_t)(mps >> 8);
            }
        }
        off = (uint16_t)(off + dlen);
    }
    #endif
    return usbif_desc_buf;
}

// Must track the descriptor: runtime_dev_open() uses this to decide which
// interfaces belong to built-in drivers, and a bound that disagrees with the
// descriptor leaves an interface claimed by nobody.
uint8_t mp_usbd_builtin_itf_max(void) {
    return usbif_ext_enabled ? USBD_ITF_BUILTIN_MAX : (uint8_t)USBD_ITF_AUDIO;
}

bool usbif_ext_is_enabled(void) {
    return usbif_ext_enabled;
}

// Re-enumerate so the host re-reads the configuration. USB has no way to
// change identity in place; a detach and re-attach is the mechanism.
void usbif_ext_set_enabled(bool enable) {
    if (enable == usbif_ext_enabled) {
        return;
    }
    usbif_ext_enabled = enable;
    if (usbif_uac_on_ext_toggled) {
        usbif_uac_on_ext_toggled();
    }
    tud_disconnect();
    mp_hal_delay_ms(120);
    tud_connect();
}

#endif // MICROPY_HW_ENABLE_USBDEV && MICROPY_HW_USB_EXT_TUSB_CONFIG
