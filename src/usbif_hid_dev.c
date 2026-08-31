// SPDX-License-Identifier: MIT
//
// HID as a *device*: the board presenting itself to a computer as a
// keyboard, a mouse, or any control surface an application cares to
// describe. The host direction has worked since Phase 2; this is the one
// the vision's board-to-board demos assumed and nobody had built.
//
// The report descriptor carries a boot keyboard and a boot mouse behind
// report IDs, which is what a host expects from a composite input device
// and what makes the two usable independently. Anything more exotic --
// a gamepad, a dial, a custom sensor collection -- is a different report
// descriptor rather than different code, and that is the natural place to
// grow when an application needs it.
//
// Reports are submitted from Python and sent by TinyUSB on its interrupt
// IN endpoint. Nothing is buffered here on purpose: a report that cannot
// be sent right now is a report the host has not polled for yet, and the
// honest answer to the caller is false rather than a queue that hides how
// far behind it is.

#include "py/mpconfig.h"

#if MICROPY_HW_ENABLE_USBDEV
#include "tusb.h"
#endif

#if defined(CFG_TUD_HID) && CFG_TUD_HID

#include <string.h>

// Report IDs, which are API: they appear in usbif's Python surface and in
// the report descriptor below.
#define USBIF_HID_REPORT_KEYBOARD (1)
#define USBIF_HID_REPORT_MOUSE    (2)

static const uint8_t usbif_hid_report_desc[] = {
    TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(USBIF_HID_REPORT_KEYBOARD)),
    TUD_HID_REPORT_DESC_MOUSE(HID_REPORT_ID(USBIF_HID_REPORT_MOUSE)),
};

uint16_t usbif_hid_report_desc_len(void) {
    return (uint16_t)sizeof(usbif_hid_report_desc);
}

// TinyUSB asks for the report descriptor by interface; this module
// contributes exactly one HID interface, so the index is not consulted.
const uint8_t *tud_hid_descriptor_report_cb(uint8_t instance) {
    (void)instance;
    return usbif_hid_report_desc;
}

// A host may poll for input by control transfer instead of waiting for the
// interrupt endpoint. Nothing here holds state to report, so the honest
// answer is "no data", which stalls the request rather than inventing one.
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id,
    hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen) {
    (void)instance;
    (void)report_id;
    (void)report_type;
    (void)buffer;
    (void)reqlen;
    return 0;
}

// Output reports from the host: for a keyboard this is the LED state
// (caps lock and friends). Kept, so Python can read what the host thinks
// the lock keys are doing -- a real thing a control surface wants to show.
static volatile uint8_t usbif_hid_leds;

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
    hid_report_type_t report_type, const uint8_t *buffer, uint16_t bufsize) {
    (void)instance;
    if (report_type == HID_REPORT_TYPE_OUTPUT
        && report_id == USBIF_HID_REPORT_KEYBOARD && bufsize >= 1) {
        usbif_hid_leds = buffer[0];
    }
}

uint8_t usbif_hid_get_leds(void) {
    return usbif_hid_leds;
}

// Submit one report. Returns false when the interface is not mounted, when
// the host has not polled since the last report, or when the previous one
// is still in flight -- all of which mean the same thing to a caller:
// not now, try again.
bool usbif_hid_send(uint8_t report_id, const uint8_t *data, uint16_t len) {
    if (!tud_hid_ready()) {
        return false;
    }
    return tud_hid_report(report_id, data, (uint8_t)len);
}

#endif // CFG_TUD_HID
