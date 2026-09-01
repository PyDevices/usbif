// SPDX-License-Identifier: MIT
//
// USB-MIDI host: the fourth class driver, and the one nobody else supplies.
// Espressif's esp-usb family ships CDC-ACM, HID, MSC, UAC and UVC host
// drivers and no MIDI one; the only upstream artifact is an unmerged,
// receive-only example, whose discussion has a maintainer saying plainly
// that MIDI host support does not exist and is not planned (recorded in
// docs/phase0-findings.md, the "Spike D" build-or-borrow decision). So this
// is built, in the same ~250-lines-we-own spirit as the CDC driver it is
// modelled on.
//
// The wire format is the only genuinely new thing here. USB-MIDI 1.0 does
// not carry a raw MIDI byte stream: it carries fixed 32-bit *event
// packets*, each one [cable<<4 | CIN][byte0][byte1][byte2], where CIN is a
// Code Index Number naming what kind of message the packet holds and,
// crucially, how many of the three data bytes are real. A three-byte
// note-on and a one-byte clock tick occupy the same four bytes on the wire.
//
// This driver's job is to make that invisible: Python reads and writes
// ordinary MIDI bytes, exactly as it already does for the *device* side
// (`midi_read`/`midi_write`), so an application that harmonises or logs
// MIDI does not care which end of the cable it is on. Packing and unpacking
// happen here.
//
// Scope, deliberately matched to the CDC driver: one MIDI session at a
// time, the MIDIStreaming interface's two bulk pipes, received bytes in a
// ring drained from Python so the producer never waits on the VM.

#include "py/mpconfig.h"

#if defined(__has_include)
#if __has_include("sdkconfig.h")
#include "sdkconfig.h"
#endif
#endif

#if defined(CONFIG_SOC_USB_OTG_SUPPORTED) && CONFIG_SOC_USB_OTG_SUPPORTED

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "usb/usb_host.h"

extern usb_host_client_handle_t usbif_host_client_get(void);
extern int usbif_host_dev_lookup(uint32_t dev_id, usb_device_handle_t *out);

#define USBIF_MIDI_RX_RING (512)
#define USBIF_MIDI_MAX_MPS (64)

// MIDIStreaming is a *subclass* of audio, not a class of its own -- the
// single most common way a USB MIDI device gets mislabelled, and the same
// distinction the portable API's class_from_interface() already makes.
#define USBIF_MIDI_SUBCLASS_MIDISTREAMING (0x03)

typedef struct {
    bool open;
    usb_device_handle_t dev;
    uint8_t itf;
    uint8_t ep_in, ep_out;
    uint16_t mps_in, mps_out;
    usb_transfer_t *xfer_in;
    usb_transfer_t *xfer_out;
    volatile bool out_busy;
    uint8_t rx[USBIF_MIDI_RX_RING];
    volatile uint16_t rx_head, rx_tail;
    volatile uint32_t rx_dropped;
} usbif_midi_host_t;

static usbif_midi_host_t usbif_midih;

static void usbif_midih_rx_push(const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        uint16_t next = (uint16_t)((usbif_midih.rx_head + 1) % USBIF_MIDI_RX_RING);
        if (next == usbif_midih.rx_tail) {
            usbif_midih.rx_dropped++;
            continue;
        }
        usbif_midih.rx[usbif_midih.rx_head] = data[i];
        usbif_midih.rx_head = next;
    }
}

// How many of a packet's three data bytes are real, by Code Index Number.
// Zero means "not a message we forward" (CIN 0 and 1 are reserved for
// vendor and cable events). Table rather than a switch because the mapping
// is the specification's, not a policy of ours -- USB-MIDI 1.0 table 4-1.
static const uint8_t usbif_midi_cin_len[16] = {
    0,  // 0x0 misc / reserved
    0,  // 0x1 cable event / reserved
    2,  // 0x2 two-byte system common
    3,  // 0x3 three-byte system common
    3,  // 0x4 SysEx starts or continues
    1,  // 0x5 SysEx ends with one byte, or single-byte system common
    2,  // 0x6 SysEx ends with two bytes
    3,  // 0x7 SysEx ends with three bytes
    3,  // 0x8 note off
    3,  // 0x9 note on
    3,  // 0xA poly key pressure
    3,  // 0xB control change
    2,  // 0xC program change
    2,  // 0xD channel pressure
    3,  // 0xE pitch bend
    1,  // 0xF single byte (realtime)
};

static void usbif_midih_in_cb(usb_transfer_t *transfer) {
    if (!usbif_midih.open) {
        return;
    }
    if (transfer->status == USB_TRANSFER_STATUS_COMPLETED && transfer->actual_num_bytes >= 4) {
        const uint8_t *p = transfer->data_buffer;
        // A single IN transfer can carry several packets; walk all of them.
        for (int i = 0; i + 4 <= transfer->actual_num_bytes; i += 4) {
            uint8_t cin = p[i] & 0x0F;
            uint8_t n = usbif_midi_cin_len[cin];
            if (n) {
                // Unpack to a plain MIDI byte stream: the cable number and
                // the CIN are transport framing, and Python is given the
                // same bytes it would see on a DIN cable.
                usbif_midih_rx_push(&p[i + 1], n);
            }
        }
    }
    if (transfer->status == USB_TRANSFER_STATUS_COMPLETED
        || transfer->status == USB_TRANSFER_STATUS_TIMED_OUT) {
        usb_host_transfer_submit(transfer);
    }
}

static void usbif_midih_out_cb(usb_transfer_t *transfer) {
    (void)transfer;
    usbif_midih.out_busy = false;
}

static bool usbif_midih_find_itf(const usb_config_desc_t *cfg) {
    const uint8_t *p = (const uint8_t *)cfg;
    const uint8_t *end = p + cfg->wTotalLength;
    int cur_itf = -1;
    bool have = false;
    p += cfg->bLength;
    while (p + 2 <= end && p[0] >= 2 && p + p[0] <= end) {
        if (p[1] == USB_B_DESCRIPTOR_TYPE_INTERFACE && p[0] >= 9) {
            cur_itf = p[2];
            if (p[5] == USB_CLASS_AUDIO
                && p[6] == USBIF_MIDI_SUBCLASS_MIDISTREAMING && !have) {
                usbif_midih.itf = (uint8_t)cur_itf;
                have = true;
            }
        }
        if (p[1] == USB_B_DESCRIPTOR_TYPE_ENDPOINT && p[0] >= 7
            && have && cur_itf == usbif_midih.itf
            && (p[3] & 0x03) == USB_TRANSFER_TYPE_BULK) {
            uint8_t addr = p[2];
            uint16_t mps = (uint16_t)(p[4] | (p[5] << 8));
            if (addr & 0x80) {
                usbif_midih.ep_in = addr;
                usbif_midih.mps_in = mps;
            } else {
                usbif_midih.ep_out = addr;
                usbif_midih.mps_out = mps;
            }
        }
        p += p[0];
    }
    // An IN pipe alone is a usable device (a keyboard that only sends), so
    // the OUT pipe is not required here -- usbif_host_midi_write() refuses
    // if it is absent rather than this refusing to open at all.
    return have && usbif_midih.ep_in;
}

int usbif_host_midi_open(uint32_t dev_id) {
    if (usbif_midih.open) {
        return -1;
    }
    usb_device_handle_t dev;
    if (usbif_host_dev_lookup(dev_id, &dev) != 0) {
        return -2;
    }
    const usb_config_desc_t *cfg;
    if (usb_host_get_active_config_descriptor(dev, &cfg) != ESP_OK) {
        return -3;
    }
    memset(&usbif_midih, 0, sizeof(usbif_midih));
    usbif_midih.dev = dev;
    if (!usbif_midih_find_itf(cfg)) {
        return -4;
    }
    if (usbif_midih.mps_in > USBIF_MIDI_MAX_MPS) {
        return -5;
    }
    if (usb_host_interface_claim(usbif_host_client_get(), dev, usbif_midih.itf, 0) != ESP_OK) {
        return -6;
    }
    if (usb_host_transfer_alloc(USBIF_MIDI_MAX_MPS, 0, &usbif_midih.xfer_in) != ESP_OK
        || usb_host_transfer_alloc(USBIF_MIDI_MAX_MPS, 0, &usbif_midih.xfer_out) != ESP_OK) {
        usb_host_transfer_free(usbif_midih.xfer_in);
        usb_host_transfer_free(usbif_midih.xfer_out);
        usb_host_interface_release(usbif_host_client_get(), dev, usbif_midih.itf);
        return -7;
    }
    usbif_midih.open = true;

    usbif_midih.xfer_in->device_handle = dev;
    usbif_midih.xfer_in->bEndpointAddress = usbif_midih.ep_in;
    usbif_midih.xfer_in->num_bytes = usbif_midih.mps_in;
    usbif_midih.xfer_in->callback = usbif_midih_in_cb;
    if (usb_host_transfer_submit(usbif_midih.xfer_in) != ESP_OK) {
        usbif_midih.open = false;
        return -8;
    }
    return 0;
}

// Pop decoded MIDI bytes. Same shape as usbif_cdc_read: whatever has
// arrived, up to max.
int usbif_host_midi_read(uint8_t *out, size_t max) {
    if (!usbif_midih.open) {
        return -1;
    }
    size_t n = 0;
    while (n < max && usbif_midih.rx_tail != usbif_midih.rx_head) {
        out[n++] = usbif_midih.rx[usbif_midih.rx_tail];
        usbif_midih.rx_tail = (uint16_t)((usbif_midih.rx_tail + 1) % USBIF_MIDI_RX_RING);
    }
    return (int)n;
}

// Pack a plain MIDI byte stream into USB-MIDI event packets.
//
// Only complete messages are sent: a trailing partial message is left
// unsent and its byte count is *not* reported as written, so the caller
// can hand the remainder in again with the rest appended. Sending a
// half-message would put a synthesiser into a state no later byte can
// explain, which is a far worse failure than a short write.
//
// SysEx is deliberately not handled yet: it needs state carried across
// calls (a message can span many writes) and no fixture on this bench
// sends it. It is refused rather than mangled -- see the return below.
int usbif_host_midi_write(const uint8_t *data, size_t len) {
    if (!usbif_midih.open) {
        return -1;
    }
    if (!usbif_midih.ep_out) {
        return -3;      // receive-only device (a keyboard, typically)
    }
    if (len == 0) {
        return 0;
    }
    for (int i = 0; i < 20 && usbif_midih.out_busy; i++) {
        vTaskDelay(1);
    }
    if (usbif_midih.out_busy) {
        return 0;
    }

    uint8_t *dst = usbif_midih.xfer_out->data_buffer;
    size_t packets = 0;
    size_t consumed = 0;
    const size_t max_packets = USBIF_MIDI_MAX_MPS / 4;

    while (consumed < len && packets < max_packets) {
        uint8_t status = data[consumed];
        if (status < 0x80) {
            // A data byte where a status byte belongs: running status is
            // not reconstructed here, and guessing would invent messages.
            // Skip it rather than emit something the caller did not say.
            consumed++;
            continue;
        }
        if (status >= 0xF8) {
            // System realtime: one byte, CIN 0xF, and legal *between* the
            // bytes of another message, which is why it is handled first.
            dst[packets * 4 + 0] = 0x0F;
            dst[packets * 4 + 1] = status;
            dst[packets * 4 + 2] = 0;
            dst[packets * 4 + 3] = 0;
            packets++;
            consumed++;
            continue;
        }
        if (status == 0xF0) {
            return (int)consumed;   // SysEx: unsupported, see the note above
        }

        uint8_t hi = (uint8_t)(status >> 4);
        // Program change and channel pressure are two bytes; every other
        // channel message is three.
        size_t need = (hi == 0x0C || hi == 0x0D) ? 2 : 3;
        if (consumed + need > len) {
            break;      // partial message: leave it for the next call
        }
        dst[packets * 4 + 0] = hi;      // cable 0, CIN == the status nibble
        dst[packets * 4 + 1] = status;
        dst[packets * 4 + 2] = data[consumed + 1];
        dst[packets * 4 + 3] = (need == 3) ? data[consumed + 2] : 0;
        packets++;
        consumed += need;
    }

    if (packets == 0) {
        return (int)consumed;
    }

    usbif_midih.xfer_out->device_handle = usbif_midih.dev;
    usbif_midih.xfer_out->bEndpointAddress = usbif_midih.ep_out;
    usbif_midih.xfer_out->num_bytes = (int)(packets * 4);
    usbif_midih.xfer_out->callback = usbif_midih_out_cb;
    usbif_midih.out_busy = true;
    if (usb_host_transfer_submit(usbif_midih.xfer_out) != ESP_OK) {
        usbif_midih.out_busy = false;
        return -2;
    }
    return (int)consumed;
}

uint32_t usbif_host_midi_rx_dropped(void) {
    return usbif_midih.rx_dropped;
}

void usbif_host_midi_close(void) {
    if (!usbif_midih.open) {
        return;
    }
    usbif_midih.open = false;
    vTaskDelay(pdMS_TO_TICKS(20));
    usb_host_endpoint_halt(usbif_midih.dev, usbif_midih.ep_in);
    usb_host_endpoint_flush(usbif_midih.dev, usbif_midih.ep_in);
    usb_host_endpoint_clear(usbif_midih.dev, usbif_midih.ep_in);
    usb_host_transfer_free(usbif_midih.xfer_in);
    usb_host_transfer_free(usbif_midih.xfer_out);
    usb_host_interface_release(usbif_host_client_get(), usbif_midih.dev, usbif_midih.itf);
}

// Teardown-only variant, for usbif_host.c's host_stop() path. Same reason
// as the CDC and HID ones: the halt/flush completions are asynchronous and
// are normally retired by the host task's own loop, which has already
// exited by the time teardown runs from inside that task.
void usbif_host_midi_close_for_host_stop(void) {
    if (!usbif_midih.open) {
        return;
    }
    usbif_midih.open = false;
    vTaskDelay(pdMS_TO_TICKS(20));
    usb_host_endpoint_halt(usbif_midih.dev, usbif_midih.ep_in);
    usb_host_endpoint_flush(usbif_midih.dev, usbif_midih.ep_in);
    usb_host_endpoint_clear(usbif_midih.dev, usbif_midih.ep_in);
    for (int i = 0; i < 10; i++) {
        usb_host_client_handle_events(usbif_host_client_get(), pdMS_TO_TICKS(10));
    }
    usb_host_transfer_free(usbif_midih.xfer_in);
    usb_host_transfer_free(usbif_midih.xfer_out);
    usb_host_interface_release(usbif_host_client_get(), usbif_midih.dev, usbif_midih.itf);
}

void usbif_host_midi_on_dev_gone(usb_device_handle_t dev) {
    if (!usbif_midih.open || usbif_midih.dev != dev) {
        return;
    }
    usbif_midih.open = false;
    usb_host_interface_release(usbif_host_client_get(), dev, usbif_midih.itf);
    vTaskDelay(pdMS_TO_TICKS(20));
    usb_host_transfer_free(usbif_midih.xfer_in);
    usb_host_transfer_free(usbif_midih.xfer_out);
    usbif_midih.xfer_in = NULL;
    usbif_midih.xfer_out = NULL;
}

#endif // CONFIG_SOC_USB_OTG_SUPPORTED
