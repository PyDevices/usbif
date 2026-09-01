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

#include "shared/usbif_midi_packet.h"

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

static void usbif_midih_in_cb(usb_transfer_t *transfer) {
    if (!usbif_midih.open) {
        return;
    }
    if (transfer->status == USB_TRANSFER_STATUS_COMPLETED && transfer->actual_num_bytes >= 4) {
        // Unpack to a plain MIDI byte stream: the cable number and the CIN
        // are transport framing, and Python is handed the same bytes it
        // would see on a DIN cable. The codec is in shared/ and tested on
        // the host (tests/test_midi_packets.c) -- this is the same code
        // those tests exercise, not a parallel copy of it.
        uint8_t midi[USBIF_MIDI_MAX_MPS];
        size_t n = usbif_midi_unpack(transfer->data_buffer,
            (size_t)transfer->actual_num_bytes, midi, sizeof(midi));
        if (n) {
            usbif_midih_rx_push(midi, n);
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

// Pack a plain MIDI byte stream into USB-MIDI event packets and send one
// transfer. Returns how many input bytes were consumed, which can be fewer
// than offered -- the codec leaves a partial trailing message for the next
// call rather than sending half of it. See shared/usbif_midi_packet.h for
// that rule and the two deliberate refusals behind it.
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

    size_t consumed = 0;
    size_t packed = usbif_midi_pack(data, len,
        usbif_midih.xfer_out->data_buffer, USBIF_MIDI_MAX_MPS, &consumed);

    if (packed == 0) {
        return (int)consumed;
    }

    usbif_midih.xfer_out->device_handle = usbif_midih.dev;
    usbif_midih.xfer_out->bEndpointAddress = usbif_midih.ep_out;
    usbif_midih.xfer_out->num_bytes = (int)packed;
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

// Quiesce both pipes before letting go of the interface.
//
// The OUT pipe matters as much as the IN one, which is not obvious and was
// found on hardware: interface_release() refuses while any endpoint still
// has an in-flight URB, and a write submitted shortly before closing leaves
// exactly that. The release then fails silently -- nothing checks it -- the
// claim survives, and the *next* host_stop() reports
// ESP_ERR_INVALID_STATE from deregister, uninstall and device_free_all, a
// long way from the cause. Halting both pipes is the fix; checking the
// release's return is how it would have been caught sooner.
static void usbif_midih_quiesce(void) {
    usb_host_endpoint_halt(usbif_midih.dev, usbif_midih.ep_in);
    usb_host_endpoint_flush(usbif_midih.dev, usbif_midih.ep_in);
    usb_host_endpoint_clear(usbif_midih.dev, usbif_midih.ep_in);
    if (usbif_midih.ep_out) {
        usb_host_endpoint_halt(usbif_midih.dev, usbif_midih.ep_out);
        usb_host_endpoint_flush(usbif_midih.dev, usbif_midih.ep_out);
        usb_host_endpoint_clear(usbif_midih.dev, usbif_midih.ep_out);
    }
}

// Non-zero if the interface could not be released -- worth surfacing rather
// than discarding, since a stuck claim only shows up much later as a
// teardown failure.
uint8_t usbif_host_midi_release_failed;

static void usbif_midih_release(void) {
    usb_host_transfer_free(usbif_midih.xfer_in);
    usb_host_transfer_free(usbif_midih.xfer_out);
    usbif_midih.xfer_in = NULL;
    usbif_midih.xfer_out = NULL;

    // Retry, because the release can legitimately be early rather than
    // wrong. interface_release() refuses while any endpoint still has an
    // in-flight URB, and the URBs that halt/flush just cancelled are only
    // retired when the client event pump next runs -- which is a *different*
    // task from the one usually calling close(). Releasing immediately
    // therefore loses a race it did not have to enter.
    //
    // Measured on a real instrument: halting both pipes alone still left the
    // claim stuck (release_failed stayed 1), and the failure only showed up
    // later as ESP_ERR_INVALID_STATE from a host_stop() far away from here.
    // Waiting for the pump is the actual fix.
    esp_err_t err = ESP_FAIL;
    for (int i = 0; i < 25; i++) {
        err = usb_host_interface_release(usbif_host_client_get(),
            usbif_midih.dev, usbif_midih.itf);
        if (err == ESP_OK) {
            break;
        }
        vTaskDelay(1);      // one 10 ms tick; ~250 ms bound in total
    }
    usbif_host_midi_release_failed = (err == ESP_OK) ? 0 : 1;
}

void usbif_host_midi_close(void) {
    if (!usbif_midih.open) {
        return;
    }
    usbif_midih.open = false;
    vTaskDelay(pdMS_TO_TICKS(20));
    usbif_midih_quiesce();
    usbif_midih_release();
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
    usbif_midih_quiesce();
    for (int i = 0; i < 10; i++) {
        usb_host_client_handle_events(usbif_host_client_get(), pdMS_TO_TICKS(10));
    }
    usbif_midih_release();
}

void usbif_host_midi_on_dev_gone(usb_device_handle_t dev) {
    if (!usbif_midih.open || usbif_midih.dev != dev) {
        return;
    }
    // Surprise detach: release the claim first so the library can free the
    // device, then drain before freeing transfers. No endpoint operations --
    // the device is physically gone and they would be meaningless.
    usbif_midih.open = false;
    usb_host_interface_release(usbif_host_client_get(), dev, usbif_midih.itf);
    vTaskDelay(pdMS_TO_TICKS(20));
    usb_host_transfer_free(usbif_midih.xfer_in);
    usb_host_transfer_free(usbif_midih.xfer_out);
    usbif_midih.xfer_in = NULL;
    usbif_midih.xfer_out = NULL;
}

#endif // CONFIG_SOC_USB_OTG_SUPPORTED
