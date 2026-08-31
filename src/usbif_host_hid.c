// SPDX-License-Identifier: MIT
//
// HID host: the second class driver, and a slimmer sibling of the CDC one.
// A HID device pushes fixed-size input reports up an interrupt IN pipe at
// its own cadence; the host's whole job is to keep that pipe primed and
// deliver the reports. They land in a byte ring as length-prefixed records
// (one length byte, then the report), because report size is a property of
// the device -- a boot mouse sends 4 bytes, a keyboard 8, and Python should
// not have to guess where one ends.
//
// Boot/report protocol, SET_IDLE, and report descriptors are deliberately
// not parsed here yet: the reports arrive fine without any of it for the
// devices this phase tests against, and interpretation is policy that
// belongs above this layer.

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

#define USBIF_HID_RX_RING (512)
#define USBIF_HID_MAX_MPS (64)

typedef struct {
    bool open;
    usb_device_handle_t dev;
    uint8_t itf;
    uint8_t ep_in;
    uint16_t mps_in;
    usb_transfer_t *xfer_in;
    uint8_t rx[USBIF_HID_RX_RING];
    volatile uint16_t rx_head, rx_tail;
    volatile uint32_t rx_dropped;
} usbif_hid_t;

static usbif_hid_t usbif_hid;

static void usbif_hid_rx_push(const uint8_t *data, size_t len) {
    // Length-prefixed record; drop whole reports, never halves.
    uint16_t free_space = (uint16_t)((usbif_hid.rx_tail - usbif_hid.rx_head - 1 + USBIF_HID_RX_RING) % USBIF_HID_RX_RING);
    if (len + 1 > free_space || len > 255) {
        usbif_hid.rx_dropped++;
        return;
    }
    uint16_t h = usbif_hid.rx_head;
    usbif_hid.rx[h] = (uint8_t)len;
    h = (uint16_t)((h + 1) % USBIF_HID_RX_RING);
    for (size_t i = 0; i < len; i++) {
        usbif_hid.rx[h] = data[i];
        h = (uint16_t)((h + 1) % USBIF_HID_RX_RING);
    }
    usbif_hid.rx_head = h;
}

static void usbif_hid_in_cb(usb_transfer_t *transfer) {
    if (!usbif_hid.open) {
        return;
    }
    if (transfer->status == USB_TRANSFER_STATUS_COMPLETED && transfer->actual_num_bytes > 0) {
        usbif_hid_rx_push(transfer->data_buffer, (size_t)transfer->actual_num_bytes);
    }
    if (transfer->status == USB_TRANSFER_STATUS_COMPLETED
        || transfer->status == USB_TRANSFER_STATUS_TIMED_OUT) {
        usb_host_transfer_submit(transfer);
    }
}

static bool usbif_hid_find_itf(const usb_config_desc_t *cfg) {
    const uint8_t *p = (const uint8_t *)cfg;
    const uint8_t *end = p + cfg->wTotalLength;
    int cur_itf = -1;
    bool have = false;
    p += cfg->bLength;
    while (p + 2 <= end && p[0] >= 2 && p + p[0] <= end) {
        if (p[1] == USB_B_DESCRIPTOR_TYPE_INTERFACE && p[0] >= 9) {
            cur_itf = p[2];
            if (p[5] == USB_CLASS_HID && !have) {
                usbif_hid.itf = (uint8_t)cur_itf;
                have = true;
            }
        }
        if (p[1] == USB_B_DESCRIPTOR_TYPE_ENDPOINT && p[0] >= 7
            && have && cur_itf == usbif_hid.itf
            && (p[3] & 0x03) == USB_TRANSFER_TYPE_INTR && (p[2] & 0x80)) {
            usbif_hid.ep_in = p[2];
            usbif_hid.mps_in = (uint16_t)(p[4] | (p[5] << 8));
        }
        p += p[0];
    }
    return have && usbif_hid.ep_in;
}

int usbif_hid_open(uint32_t dev_id) {
    if (usbif_hid.open) {
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
    memset(&usbif_hid, 0, sizeof(usbif_hid));
    usbif_hid.dev = dev;
    if (!usbif_hid_find_itf(cfg)) {
        return -4;
    }
    if (usbif_hid.mps_in > USBIF_HID_MAX_MPS) {
        return -5;
    }
    if (usb_host_interface_claim(usbif_host_client_get(), dev, usbif_hid.itf, 0) != ESP_OK) {
        return -6;
    }
    if (usb_host_transfer_alloc(USBIF_HID_MAX_MPS, 0, &usbif_hid.xfer_in) != ESP_OK) {
        usb_host_interface_release(usbif_host_client_get(), dev, usbif_hid.itf);
        return -7;
    }
    usbif_hid.open = true;
    usbif_hid.xfer_in->device_handle = dev;
    usbif_hid.xfer_in->bEndpointAddress = usbif_hid.ep_in;
    usbif_hid.xfer_in->num_bytes = usbif_hid.mps_in;
    usbif_hid.xfer_in->callback = usbif_hid_in_cb;
    if (usb_host_transfer_submit(usbif_hid.xfer_in) != ESP_OK) {
        usbif_hid.open = false;
        return -8;
    }
    return 0;
}

// Pop one length-prefixed report; returns its length, 0 if none waiting.
int usbif_hid_read(uint8_t *out, size_t max) {
    if (!usbif_hid.open) {
        return -1;
    }
    if (usbif_hid.rx_tail == usbif_hid.rx_head) {
        return 0;
    }
    uint16_t t = usbif_hid.rx_tail;
    uint8_t len = usbif_hid.rx[t];
    t = (uint16_t)((t + 1) % USBIF_HID_RX_RING);
    if (len > max) {
        // Caller's buffer too small: consume and truncate rather than jam.
        for (uint8_t i = 0; i < len; i++) {
            if (i < max) {
                out[i] = usbif_hid.rx[t];
            }
            t = (uint16_t)((t + 1) % USBIF_HID_RX_RING);
        }
        usbif_hid.rx_tail = t;
        return (int)max;
    }
    for (uint8_t i = 0; i < len; i++) {
        out[i] = usbif_hid.rx[t];
        t = (uint16_t)((t + 1) % USBIF_HID_RX_RING);
    }
    usbif_hid.rx_tail = t;
    return (int)len;
}

void usbif_hid_close(void) {
    if (!usbif_hid.open) {
        return;
    }
    usbif_hid.open = false;
    vTaskDelay(pdMS_TO_TICKS(20));
    usb_host_endpoint_halt(usbif_hid.dev, usbif_hid.ep_in);
    usb_host_endpoint_flush(usbif_hid.dev, usbif_hid.ep_in);
    usb_host_endpoint_clear(usbif_hid.dev, usbif_hid.ep_in);
    usb_host_transfer_free(usbif_hid.xfer_in);
    usb_host_interface_release(usbif_host_client_get(), usbif_hid.dev, usbif_hid.itf);
}

void usbif_hid_on_dev_gone(usb_device_handle_t dev) {
    if (!usbif_hid.open || usbif_hid.dev != dev) {
        return;
    }
    // Same order as the CDC driver: stop the callbacks resubmitting, release
    // the claim so the library can free the device (holding it is what
    // wedged re-enumeration after the first surprise detach), let any queued
    // callback drain, then free.
    usbif_hid.open = false;
    usb_host_interface_release(usbif_host_client_get(), dev, usbif_hid.itf);
    vTaskDelay(pdMS_TO_TICKS(20));
    usb_host_transfer_free(usbif_hid.xfer_in);
    usbif_hid.xfer_in = NULL;
}

uint32_t usbif_hid_rx_dropped(void) {
    return usbif_hid.rx_dropped;
}

#endif // CONFIG_SOC_USB_OTG_SUPPORTED
