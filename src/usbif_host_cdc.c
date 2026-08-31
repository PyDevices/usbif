// SPDX-License-Identifier: MIT
//
// CDC-ACM host: the first class driver, built directly on the IDF host
// library's transfer API rather than a managed component -- ~250 lines we
// own outright, in the same spirit as the rest of this module.
//
// Scope, deliberately minimal: one CDC session at a time, the data
// interface's two bulk pipes, and the one control request (DTR/RTS) that a
// MicroPython VCP wants before it treats its terminal as open. Received
// bytes land in a byte ring drained from Python, mirroring the event
// transport's design: the producer (the host task, where transfer
// callbacks run) never waits on the VM, and a late read costs latency
// rather than data. Overflow drops the oldest unread bytes and counts.

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

// From usbif_host.c: the client handle and a device lookup by the dev_id
// the event transport reported to Python.
extern usb_host_client_handle_t usbif_host_client_get(void);
extern int usbif_host_dev_lookup(uint32_t dev_id, usb_device_handle_t *out);

#define USBIF_CDC_RX_RING (1024)
#define USBIF_CDC_MAX_MPS (64)     // FS bulk maximum; HS devices wait for HS host work

typedef struct {
    bool open;
    usb_device_handle_t dev;
    uint8_t comm_itf;      // CDC control interface (for SET_CONTROL_LINE_STATE)
    uint8_t data_itf;      // CDC data interface (claimed)
    uint8_t ep_in, ep_out;
    uint16_t mps_in, mps_out;
    usb_transfer_t *xfer_in;
    usb_transfer_t *xfer_out;
    usb_transfer_t *xfer_ctrl;
    volatile bool out_busy;
    volatile bool ctrl_busy;
    // RX byte ring: written only by transfer callbacks (host task), read
    // only by Python (VM task). Same SPSC discipline as the event ring.
    uint8_t rx[USBIF_CDC_RX_RING];
    volatile uint16_t rx_head, rx_tail;
    volatile uint32_t rx_dropped;
} usbif_cdc_t;

static usbif_cdc_t usbif_cdc;

static void usbif_cdc_rx_push(const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        uint16_t next = (uint16_t)((usbif_cdc.rx_head + 1) % USBIF_CDC_RX_RING);
        if (next == usbif_cdc.rx_tail) {
            usbif_cdc.rx_dropped++;
            continue;   // full: drop, counted
        }
        usbif_cdc.rx[usbif_cdc.rx_head] = data[i];
        usbif_cdc.rx_head = next;
    }
}

static void usbif_cdc_in_cb(usb_transfer_t *transfer) {
    if (!usbif_cdc.open) {
        return;         // closing or device gone: do not resubmit
    }
    if (transfer->status == USB_TRANSFER_STATUS_COMPLETED && transfer->actual_num_bytes > 0) {
        usbif_cdc_rx_push(transfer->data_buffer, (size_t)transfer->actual_num_bytes);
    }
    if (transfer->status == USB_TRANSFER_STATUS_COMPLETED
        || transfer->status == USB_TRANSFER_STATUS_TIMED_OUT) {
        usb_host_transfer_submit(transfer);   // keep the pipe primed
    }
    // Any other status (STALL, NO_DEVICE, CANCELED): stop resubmitting; a
    // read after that returns only what is already in the ring, which is
    // the honest answer.
}

static void usbif_cdc_out_cb(usb_transfer_t *transfer) {
    (void)transfer;
    usbif_cdc.out_busy = false;
}

static void usbif_cdc_ctrl_cb(usb_transfer_t *transfer) {
    (void)transfer;
    usbif_cdc.ctrl_busy = false;
}

// Walk the active configuration for the CDC control (0x02) and data (0x0A)
// interfaces and the data interface's bulk endpoint pair.
static bool usbif_cdc_find_itfs(const usb_config_desc_t *cfg) {
    const uint8_t *p = (const uint8_t *)cfg;
    const uint8_t *end = p + cfg->wTotalLength;
    int cur_itf = -1;
    bool have_data = false, have_comm = false;
    p += cfg->bLength;
    while (p + 2 <= end && p[0] >= 2 && p + p[0] <= end) {
        if (p[1] == USB_B_DESCRIPTOR_TYPE_INTERFACE && p[0] >= 9) {
            cur_itf = p[2];
            if (p[5] == USB_CLASS_COMM && !have_comm) {
                usbif_cdc.comm_itf = (uint8_t)cur_itf;
                have_comm = true;
            }
            if (p[5] == USB_CLASS_CDC_DATA && !have_data) {
                usbif_cdc.data_itf = (uint8_t)cur_itf;
                have_data = true;
            }
        }
        if (p[1] == USB_B_DESCRIPTOR_TYPE_ENDPOINT && p[0] >= 7
            && have_data && cur_itf == usbif_cdc.data_itf
            && (p[3] & 0x03) == USB_TRANSFER_TYPE_BULK) {
            uint8_t addr = p[2];
            uint16_t mps = (uint16_t)(p[4] | (p[5] << 8));
            if (addr & 0x80) {
                usbif_cdc.ep_in = addr;
                usbif_cdc.mps_in = mps;
            } else {
                usbif_cdc.ep_out = addr;
                usbif_cdc.mps_out = mps;
            }
        }
        p += p[0];
    }
    return have_data && usbif_cdc.ep_in && usbif_cdc.ep_out;
}

int usbif_cdc_open(uint32_t dev_id) {
    if (usbif_cdc.open) {
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
    memset(&usbif_cdc, 0, sizeof(usbif_cdc));
    usbif_cdc.dev = dev;
    if (!usbif_cdc_find_itfs(cfg)) {
        return -4;
    }
    if (usbif_cdc.mps_in > USBIF_CDC_MAX_MPS || usbif_cdc.mps_out > USBIF_CDC_MAX_MPS) {
        return -5;      // an HS-sized pipe; revisit with the HS host work
    }
    usb_host_client_handle_t client = usbif_host_client_get();
    if (usb_host_interface_claim(client, dev, usbif_cdc.data_itf, 0) != ESP_OK) {
        return -6;
    }
    if (usb_host_transfer_alloc(USBIF_CDC_MAX_MPS, 0, &usbif_cdc.xfer_in) != ESP_OK
        || usb_host_transfer_alloc(USBIF_CDC_MAX_MPS, 0, &usbif_cdc.xfer_out) != ESP_OK
        || usb_host_transfer_alloc(16, 0, &usbif_cdc.xfer_ctrl) != ESP_OK) {
        usb_host_transfer_free(usbif_cdc.xfer_in);
        usb_host_transfer_free(usbif_cdc.xfer_out);
        usb_host_transfer_free(usbif_cdc.xfer_ctrl);
        usb_host_interface_release(client, dev, usbif_cdc.data_itf);
        return -7;
    }
    usbif_cdc.open = true;

    // Prime the IN pipe.
    usbif_cdc.xfer_in->device_handle = dev;
    usbif_cdc.xfer_in->bEndpointAddress = usbif_cdc.ep_in;
    usbif_cdc.xfer_in->num_bytes = usbif_cdc.mps_in;
    usbif_cdc.xfer_in->callback = usbif_cdc_in_cb;
    if (usb_host_transfer_submit(usbif_cdc.xfer_in) != ESP_OK) {
        usbif_cdc.open = false;
        return -8;
    }

    // SET_CONTROL_LINE_STATE: DTR|RTS. A MicroPython VCP treats DTR as
    // "a terminal is attached" -- without it the REPL stays silent.
    uint8_t *s = usbif_cdc.xfer_ctrl->data_buffer;
    s[0] = 0x21;    // host->device, class, interface
    s[1] = 0x22;    // SET_CONTROL_LINE_STATE
    s[2] = 0x03;    // DTR | RTS
    s[3] = 0x00;
    s[4] = usbif_cdc.comm_itf;
    s[5] = 0x00;
    s[6] = 0x00;    // wLength 0
    s[7] = 0x00;
    usbif_cdc.xfer_ctrl->device_handle = dev;
    usbif_cdc.xfer_ctrl->bEndpointAddress = 0;
    usbif_cdc.xfer_ctrl->num_bytes = 8;
    usbif_cdc.xfer_ctrl->callback = usbif_cdc_ctrl_cb;
    usbif_cdc.ctrl_busy = true;
    if (usb_host_transfer_submit_control(usbif_host_client_get(), usbif_cdc.xfer_ctrl) != ESP_OK) {
        usbif_cdc.ctrl_busy = false;   // non-fatal: some devices need no DTR
    }
    return 0;
}

int usbif_cdc_write(const uint8_t *data, size_t len) {
    if (!usbif_cdc.open || len == 0) {
        return usbif_cdc.open ? 0 : -1;
    }
    // One OUT transfer in flight at a time; REPL traffic never notices.
    for (int i = 0; i < 20 && usbif_cdc.out_busy; i++) {
        vTaskDelay(1);
    }
    if (usbif_cdc.out_busy) {
        return 0;
    }
    size_t n = len > USBIF_CDC_MAX_MPS ? USBIF_CDC_MAX_MPS : len;
    memcpy(usbif_cdc.xfer_out->data_buffer, data, n);
    usbif_cdc.xfer_out->device_handle = usbif_cdc.dev;
    usbif_cdc.xfer_out->bEndpointAddress = usbif_cdc.ep_out;
    usbif_cdc.xfer_out->num_bytes = (int)n;
    usbif_cdc.xfer_out->callback = usbif_cdc_out_cb;
    usbif_cdc.out_busy = true;
    if (usb_host_transfer_submit(usbif_cdc.xfer_out) != ESP_OK) {
        usbif_cdc.out_busy = false;
        return -2;
    }
    return (int)n;
}

int usbif_cdc_read(uint8_t *out, size_t max) {
    if (!usbif_cdc.open) {
        return -1;
    }
    size_t n = 0;
    while (n < max && usbif_cdc.rx_tail != usbif_cdc.rx_head) {
        out[n++] = usbif_cdc.rx[usbif_cdc.rx_tail];
        usbif_cdc.rx_tail = (uint16_t)((usbif_cdc.rx_tail + 1) % USBIF_CDC_RX_RING);
    }
    return (int)n;
}

void usbif_cdc_close(void) {
    if (!usbif_cdc.open) {
        return;
    }
    usbif_cdc.open = false;             // callbacks stop resubmitting
    vTaskDelay(pdMS_TO_TICKS(20));      // let in-flight callbacks drain
    usb_host_endpoint_halt(usbif_cdc.dev, usbif_cdc.ep_in);
    usb_host_endpoint_flush(usbif_cdc.dev, usbif_cdc.ep_in);
    usb_host_endpoint_clear(usbif_cdc.dev, usbif_cdc.ep_in);
    usb_host_transfer_free(usbif_cdc.xfer_in);
    usb_host_transfer_free(usbif_cdc.xfer_out);
    usb_host_transfer_free(usbif_cdc.xfer_ctrl);
    usb_host_interface_release(usbif_host_client_get(), usbif_cdc.dev, usbif_cdc.data_itf);
}

// Teardown-only variant -- see usbif_hid_close_for_host_stop()'s comment in
// usbif_host_hid.c for the full reasoning. Same asynchronous halt/flush,
// same host-task-has-no-concurrent-pump problem when called from inside
// usbif_host_task()'s own teardown rather than live from Python.
void usbif_cdc_close_for_host_stop(void) {
    if (!usbif_cdc.open) {
        return;
    }
    usbif_cdc.open = false;
    vTaskDelay(pdMS_TO_TICKS(20));
    usb_host_endpoint_halt(usbif_cdc.dev, usbif_cdc.ep_in);
    usb_host_endpoint_flush(usbif_cdc.dev, usbif_cdc.ep_in);
    usb_host_endpoint_clear(usbif_cdc.dev, usbif_cdc.ep_in);
    for (int i = 0; i < 10; i++) {
        usb_host_client_handle_events(usbif_host_client_get(), pdMS_TO_TICKS(10));
    }
    usb_host_transfer_free(usbif_cdc.xfer_in);
    usb_host_transfer_free(usbif_cdc.xfer_out);
    usb_host_transfer_free(usbif_cdc.xfer_ctrl);
    usb_host_interface_release(usbif_host_client_get(), usbif_cdc.dev, usbif_cdc.data_itf);
}

// Called by usbif_host.c when a device disappears mid-session.
void usbif_cdc_on_dev_gone(usb_device_handle_t dev) {
    if (!usbif_cdc.open || usbif_cdc.dev != dev) {
        return;
    }
    // Order matters on this path. Clearing `open` first stops the transfer
    // callbacks resubmitting; releasing the claim lets the library actually
    // free the device, without which the stale device wedges it and the
    // returning device never re-enumerates (observed with the NUCLEO's mode
    // switch).
    usbif_cdc.open = false;
    usb_host_interface_release(usbif_host_client_get(), dev, usbif_cdc.data_itf);
    // Then free the transfers. The endpoints are gone with the device, so
    // there is nothing to halt or flush first -- but an in-flight transfer's
    // callback may still be queued in the client, so give the host task one
    // pass to drain before freeing what those callbacks would touch.
    vTaskDelay(pdMS_TO_TICKS(20));
    usb_host_transfer_free(usbif_cdc.xfer_in);
    usb_host_transfer_free(usbif_cdc.xfer_out);
    usb_host_transfer_free(usbif_cdc.xfer_ctrl);
    usbif_cdc.xfer_in = NULL;
    usbif_cdc.xfer_out = NULL;
    usbif_cdc.xfer_ctrl = NULL;
}

uint32_t usbif_cdc_rx_dropped(void) {
    return usbif_cdc.rx_dropped;
}

#endif // CONFIG_SOC_USB_OTG_SUPPORTED
