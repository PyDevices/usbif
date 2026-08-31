// SPDX-License-Identifier: MIT
//
// MSC host: Bulk-Only Transport with the minimum SCSI vocabulary needed to
// identify a drive and read its blocks -- INQUIRY, READ CAPACITY(10) and
// READ(10). The third class driver, and the first with a command protocol
// rather than a byte stream: every operation is a 31-byte CBW out, an
// optional data stage, and a 13-byte CSW back whose tag and status are
// checked, not assumed.
//
// Synchronous by design: block reads are request/response, the caller is
// Python, and a busy-flag poll (the CDC writer's pattern) keeps the
// transfer callbacks -- which run in the host task -- decoupled from the
// VM without a queue nobody needs yet. Writes are deliberately absent:
// reading proves the transport, and writing someone's filesystem is a
// responsibility this layer takes on only when something above it has a
// reason to ask.

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

#define USBIF_MSC_MAX_MPS   (64)
#define USBIF_MSC_BLOCK_MAX (512)
#define USBIF_MSC_TIMEOUT_TICKS (100)   // 1 s of 10 ms ticks per stage

typedef struct {
    bool open;
    usb_device_handle_t dev;
    uint8_t itf;
    uint8_t ep_in, ep_out;
    uint32_t tag;
    uint32_t num_blocks;
    uint32_t block_size;
    char inquiry[29];               // "VENDOR12PRODUCT8901234567REV4", NUL-terminated
    usb_transfer_t *xfer;           // one transfer object, reused per stage
    volatile bool done;
} usbif_msc_t;

static usbif_msc_t usbif_msc;

void usbif_msc_close(void);

static void usbif_msc_cb(usb_transfer_t *transfer) {
    (void)transfer;
    usbif_msc.done = true;
}

// Run one transfer stage to completion. Returns actual bytes, or -1.
static int usbif_msc_stage(uint8_t ep, const void *out_data, size_t len) {
    usbif_msc.xfer->device_handle = usbif_msc.dev;
    usbif_msc.xfer->bEndpointAddress = ep;
    usbif_msc.xfer->callback = usbif_msc_cb;
    if (out_data != NULL) {
        memcpy(usbif_msc.xfer->data_buffer, out_data, len);
        usbif_msc.xfer->num_bytes = (int)len;
    } else {
        // IN stages must ask in whole packets; a short packet ends them.
        usbif_msc.xfer->num_bytes = (int)((len + USBIF_MSC_MAX_MPS - 1)
            / USBIF_MSC_MAX_MPS * USBIF_MSC_MAX_MPS);
    }
    usbif_msc.done = false;
    if (usb_host_transfer_submit(usbif_msc.xfer) != ESP_OK) {
        return -1;
    }
    for (int i = 0; i < USBIF_MSC_TIMEOUT_TICKS && !usbif_msc.done; i++) {
        vTaskDelay(1);
    }
    if (!usbif_msc.done || usbif_msc.xfer->status != USB_TRANSFER_STATUS_COMPLETED) {
        return -1;
    }
    return usbif_msc.xfer->actual_num_bytes;
}

// One full BOT round trip. cb: SCSI command block. data_in: buffer for a
// device-to-host data stage (or NULL). Returns data bytes moved, or -1.
static int usbif_msc_xact(const uint8_t *cb, uint8_t cb_len,
    uint8_t *data_in, uint32_t data_len) {
    uint8_t cbw[31];
    memset(cbw, 0, sizeof(cbw));
    uint32_t tag = ++usbif_msc.tag;
    cbw[0] = 'U'; cbw[1] = 'S'; cbw[2] = 'B'; cbw[3] = 'C';
    memcpy(&cbw[4], &tag, 4);
    memcpy(&cbw[8], &data_len, 4);
    cbw[12] = data_in ? 0x80 : 0x00;    // direction: IN when a buffer waits
    cbw[13] = 0;                        // LUN 0
    cbw[14] = cb_len;
    memcpy(&cbw[15], cb, cb_len);

    if (usbif_msc_stage(usbif_msc.ep_out, cbw, sizeof(cbw)) < 0) {
        return -1;
    }
    int moved = 0;
    if (data_in != NULL && data_len > 0) {
        moved = usbif_msc_stage(usbif_msc.ep_in, NULL, data_len);
        if (moved < 0) {
            return -1;
        }
        memcpy(data_in, usbif_msc.xfer->data_buffer, (size_t)moved);
    }
    int csw_n = usbif_msc_stage(usbif_msc.ep_in, NULL, 13);
    if (csw_n < 13) {
        return -1;
    }
    const uint8_t *csw = usbif_msc.xfer->data_buffer;
    uint32_t csw_tag;
    memcpy(&csw_tag, &csw[4], 4);
    if (memcmp(csw, "USBS", 4) != 0 || csw_tag != tag || csw[12] != 0) {
        return -1;
    }
    return moved;
}

static bool usbif_msc_find_itf(const usb_config_desc_t *cfg) {
    const uint8_t *p = (const uint8_t *)cfg;
    const uint8_t *end = p + cfg->wTotalLength;
    int cur_itf = -1;
    bool have = false;
    p += cfg->bLength;
    while (p + 2 <= end && p[0] >= 2 && p + p[0] <= end) {
        if (p[1] == USB_B_DESCRIPTOR_TYPE_INTERFACE && p[0] >= 9) {
            cur_itf = p[2];
            if (p[5] == USB_CLASS_MASS_STORAGE && !have) {
                usbif_msc.itf = (uint8_t)cur_itf;
                have = true;
            }
        }
        if (p[1] == USB_B_DESCRIPTOR_TYPE_ENDPOINT && p[0] >= 7
            && have && cur_itf == usbif_msc.itf
            && (p[3] & 0x03) == USB_TRANSFER_TYPE_BULK) {
            if (p[2] & 0x80) {
                usbif_msc.ep_in = p[2];
            } else {
                usbif_msc.ep_out = p[2];
            }
        }
        p += p[0];
    }
    return have && usbif_msc.ep_in && usbif_msc.ep_out;
}

int usbif_msc_open(uint32_t dev_id) {
    if (usbif_msc.open) {
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
    memset(&usbif_msc, 0, sizeof(usbif_msc));
    usbif_msc.dev = dev;
    if (!usbif_msc_find_itf(cfg)) {
        return -4;
    }
    if (usb_host_interface_claim(usbif_host_client_get(), dev, usbif_msc.itf, 0) != ESP_OK) {
        return -5;
    }
    if (usb_host_transfer_alloc(USBIF_MSC_BLOCK_MAX + USBIF_MSC_MAX_MPS, 0, &usbif_msc.xfer) != ESP_OK) {
        usb_host_interface_release(usbif_host_client_get(), dev, usbif_msc.itf);
        return -6;
    }
    usbif_msc.open = true;

    // INQUIRY: identity, and proof the command path works at all.
    uint8_t inq[36];
    const uint8_t cb_inquiry[6] = { 0x12, 0, 0, 0, 36, 0 };
    if (usbif_msc_xact(cb_inquiry, 6, inq, sizeof(inq)) < 36) {
        usbif_msc_close();
        return -7;
    }
    memcpy(usbif_msc.inquiry, &inq[8], 28);
    usbif_msc.inquiry[28] = 0;

    // READ CAPACITY(10): last LBA and block size, big-endian.
    uint8_t cap[8];
    const uint8_t cb_cap[10] = { 0x25, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    if (usbif_msc_xact(cb_cap, 10, cap, sizeof(cap)) < 8) {
        usbif_msc_close();
        return -8;
    }
    uint32_t last_lba = ((uint32_t)cap[0] << 24) | ((uint32_t)cap[1] << 16)
        | ((uint32_t)cap[2] << 8) | cap[3];
    usbif_msc.block_size = ((uint32_t)cap[4] << 24) | ((uint32_t)cap[5] << 16)
        | ((uint32_t)cap[6] << 8) | cap[7];
    usbif_msc.num_blocks = last_lba + 1;
    if (usbif_msc.block_size > USBIF_MSC_BLOCK_MAX) {
        usbif_msc_close();
        return -9;
    }
    return 0;
}

int usbif_msc_info(uint32_t *num_blocks, uint32_t *block_size, const char **inquiry) {
    if (!usbif_msc.open) {
        return -1;
    }
    *num_blocks = usbif_msc.num_blocks;
    *block_size = usbif_msc.block_size;
    *inquiry = usbif_msc.inquiry;
    return 0;
}

int usbif_msc_read_block(uint32_t lba, uint8_t *out, size_t max) {
    if (!usbif_msc.open) {
        return -1;
    }
    if (lba >= usbif_msc.num_blocks || max < usbif_msc.block_size) {
        return -2;
    }
    uint8_t cb[10];
    memset(cb, 0, sizeof(cb));
    cb[0] = 0x28;               // READ(10)
    cb[2] = (uint8_t)(lba >> 24);
    cb[3] = (uint8_t)(lba >> 16);
    cb[4] = (uint8_t)(lba >> 8);
    cb[5] = (uint8_t)lba;
    cb[8] = 1;                  // one block per round trip in this cut
    int n = usbif_msc_xact(cb, 10, out, usbif_msc.block_size);
    return n == (int)usbif_msc.block_size ? n : -3;
}

void usbif_msc_close(void) {
    if (!usbif_msc.open) {
        return;
    }
    usbif_msc.open = false;
    vTaskDelay(pdMS_TO_TICKS(20));
    usb_host_transfer_free(usbif_msc.xfer);
    usb_host_interface_release(usbif_host_client_get(), usbif_msc.dev, usbif_msc.itf);
}

void usbif_msc_on_dev_gone(usb_device_handle_t dev) {
    if (!usbif_msc.open || usbif_msc.dev != dev) {
        return;
    }
    usbif_msc.open = false;
    usb_host_interface_release(usbif_host_client_get(), dev, usbif_msc.itf);
    vTaskDelay(pdMS_TO_TICKS(20));
    usb_host_transfer_free(usbif_msc.xfer);
    usbif_msc.xfer = NULL;
}

#endif // CONFIG_SOC_USB_OTG_SUPPORTED
