// SPDX-License-Identifier: MIT
//
// USB Video Class host: the board driving a commercial camera.
//
// Same division of labour as the UAC host driver this is modelled on, and
// for the same reason: **this driver does not walk descriptors.** A camera's
// configuration blob is worse than an audio device's -- the C920e on the
// bench publishes two formats over thirty-six frame descriptors across
// eleven bandwidth tiers -- and choosing among those is configuration.
// usbif.uvc already reads the whole blob in Python and choose() already picks
// a mode. Python hands this driver the answer.
//
// What UVC adds over UAC, and why this is not just the audio driver with a
// different endpoint:
//
// 1. A camera does not stream because you selected an alternate setting. The
//    mode is *negotiated* first, over the control endpoint, with a
//    PROBE/COMMIT handshake: the host proposes a format, frame and interval;
//    the device answers with the same block filled in -- crucially including
//    dwMaxPayloadTransferSize, which is the number that selects the alternate
//    setting. The host does not compute that. It asks.
//
// 2. The isochronous stream is not a byte stream. Every packet begins with a
//    payload header carrying an end-of-frame bit and a frame-ID bit that
//    toggles between frames. Video frames are reassembled from those, and a
//    frame that arrives torn must be dropped whole rather than handed on --
//    half a JPEG is not a smaller JPEG.
//
// So the ring here holds frames, not bytes.

#include "py/mpconfig.h"

#if defined(__has_include)
#if __has_include("sdkconfig.h")
#include "sdkconfig.h"
#endif
#endif

#if defined(CONFIG_SOC_USB_OTG_SUPPORTED) && CONFIG_SOC_USB_OTG_SUPPORTED

#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "usb/usb_host.h"

extern usb_host_client_handle_t usbif_host_client_get(void);
extern int usbif_host_dev_lookup(uint32_t dev_id, usb_device_handle_t *out);
extern void usbif_host_lock(void);
extern void usbif_host_unlock(void);
extern int usbif_host_lock_suspend(void);
extern void usbif_host_lock_resume(int held);

// Packets per transfer and transfers in flight. On a high-speed bus a
// bInterval of 1 means one transaction per 125 us microframe, so eight
// packets is one millisecond of schedule -- the same unit the UAC driver
// uses, which keeps the two drivers' latency arithmetic comparable.
#define USBIF_UVC_PKTS_PER_XFER (8)
#define USBIF_UVC_NUM_XFER      (3)

// Ceiling on one packet's DMA buffer: 1024 bytes times three transactions per
// microframe is the most a high-speed isochronous endpoint can ask for.
#define USBIF_UVC_MAX_PACKET    (3072)

// UVC 1.1 VideoStreaming interface control requests.
#define UVC_SET_CUR                 (0x01)
#define UVC_GET_CUR                 (0x81)
#define UVC_VS_PROBE_CONTROL        (0x01)
#define UVC_VS_COMMIT_CONTROL       (0x02)
#define UVC_REQTYPE_SET_ITF         (0x21)   // host->device, class, interface
#define UVC_REQTYPE_GET_ITF         (0xA1)   // device->host, class, interface

// The negotiation block. 26 bytes in UVC 1.0, 34 in 1.1, 48 in 1.5. We send
// and expect 34: a 1.0 device answers the leading 26 and ignores the rest,
// and reading a short answer is handled by only trusting the fields a 1.0
// device also fills.
#define UVC_PROBE_LEN               (34)
#define UVC_PROBE_FORMAT_INDEX      (2)
#define UVC_PROBE_FRAME_INDEX       (3)
#define UVC_PROBE_INTERVAL          (4)
#define UVC_PROBE_MAX_FRAME_SIZE    (18)
#define UVC_PROBE_MAX_PAYLOAD       (22)

// Payload header bits (UVC 1.1 s2.4.3.3).
#define UVC_HDR_FID                 (0x01)   // toggles between frames
#define UVC_HDR_EOF                 (0x02)
#define UVC_HDR_ERR                 (0x40)

typedef struct {
    bool open;
    usb_device_handle_t dev;
    uint8_t itf, alt, ep;
    uint16_t packet;            // bytes per (micro)frame, mult already applied
    usb_transfer_t *xfer[USBIF_UVC_NUM_XFER];
    volatile uint8_t inflight;

    // Frame assembly. Two buffers, not a byte ring: a video frame is only
    // useful whole, so the producer fills one while the consumer holds the
    // other, and a frame that completes with nowhere to go is dropped as a
    // frame rather than corrupting the next one.
    uint8_t *buf[2];
    uint32_t buf_size;
    volatile uint8_t filling;       // index the callback writes into
    volatile uint32_t fill_len;
    volatile uint32_t ready_len;    // non-zero when buf[!filling] holds a frame
    volatile bool has_fid;
    volatile uint8_t fid;
    volatile bool torn;             // current frame already unusable

    volatile uint32_t frames, dropped_full, dropped_torn, dropped_big;
    volatile uint32_t packets, bytes, errors, empty;
} usbif_uvc_host_t;

static usbif_uvc_host_t usbif_uvch;

// --- control transfers --------------------------------------------------

static volatile bool usbif_uvc_ctrl_done;
static usb_transfer_t *volatile usbif_uvc_ctrl_active;

static void usbif_uvc_ctrl_cb(usb_transfer_t *xfer) {
    if (xfer == usbif_uvc_ctrl_active) {
        usbif_uvc_ctrl_done = true;
    }
}

// pdMS_TO_TICKS() truncates and this port runs at 100 Hz, so anything under
// 10 ms is ZERO ticks and vTaskDelay(0) does not block at all. See the long
// note on USBIF_DELAY_TICKS in usbif_host_uac.c: a wait built from those is
// not a wait, and the driver that had one ended up freeing transfers the USB
// library still owned.
#define USBIF_UVC_DELAY_TICKS(ms) ((pdMS_TO_TICKS(ms) > 0) ? pdMS_TO_TICKS(ms) : 1)
#define USBIF_UVC_CTRL_TIMEOUT_MS (3000)

// One class-specific interface request. `payload` is written for a host->
// device request and filled for a device->host one; `len` is its length.
static int usbif_uvc_control(uint8_t req_type, uint8_t request, uint16_t value,
    uint16_t index, uint8_t *payload, uint16_t len) {
    usb_transfer_t *ctrl;
    if (usb_host_transfer_alloc(sizeof(usb_setup_packet_t) + len, 0, &ctrl) != ESP_OK) {
        return -1;
    }
    usb_setup_packet_t *setup = (usb_setup_packet_t *)ctrl->data_buffer;
    setup->bmRequestType = req_type;
    setup->bRequest = request;
    setup->wValue = value;
    setup->wIndex = index;
    setup->wLength = len;
    bool device_to_host = (req_type & 0x80) != 0;
    if (!device_to_host && payload && len) {
        memcpy(ctrl->data_buffer + sizeof(usb_setup_packet_t), payload, len);
    }
    ctrl->device_handle = usbif_uvch.dev;
    ctrl->bEndpointAddress = 0;
    ctrl->num_bytes = sizeof(usb_setup_packet_t) + len;
    ctrl->callback = usbif_uvc_ctrl_cb;
    ctrl->timeout_ms = USBIF_UVC_CTRL_TIMEOUT_MS;
    usbif_uvc_ctrl_done = false;
    usbif_uvc_ctrl_active = ctrl;
    if (usb_host_transfer_submit_control(usbif_host_client_get(), ctrl) != ESP_OK) {
        usbif_uvc_ctrl_active = NULL;
        usb_host_transfer_free(ctrl);
        return -2;
    }
    // The completion callback is delivered by the host task, which the client
    // lock excludes -- so drop it for the wait or deadlock deterministically.
    int held = usbif_host_lock_suspend();
    const TickType_t limit = USBIF_UVC_DELAY_TICKS(USBIF_UVC_CTRL_TIMEOUT_MS);
    for (TickType_t i = 0; i < limit && !usbif_uvc_ctrl_done; i++) {
        vTaskDelay(1);
    }
    usbif_host_lock_resume(held);
    if (!usbif_uvc_ctrl_done) {
        // Still queued on EP0. Freeing it now is how the library ends up
        // dereferencing freed memory later; leak it instead.
        usbif_uvc_ctrl_active = NULL;
        return -3;
    }
    int status = (int)ctrl->status;
    int actual = (int)ctrl->actual_num_bytes;
    if (device_to_host && payload && len && status == USB_TRANSFER_STATUS_COMPLETED) {
        int got = actual - (int)sizeof(usb_setup_packet_t);
        if (got > (int)len) {
            got = len;
        }
        if (got > 0) {
            memcpy(payload, ctrl->data_buffer + sizeof(usb_setup_packet_t), got);
        }
    }
    usbif_uvc_ctrl_active = NULL;
    usb_host_transfer_free(ctrl);
    return status == USB_TRANSFER_STATUS_COMPLETED ? 0 : -4;
}

// Negotiate a mode. Returns 0 and fills *payload_out / *frame_out with what
// the device agreed to -- which is the point of the exercise: those are the
// numbers that pick the alternate setting and size the frame buffer, and the
// host is not entitled to guess either.
int usbif_host_uvc_negotiate(uint32_t dev_id, uint8_t itf, uint8_t format_index,
    uint8_t frame_index, uint32_t interval, uint32_t *payload_out,
    uint32_t *frame_out) {
    usb_device_handle_t dev;
    if (usbif_host_dev_lookup(dev_id, &dev) != 0) {
        return -1;
    }
    usbif_host_lock();
    usbif_uvch.dev = dev;

    uint8_t probe[UVC_PROBE_LEN];
    memset(probe, 0, sizeof(probe));
    probe[UVC_PROBE_FORMAT_INDEX] = format_index;
    probe[UVC_PROBE_FRAME_INDEX] = frame_index;
    probe[UVC_PROBE_INTERVAL + 0] = (uint8_t)(interval);
    probe[UVC_PROBE_INTERVAL + 1] = (uint8_t)(interval >> 8);
    probe[UVC_PROBE_INTERVAL + 2] = (uint8_t)(interval >> 16);
    probe[UVC_PROBE_INTERVAL + 3] = (uint8_t)(interval >> 24);

    int rc = usbif_uvc_control(UVC_REQTYPE_SET_ITF, UVC_SET_CUR,
        UVC_VS_PROBE_CONTROL << 8, itf, probe, sizeof(probe));
    if (rc != 0) {
        usbif_host_unlock();
        return -2;
    }
    // Read back what the device is actually willing to do. It may lower the
    // frame rate, change the frame size, or both.
    rc = usbif_uvc_control(UVC_REQTYPE_GET_ITF, UVC_GET_CUR,
        UVC_VS_PROBE_CONTROL << 8, itf, probe, sizeof(probe));
    if (rc != 0) {
        usbif_host_unlock();
        return -3;
    }
    uint32_t frame_size =
        (uint32_t)probe[UVC_PROBE_MAX_FRAME_SIZE]
        | ((uint32_t)probe[UVC_PROBE_MAX_FRAME_SIZE + 1] << 8)
        | ((uint32_t)probe[UVC_PROBE_MAX_FRAME_SIZE + 2] << 16)
        | ((uint32_t)probe[UVC_PROBE_MAX_FRAME_SIZE + 3] << 24);
    uint32_t payload =
        (uint32_t)probe[UVC_PROBE_MAX_PAYLOAD]
        | ((uint32_t)probe[UVC_PROBE_MAX_PAYLOAD + 1] << 8)
        | ((uint32_t)probe[UVC_PROBE_MAX_PAYLOAD + 2] << 16)
        | ((uint32_t)probe[UVC_PROBE_MAX_PAYLOAD + 3] << 24);

    // Commit the negotiated block verbatim. Sending back anything other than
    // what the device just returned is how a camera ends up streaming a mode
    // neither side agreed on.
    rc = usbif_uvc_control(UVC_REQTYPE_SET_ITF, UVC_SET_CUR,
        UVC_VS_COMMIT_CONTROL << 8, itf, probe, sizeof(probe));
    usbif_host_unlock();
    if (rc != 0) {
        return -4;
    }
    printf("usbif_uvc: negotiated fmt %u frame %u -> payload %u, frame %u bytes\n",
        (unsigned)probe[UVC_PROBE_FORMAT_INDEX],
        (unsigned)probe[UVC_PROBE_FRAME_INDEX],
        (unsigned)payload, (unsigned)frame_size);
    if (payload_out) {
        *payload_out = payload;
    }
    if (frame_out) {
        *frame_out = frame_size;
    }
    return 0;
}

// --- streaming ----------------------------------------------------------

static void usbif_uvc_finish_frame(void) {
    if (usbif_uvch.torn || usbif_uvch.fill_len == 0) {
        if (usbif_uvch.fill_len) {
            usbif_uvch.dropped_torn++;
        }
    } else if (usbif_uvch.ready_len != 0) {
        // The consumer has not taken the previous frame. Drop this one whole
        // rather than overwrite a buffer Python may be reading.
        usbif_uvch.dropped_full++;
    } else {
        usbif_uvch.ready_len = usbif_uvch.fill_len;
        usbif_uvch.filling ^= 1;
        usbif_uvch.frames++;
    }
    usbif_uvch.fill_len = 0;
    usbif_uvch.torn = false;
}

static void usbif_uvc_payload(const uint8_t *data, uint32_t len) {
    if (len < 2) {
        return;
    }
    uint8_t hlen = data[0];
    uint8_t flags = data[1];
    if (hlen < 2 || hlen > len) {
        usbif_uvch.torn = true;
        return;
    }
    uint8_t fid = flags & UVC_HDR_FID;
    if (!usbif_uvch.has_fid) {
        usbif_uvch.has_fid = true;
        usbif_uvch.fid = fid;
    } else if (fid != usbif_uvch.fid) {
        // The frame ID toggled without an end-of-frame marker: the previous
        // frame ended and we missed its last packet.
        usbif_uvch.fid = fid;
        usbif_uvch.torn = true;
        usbif_uvc_finish_frame();
    }
    if (flags & UVC_HDR_ERR) {
        usbif_uvch.torn = true;
    }
    uint32_t payload = len - hlen;
    if (payload) {
        if (usbif_uvch.fill_len + payload > usbif_uvch.buf_size) {
            // A frame larger than the buffer negotiated for it. Counted
            // separately from a torn one: this is a sizing mistake on our
            // side, not a bus problem, and conflating the two would hide it.
            usbif_uvch.dropped_big++;
            usbif_uvch.torn = true;
        } else {
            memcpy(usbif_uvch.buf[usbif_uvch.filling] + usbif_uvch.fill_len,
                data + hlen, payload);
            usbif_uvch.fill_len += payload;
        }
    }
    if (flags & UVC_HDR_EOF) {
        usbif_uvc_finish_frame();
    }
}

static void usbif_uvc_cb(usb_transfer_t *xfer) {
    if (!usbif_uvch.open) {
        usbif_uvch.inflight--;
        return;
    }
    uint32_t offset = 0;
    for (int i = 0; i < xfer->num_isoc_packets; i++) {
        usb_isoc_packet_desc_t *pkt = &xfer->isoc_packet_desc[i];
        if (pkt->status == USB_TRANSFER_STATUS_COMPLETED && pkt->actual_num_bytes) {
            usbif_uvch.packets++;
            usbif_uvch.bytes += pkt->actual_num_bytes;
            usbif_uvc_payload(xfer->data_buffer + offset, pkt->actual_num_bytes);
        } else if (pkt->status != USB_TRANSFER_STATUS_COMPLETED) {
            usbif_uvch.errors++;
        } else {
            // A camera with nothing to send still occupies its schedule, so
            // empty packets are normal here in a way they are not for audio.
            usbif_uvch.empty++;
        }
        // Packets are laid out at their requested size, not their actual one.
        offset += usbif_uvch.packet;
    }
    if (usb_host_transfer_submit(xfer) != ESP_OK) {
        usbif_uvch.inflight--;
        usbif_uvch.errors++;
    }
}

static int usbif_host_uvc_open_locked(uint32_t dev_id, uint8_t itf, uint8_t alt,
    uint8_t ep, uint16_t packet, uint32_t frame_bytes) {
    if (usbif_uvch.open) {
        return -1;
    }
    if (packet == 0 || packet > USBIF_UVC_MAX_PACKET || alt == 0) {
        return -2;
    }
    usb_device_handle_t dev;
    if (usbif_host_dev_lookup(dev_id, &dev) != 0) {
        return -3;
    }
    uint8_t *b0 = malloc(frame_bytes);
    uint8_t *b1 = malloc(frame_bytes);
    if (b0 == NULL || b1 == NULL) {
        free(b0);
        free(b1);
        return -4;
    }
    memset(&usbif_uvch, 0, sizeof(usbif_uvch));
    usbif_uvch.dev = dev;
    usbif_uvch.itf = itf;
    usbif_uvch.alt = alt;
    usbif_uvch.ep = ep;
    usbif_uvch.packet = packet;
    usbif_uvch.buf[0] = b0;
    usbif_uvch.buf[1] = b1;
    usbif_uvch.buf_size = frame_bytes;

    esp_err_t cerr = usb_host_interface_claim(usbif_host_client_get(), dev, itf, alt);
    printf("usbif_uvc: interface_claim(itf=%u alt=%u) -> 0x%x\n",
        (unsigned)itf, (unsigned)alt, (unsigned)cerr);
    if (cerr != ESP_OK) {
        free(b0);
        free(b1);
        usbif_uvch.buf[0] = usbif_uvch.buf[1] = NULL;
        return -5;
    }
    // Claiming is host-side bookkeeping; the device is still on alt 0 and
    // still silent until told otherwise. IDF sends no SET_INTERFACE of its
    // own -- the same trap the UAC driver documents at length.
    int sif = usbif_uvc_control(0x01, 0x0B, alt, itf, NULL, 0);
    printf("usbif_uvc: SET_INTERFACE(%u, %u) -> %d\n",
        (unsigned)itf, (unsigned)alt, sif);

    const size_t buf = (size_t)packet * USBIF_UVC_PKTS_PER_XFER;
    for (int i = 0; i < USBIF_UVC_NUM_XFER; i++) {
        if (usb_host_transfer_alloc(buf, USBIF_UVC_PKTS_PER_XFER,
                &usbif_uvch.xfer[i]) != ESP_OK) {
            for (int j = 0; j < i; j++) {
                usb_host_transfer_free(usbif_uvch.xfer[j]);
                usbif_uvch.xfer[j] = NULL;
            }
            usb_host_interface_release(usbif_host_client_get(), dev, itf);
            free(b0);
            free(b1);
            usbif_uvch.buf[0] = usbif_uvch.buf[1] = NULL;
            return -6;
        }
        usbif_uvch.xfer[i]->device_handle = dev;
        usbif_uvch.xfer[i]->bEndpointAddress = ep;
        usbif_uvch.xfer[i]->callback = usbif_uvc_cb;
        usbif_uvch.xfer[i]->num_bytes = (int)buf;
        for (int p = 0; p < USBIF_UVC_PKTS_PER_XFER; p++) {
            usbif_uvch.xfer[i]->isoc_packet_desc[p].num_bytes = packet;
        }
    }
    usbif_uvch.open = true;
    for (int i = 0; i < USBIF_UVC_NUM_XFER; i++) {
        if (usb_host_transfer_submit(usbif_uvch.xfer[i]) == ESP_OK) {
            usbif_uvch.inflight++;
        } else {
            usbif_uvch.errors++;
        }
    }
    if (usbif_uvch.inflight == 0) {
        usbif_uvch.open = false;
        for (int i = 0; i < USBIF_UVC_NUM_XFER; i++) {
            usb_host_transfer_free(usbif_uvch.xfer[i]);
            usbif_uvch.xfer[i] = NULL;
        }
        usb_host_interface_release(usbif_host_client_get(), dev, itf);
        free(b0);
        free(b1);
        usbif_uvch.buf[0] = usbif_uvch.buf[1] = NULL;
        return -7;
    }
    printf("usbif_uvc: streaming itf %u alt %u ep 0x%02x packet %u frame buf %u\n",
        (unsigned)itf, (unsigned)alt, (unsigned)ep, (unsigned)packet,
        (unsigned)frame_bytes);
    return 0;
}

int usbif_host_uvc_open(uint32_t dev_id, uint8_t itf, uint8_t alt, uint8_t ep,
    uint16_t packet, uint32_t frame_bytes) {
    usbif_host_lock();
    int r = usbif_host_uvc_open_locked(dev_id, itf, alt, ep, packet, frame_bytes);
    usbif_host_unlock();
    return r;
}

// Copy out one complete frame, or 0 if none is ready. Never a partial frame:
// the caller cannot tell a short JPEG from a whole one, so it is not offered.
int usbif_host_uvc_read_frame(uint8_t *out, size_t max) {
    if (!usbif_uvch.open) {
        return -1;
    }
    uint32_t ready = usbif_uvch.ready_len;
    if (ready == 0) {
        return 0;
    }
    if (ready > max) {
        // Refuse rather than truncate, and leave the frame in place so the
        // caller can retry with a big enough buffer.
        return -2;
    }
    memcpy(out, usbif_uvch.buf[usbif_uvch.filling ^ 1], ready);
    usbif_uvch.ready_len = 0;
    return (int)ready;
}

int usbif_host_uvc_frame_ready(void) {
    return usbif_uvch.open ? (int)usbif_uvch.ready_len : 0;
}

void usbif_host_uvc_stats(uint32_t *frames, uint32_t *packets, uint32_t *bytes,
    uint32_t *dropped_full, uint32_t *dropped_torn, uint32_t *dropped_big,
    uint32_t *errors, uint32_t *empty) {
    *frames = usbif_uvch.frames;
    *packets = usbif_uvch.packets;
    *bytes = usbif_uvch.bytes;
    *dropped_full = usbif_uvch.dropped_full;
    *dropped_torn = usbif_uvch.dropped_torn;
    *dropped_big = usbif_uvch.dropped_big;
    *errors = usbif_uvch.errors;
    *empty = usbif_uvch.empty;
}

static void usbif_host_uvc_close_locked(void) {
    if (!usbif_uvch.open) {
        return;
    }
    usbif_uvch.open = false;
    // The in-flight transfers must retire before their buffers go away.
    // Suspended, because only the host task can retire them.
    int held = usbif_host_lock_suspend();
    const TickType_t limit = USBIF_UVC_DELAY_TICKS(200);
    for (TickType_t i = 0; i < limit && usbif_uvch.inflight; i++) {
        vTaskDelay(1);
    }
    usbif_host_lock_resume(held);
    for (int i = 0; i < USBIF_UVC_NUM_XFER; i++) {
        if (usbif_uvch.xfer[i]) {
            usb_host_transfer_free(usbif_uvch.xfer[i]);
            usbif_uvch.xfer[i] = NULL;
        }
    }
    // Park the device back on alt 0, its zero-bandwidth setting, so it stops
    // occupying isochronous schedule the moment we stop reading.
    usbif_uvc_control(0x01, 0x0B, 0, usbif_uvch.itf, NULL, 0);
    usb_host_interface_release(usbif_host_client_get(), usbif_uvch.dev, usbif_uvch.itf);
    free(usbif_uvch.buf[0]);
    free(usbif_uvch.buf[1]);
    usbif_uvch.buf[0] = usbif_uvch.buf[1] = NULL;
}

void usbif_host_uvc_close(void) {
    usbif_host_lock();
    usbif_host_uvc_close_locked();
    usbif_host_unlock();
}

// Called from the host task during teardown, where it already holds the lock
// and where nothing else will pump the completions this needs.
void usbif_host_uvc_close_for_host_stop(void) {
    if (usbif_uvch.open) {
        usbif_host_uvc_close_locked();
    }
}

#endif // CONFIG_SOC_USB_OTG_SUPPORTED
