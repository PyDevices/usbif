// SPDX-License-Identifier: MIT
//
// USB Audio Class host: the board driving a commercial sound device.
//
// One deliberate departure from the MIDI and CDC host drivers this is
// otherwise modelled on: **this driver does not walk descriptors.** The MIDI
// driver finds its own interface and endpoints in C because a MIDIStreaming
// interface is simple enough to recognise in twenty lines. A UAC device is
// not: the Burr-Brown CODEC on this bench publishes 24 alternate settings
// across two interfaces, at six sample rates, in 8- and 16-bit, with three
// different synchronisation types. Choosing among those is configuration, and
// configuration belongs in Python -- where usbif.uac already reads the whole
// descriptor and choose() already picks a stream.
//
// So Python hands this driver the answer: interface, alternate setting,
// endpoint, packet size, rate. C claims, negotiates the rate, and moves
// isochronous bytes. That is this module's stated division of labour, and it
// keeps the part that changes per device out of the part that needs a
// reflash to iterate.
//
// Isochronous is not bulk. A bulk transfer moves a byte stream and retries;
// an isochronous transfer is a *schedule* -- one packet per bus interval,
// delivered late or not at all, never retried. So transfers carry many
// packets each, several transfers stay in flight at once, and a gap in the
// data is a real event to be counted rather than an error to be raised. The
// ring exists so neither side ever waits on the other: the bus never waits
// for Python, and Python never waits for the bus.

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
#include "esp_heap_caps.h"
#include "esp_memory_utils.h"
#include "usb/usb_host.h"

#include "shared/usbif_byte_ring.h"

extern usb_host_client_handle_t usbif_host_client_get(void);
extern void usbif_host_lock(void);
extern void usbif_host_unlock(void);
extern volatile uint32_t usbif_host_pump_count;
extern void usbif_host_lock_debug(const char *tag);
extern int usbif_host_lock_suspend(void);
extern void usbif_host_lock_resume(int held);
extern int usbif_host_dev_lookup(uint32_t dev_id, usb_device_handle_t *out);

// Packets per transfer, and transfers in flight. 8 x 1 ms packets per
// transfer with 3 in flight gives ~24 ms of scheduled bus time, which is
// enough that a Python service loop can be late by a frame render without
// the stream gapping, and small enough that stopping is prompt.
#define USBIF_UAC_PKTS_PER_XFER (8)
#define USBIF_UAC_NUM_XFER      (3)
#define USBIF_UAC_MAX_MPS       (256)
// The ring between Python and the bus. Its size is the caller's (usbif#36):
// the default is the 8 KB this driver always had, 43 ms of 48 kHz stereo,
// which an interpreter busy with a UI redraw or a Web API call outlasts. A
// caller that feeds the stream from the interpreter thread asks for seconds.
// Up to USBIF_UAC_RING_INTERNAL the ring prefers internal RAM, which is the
// scarcer pool but the faster one; above it, PSRAM, because 2 s of 48 kHz
// stereo is 375 KB and internal RAM on an S3 holds about 300 KB in all.
// Either falls back to the other. It is never the MicroPython heap: the
// transfers keep reading the ring from the host task after a soft reset has
// wiped that heap, until something closes the stream.
#define USBIF_UAC_RING_DEFAULT  (8192)
#define USBIF_UAC_RING_INTERNAL (16384)
#define USBIF_UAC_RING_MAX      (4u * 1024u * 1024u)

// How long a control transfer may take to be *retired*, which is not the same
// as how long the device takes to answer. EP0 is shared, and a hub enumerating
// another device behind this one holds it: opening the CODEC while the mic
// behind it was still enumerating, the SET_INTERFACE callback arrived after
// more than 500 ms -- measured, by counting callbacks, as zero during the
// wait and two by the time the next request finished. The old 500 ms bound
// gave up on a transfer that was merely queued, and (before the ownership fix
// above) freed it while the library still owned it. Three seconds is far
// longer than any device needs and still bounded.
#define USBIF_UAC_CTRL_TIMEOUT_MS   (3000)

// pdMS_TO_TICKS() truncates, and this port runs at CONFIG_FREERTOS_HZ=100 --
// one tick is 10 ms, so *any* value below 10 ms becomes ZERO ticks. And
// vTaskDelay(0) does not block: it yields to equal-priority tasks and returns
// immediately. A wait loop built from those is not a wait at all. It runs to
// completion in microseconds, while the task whose callback it is waiting for
// never gets scheduled.
//
// That is the root of this module's isochronous crashes. The control-transfer
// "500 ms" wait was 100 iterations of vTaskDelay(0); it always fell through
// with the callback still pending, concluded the transfer had failed, and
// freed it while the USB host library still owned it. The panic then landed
// wherever the heap was next touched -- inside TLSF, or inside IDF's
// handle_ep0_dequeue() -- which is why it read as memory corruption from an
// unrelated allocation rather than as a timeout here.
//
// Always at least one real tick.
#define USBIF_DELAY_TICKS(ms) ((pdMS_TO_TICKS(ms) > 0) ? pdMS_TO_TICKS(ms) : 1)


// UAC 1.0 endpoint control request: SET_CUR of SAMPLING_FREQ_CONTROL.
#define UAC_SET_CUR                 (0x01)
#define UAC_SAMPLING_FREQ_CONTROL   (0x01)
#define UAC_REQTYPE_SET_EP          (0x22)   // host->device, class, endpoint

// USB Audio 2.0 moves the sampling frequency to the clock source entity
// (5.2.5.1.2): RANGE answers wNumSubRanges then (dMIN, dMAX, dRES) triplets
// of 32 bits, CUR sets one; the recipient is the AudioControl interface,
// wIndex = clock id << 8 | interface. usbif's own sound card is a 2.0
// device, and until this existed the S3 could not host the P4 (usbif#28).
#define UAC2_CUR                    (0x01)
#define UAC2_RANGE                  (0x02)
#define UAC2_CS_SAM_FREQ_CONTROL    (0x01)
#define UAC2_REQTYPE_SET_ITF        (0x21)   // host->device, class, interface
#define UAC2_REQTYPE_GET_ITF        (0xA1)   // device->host, class, interface
#define UAC2_MAX_SUBRANGES          (8)

typedef struct {
    bool open;
    bool is_in;
    usb_device_handle_t dev;
    uint8_t itf, alt, ep;
    uint16_t mps;
    uint32_t rate;
    uint8_t clock, control;     // 2.0: the clock source and its interface; 0 for 1.0
    // Playback packet sizing. `frame` is bytes per audio frame (channels x
    // sample bytes); with it known, a packet carries the frames one
    // millisecond holds at `rate`, the fraction carried in `acc`, rather
    // than a full `mps`. Sending mps every interval to a sink whose maximum
    // packet is a millisecond plus one frame (as 2.0 devices size them) runs
    // 2 % fast, and the sink drops what it cannot hold.
    uint16_t frame;
    uint32_t acc;
    usb_transfer_t *xfer[USBIF_UAC_NUM_XFER];
    volatile uint8_t inflight;
    usbif_byte_ring_t ring;
    uint8_t *ring_mem;
    // Diagnostics, same philosophy as the device-side UAC counters: when a
    // stream sounds wrong the first question is always whether bytes are
    // being lost, and where.
    volatile uint32_t packets, bytes, dropped, starved, errors, empty;
} usbif_uac_host_t;

static usbif_uac_host_t usbif_uach;

// --- ring ---------------------------------------------------------------
//
// Single producer, single consumer, no lock: on IN the callback writes and
// Python reads; on OUT the reverse. The ring itself is shared/usbif_byte_ring,
// tested on the host; this file only sizes and places its storage.

static inline uint32_t usbif_uac_ring_used(void) {
    return usbif_byte_ring_used(&usbif_uach.ring);
}

static inline uint32_t usbif_uac_ring_free(void) {
    return usbif_byte_ring_free(&usbif_uach.ring);
}

static void usbif_uac_ring_push(const uint8_t *data, uint32_t len) {
    if (usbif_byte_ring_push(&usbif_uach.ring, data, len) < len) {
        usbif_uach.dropped++;       // the tail of this packet, not the ring
    }
}

static uint32_t usbif_uac_ring_pop(uint8_t *out, uint32_t max) {
    return usbif_byte_ring_pop(&usbif_uach.ring, out, max);
}

static bool usbif_uac_ring_alloc(uint32_t size) {
    const uint32_t internal = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
    const uint32_t psram = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;
    const bool small = size <= USBIF_UAC_RING_INTERNAL;
    uint8_t *mem = heap_caps_malloc(size, small ? internal : psram);
    if (mem == NULL) {
        mem = heap_caps_malloc(size, small ? psram : internal);
    }
    if (mem == NULL) {
        return false;
    }
    usbif_uach.ring_mem = mem;
    usbif_byte_ring_init(&usbif_uach.ring, mem, size);
    return true;
}

static void usbif_uac_ring_release(void) {
    if (usbif_uach.ring_mem) {
        heap_caps_free(usbif_uach.ring_mem);
        usbif_uach.ring_mem = NULL;
    }
    usbif_byte_ring_init(&usbif_uach.ring, NULL, 0);
}

// --- transfer plumbing --------------------------------------------------

static void usbif_uac_prepare(usb_transfer_t *xfer) {
    // A capture transfer asks for a full packet in every interval. A playback
    // transfer carries whatever the ring has: a short packet is legal and is
    // how an underrun is expressed on the wire, rather than by stalling.
    xfer->device_handle = usbif_uach.dev;
    xfer->bEndpointAddress = usbif_uach.ep;
    // num_isoc_packets is const in usb_transfer_t: it is fixed by
    // usb_host_transfer_alloc() and describes how the buffer is carved up, so
    // it is a property of the allocation rather than of this submission.
    uint32_t total = 0;
    for (int i = 0; i < USBIF_UAC_PKTS_PER_XFER; i++) {
        uint32_t want = usbif_uach.mps;
        if (!usbif_uach.is_in && usbif_uach.frame && usbif_uach.rate) {
            // One millisecond of audio per packet, the remainder carried:
            // 44.1 kHz sends 44 frames then 45 in the right proportion. The
            // host is full-speed today (usbif#3 parks the P4's), so the
            // interval is a millisecond.
            usbif_uach.acc += usbif_uach.rate;
            const uint32_t frames = usbif_uach.acc / 1000;
            usbif_uach.acc -= frames * 1000;
            want = frames * usbif_uach.frame;
            if (want > usbif_uach.mps) {
                want = usbif_uach.mps;
            }
        }
        if (!usbif_uach.is_in) {
            // Playback always sends a full packet. Where the ring is short,
            // the remainder is silence -- an underrun in audio is silence,
            // not a stall, and the stream must keep its slot on the bus.
            uint32_t have = usbif_uac_ring_used();
            uint32_t take = have < want ? have : want;
            if (usbif_uach.frame) {
                // Whole frames only: popping half a frame into a packet
                // padded with silence would shift every sample after it.
                take -= take % usbif_uach.frame;
            }
            if (take) {
                usbif_uac_ring_pop(xfer->data_buffer + total, take);
            }
            if (take < want) {
                memset(xfer->data_buffer + total + take, 0, want - take);
                usbif_uach.starved++;
            }
        }
        xfer->isoc_packet_desc[i].num_bytes = want;
        total += want;
    }
    // IDF requires num_bytes to equal the sum of the packet lengths exactly:
    // usbh.c's transfer_check_usb_compliance() rejects any mismatch with a
    // bare ESP_ERR_INVALID_ARG. An earlier version shortened packets to
    // whatever the ring held and left num_bytes at mps, so every submit was
    // refused the moment the ring was empty -- which is always, at open.
    xfer->num_bytes = total;
}

static volatile uint32_t usbif_uac_cb_count;

static void usbif_uac_cb(usb_transfer_t *xfer) {
    if (usbif_uac_cb_count < 4) {
        printf("usbif_uac: cb#%u status=%d actual=%d pkts=%d pkt0=%d/%d\n",
            (unsigned)usbif_uac_cb_count, (int)xfer->status,
            (int)xfer->actual_num_bytes, (int)xfer->num_isoc_packets,
            (int)xfer->isoc_packet_desc[0].status,
            (int)xfer->isoc_packet_desc[0].actual_num_bytes);
    }
    usbif_uac_cb_count++;
    if (!usbif_uach.open) {
        usbif_uach.inflight--;
        return;
    }
    if (usbif_uach.is_in) {
        uint32_t offset = 0;
        for (int i = 0; i < xfer->num_isoc_packets; i++) {
            usb_isoc_packet_desc_t *pkt = &xfer->isoc_packet_desc[i];
            if (pkt->status == USB_TRANSFER_STATUS_COMPLETED && pkt->actual_num_bytes) {
                usbif_uac_ring_push(xfer->data_buffer + offset, pkt->actual_num_bytes);
                usbif_uach.bytes += pkt->actual_num_bytes;
                usbif_uach.packets++;
            } else if (pkt->status != USB_TRANSFER_STATUS_COMPLETED) {
                usbif_uach.errors++;
            } else {
                // Completed carrying nothing. Counted separately because it
                // is neither success nor error, and leaving it uncounted made
                // every statistic read zero while the device streamed
                // silence -- indistinguishable from never having started.
                usbif_uach.empty++;
            }
            // Packets are laid out at their *requested* size, not their
            // actual one -- the next packet's data starts where this one's
            // buffer ended, however few bytes actually arrived. Advancing by
            // actual_num_bytes here is the classic isochronous read bug and
            // produces audio that is subtly, progressively wrong.
            offset += usbif_uach.mps;
        }
    } else {
        for (int i = 0; i < xfer->num_isoc_packets; i++) {
            if (xfer->isoc_packet_desc[i].status == USB_TRANSFER_STATUS_COMPLETED) {
                usbif_uach.bytes += xfer->isoc_packet_desc[i].actual_num_bytes;
                usbif_uach.packets++;
            } else {
                usbif_uach.errors++;
            }
        }
    }
    usbif_uac_prepare(xfer);
    if (usb_host_transfer_submit(xfer) != ESP_OK) {
        usbif_uach.inflight--;
        usbif_uach.errors++;
    }
}

static volatile bool usbif_uac_ctrl_done;

// IDF rejects any URB whose callback is NULL -- urb_check_args() in usbh.c
// fails it before the transfer is ever attempted, returning ESP_ERR_INVALID_ARG
// with no hint that the callback is what it objected to. A control transfer
// this code intends to wait for still needs one, so this exists to set a flag.
// Matched against the transfer actually being waited on. A control transfer
// that timed out is not cancelled -- it is still queued, and its callback can
// arrive later. Without this check that late callback would land on whatever
// transfer the next request is waiting for and declare it complete while it
// is still in flight.
static usb_transfer_t *volatile usbif_uac_ctrl_active;

static void usbif_uac_ctrl_cb(usb_transfer_t *xfer) {
    if (xfer == usbif_uac_ctrl_active) {
        usbif_uac_ctrl_done = true;
    }
}

// One control transfer, synchronous: submit, wait for the callback, free.
// Setup-only when payload is NULL. For a device-to-host request `rx` takes
// what came back (at most `len` bytes; the count is written to `rx_len`).
// `dev` is any handle the host holds, so this also serves a device this
// driver has not opened as a stream -- the 2.0 rate query below.
static int usbif_uac_control_on(usb_device_handle_t dev, uint8_t req_type, uint8_t request,
    uint16_t value, uint16_t index, const uint8_t *payload, uint16_t len,
    uint8_t *rx, uint16_t *rx_len) {
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
    if (payload && len) {
        memcpy(ctrl->data_buffer + sizeof(usb_setup_packet_t), payload, len);
    }
    ctrl->device_handle = dev;
    ctrl->bEndpointAddress = 0;
    ctrl->num_bytes = sizeof(usb_setup_packet_t) + len;
    ctrl->callback = usbif_uac_ctrl_cb;
    ctrl->timeout_ms = USBIF_UAC_CTRL_TIMEOUT_MS;
    usbif_uac_ctrl_done = false;
    usbif_uac_ctrl_active = ctrl;
    esp_err_t err = usb_host_transfer_submit_control(usbif_host_client_get(), ctrl);
    if (err != ESP_OK) {
        // Never submitted, so never queued: ours to free.
        usbif_uac_ctrl_active = NULL;
        usb_host_transfer_free(ctrl);
        return -2;
    }
    // WAIT, do not pump. The host task is already calling
    // usb_host_client_handle_events() on this same client in its own loop,
    // and a second caller from the MicroPython thread races it: observed as
    // heap corruption, surfacing later as a StoreProhibited panic inside
    // TLSF's remove_free_block during an unrelated allocation. The class
    // drivers' _for_host_stop variants pump explicitly and are safe doing so
    // precisely because the host task has already exited by then; during
    // normal operation it has not.
    //
    // So sleep and let the task deliver the callback. Bounded so a device
    // that never answers cannot hang setup.
    // Suspended across the wait: the completion callback is delivered by the
    // host task, which this lock excludes. Holding it here cannot be slow,
    // only fatal.
    int held = usbif_host_lock_suspend();
    const TickType_t limit = USBIF_DELAY_TICKS(USBIF_UAC_CTRL_TIMEOUT_MS);
    for (TickType_t i = 0; i < limit && !usbif_uac_ctrl_done; i++) {
        vTaskDelay(1);
    }
    usbif_host_lock_resume(held);
    printf("usbif_uac: ctrl req=0x%02x val=0x%04x idx=0x%04x len=%u submit=0x%x status=%d actual=%d\n",
        (unsigned)request, (unsigned)value, (unsigned)index, (unsigned)len,
        (unsigned)err, (int)ctrl->status, (int)ctrl->actual_num_bytes);
    if (!usbif_uac_ctrl_done) {
        // The device never answered. The transfer is still queued on EP0, so
        // the library will dequeue it eventually and touch this memory --
        // freeing it here is what produced the LoadProhibited panic inside
        // handle_ep0_dequeue(). Deliberately leaked: one 8-byte transfer,
        // once, on a device that is already misbehaving, against corrupting
        // the heap of a board that has to keep running.
        usbif_uac_ctrl_active = NULL;
        return -3;
    }
    usbif_uac_ctrl_active = NULL;
    int rc = (ctrl->status == USB_TRANSFER_STATUS_COMPLETED) ? 0 : -4;
    if (rx && rx_len) {
        // actual_num_bytes counts the setup packet too.
        uint16_t got = 0;
        if (rc == 0 && ctrl->actual_num_bytes > (int)sizeof(usb_setup_packet_t)) {
            got = (uint16_t)(ctrl->actual_num_bytes - sizeof(usb_setup_packet_t));
            if (got > len) {
                got = len;
            }
            memcpy(rx, ctrl->data_buffer + sizeof(usb_setup_packet_t), got);
        }
        *rx_len = got;
    }
    usb_host_transfer_free(ctrl);
    return rc;
}

static int usbif_uac_control(uint8_t req_type, uint8_t request, uint16_t value,
    uint16_t index, const uint8_t *payload, uint16_t len) {
    return usbif_uac_control_on(usbif_uach.dev, req_type, request, value, index,
        payload, len, NULL, NULL);
}

// Tell the DEVICE to switch to the streaming alternate setting.
//
// This is not optional and is not what usb_host_interface_claim() does.
// Claiming is a host-side matter -- it takes ownership and allocates the
// endpoints belonging to that alt setting -- and IDF's usb_host.c contains no
// SET_INTERFACE at all. Without this the device stays on alt 0, which by
// specification carries no endpoint and produces no data, so the host happily
// polls and every isochronous packet completes with zero bytes. Measured
// exactly that way before this call existed: submits fine, callbacks fire,
// status COMPLETED, actual_num_bytes 0, forever.
static int usbif_uac_set_interface(uint8_t itf, uint8_t alt) {
    return usbif_uac_control(0x01, 0x0B, alt, itf, NULL, 0);
}

// Ask the device to run at `rate`. UAC 1.0 puts sampling frequency in an
// *endpoint* control, three bytes little-endian. A device with a single fixed
// rate may STALL this, which is not fatal -- it is already running at the only
// rate it has -- so the result is reported and not treated as failure.
static int usbif_uac_set_rate(uint32_t rate) {
    if (usbif_uach.clock) {
        // 2.0: CUR on the clock source, four bytes.
        uint8_t cur[4] = {
            (uint8_t)(rate & 0xFF), (uint8_t)((rate >> 8) & 0xFF),
            (uint8_t)((rate >> 16) & 0xFF), (uint8_t)((rate >> 24) & 0xFF),
        };
        return usbif_uac_control(UAC2_REQTYPE_SET_ITF, UAC2_CUR,
            (uint16_t)(UAC2_CS_SAM_FREQ_CONTROL << 8),
            (uint16_t)((usbif_uach.clock << 8) | usbif_uach.control), cur, 4);
    }
    uint8_t payload[3] = {
        (uint8_t)(rate & 0xFF),
        (uint8_t)((rate >> 8) & 0xFF),
        (uint8_t)((rate >> 16) & 0xFF),
    };
    return usbif_uac_control(UAC_REQTYPE_SET_EP, UAC_SET_CUR,
        (uint16_t)(UAC_SAMPLING_FREQ_CONTROL << 8), usbif_uach.ep, payload, 3);
}

// --- public API ---------------------------------------------------------

// A 2.0 clock source's sampling-frequency ranges, as (min, max, res)
// triplets into `out` (room for `max_triplets`). The device need not be
// open as a stream; the host's own handle for it serves. Returns the count,
// or negative: -1 no such device, -2 the request failed, -3 short answer.
int usbif_host_uac_clock_ranges(uint32_t dev_id, uint8_t control_itf, uint8_t clock_id,
    uint32_t *out, int max_triplets) {
    usb_device_handle_t dev;
    usbif_host_lock();
    if (usbif_host_dev_lookup(dev_id, &dev) != 0) {
        usbif_host_unlock();
        return -1;
    }
    uint8_t rx[2 + 12 * UAC2_MAX_SUBRANGES];
    uint16_t got = 0;
    int rc = usbif_uac_control_on(dev, UAC2_REQTYPE_GET_ITF, UAC2_RANGE,
        (uint16_t)(UAC2_CS_SAM_FREQ_CONTROL << 8),
        (uint16_t)((clock_id << 8) | control_itf), NULL, sizeof(rx), rx, &got);
    usbif_host_unlock();
    if (rc != 0) {
        return -2;
    }
    if (got < 2) {
        return -3;
    }
    int n = rx[0] | (rx[1] << 8);
    if (n > max_triplets) {
        n = max_triplets;
    }
    if (n > UAC2_MAX_SUBRANGES) {
        n = UAC2_MAX_SUBRANGES;
    }
    int filled = 0;
    for (int i = 0; i < n; i++) {
        const uint8_t *t = rx + 2 + 12 * i;
        if (2 + 12 * (i + 1) > got) {
            break;
        }
        for (int k = 0; k < 3; k++) {
            const uint8_t *v = t + 4 * k;
            out[3 * i + k] = (uint32_t)v[0] | ((uint32_t)v[1] << 8)
                | ((uint32_t)v[2] << 16) | ((uint32_t)v[3] << 24);
        }
        filled++;
    }
    return filled;
}

static int usbif_host_uac_open_locked(uint32_t dev_id, uint8_t itf, uint8_t alt, uint8_t ep,
    uint16_t mps, uint32_t rate, uint8_t clock, uint8_t control, uint16_t frame,
    uint32_t ring_bytes) {
    if (usbif_uach.open) {
        return -1;
    }
    if (mps == 0 || mps > USBIF_UAC_MAX_MPS) {
        return -2;
    }
    if (ring_bytes == 0) {
        ring_bytes = USBIF_UAC_RING_DEFAULT;
    }
    if (frame) {
        // Whole frames, so a write the ring cuts short never splits one.
        ring_bytes -= ring_bytes % frame;
    }
    if (ring_bytes < (uint32_t)mps * 2 || ring_bytes > USBIF_UAC_RING_MAX) {
        return -9;
    }
    usb_device_handle_t dev;
    if (usbif_host_dev_lookup(dev_id, &dev) != 0) {
        return -3;
    }
    memset(&usbif_uach, 0, sizeof(usbif_uach));
    usbif_uach.dev = dev;
    usbif_uach.itf = itf;
    usbif_uach.alt = alt;
    usbif_uach.ep = ep;
    usbif_uach.mps = mps;
    usbif_uach.rate = rate;
    usbif_uach.clock = clock;
    usbif_uach.control = control;
    usbif_uach.frame = frame;
    usbif_uach.acc = 0;
    usbif_uach.is_in = (ep & 0x80) != 0;

    // Claiming with the chosen alternate setting is what starts the device
    // reserving isochronous bandwidth. Alt 0 carries no endpoint by
    // specification, so claiming it and then submitting would fail in a way
    // that looks like a driver bug rather than a wrong argument.
    if (alt == 0) {
        return -4;
    }
    if (!usbif_uac_ring_alloc(ring_bytes)) {
        return -8;
    }
    esp_err_t cerr = usb_host_interface_claim(usbif_host_client_get(), dev, itf, alt);
    printf("usbif_uac: interface_claim(itf=%u alt=%u) -> 0x%x\n",
        (unsigned)itf, (unsigned)alt, (unsigned)cerr);
    if (cerr != ESP_OK) {
        usbif_uac_ring_release();
        return -5;
    }
    int sif = usbif_uac_set_interface(itf, alt);
    printf("usbif_uac: SET_INTERFACE(%u, %u) -> %d\n", (unsigned)itf, (unsigned)alt, sif);
    if (rate) {
        usbif_uac_set_rate(rate);   // advisory: a fixed-rate device may STALL
    }

    const size_t buf = (size_t)mps * USBIF_UAC_PKTS_PER_XFER;
    for (int i = 0; i < USBIF_UAC_NUM_XFER; i++) {
        if (usb_host_transfer_alloc(buf, USBIF_UAC_PKTS_PER_XFER, &usbif_uach.xfer[i]) != ESP_OK) {
            for (int j = 0; j < i; j++) {
                usb_host_transfer_free(usbif_uach.xfer[j]);
                usbif_uach.xfer[j] = NULL;
            }
            usb_host_interface_release(usbif_host_client_get(), dev, itf);
            usbif_uac_ring_release();
            return -6;
        }
        usbif_uach.xfer[i]->callback = usbif_uac_cb;
    }

    usbif_uach.open = true;
    printf("usbif_uac: claimed itf %u alt %u ep 0x%02x mps %u rate %u ring %u (%s)\n",
        (unsigned)itf, (unsigned)alt, (unsigned)ep, (unsigned)mps, (unsigned)rate,
        (unsigned)ring_bytes,
        esp_ptr_external_ram(usbif_uach.ring_mem) ? "psram" : "internal");
    for (int i = 0; i < USBIF_UAC_NUM_XFER; i++) {
        usbif_uac_prepare(usbif_uach.xfer[i]);
        esp_err_t serr = usb_host_transfer_submit(usbif_uach.xfer[i]);
        printf("usbif_uac: submit[%d] num_bytes=%d pkts=%d -> 0x%x\n",
            i, (int)usbif_uach.xfer[i]->num_bytes,
            (int)usbif_uach.xfer[i]->num_isoc_packets, (unsigned)serr);
        if (serr == ESP_OK) {
            usbif_uach.inflight++;
        } else {
            usbif_uach.errors++;
        }
    }
    if (usbif_uach.inflight == 0) {
        usbif_uach.open = false;
        for (int i = 0; i < USBIF_UAC_NUM_XFER; i++) {
            usb_host_transfer_free(usbif_uach.xfer[i]);
            usbif_uach.xfer[i] = NULL;
        }
        usb_host_interface_release(usbif_host_client_get(), dev, itf);
        usbif_uac_ring_release();
        return -7;
    }
    return 0;
}

int usbif_host_uac_open(uint32_t dev_id, uint8_t itf, uint8_t alt, uint8_t ep,
    uint16_t mps, uint32_t rate, uint8_t clock, uint8_t control, uint16_t frame,
    uint32_t ring_bytes) {
    usbif_host_lock();
    int r = usbif_host_uac_open_locked(dev_id, itf, alt, ep, mps, rate, clock, control, frame,
        ring_bytes);
    usbif_host_unlock();
    return r;
}

int usbif_host_uac_read(uint8_t *out, size_t max) {
    if (!usbif_uach.open || !usbif_uach.is_in) {
        return -1;
    }
    return (int)usbif_uac_ring_pop(out, (uint32_t)max);
}

int usbif_host_uac_write(const uint8_t *data, size_t len) {
    if (!usbif_uach.open || usbif_uach.is_in) {
        return -1;
    }
    // A short write is the caller's to retry, not a drop: no count here.
    return (int)usbif_byte_ring_push(&usbif_uach.ring, data, (uint32_t)len);
}

int usbif_host_uac_queued(void) {
    return usbif_uach.open ? (int)usbif_uac_ring_used() : -1;
}

// Room for a write that will not be short, in bytes; -1 when closed.
int usbif_host_uac_space(void) {
    return usbif_uach.open ? (int)usbif_uac_ring_free() : -1;
}

// The ring's size in bytes, as opened; -1 when closed.
int usbif_host_uac_capacity(void) {
    return usbif_uach.open ? (int)usbif_uach.ring.size : -1;
}

void usbif_host_uac_stats(uint32_t *packets, uint32_t *bytes, uint32_t *dropped,
    uint32_t *starved, uint32_t *errors, uint32_t *empty) {
    *packets = usbif_uach.packets;
    *bytes = usbif_uach.bytes;
    *dropped = usbif_uach.dropped;
    *starved = usbif_uach.starved;
    *errors = usbif_uach.errors;
    *empty = usbif_uach.empty;
}

static void usbif_host_uac_close_locked(void) {
    if (!usbif_uach.open) {
        return;
    }
    usbif_uach.open = false;
    // Let the in-flight transfers retire before their buffers go away: an
    // isochronous transfer is scheduled bus time, and freeing underneath one
    // is how a host stack gets corrupted rather than merely stopped.
    // Suspended for the same reason as the control wait: `inflight` only
    // falls when the host task delivers the completion callbacks, and this
    // lock is what keeps it out.
    int drain_held = usbif_host_lock_suspend();
    const TickType_t drain_limit = USBIF_DELAY_TICKS(200);
    for (TickType_t i = 0; i < drain_limit && usbif_uach.inflight; i++) {
        vTaskDelay(1);
    }
    usbif_host_lock_resume(drain_held);
    for (int i = 0; i < USBIF_UAC_NUM_XFER; i++) {
        if (usbif_uach.xfer[i]) {
            usb_host_transfer_free(usbif_uach.xfer[i]);
            usbif_uach.xfer[i] = NULL;
        }
    }
    // Drop back to alt 0: the device stops reserving isochronous bandwidth,
    // which matters on a full-speed bus where that reservation is the scarce
    // resource every other device is competing for.
    usb_host_interface_release(usbif_host_client_get(), usbif_uach.dev, usbif_uach.itf);
    // Only once the transfers have retired: a callback still in flight pops
    // from this storage. One that outlived the drain above returns early on
    // `open` and never reaches the ring.
    usbif_uac_ring_release();
}

void usbif_host_uac_close(void) {
    usbif_host_lock();
    usbif_host_uac_close_locked();
    usbif_host_unlock();
}

// Called from host_stop()'s teardown, with the event pump explicit because
// the host task's own loop has already exited by then -- the same reason the
// other class drivers have a _for_host_stop variant.
void usbif_host_uac_close_for_host_stop(void) {
    usbif_host_uac_close();
    for (int i = 0; i < 10; i++) {
        usb_host_client_handle_events(usbif_host_client_get(), pdMS_TO_TICKS(10));
    }
}

#endif // CONFIG_SOC_USB_OTG_SUPPORTED
