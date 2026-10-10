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
#include "shared/usbif_pcm_sink.h"
#include "pcm_c_sink.h"
#include "usbif_classes.h"

extern usb_host_client_handle_t usbif_host_client_get(void);
extern void usbif_host_lock(void);
extern void usbif_host_unlock(void);
extern volatile uint32_t usbif_host_pump_count;
extern void usbif_host_lock_debug(const char *tag);
extern int usbif_host_lock_suspend(void);
extern void usbif_host_lock_resume(int held);
extern int usbif_host_dev_lookup(uint32_t dev_id, usb_device_handle_t *out);
// usbif_host.c: a control transfer this driver stopped waiting for is handed
// over, and its late callback frees it (usbif#73).
extern void usbif_host_ctrl_abandon(usb_transfer_t *xfer);
extern bool usbif_host_ctrl_reclaim(usb_transfer_t *xfer);

// Packets per transfer, and transfers in flight. 8 x 1 ms packets per
// transfer with 3 in flight gives ~24 ms of scheduled bus time, which is
// enough that a Python service loop can be late by a frame render without
// the stream gapping, and small enough that stopping is prompt.
#define USBIF_UAC_PKTS_PER_XFER (8)
#define USBIF_UAC_NUM_XFER      (3)
// The largest packet an isochronous endpoint may declare: 1023 bytes at full
// speed, 1024 per transaction at high speed. This used to be 256, which
// refused a cheap USB-C DAC whose OUT endpoint declares 384 bytes before the
// host was ever asked. The real limit is the controller's FIFO split
// (usbif_host.c), and IDF enforces it at the interface claim; this bound only
// guards the transfer buffers, which are sized from the stream's own packet
// (mps x USBIF_UAC_PKTS_PER_XFER), never from this.
#define USBIF_UAC_MAX_MPS       (1024)
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
    uint32_t cb_count;          // the first few callbacks are printed
} usbif_uac_host_t;

// Two streams, one per direction, so a board can play to a device and record
// from it at once -- a headset, or a DAC with its output looped back into its
// input -- or play to one device while recording from another. Each has its
// own interface, endpoint, transfers, ring and counters; a transfer carries
// its stream in `context`, so the callbacks never have to ask which one.
// One per direction rather than a pool because that is what the class needs:
// a second playback stream to another device is a mixer's job, not a bus's.
#define USBIF_UAC_OUT (0)
#define USBIF_UAC_IN  (1)
static usbif_uac_host_t usbif_uach[2];

static inline usbif_uac_host_t *usbif_uac_slot(uint8_t ep) {
    return &usbif_uach[(ep & 0x80) ? USBIF_UAC_IN : USBIF_UAC_OUT];
}

// The playback ring's writers go through this gate (usbif#43), the
// interpreter's and a usermod task's alike. Only the OUT stream has one. It
// lives outside usbif_uach because open() zeroes that stream, and the gate's
// generation count must survive from one stream to the next: a sink issued
// for a stream that has closed stays closed. See shared/usbif_pcm_sink.h for the guarantee.
static usbif_pcm_sink_t usbif_uac_sink;
static bool usbif_uac_sink_ready;

static void usbif_uac_sink_wait(void) {
    vTaskDelay(1);      // one real tick: lets a writer on this core finish
}

static void usbif_uac_sink_setup(void) {
    if (!usbif_uac_sink_ready) {
        usbif_pcm_sink_init(&usbif_uac_sink, &usbif_uach[USBIF_UAC_OUT].ring, usbif_uac_sink_wait);
        usbif_uac_sink_ready = true;
    }
}

// --- ring ---------------------------------------------------------------
//
// Single producer, single consumer, no lock: on IN the callback writes and
// Python reads; on OUT the reverse. The ring itself is shared/usbif_byte_ring,
// tested on the host; this file only sizes and places its storage.

static inline uint32_t usbif_uac_ring_used(usbif_uac_host_t *s) {
    return usbif_byte_ring_used(&s->ring);
}

static inline uint32_t usbif_uac_ring_free(usbif_uac_host_t *s) {
    return usbif_byte_ring_free(&s->ring);
}

static void usbif_uac_ring_push(usbif_uac_host_t *s, const uint8_t *data, uint32_t len) {
    if (usbif_byte_ring_push(&s->ring, data, len) < len) {
        s->dropped++;       // the tail of this packet, not the ring
    }
}

static uint32_t usbif_uac_ring_pop(usbif_uac_host_t *s, uint8_t *out, uint32_t max) {
    return usbif_byte_ring_pop(&s->ring, out, max);
}

static bool usbif_uac_ring_alloc(usbif_uac_host_t *s, uint32_t size) {
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
    s->ring_mem = mem;
    usbif_byte_ring_init(&s->ring, mem, size);
    return true;
}

static void usbif_uac_ring_release(usbif_uac_host_t *s) {
    if (s->ring_mem) {
        heap_caps_free(s->ring_mem);
        s->ring_mem = NULL;
    }
    usbif_byte_ring_init(&s->ring, NULL, 0);
}

// --- transfer plumbing --------------------------------------------------

static void usbif_uac_prepare(usbif_uac_host_t *s, usb_transfer_t *xfer) {
    // A capture transfer asks for a full packet in every interval. A playback
    // transfer carries whatever the ring has: a short packet is legal and is
    // how an underrun is expressed on the wire, rather than by stalling.
    xfer->device_handle = s->dev;
    xfer->bEndpointAddress = s->ep;
    // num_isoc_packets is const in usb_transfer_t: it is fixed by
    // usb_host_transfer_alloc() and describes how the buffer is carved up, so
    // it is a property of the allocation rather than of this submission.
    uint32_t total = 0;
    for (int i = 0; i < USBIF_UAC_PKTS_PER_XFER; i++) {
        uint32_t want = s->mps;
        if (!s->is_in && s->frame && s->rate) {
            // One millisecond of audio per packet, the remainder carried:
            // 44.1 kHz sends 44 frames then 45 in the right proportion. The
            // host is full-speed today (usbif#3 parks the P4's), so the
            // interval is a millisecond.
            s->acc += s->rate;
            const uint32_t frames = s->acc / 1000;
            s->acc -= frames * 1000;
            want = frames * s->frame;
            if (want > s->mps) {
                want = s->mps;
            }
        }
        if (!s->is_in) {
            // Playback always sends a full packet. Where the ring is short,
            // the remainder is silence -- an underrun in audio is silence,
            // not a stall, and the stream must keep its slot on the bus.
            uint32_t have = usbif_uac_ring_used(s);
            uint32_t take = have < want ? have : want;
            if (s->frame) {
                // Whole frames only: popping half a frame into a packet
                // padded with silence would shift every sample after it.
                take -= take % s->frame;
            }
            if (take) {
                usbif_uac_ring_pop(s, xfer->data_buffer + total, take);
            }
            if (take < want) {
                memset(xfer->data_buffer + total + take, 0, want - take);
                s->starved++;
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

static void usbif_uac_cb(usb_transfer_t *xfer) {
    usbif_uac_host_t *s = (usbif_uac_host_t *)xfer->context;
    if (s->cb_count < 4) {
        printf("usbif_uac: %s cb#%u status=%d actual=%d pkts=%d pkt0=%d/%d\n",
            s->is_in ? "in" : "out", (unsigned)s->cb_count, (int)xfer->status,
            (int)xfer->actual_num_bytes, (int)xfer->num_isoc_packets,
            (int)xfer->isoc_packet_desc[0].status,
            (int)xfer->isoc_packet_desc[0].actual_num_bytes);
    }
    s->cb_count++;
    if (!s->open) {
        s->inflight--;
        return;
    }
    if (s->is_in) {
        uint32_t offset = 0;
        for (int i = 0; i < xfer->num_isoc_packets; i++) {
            usb_isoc_packet_desc_t *pkt = &xfer->isoc_packet_desc[i];
            if (pkt->status == USB_TRANSFER_STATUS_COMPLETED && pkt->actual_num_bytes) {
                usbif_uac_ring_push(s, xfer->data_buffer + offset, pkt->actual_num_bytes);
                s->bytes += pkt->actual_num_bytes;
                s->packets++;
            } else if (pkt->status != USB_TRANSFER_STATUS_COMPLETED) {
                s->errors++;
            } else {
                // Completed carrying nothing. Counted separately because it
                // is neither success nor error, and leaving it uncounted made
                // every statistic read zero while the device streamed
                // silence -- indistinguishable from never having started.
                s->empty++;
            }
            // Packets are laid out at their *requested* size, not their
            // actual one -- the next packet's data starts where this one's
            // buffer ended, however few bytes actually arrived. Advancing by
            // actual_num_bytes here is the classic isochronous read bug and
            // produces audio that is subtly, progressively wrong.
            offset += s->mps;
        }
    } else {
        for (int i = 0; i < xfer->num_isoc_packets; i++) {
            if (xfer->isoc_packet_desc[i].status == USB_TRANSFER_STATUS_COMPLETED) {
                s->bytes += xfer->isoc_packet_desc[i].actual_num_bytes;
                s->packets++;
            } else {
                s->errors++;
            }
        }
    }
    usbif_uac_prepare(s, xfer);
    if (usb_host_transfer_submit(xfer) != ESP_OK) {
        s->inflight--;
        s->errors++;
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
    if (usbif_host_ctrl_reclaim(xfer)) {
        return;
    }
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
    ctrl->context = NULL;
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
    // Decided under the lock, which the host task holds while it delivers
    // callbacks, so a completion can't land between the check and the
    // hand-over below.
    usbif_host_lock();
    bool answered = usbif_uac_ctrl_done;
    if (!answered) {
        // The device never answered. The transfer is still queued on EP0, so
        // the library will dequeue it eventually and touch this memory --
        // freeing it here is what produced the LoadProhibited panic inside
        // handle_ep0_dequeue(). Hand it to the host instead: its callback
        // frees it when it comes, and host_stop() makes sure it comes.
        usbif_uac_ctrl_active = NULL;
        usbif_host_ctrl_abandon(ctrl);
    }
    usbif_host_unlock();
    if (!answered) {
        printf("usbif_uac: ctrl req=0x%02x val=0x%04x idx=0x%04x len=%u: no answer in %d ms, "
            "left queued\n", (unsigned)request, (unsigned)value, (unsigned)index,
            (unsigned)len, USBIF_UAC_CTRL_TIMEOUT_MS);
        return -3;
    }
    printf("usbif_uac: ctrl req=0x%02x val=0x%04x idx=0x%04x len=%u submit=0x%x status=%d actual=%d\n",
        (unsigned)request, (unsigned)value, (unsigned)index, (unsigned)len,
        (unsigned)err, (int)ctrl->status, (int)ctrl->actual_num_bytes);
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

static int usbif_uac_control(usbif_uac_host_t *s, uint8_t req_type, uint8_t request,
    uint16_t value, uint16_t index, const uint8_t *payload, uint16_t len) {
    return usbif_uac_control_on(s->dev, req_type, request, value, index,
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
static int usbif_uac_set_interface(usbif_uac_host_t *s, uint8_t itf, uint8_t alt) {
    return usbif_uac_control(s, 0x01, 0x0B, alt, itf, NULL, 0);
}

// Ask the device to run at `rate`. UAC 1.0 puts sampling frequency in an
// *endpoint* control, three bytes little-endian. A device with a single fixed
// rate may STALL this, which is not fatal -- it is already running at the only
// rate it has -- so the result is reported and not treated as failure.
static int usbif_uac_set_rate(usbif_uac_host_t *s, uint32_t rate) {
    if (s->clock) {
        // 2.0: CUR on the clock source, four bytes.
        uint8_t cur[4] = {
            (uint8_t)(rate & 0xFF), (uint8_t)((rate >> 8) & 0xFF),
            (uint8_t)((rate >> 16) & 0xFF), (uint8_t)((rate >> 24) & 0xFF),
        };
        return usbif_uac_control(s, UAC2_REQTYPE_SET_ITF, UAC2_CUR,
            (uint16_t)(UAC2_CS_SAM_FREQ_CONTROL << 8),
            (uint16_t)((s->clock << 8) | s->control), cur, 4);
    }
    uint8_t payload[3] = {
        (uint8_t)(rate & 0xFF),
        (uint8_t)((rate >> 8) & 0xFF),
        (uint8_t)((rate >> 16) & 0xFF),
    };
    return usbif_uac_control(s, UAC_REQTYPE_SET_EP, UAC_SET_CUR,
        (uint16_t)(UAC_SAMPLING_FREQ_CONTROL << 8), s->ep, payload, 3);
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
    // The endpoint's direction picks the stream: one playback and one capture
    // may run at once, on one device or on two.
    usbif_uac_host_t *s = usbif_uac_slot(ep);
    if (s->open) {
        return -1;
    }
    const usbif_uac_host_t *other = &usbif_uach[(ep & 0x80) ? USBIF_UAC_OUT : USBIF_UAC_IN];
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
    // Alt 0 carries no endpoint by specification, so claiming it and then
    // submitting would fail in a way that looks like a driver bug rather than
    // a wrong argument.
    if (alt == 0) {
        return -4;
    }
    // The two streams of one device are two interfaces. A device that put
    // both directions on one interface would have them share an alternate
    // setting, and claiming it twice is not something either stream can own.
    if (other->open && other->dev == dev && other->itf == itf) {
        return -10;
    }
    memset(s, 0, sizeof(*s));
    s->dev = dev;
    s->itf = itf;
    s->alt = alt;
    s->ep = ep;
    s->mps = mps;
    s->rate = rate;
    s->clock = clock;
    s->control = control;
    s->frame = frame;
    s->acc = 0;
    s->is_in = (ep & 0x80) != 0;

    // Claiming with the chosen alternate setting is what starts the device
    // reserving isochronous bandwidth.
    if (!usbif_uac_ring_alloc(s, ring_bytes)) {
        return -8;
    }
    esp_err_t cerr = usb_host_interface_claim(usbif_host_client_get(), dev, itf, alt);
    printf("usbif_uac: interface_claim(itf=%u alt=%u) -> 0x%x\n",
        (unsigned)itf, (unsigned)alt, (unsigned)cerr);
    if (cerr != ESP_OK) {
        usbif_uac_ring_release(s);
        return -5;
    }
    int sif = usbif_uac_set_interface(s, itf, alt);
    printf("usbif_uac: SET_INTERFACE(%u, %u) -> %d\n", (unsigned)itf, (unsigned)alt, sif);
    if (rate) {
        usbif_uac_set_rate(s, rate);   // advisory: a fixed-rate device may STALL
    }

    const size_t buf = (size_t)mps * USBIF_UAC_PKTS_PER_XFER;
    for (int i = 0; i < USBIF_UAC_NUM_XFER; i++) {
        if (usb_host_transfer_alloc(buf, USBIF_UAC_PKTS_PER_XFER, &s->xfer[i]) != ESP_OK) {
            for (int j = 0; j < i; j++) {
                usb_host_transfer_free(s->xfer[j]);
                s->xfer[j] = NULL;
            }
            usb_host_interface_release(usbif_host_client_get(), dev, itf);
            usbif_uac_ring_release(s);
            return -6;
        }
        s->xfer[i]->callback = usbif_uac_cb;
        s->xfer[i]->context = s;
    }

    s->open = true;
    if (!s->is_in) {
        usbif_uac_sink_setup();
        usbif_pcm_sink_open(&usbif_uac_sink, frame);
    }
    printf("usbif_uac: claimed itf %u alt %u ep 0x%02x mps %u rate %u ring %u (%s)%s\n",
        (unsigned)itf, (unsigned)alt, (unsigned)ep, (unsigned)mps, (unsigned)rate,
        (unsigned)ring_bytes,
        esp_ptr_external_ram(s->ring_mem) ? "psram" : "internal",
        other->open ? ", beside the other direction" : "");
    for (int i = 0; i < USBIF_UAC_NUM_XFER; i++) {
        usbif_uac_prepare(s, s->xfer[i]);
        esp_err_t serr = usb_host_transfer_submit(s->xfer[i]);
        printf("usbif_uac: submit[%d] num_bytes=%d pkts=%d -> 0x%x\n",
            i, (int)s->xfer[i]->num_bytes,
            (int)s->xfer[i]->num_isoc_packets, (unsigned)serr);
        if (serr == ESP_OK) {
            s->inflight++;
        } else {
            s->errors++;
        }
    }
    if (s->inflight == 0) {
        s->open = false;
        if (!s->is_in && usbif_uac_sink_ready) {
            usbif_pcm_sink_close(&usbif_uac_sink);
        }
        for (int i = 0; i < USBIF_UAC_NUM_XFER; i++) {
            usb_host_transfer_free(s->xfer[i]);
            s->xfer[i] = NULL;
        }
        usb_host_interface_release(usbif_host_client_get(), dev, itf);
        usbif_uac_ring_release(s);
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

// Which stream a query means. `dir` is USBIF_HOST_UAC_OUT or _IN, or
// USBIF_HOST_UAC_ANY for the calls that predate two streams: then the one
// that is open, playback first when both are. NULL when that is none.
static usbif_uac_host_t *usbif_uac_pick(int dir) {
    if (dir == USBIF_HOST_UAC_OUT || dir == USBIF_HOST_UAC_IN) {
        usbif_uac_host_t *s = &usbif_uach[dir == USBIF_HOST_UAC_IN ? USBIF_UAC_IN : USBIF_UAC_OUT];
        return s->open ? s : NULL;
    }
    if (usbif_uach[USBIF_UAC_OUT].open) {
        return &usbif_uach[USBIF_UAC_OUT];
    }
    return usbif_uach[USBIF_UAC_IN].open ? &usbif_uach[USBIF_UAC_IN] : NULL;
}

int usbif_host_uac_read(uint8_t *out, size_t max) {
    usbif_uac_host_t *s = &usbif_uach[USBIF_UAC_IN];
    if (!s->open) {
        return -1;
    }
    return (int)usbif_uac_ring_pop(s, out, (uint32_t)max);
}

int usbif_host_uac_write(const uint8_t *data, size_t len) {
    if (!usbif_uach[USBIF_UAC_OUT].open) {
        return -1;
    }
    // Through the gate, like a C sink's writes: a usermod's task may be
    // writing too. A short write is the caller's to retry, not a drop.
    return usbif_pcm_sink_write(&usbif_uac_sink, usbif_pcm_sink_current(&usbif_uac_sink),
        data, len);
}

// --- the C sink (usbif#43) ------------------------------------------------
//
// What a usermod gets from UacHostOutput.c_sink(): pcm_c_sink.h's struct,
// with this stream's generation as ctx. Both functions are safe from any
// task and never block; once this stream closes they answer -1 for good.

static int usbif_uac_c_write(void *ctx, const uint8_t *data, size_t len) {
    return usbif_pcm_sink_write(&usbif_uac_sink, (uint32_t)(uintptr_t)ctx, data, len);
}

static int usbif_uac_c_space(void *ctx) {
    return usbif_pcm_sink_space(&usbif_uac_sink, (uint32_t)(uintptr_t)ctx);
}

// Fill `out` for the playback stream open now. channels and bits come from
// the caller (Python chose the format and knows them); rate and frame size
// are the driver's own. 0, or -1 when no playback stream is open.
int usbif_host_uac_c_sink(pcm_c_sink_t *out, uint32_t channels, uint32_t bits) {
    const usbif_uac_host_t *s = &usbif_uach[USBIF_UAC_OUT];
    uint32_t gen = usbif_uac_sink_ready ? usbif_pcm_sink_current(&usbif_uac_sink) : 0;
    if (!s->open || gen == 0) {
        return -1;
    }
    memset(out, 0, sizeof(*out));
    out->magic = PCM_C_SINK_MAGIC;
    out->version = PCM_C_SINK_VERSION;
    out->ctx = (void *)(uintptr_t)gen;
    out->write = usbif_uac_c_write;
    out->space = usbif_uac_c_space;
    out->rate = s->rate;
    out->channels = channels;
    out->bits = bits;
    out->frame_bytes = s->frame;
    return 0;
}

int usbif_host_uac_queued(int dir) {
    usbif_uac_host_t *s = usbif_uac_pick(dir);
    return s ? (int)usbif_uac_ring_used(s) : -1;
}

// Room for a write that will not be short, in bytes; -1 when closed. For a
// capture stream, the room left before the ring drops packets.
int usbif_host_uac_space(int dir) {
    usbif_uac_host_t *s = usbif_uac_pick(dir);
    if (s == NULL) {
        return -1;
    }
    if (!s->is_in) {
        return usbif_pcm_sink_space(&usbif_uac_sink, usbif_pcm_sink_current(&usbif_uac_sink));
    }
    return (int)usbif_uac_ring_free(s);
}

// The ring's size in bytes, as opened; -1 when closed.
int usbif_host_uac_capacity(int dir) {
    usbif_uac_host_t *s = usbif_uac_pick(dir);
    return s ? (int)s->ring.size : -1;
}

// The counters of a stream. They outlive its close, so a caller can read
// how a run went after stopping it; a direction never opened reads zeros.
// USBIF_HOST_UAC_ANY reads the open stream, or the playback one when
// neither is.
void usbif_host_uac_stats(int dir, uint32_t *packets, uint32_t *bytes, uint32_t *dropped,
    uint32_t *starved, uint32_t *errors, uint32_t *empty) {
    const usbif_uac_host_t *s = usbif_uac_pick(dir);
    if (s == NULL) {
        s = &usbif_uach[dir == USBIF_HOST_UAC_IN ? USBIF_UAC_IN : USBIF_UAC_OUT];
        if (dir == USBIF_HOST_UAC_ANY && !s->packets && usbif_uach[USBIF_UAC_IN].packets) {
            s = &usbif_uach[USBIF_UAC_IN];
        }
    }
    *packets = s->packets;
    *bytes = s->bytes;
    *dropped = s->dropped;
    *starved = s->starved;
    *errors = s->errors;
    *empty = s->empty;
}

// `tell_device`: send SET_INTERFACE alt 0 before releasing. Not from
// host_stop()'s teardown, where the host task that delivers control
// completions has already exited and the request could only time out.
static void usbif_host_uac_close_locked(usbif_uac_host_t *s, bool tell_device) {
    if (!s->open) {
        return;
    }
    s->open = false;
    // Let the in-flight transfers retire before their buffers go away: an
    // isochronous transfer is scheduled bus time, and freeing underneath one
    // is how a host stack gets corrupted rather than merely stopped.
    // Suspended for the same reason as the control wait: `inflight` only
    // falls when the host task delivers the completion callbacks, and this
    // lock is what keeps it out. The other direction's stream, if any, keeps
    // running throughout: its transfers and ring are its own.
    int drain_held = usbif_host_lock_suspend();
    // First the producers: a usermod's task may be inside a write right now,
    // on the other core. This marks the sink closed and waits it out, so
    // from here on nothing but the transfer callbacks can reach the ring
    // (usbif#43, and the guarantee in shared/usbif_pcm_sink.h).
    if (!s->is_in && usbif_uac_sink_ready) {
        usbif_pcm_sink_close(&usbif_uac_sink);
    }
    const TickType_t drain_limit = USBIF_DELAY_TICKS(200);
    for (TickType_t i = 0; i < drain_limit && s->inflight; i++) {
        vTaskDelay(1);
    }
    usbif_host_lock_resume(drain_held);
    for (int i = 0; i < USBIF_UAC_NUM_XFER; i++) {
        if (s->xfer[i]) {
            usb_host_transfer_free(s->xfer[i]);
            s->xfer[i] = NULL;
        }
    }
    // Drop back to alt 0: the device stops streaming and stops reserving
    // isochronous bandwidth, which matters on a full-speed bus where that
    // reservation is the scarce resource every other device is competing
    // for. Releasing the interface does not do this -- IDF sends no
    // SET_INTERFACE at all, as the note on usbif_uac_set_interface() says --
    // so without it a closed stream left the device on its streaming
    // setting, and the next open's SET_INTERFACE was no change to it.
    if (tell_device) {
        usbif_uac_set_interface(s, s->itf, 0);
    }
    usb_host_interface_release(usbif_host_client_get(), s->dev, s->itf);
    // Only once the transfers have retired: a callback still in flight pops
    // from this storage. One that outlived the drain above returns early on
    // `open` and never reaches the ring.
    usbif_uac_ring_release(s);
}

static void usbif_host_uac_close_dir(int dir, bool tell_device) {
    usbif_host_lock();
    if (dir != USBIF_HOST_UAC_IN) {
        usbif_host_uac_close_locked(&usbif_uach[USBIF_UAC_OUT], tell_device);
    }
    if (dir != USBIF_HOST_UAC_OUT) {
        usbif_host_uac_close_locked(&usbif_uach[USBIF_UAC_IN], tell_device);
    }
    usbif_host_unlock();
}

// Close one direction's stream, or both with USBIF_HOST_UAC_ANY.
void usbif_host_uac_close(int dir) {
    usbif_host_uac_close_dir(dir, true);
}

// Called from host_stop()'s teardown, with the event pump explicit because
// the host task's own loop has already exited by then -- the same reason the
// other class drivers have a _for_host_stop variant.
void usbif_host_uac_close_for_host_stop(void) {
    usbif_host_uac_close_dir(USBIF_HOST_UAC_ANY, false);
    for (int i = 0; i < 10; i++) {
        usb_host_client_handle_events(usbif_host_client_get(), pdMS_TO_TICKS(10));
    }
}

#endif // CONFIG_SOC_USB_OTG_SUPPORTED
