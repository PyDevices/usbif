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
#include "usb/usb_host.h"

extern usb_host_client_handle_t usbif_host_client_get(void);
extern int usbif_host_dev_lookup(uint32_t dev_id, usb_device_handle_t *out);

// Packets per transfer, and transfers in flight. 8 x 1 ms packets per
// transfer with 3 in flight gives ~24 ms of scheduled bus time, which is
// enough that a Python service loop can be late by a frame render without
// the stream gapping, and small enough that stopping is prompt.
#define USBIF_UAC_PKTS_PER_XFER (8)
#define USBIF_UAC_NUM_XFER      (3)
#define USBIF_UAC_MAX_MPS       (256)
#define USBIF_UAC_RING          (8192)

// UAC 1.0 endpoint control request: SET_CUR of SAMPLING_FREQ_CONTROL.
#define UAC_SET_CUR                 (0x01)
#define UAC_SAMPLING_FREQ_CONTROL   (0x01)
#define UAC_REQTYPE_SET_EP          (0x22)   // host->device, class, endpoint

typedef struct {
    bool open;
    bool is_in;
    usb_device_handle_t dev;
    uint8_t itf, alt, ep;
    uint16_t mps;
    uint32_t rate;
    usb_transfer_t *xfer[USBIF_UAC_NUM_XFER];
    volatile uint8_t inflight;
    uint8_t ring[USBIF_UAC_RING];
    volatile uint32_t head, tail;
    // Diagnostics, same philosophy as the device-side UAC counters: when a
    // stream sounds wrong the first question is always whether bytes are
    // being lost, and where.
    volatile uint32_t packets, bytes, dropped, starved, errors;
} usbif_uac_host_t;

static usbif_uac_host_t usbif_uach;

// --- ring ---------------------------------------------------------------
//
// Single producer, single consumer, no lock: on IN the callback writes and
// Python reads; on OUT the reverse. head and tail are each written by exactly
// one side, which is what makes that safe without a mutex on a single core.

static inline uint32_t usbif_uac_ring_used(void) {
    return (usbif_uach.head - usbif_uach.tail) % USBIF_UAC_RING;
}

static inline uint32_t usbif_uac_ring_free(void) {
    return USBIF_UAC_RING - 1 - usbif_uac_ring_used();
}

static void usbif_uac_ring_push(const uint8_t *data, uint32_t len) {
    for (uint32_t i = 0; i < len; i++) {
        uint32_t next = (usbif_uach.head + 1) % USBIF_UAC_RING;
        if (next == usbif_uach.tail) {
            usbif_uach.dropped++;
            return;                 // drop the tail of this packet, not the ring
        }
        usbif_uach.ring[usbif_uach.head] = data[i];
        usbif_uach.head = next;
    }
}

static uint32_t usbif_uac_ring_pop(uint8_t *out, uint32_t max) {
    uint32_t n = 0;
    while (n < max && usbif_uach.tail != usbif_uach.head) {
        out[n++] = usbif_uach.ring[usbif_uach.tail];
        usbif_uach.tail = (usbif_uach.tail + 1) % USBIF_UAC_RING;
    }
    return n;
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
        if (!usbif_uach.is_in) {
            uint32_t have = usbif_uac_ring_used();
            if (have < want) {
                want = have;
                if (want == 0) {
                    usbif_uach.starved++;
                }
            }
            if (want) {
                usbif_uac_ring_pop(xfer->data_buffer + total, want);
            }
        }
        xfer->isoc_packet_desc[i].num_bytes = want;
        total += want;
    }
    xfer->num_bytes = total ? total : usbif_uach.mps;
}

static void usbif_uac_cb(usb_transfer_t *xfer) {
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

// Ask the device to run at `rate`. UAC 1.0 puts sampling frequency in an
// *endpoint* control, three bytes little-endian. A device with a single fixed
// rate may STALL this, which is not fatal -- it is already running at the only
// rate it has -- so the result is reported and not treated as failure.
static int usbif_uac_set_rate(uint32_t rate) {
    usb_transfer_t *ctrl;
    if (usb_host_transfer_alloc(sizeof(usb_setup_packet_t) + 3, 0, &ctrl) != ESP_OK) {
        return -1;
    }
    usb_setup_packet_t *setup = (usb_setup_packet_t *)ctrl->data_buffer;
    setup->bmRequestType = UAC_REQTYPE_SET_EP;
    setup->bRequest = UAC_SET_CUR;
    setup->wValue = (uint16_t)(UAC_SAMPLING_FREQ_CONTROL << 8);
    setup->wIndex = usbif_uach.ep;
    setup->wLength = 3;
    uint8_t *payload = ctrl->data_buffer + sizeof(usb_setup_packet_t);
    payload[0] = (uint8_t)(rate & 0xFF);
    payload[1] = (uint8_t)((rate >> 8) & 0xFF);
    payload[2] = (uint8_t)((rate >> 16) & 0xFF);
    ctrl->device_handle = usbif_uach.dev;
    ctrl->bEndpointAddress = 0;
    ctrl->num_bytes = sizeof(usb_setup_packet_t) + 3;
    ctrl->callback = NULL;
    ctrl->timeout_ms = 500;
    esp_err_t err = usb_host_transfer_submit_control(usbif_host_client_get(), ctrl);
    // Pump so the control transfer can actually retire before we free it.
    for (int i = 0; i < 20; i++) {
        usb_host_client_handle_events(usbif_host_client_get(), pdMS_TO_TICKS(5));
    }
    usb_host_transfer_free(ctrl);
    return err == ESP_OK ? 0 : -2;
}

// --- public API ---------------------------------------------------------

int usbif_host_uac_open(uint32_t dev_id, uint8_t itf, uint8_t alt, uint8_t ep,
    uint16_t mps, uint32_t rate) {
    if (usbif_uach.open) {
        return -1;
    }
    if (mps == 0 || mps > USBIF_UAC_MAX_MPS) {
        return -2;
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
    usbif_uach.is_in = (ep & 0x80) != 0;

    // Claiming with the chosen alternate setting is what starts the device
    // reserving isochronous bandwidth. Alt 0 carries no endpoint by
    // specification, so claiming it and then submitting would fail in a way
    // that looks like a driver bug rather than a wrong argument.
    if (alt == 0) {
        return -4;
    }
    if (usb_host_interface_claim(usbif_host_client_get(), dev, itf, alt) != ESP_OK) {
        return -5;
    }
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
            return -6;
        }
        usbif_uach.xfer[i]->callback = usbif_uac_cb;
    }

    usbif_uach.open = true;
    for (int i = 0; i < USBIF_UAC_NUM_XFER; i++) {
        usbif_uac_prepare(usbif_uach.xfer[i]);
        if (usb_host_transfer_submit(usbif_uach.xfer[i]) == ESP_OK) {
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
        return -7;
    }
    return 0;
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
    uint32_t room = usbif_uac_ring_free();
    uint32_t n = (uint32_t)len < room ? (uint32_t)len : room;
    usbif_uac_ring_push(data, n);
    return (int)n;
}

int usbif_host_uac_queued(void) {
    return usbif_uach.open ? (int)usbif_uac_ring_used() : -1;
}

void usbif_host_uac_stats(uint32_t *packets, uint32_t *bytes, uint32_t *dropped,
    uint32_t *starved, uint32_t *errors) {
    *packets = usbif_uach.packets;
    *bytes = usbif_uach.bytes;
    *dropped = usbif_uach.dropped;
    *starved = usbif_uach.starved;
    *errors = usbif_uach.errors;
}

void usbif_host_uac_close(void) {
    if (!usbif_uach.open) {
        return;
    }
    usbif_uach.open = false;
    // Let the in-flight transfers retire before their buffers go away: an
    // isochronous transfer is scheduled bus time, and freeing underneath one
    // is how a host stack gets corrupted rather than merely stopped.
    for (int i = 0; i < 40 && usbif_uach.inflight; i++) {
        usb_host_client_handle_events(usbif_host_client_get(), pdMS_TO_TICKS(5));
    }
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
