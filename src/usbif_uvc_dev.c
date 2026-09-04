// SPDX-License-Identifier: MIT
//
// USB Video Class device: the board *is* the webcam.
//
// The mirror of usbif_host_uvc.c. That module consumes UVC from somebody
// else's camera; this one produces it, so a PC sees this board as an
// ordinary webcam and any application that opens a camera can open it.
//
// TinyUSB carries the class itself -- the payload headers with their
// frame-ID and end-of-frame bits, and the PROBE/COMMIT negotiation the host
// runs before streaming -- so this module is deliberately thin. It owns one
// frame buffer, hands it to tud_video_n_frame_xfer(), and tells Python when
// the next frame may be written. Everything about *what* to show stays in
// Python, exactly as the host side keeps descriptor parsing there.
//
// The frame format is fixed at build time by the descriptor in
// usbif_tusb_ext.h -- 160x120 YUY2 -- because a UVC device advertises its
// formats in descriptors the host reads once at enumeration. Changing the
// format is a reflash, not a runtime call, and that is a property of the
// class rather than of this implementation.

#include "py/mpconfig.h"

#if defined(__has_include)
#if __has_include("sdkconfig.h")
#include "sdkconfig.h"
#endif
#endif

#include <string.h>

#include "tusb.h"

#if CFG_TUD_VIDEO && CFG_TUD_VIDEO_STREAMING

#include "usbif_tusb_ext.h"

// One frame, owned here. Double buffering would let Python write frame N+1
// while the bus sends frame N, but a UVC frame at this size is 38 KB and the
// win is one frame of latency at 10 fps -- not worth twice the RAM until a
// real sensor is filling it, at which point the sensor's own buffers are the
// right place to double.
static uint8_t usbif_uvc_frame[USBIF_VIDEO_FRAME_BYTES];

static volatile bool usbif_uvc_in_flight;
static volatile bool usbif_uvc_committed;
static volatile uint32_t usbif_uvc_frames;      // handed to the bus
static volatile uint32_t usbif_uvc_completed;   // confirmed sent
static volatile uint32_t usbif_uvc_refused;     // submitted while busy

// TinyUSB calls this when the host has finished negotiating a format. It is
// the device-side end of the PROBE/COMMIT exchange usbif_host_uvc.c drives
// from the other direction.
int tud_video_commit_cb(uint_fast8_t ctl_idx, uint_fast8_t stm_idx,
    video_probe_and_commit_control_t const *parameters) {
    (void)ctl_idx;
    (void)stm_idx;
    (void)parameters;
    usbif_uvc_committed = true;
    // Any frame that was in flight belongs to the previous negotiation.
    usbif_uvc_in_flight = false;
    return VIDEO_ERROR_NONE;
}

void tud_video_frame_xfer_complete_cb(uint_fast8_t ctl_idx, uint_fast8_t stm_idx) {
    (void)ctl_idx;
    (void)stm_idx;
    usbif_uvc_in_flight = false;
    usbif_uvc_completed++;
}

// True when the host has opened the stream and is waiting for pixels.
int usbif_uvc_dev_streaming(void) {
    return (tud_video_n_streaming(0, 0) && usbif_uvc_committed) ? 1 : 0;
}

// True when a frame may be written. Separate from "streaming" on purpose:
// a caller that cannot tell "the host is not watching" from "the host is
// watching but the last frame has not gone yet" ends up either spinning or
// dropping frames, and the two want opposite responses.
int usbif_uvc_dev_ready(void) {
    return (usbif_uvc_dev_streaming() && !usbif_uvc_in_flight) ? 1 : 0;
}

int usbif_uvc_dev_frame_bytes(void) {
    return USBIF_VIDEO_FRAME_BYTES;
}

void usbif_uvc_dev_size(int *width, int *height) {
    *width = USBIF_VIDEO_WIDTH;
    *height = USBIF_VIDEO_HEIGHT;
}

// Copy one frame in and hand it to the bus.
//
// Returns the number of bytes accepted, 0 if the host is not streaming or
// the previous frame is still going, or a negative error. A short frame is
// refused rather than padded: a UVC frame is a fixed size for a given
// format, and silently sending a partial one produces a torn image that
// looks like a bus problem rather than a caller mistake.
int usbif_uvc_dev_submit(const uint8_t *data, size_t len) {
    if (!usbif_uvc_dev_streaming()) {
        return 0;
    }
    if (usbif_uvc_in_flight) {
        usbif_uvc_refused++;
        return 0;
    }
    if (len != USBIF_VIDEO_FRAME_BYTES) {
        return -1;
    }
    memcpy(usbif_uvc_frame, data, len);
    usbif_uvc_in_flight = true;
    if (!tud_video_n_frame_xfer(0, 0, usbif_uvc_frame, len)) {
        usbif_uvc_in_flight = false;
        return -2;
    }
    usbif_uvc_frames++;
    return (int)len;
}

void usbif_uvc_dev_stats(uint32_t *frames, uint32_t *completed, uint32_t *refused,
    uint32_t *streaming) {
    *frames = usbif_uvc_frames;
    *completed = usbif_uvc_completed;
    *refused = usbif_uvc_refused;
    *streaming = (uint32_t)usbif_uvc_dev_streaming();
}

void usbif_uvc_dev_reset(void) {
    usbif_uvc_in_flight = false;
    usbif_uvc_committed = false;
    usbif_uvc_frames = 0;
    usbif_uvc_completed = 0;
    usbif_uvc_refused = 0;
}

#else // !CFG_TUD_VIDEO

int usbif_uvc_dev_streaming(void) { return 0; }
int usbif_uvc_dev_ready(void) { return 0; }
int usbif_uvc_dev_frame_bytes(void) { return 0; }
void usbif_uvc_dev_size(int *width, int *height) { *width = 0; *height = 0; }
int usbif_uvc_dev_submit(const uint8_t *data, size_t len) {
    (void)data; (void)len; return -3;
}
void usbif_uvc_dev_stats(uint32_t *frames, uint32_t *completed, uint32_t *refused,
    uint32_t *streaming) {
    *frames = 0; *completed = 0; *refused = 0; *streaming = 0;
}
void usbif_uvc_dev_reset(void) { }

#endif // CFG_TUD_VIDEO
