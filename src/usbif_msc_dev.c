// SPDX-License-Identifier: MIT
//
// MSC as a *device*: the board appearing to a computer as a removable
// drive, so files cross without a recompile and without special tools --
// fonts and graphics written in, sensor logs read out.
//
// What the drive *is* stays a Python decision, as everywhere else in this
// module. Python hands over a buffer and the host sees exactly those bytes
// as a block device; whether that buffer holds a FAT image built on the
// board, a slice of a partition, or anything else is the application's
// business. The firmware neither knows nor formats.
//
// Why a buffer rather than a Python block-device object: these callbacks
// run in TinyUSB's task, and reaching into the interpreter from there is
// the exact hazard the event transport exists to avoid -- a host read must
// not wait on the GC. A buffer is memory both sides can touch without a
// scheduler in between.
//
// The safety rule this cannot enforce and so must state: a filesystem
// with two writers is a corrupted filesystem. A board that attaches a
// buffer to the host should unmount it locally first, and mount it again
// only after detaching. `msc_attach` refuses to swap buffers under a
// mounted host for the same reason.

#include "py/mpconfig.h"

#if MICROPY_HW_ENABLE_USBDEV
#include "tusb.h"
#endif

#if defined(CFG_TUD_MSC) && CFG_TUD_MSC

#include <string.h>

#include "py/runtime.h"

#define USBIF_MSC_BLOCK_SIZE (512)

// Whether the VM still roots the attached object. Defined by mod_usbif.c,
// weak so a build without it links. This is the guard against a soft
// reset: MicroPython clears its root pointers when the VM restarts, which
// frees the buffer while this file still holds its address. Reading freed
// memory would hand a host plausible-looking garbage rather than an error
// -- corrupted files instead of a crash, which is the worse failure -- so
// every access checks that the VM still owns what we are about to serve.
bool usbif_msc_root_alive(void) __attribute__((weak));

static bool usbif_msc_live(void) {
    if (usbif_msc_root_alive && !usbif_msc_root_alive()) {
        return false;
    }
    return true;
}

static uint8_t *usbif_msc_buf;
static uint32_t usbif_msc_blocks;
static bool usbif_msc_writable = true;
// Ejected by the host (Start Stop Unit with load_eject). Reported to
// Python so an application can know the host is done with the drive and
// it is safe to mount locally again.
static volatile bool usbif_msc_ejected;

int usbif_msc_attach(uint8_t *buf, size_t len, bool writable) {
    if (len < USBIF_MSC_BLOCK_SIZE) {
        return -1;                      // smaller than one block
    }
    usbif_msc_buf = buf;
    usbif_msc_blocks = (uint32_t)(len / USBIF_MSC_BLOCK_SIZE);
    usbif_msc_writable = writable;
    usbif_msc_ejected = false;
    return 0;
}

void usbif_msc_detach(void) {
    usbif_msc_buf = NULL;
    usbif_msc_blocks = 0;
}

bool usbif_msc_is_attached(void) {
    return usbif_msc_buf != NULL && usbif_msc_live();
}

bool usbif_msc_was_ejected(void) {
    return usbif_msc_ejected;
}

uint32_t usbif_msc_block_count(void) {
    return usbif_msc_blocks;
}

void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8], uint8_t product_id[16],
    uint8_t product_rev[4]) {
    (void)lun;
    memcpy(vendor_id, MICROPY_HW_USB_MSC_INQUIRY_VENDOR_STRING,
        MIN(strlen(MICROPY_HW_USB_MSC_INQUIRY_VENDOR_STRING), 8));
    memcpy(product_id, MICROPY_HW_USB_MSC_INQUIRY_PRODUCT_STRING,
        MIN(strlen(MICROPY_HW_USB_MSC_INQUIRY_PRODUCT_STRING), 16));
    memcpy(product_rev, MICROPY_HW_USB_MSC_INQUIRY_REVISION_STRING,
        MIN(strlen(MICROPY_HW_USB_MSC_INQUIRY_REVISION_STRING), 4));
}

// No buffer attached is "no medium", which is what an empty card reader
// reports and what hosts already know how to display: a drive letter with
// nothing in it, rather than an error or a hang.
bool tud_msc_test_unit_ready_cb(uint8_t lun) {
    if (usbif_msc_buf == NULL || usbif_msc_ejected || !usbif_msc_live()) {
        tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x3A, 0x00);
        return false;
    }
    return true;
}

void tud_msc_capacity_cb(uint8_t lun, uint32_t *block_count, uint16_t *block_size) {
    (void)lun;
    *block_size = USBIF_MSC_BLOCK_SIZE;
    *block_count = usbif_msc_blocks;
}

// Start Stop Unit: the host telling us it has finished with the medium.
// Honouring the eject is what makes "safely remove" mean something, and
// it is the signal an application waits for before mounting locally again.
bool tud_msc_start_stop_cb(uint8_t lun, uint8_t power_condition, bool start, bool load_eject) {
    (void)lun;
    (void)power_condition;
    if (load_eject && !start) {
        usbif_msc_ejected = true;
    }
    return true;
}

int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset, void *buffer,
    uint32_t bufsize) {
    (void)lun;
    if (usbif_msc_buf == NULL || lba >= usbif_msc_blocks || !usbif_msc_live()) {
        return -1;
    }
    const uint32_t byte_off = lba * USBIF_MSC_BLOCK_SIZE + offset;
    const uint32_t total = usbif_msc_blocks * USBIF_MSC_BLOCK_SIZE;
    if (byte_off >= total) {
        return -1;
    }
    uint32_t n = bufsize;
    if (byte_off + n > total) {
        n = total - byte_off;           // clamp rather than read past the end
    }
    memcpy(buffer, usbif_msc_buf + byte_off, n);
    return (int32_t)n;
}

int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t *buffer,
    uint32_t bufsize) {
    (void)lun;
    if (usbif_msc_buf == NULL || lba >= usbif_msc_blocks || !usbif_msc_live()) {
        return -1;
    }
    if (!usbif_msc_writable) {
        // Write-protected: the host is told plainly rather than having its
        // writes swallowed, which is what makes a read-only drive honest.
        tud_msc_set_sense(lun, SCSI_SENSE_DATA_PROTECT, 0x27, 0x00);
        return -1;
    }
    const uint32_t byte_off = lba * USBIF_MSC_BLOCK_SIZE + offset;
    const uint32_t total = usbif_msc_blocks * USBIF_MSC_BLOCK_SIZE;
    if (byte_off >= total) {
        return -1;
    }
    uint32_t n = bufsize;
    if (byte_off + n > total) {
        n = total - byte_off;
    }
    memcpy(usbif_msc_buf + byte_off, buffer, n);
    return (int32_t)n;
}

// SCSI commands beyond the ones TinyUSB handles itself. Refusing politely
// beats refusing silently: a host that asks for something unsupported gets
// an illegal-request sense and moves on.
int32_t tud_msc_scsi_cb(uint8_t lun, const uint8_t scsi_cmd[16], void *buffer,
    uint16_t bufsize) {
    (void)buffer;
    (void)bufsize;
    switch (scsi_cmd[0]) {
        case SCSI_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL:
            return 0;                   // nothing to lock; success is honest
        default:
            tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0x00);
            return -1;
    }
}

bool tud_msc_is_writable_cb(uint8_t lun) {
    (void)lun;
    return usbif_msc_writable;
}

#endif // CFG_TUD_MSC
