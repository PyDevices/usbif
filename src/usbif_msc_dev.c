// SPDX-License-Identifier: MIT
//
// MSC as a *device*: the board appearing to a computer as a removable
// drive, so files cross without a recompile and without special tools --
// fonts and graphics written in, sensor logs read out.
//
// What the drive *is* stays a Python decision, as everywhere else in this
// module. `msc_attach` hands over a buffer and the host sees exactly those
// bytes as a block device -- whether that buffer holds a FAT image built on
// the board, a slice of a partition, or anything else is the application's
// business. `msc_attach_blockdev` (below) hands over an object instead, for
// real storage a buffer can't reach: an SD card or a flash partition.
//
// The original version of this comment claimed calling into the
// interpreter from these callbacks was unsafe, reasoning that they "run in
// TinyUSB's task." That's imprecise for this port: this esp32 build feeds
// tud_task_ext() through MicroPython's own scheduler
// (mp_sched_schedule_node, in mp_usbd.c/mp_usbd_runtime.c), so these
// callbacks run interleaved with ordinary bytecode on the *same* thread as
// the VM, not on a foreign one -- exactly the mechanism upstream's own
// machine.USBDevice runtime already uses to call user Python from inside a
// TinyUSB callback (mp_usbd_runtime.c's usbd_callback_function_n). There is
// no thread-safety hazard calling Python from here. The buffer path stays
// the default anyway, because the real cost is latency, not safety: every
// other USB event -- CDC, HID, MIDI -- waits behind whatever a scheduled
// callback does, and a slow block-device object (bit-banged SPI, say) adds
// real time to that queue and risks the host's own SCSI command timeout. A
// buffer never has that cost. Measure before trusting a slow backing store
// under load.
//
// The safety rule this cannot enforce and so must state: a filesystem
// with two writers is a corrupted filesystem. A board that attaches a
// buffer to the host should unmount it locally first, and mount it again
// only after detaching. `msc_attach`/`msc_attach_blockdev` refuse to swap
// what's attached under a mounted host for the same reason.

#include "py/mpconfig.h"

#if MICROPY_HW_ENABLE_USBDEV
#include "tusb.h"
#endif

#if defined(CFG_TUD_MSC) && CFG_TUD_MSC

#include <string.h>

#include "py/runtime.h"
#include "py/objarray.h"    // MP_OBJ_ARRAY_TYPECODE_FLAG_RW

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

// Real-storage backing: read10/write10 call straight into a Python object's
// readblocks(block_num, buf)/writeblocks(block_num, buf) -- MicroPython's
// standard block-device protocol, which mip's `sdcard.SDCard` and a raw
// flash partition object both already speak. See this file's header
// comment for why calling into the interpreter from here is safe on this
// port. mod_usbif.c owns the actual object reference (as a VM root, same
// soft-reset hazard as the buffer path) and exposes it through this getter,
// weak for the same reason usbif_msc_root_alive() is.
static bool usbif_msc_blockdev_mode;
extern mp_obj_t usbif_msc_get_obj(void) __attribute__((weak));

int usbif_msc_attach(uint8_t *buf, size_t len, bool writable) {
    if (usbif_msc_blockdev_mode) {
        return -3;                      // one MSC session at a time
    }
    if (len < USBIF_MSC_BLOCK_SIZE) {
        return -1;                      // smaller than one block
    }
    usbif_msc_buf = buf;
    usbif_msc_blocks = (uint32_t)(len / USBIF_MSC_BLOCK_SIZE);
    usbif_msc_writable = writable;
    usbif_msc_ejected = false;
    return 0;
}

// num_blocks/block_size come from the object's own ioctl(4, 0)/ioctl(5, 0)
// (mod_usbif.c reads them before calling this) rather than being guessed
// here. block_size must be the standard 512-byte sector -- refused rather
// than silently misinterpreted if a backing store ever reports otherwise,
// since every offset computation below assumes it.
int usbif_msc_attach_blockdev(uint32_t num_blocks, uint32_t block_size, bool writable) {
    if (usbif_msc_buf != NULL || usbif_msc_blockdev_mode) {
        return -3;                      // one MSC session at a time
    }
    if (block_size != USBIF_MSC_BLOCK_SIZE) {
        return -4;
    }
    usbif_msc_blocks = num_blocks;
    usbif_msc_writable = writable;
    usbif_msc_ejected = false;
    usbif_msc_blockdev_mode = true;
    return 0;
}

void usbif_msc_detach(void) {
    usbif_msc_buf = NULL;
    usbif_msc_blocks = 0;
    usbif_msc_blockdev_mode = false;
}

bool usbif_msc_is_attached(void) {
    return (usbif_msc_buf != NULL || usbif_msc_blockdev_mode) && usbif_msc_live();
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
    if ((usbif_msc_buf == NULL && !usbif_msc_blockdev_mode) || usbif_msc_ejected || !usbif_msc_live()) {
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

// Scratch for the blockdev path, sized to the largest single transfer
// TinyUSB will ever hand a read10/write10 callback in this build
// (CFG_TUD_MSC_BUFSIZE, deliberately set to MICROPY_FATFS_MAX_SS -- see
// the tusb_config.h comment "avoid partial read/writes"). A whole-block
// read/write copies straight from/to it; a sub-block write additionally
// needs it as a read-modify-write staging area.
#define USBIF_MSC_BD_SCRATCH_LEN (CFG_TUD_MSC_BUFSIZE)
static uint8_t usbif_msc_bd_scratch[USBIF_MSC_BD_SCRATCH_LEN];

// Diagnostic counters: the first question when the blockdev path
// misbehaves is whether it's being called at all and, if so, whether the
// Python call underneath it is succeeding.
uint32_t usbif_msc_bd_calls, usbif_msc_bd_errors;

// obj.readblocks(first_block, buf) / obj.writeblocks(first_block, buf) --
// MicroPython's standard block-device protocol. Any Python exception is
// swallowed here rather than printed: printing from inside a TinyUSB
// callback risks recursing into TinyUSB itself if a CDC console shares
// this build (the same hazard upstream's usbd_callback_function_n avoids
// by deferring exceptions rather than printing them inline). A swallowed
// exception here becomes a failed SCSI command, which is a visible I/O
// error to the host rather than a silent wrong answer.
static bool usbif_msc_bd_call(qstr method, uint32_t first_block, uint32_t len) {
    usbif_msc_bd_calls++;
    mp_obj_t obj = (usbif_msc_get_obj != NULL) ? usbif_msc_get_obj() : MP_OBJ_NULL;
    if (obj == MP_OBJ_NULL || obj == mp_const_none) {
        usbif_msc_bd_errors++;
        return false;
    }
    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0) {
        mp_obj_t dest[4];
        mp_load_method(obj, method, dest);
        dest[2] = mp_obj_new_int_from_uint(first_block);
        // The RW flag is not optional: mp_obj_new_memoryview() stores the
        // typecode verbatim, and a memoryview without it refuses every
        // MP_BUFFER_WRITE request (py/objarray.c). readblocks() writes into
        // the buffer it is handed, so a read-only view makes every single
        // call raise -- which is exactly what it did: 13591 calls, 13591
        // failures, a host retrying forever and a drive that reported its
        // size correctly (no Python needed for capacity) but could never be
        // read. Found by counter, not by inspection.
        dest[3] = mp_obj_new_memoryview('B' | MP_OBJ_ARRAY_TYPECODE_FLAG_RW,
            len, usbif_msc_bd_scratch);
        mp_call_method_n_kw(2, 0, dest);
        nlr_pop();
        return true;
    } else {
        usbif_msc_bd_errors++;
        return false;
    }
}

uint32_t usbif_msc_bd_stats(uint32_t *errors) {
    *errors = usbif_msc_bd_errors;
    return usbif_msc_bd_calls;
}

int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset, void *buffer,
    uint32_t bufsize) {
    (void)lun;
    if ((usbif_msc_buf == NULL && !usbif_msc_blockdev_mode)
        || lba >= usbif_msc_blocks || !usbif_msc_live()) {
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
    if (usbif_msc_blockdev_mode) {
        const uint32_t first_block = byte_off / USBIF_MSC_BLOCK_SIZE;
        const uint32_t last_block = (byte_off + n - 1) / USBIF_MSC_BLOCK_SIZE;
        const uint32_t count = last_block - first_block + 1;
        if (count * USBIF_MSC_BLOCK_SIZE > USBIF_MSC_BD_SCRATCH_LEN) {
            return -1;                  // shouldn't happen given CFG_TUD_MSC_BUFSIZE
        }
        if (!usbif_msc_bd_call(MP_QSTR_readblocks, first_block, count * USBIF_MSC_BLOCK_SIZE)) {
            return -1;
        }
        const uint32_t sub_off = byte_off - first_block * USBIF_MSC_BLOCK_SIZE;
        memcpy(buffer, usbif_msc_bd_scratch + sub_off, n);
        return (int32_t)n;
    }
    memcpy(buffer, usbif_msc_buf + byte_off, n);
    return (int32_t)n;
}

int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t *buffer,
    uint32_t bufsize) {
    (void)lun;
    if ((usbif_msc_buf == NULL && !usbif_msc_blockdev_mode)
        || lba >= usbif_msc_blocks || !usbif_msc_live()) {
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
    if (usbif_msc_blockdev_mode) {
        const uint32_t first_block = byte_off / USBIF_MSC_BLOCK_SIZE;
        const uint32_t last_block = (byte_off + n - 1) / USBIF_MSC_BLOCK_SIZE;
        const uint32_t count = last_block - first_block + 1;
        if (count * USBIF_MSC_BLOCK_SIZE > USBIF_MSC_BD_SCRATCH_LEN) {
            return -1;
        }
        const uint32_t sub_off = byte_off - first_block * USBIF_MSC_BLOCK_SIZE;
        const bool whole_blocks = (sub_off == 0) && (n == count * USBIF_MSC_BLOCK_SIZE);
        if (!whole_blocks) {
            // Sub-block write: preserve the untouched bytes in the first/last
            // block by reading the full range first, in practice a path this
            // build's CFG_TUD_MSC_BUFSIZE tuning is meant to avoid -- but
            // never assumed away, since silently dropping neighbouring bytes
            // would corrupt the filesystem rather than just fail loudly.
            if (!usbif_msc_bd_call(MP_QSTR_readblocks, first_block, count * USBIF_MSC_BLOCK_SIZE)) {
                return -1;
            }
        }
        memcpy(usbif_msc_bd_scratch + sub_off, buffer, n);
        if (!usbif_msc_bd_call(MP_QSTR_writeblocks, first_block, count * USBIF_MSC_BLOCK_SIZE)) {
            return -1;
        }
        return (int32_t)n;
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
