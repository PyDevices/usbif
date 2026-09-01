// SPDX-License-Identifier: MIT
//
// The `_usbif` native module: the MicroPython-facing surface of usbif.
//
// Scope note (Phase 1): this wires up the module, the capability report and
// the event transport. The IDF USB Host Library and its class drivers arrive
// in Phase 2 and push into the same ring declared here, so nothing about the
// Python-facing contract changes when they do -- which is the point of landing
// the transport first.
//
// The Python half of this backend lives in pydevices `lib/usbif/native_usb.py`
// and documents the same surface from the caller's side. The two are meant to
// be read together.

#include "py/runtime.h"
#include "py/obj.h"
#include "py/mperrno.h"
#include <string.h>

#include "shared/usbif_ringbuf.h"

#if MICROPY_HW_ENABLE_USBDEV
#include "tusb.h"
#include "mp_usbd.h"
#endif

#include "usbif_classes.h"

// From usbif_desc.c, present on any build with the TinyUSB extension
// header. The USB identity is chosen at runtime from these bits.
#if defined(MICROPY_HW_USB_EXT_TUSB_CONFIG) && MICROPY_HW_ENABLE_USBDEV
#define USBIF_HAVE_FN (1)
extern uint16_t usbif_fn_get(void);
extern uint16_t usbif_fn_built(void);
extern int usbif_fn_set(uint16_t mask);
extern int usbif_desc_check(void);
#define USBIF_FN_CDC   (1u << 0)
#define USBIF_FN_MSC   (1u << 1)
#define USBIF_FN_AUDIO (1u << 2)
#define USBIF_FN_MIDI  (1u << 3)
#define USBIF_FN_HID   (1u << 4)
#else
#define USBIF_HAVE_FN (0)
#endif

// Host engine availability: the IDF USB Host Library, on chips with an OTG
// controller. The same gate guards usbif_host.c; elsewhere the host_* calls
// keep their Phase-1 stub behaviour (an honest empty capability set).
// Usermod sources build with MicroPython's define set, not the IDF's, so
// ESP_PLATFORM is NOT defined here -- learned the hard way when this gate
// silently compiled the engine out and the Phase 1 stubs answered in its
// place. sdkconfig.h existing (and naming an OTG controller) is the real
// signal that the IDF host library is available.
#if defined(__has_include)
#if __has_include("sdkconfig.h")
#include "sdkconfig.h"
#endif
#endif
#if defined(CONFIG_SOC_USB_OTG_SUPPORTED) && CONFIG_SOC_USB_OTG_SUPPORTED
#define USBIF_HAVE_HOST (1)
extern int usbif_host_start_c(void);
extern void usbif_host_stop_c(void);
extern bool usbif_host_is_running(void);
extern int usbif_host_snapshot(usbif_event_t *out, int max);
extern uint32_t usbif_host_attaches, usbif_host_detaches, usbif_host_errors;
extern int usbif_host_lib_counts(int *num_devices, int *num_clients);
extern int usbif_host_port_cycle(void);
extern void usbif_host_set_class_filter(uint16_t mask);
extern void usbif_host_intr_dump(void);
extern int usbif_cdc_open(uint32_t dev_id);
extern int usbif_cdc_write(const uint8_t *data, size_t len);
extern int usbif_cdc_read(uint8_t *out, size_t max);
extern void usbif_cdc_close(void);
extern uint32_t usbif_cdc_rx_dropped(void);
extern int usbif_hid_open(uint32_t dev_id);
extern int usbif_hid_read(uint8_t *out, size_t max);
extern void usbif_hid_close(void);
extern int usbif_msc_open(uint32_t dev_id);
extern int usbif_msc_info(uint32_t *num_blocks, uint32_t *block_size, const char **inquiry);
extern int usbif_msc_read_block(uint32_t lba, uint8_t *out, size_t max);
extern void usbif_msc_close(void);
#else
#define USBIF_HAVE_HOST (0)
#endif

// Event capacity. Sized from measurement rather than a round number: the worst
// VM stall observed on an ESP32-S3 was 1537 ms during flash writes, and a
// 1 kHz event stream over that window is ~1500 events. Full HID/MIDI traffic
// at that rate through a stall this long is a pathological case rather than a
// working one, so the ring holds a generous fraction of it and reports
// overflow instead of pretending. 256 records is 2 KB at 8 bytes each.
#define USBIF_EVENT_CAPACITY (256)

static usbif_event_t usbif_event_slots[USBIF_EVENT_CAPACITY];
static usbif_ringbuf_t usbif_events;
static bool usbif_host_running = false;

// Producer hook for the host engine (usbif_host.c). Called only from the
// host task, preserving the ring's single-producer contract.
void usbif_host_emit(const usbif_event_t *event) {
    usbif_rb_push(&usbif_events, event);
}

static const struct {
    uint16_t bit;
    qstr name;
} usbif_class_names[] = {
    { USBIF_CLASS_HID, MP_QSTR_hid },
    { USBIF_CLASS_MSC, MP_QSTR_msc },
    { USBIF_CLASS_CDC, MP_QSTR_cdc },
    { USBIF_CLASS_MIDI, MP_QSTR_midi },
    { USBIF_CLASS_UAC, MP_QSTR_uac },
    { USBIF_CLASS_UVC, MP_QSTR_uvc },
};

static const qstr usbif_speed_names[] = {
    MP_QSTRnull, MP_QSTR_low, MP_QSTR_full, MP_QSTR_high,
};

// Classes this firmware was actually built with. Phase 1 left this at zero
// pending class drivers; CDC, HID and MSC host drivers now exist (unconditionally,
// under the same USBIF_HAVE_HOST gate), so an honest answer names those three --
// MIDI/UAC/UVC host drivers don't exist yet, so those bits stay unset regardless
// of what a caller asks host_start() for.
static uint16_t usbif_supported_classes(void) {
    #if USBIF_HAVE_HOST
    return USBIF_CLASS_CDC | USBIF_CLASS_HID | USBIF_CLASS_MSC;
    #else
    return 0;
    #endif
}

static mp_obj_t usbif_classes_to_set(uint16_t mask) {
    mp_obj_t set = mp_obj_new_set(0, NULL);
    for (size_t i = 0; i < MP_ARRAY_SIZE(usbif_class_names); i++) {
        if (mask & usbif_class_names[i].bit) {
            mp_obj_set_store(set, MP_OBJ_NEW_QSTR(usbif_class_names[i].name));
        }
    }
    return set;
}

// Reverse of usbif_classes_to_set: an iterable of class-name strings (a tuple,
// per the native_usb.py contract) to a bitmask. Unknown names are ignored
// rather than raising -- host_start() intersects the result against
// usbif_supported_classes() anyway, so an unrecognised or unsupported name
// simply doesn't start, which is what "not built into this firmware" should
// look like on the host side too (see NativeDevice.functions() for the same
// philosophy on the device side).
static uint16_t usbif_classes_from_iterable(mp_obj_t classes_in) {
    size_t len;
    mp_obj_t *items;
    mp_obj_get_array(classes_in, &len, &items);
    uint16_t mask = 0;
    for (size_t j = 0; j < len; j++) {
        qstr name = mp_obj_str_get_qstr(items[j]);
        for (size_t i = 0; i < MP_ARRAY_SIZE(usbif_class_names); i++) {
            if (usbif_class_names[i].name == name) {
                mask |= usbif_class_names[i].bit;
                break;
            }
        }
    }
    return mask;
}

static mp_obj_t usbif_capabilities(void) {
    return usbif_classes_to_set(usbif_supported_classes());
}
static MP_DEFINE_CONST_FUN_OBJ_0(usbif_capabilities_obj, usbif_capabilities);

// Build the DeviceInfo row the Python side turns into a namedtuple. Field
// order matches usbif.DeviceInfo exactly; the parity harness asserts on those
// names, so a change here is a change to the portable contract.
static mp_obj_t usbif_device_row(const usbif_event_t *event) {
    mp_obj_t items[7];
    items[0] = mp_obj_new_int_from_uint(event->dev_id);
    items[1] = mp_obj_new_int_from_uint(event->vid);
    items[2] = mp_obj_new_int_from_uint(event->pid);
    items[3] = mp_const_none;   // product: filled by the class drivers
    items[4] = mp_const_none;   // serial
    items[5] = usbif_classes_to_set(event->classes);
    items[6] = event->speed < MP_ARRAY_SIZE(usbif_speed_names) && event->speed
        ? MP_OBJ_NEW_QSTR(usbif_speed_names[event->speed])
        : mp_const_none;
    return mp_obj_new_tuple(7, items);
}

static mp_obj_t usbif_host_start(mp_obj_t classes_in) {
    // Intersected against what this firmware can actually drive: a caller
    // naming an unbuilt or unknown class doesn't start it, matching the
    // return value ("the set actually started") that native_usb.py documents.
    uint16_t wanted = usbif_classes_from_iterable(classes_in) & usbif_supported_classes();
    // Only a genuine start initialises the ring: a second host_start() on a
    // running host must be a no-op, not a wipe of queued events -- learned
    // when the NUCLEO's attach events vanished under an idempotent restart.
    if (!usbif_host_running) {
        usbif_rb_init(&usbif_events, usbif_event_slots, USBIF_EVENT_CAPACITY);
    }
    #if USBIF_HAVE_HOST
    usbif_host_set_class_filter(wanted);
    int err = usbif_host_start_c();
    if (err != 0) {
        mp_raise_OSError(err > 0 ? err : MP_EIO);
    }
    #endif
    usbif_host_running = true;
    return usbif_classes_to_set(wanted);
}
static MP_DEFINE_CONST_FUN_OBJ_1(usbif_host_start_obj, usbif_host_start);

static mp_obj_t usbif_host_stop(void) {
    #if USBIF_HAVE_HOST
    usbif_host_stop_c();
    #endif
    usbif_host_running = false;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(usbif_host_stop_obj, usbif_host_stop);

static mp_obj_t usbif_host_devices(void) {
    #if USBIF_HAVE_HOST
    usbif_event_t rows[8];
    int n = usbif_host_snapshot(rows, 8);
    mp_obj_t items[8];
    for (int i = 0; i < n; i++) {
        items[i] = usbif_device_row(&rows[i]);
    }
    return mp_obj_new_tuple(n, items);
    #else
    return mp_obj_new_tuple(0, NULL);
    #endif
}
static MP_DEFINE_CONST_FUN_OBJ_0(usbif_host_devices_obj, usbif_host_devices);

// Pop up to `limit` records and report whether anything was lost since the
// last call. Bounded so one poll cannot turn a burst into an unbounded
// allocation on a device with a small heap; whatever is left stays queued.
static mp_obj_t usbif_host_drain(mp_obj_t limit_in) {
    mp_int_t limit = mp_obj_get_int(limit_in);
    if (limit < 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("limit must not be negative"));
    }

    mp_obj_t list = mp_obj_new_list(0, NULL);
    usbif_event_t event;
    while (limit-- > 0 && usbif_rb_pop(&usbif_events, &event)) {
        mp_obj_t pair[2] = {
            MP_OBJ_NEW_SMALL_INT(event.kind),
            usbif_device_row(&event),
        };
        mp_obj_list_append(list, mp_obj_new_tuple(2, pair));
    }

    mp_obj_t result[2] = {
        list,
        mp_obj_new_bool(usbif_rb_take_dropped(&usbif_events) != 0),
    };
    return mp_obj_new_tuple(2, result);
}
static MP_DEFINE_CONST_FUN_OBJ_1(usbif_host_drain_obj, usbif_host_drain);

// Debug aid: hand back the built-in configuration descriptor exactly as the
// host receives it, plus the length the build computed for it. A descriptor
// whose declared wTotalLength disagrees with the bytes actually emitted is
// the classic cause of a function that enumerates and then refuses to start,
// and it is invisible from the host side -- Windows reports "cannot start"
// either way.
static mp_obj_t usbif_builtin_desc_cfg(void) {
    // Only USB-device ports have a built-in descriptor to show; on desktop
    // backends (unix, windows) the honest answer is None, same shape as the
    // other capability-gated accessors. Unguarded, this reference broke
    // every desktop build in the workspace -- caught by the first unix
    // engine rebuild after usbif joined the USER_C_MODULES glob.
    #if MICROPY_HW_ENABLE_USBDEV
    mp_obj_t items[2] = {
        mp_obj_new_bytes(mp_usbd_builtin_desc_cfg, MP_USBD_BUILTIN_DESC_CFG_LEN),
        MP_OBJ_NEW_SMALL_INT(MP_USBD_BUILTIN_DESC_CFG_LEN),
    };
    return mp_obj_new_tuple(2, items);
    #else
    return mp_const_none;
    #endif
}
static MP_DEFINE_CONST_FUN_OBJ_0(usbif_builtin_desc_cfg_obj, usbif_builtin_desc_cfg);

#if CFG_TUD_AUDIO
extern uint32_t usbif_uac_get_reqs, usbif_uac_set_reqs, usbif_uac_itf_sets, usbif_uac_unhandled;
extern uint32_t usbif_uac_overflows;
extern bool usbif_uac_is_streaming(void);
extern uint32_t usbif_uac_current_rate(void);
extern bool usbif_uac_is_muted(void);
extern int usbif_uac_volume_db256(void);
extern uint32_t usbif_uac_gain(void);

extern void usbif_uac_note_read(void);
extern int usbif_pump_start(int i2s_id, int bclk, int ws, int dout, int mclk,
    uint32_t rate, int bits, int channels);
extern void usbif_pump_stop(void);
extern bool usbif_pump_is_running(void);
extern uint32_t usbif_pump_bytes, usbif_pump_idle, usbif_pump_timeouts, usbif_pump_shed;
#endif

// Diagnostic: what the host has actually asked the audio function for.
static mp_obj_t usbif_uac_stats(void) {
    #if CFG_TUD_AUDIO
    mp_obj_t items[7] = {
        mp_obj_new_int_from_uint(usbif_uac_get_reqs),
        mp_obj_new_int_from_uint(usbif_uac_set_reqs),
        mp_obj_new_int_from_uint(usbif_uac_itf_sets),
        mp_obj_new_int_from_uint(usbif_uac_unhandled),
        mp_obj_new_bool(usbif_uac_is_streaming()),
        mp_obj_new_int_from_uint(usbif_uac_current_rate()),
        mp_obj_new_int_from_uint(usbif_uac_overflows),
    };
    return mp_obj_new_tuple(7, items);
    #else
    return mp_const_none;
    #endif
}
static MP_DEFINE_CONST_FUN_OBJ_0(usbif_uac_stats_obj, usbif_uac_stats);

// --- UAC playback: move received audio out of TinyUSB's FIFO.
//
// The isochronous endpoint itself is serviced entirely in C, on TinyUSB's
// task, whatever Python is doing -- that is the whole point of the split.
// What Python does here is move already-buffered blocks on to an audio sink,
// a soft deadline governed by FIFO depth rather than a per-frame one.

static mp_obj_t usbif_uac_available(void) {
    #if defined(CFG_TUD_AUDIO) && CFG_TUD_AUDIO
    return mp_obj_new_int_from_uint(tud_audio_available());
    #else
    return MP_OBJ_NEW_SMALL_INT(0);
    #endif
}
static MP_DEFINE_CONST_FUN_OBJ_0(usbif_uac_available_obj, usbif_uac_available);

// Read up to len(buf) bytes of received PCM into a writable buffer, returning
// the byte count. Returns 0 rather than blocking when the host is not
// streaming, so a caller can poll it from an ordinary loop.
static mp_obj_t usbif_uac_read(mp_obj_t buf_in) {
    #if defined(CFG_TUD_AUDIO) && CFG_TUD_AUDIO
    mp_buffer_info_t buf;
    mp_get_buffer_raise(buf_in, &buf, MP_BUFFER_WRITE);
    // Records that a consumer is alive, so the C side stops discarding.
    usbif_uac_note_read();
    uint16_t count = tud_audio_read(buf.buf, (uint16_t)MIN(buf.len, UINT16_MAX));
    // Host volume and mute, same as the C pump applies, so the two pump
    // paths sound identical. Whole samples only: a trailing odd byte means
    // the stream is misframed and scaling half a sample cannot help it.
    uint32_t gain = usbif_uac_gain();
    if (gain != 65536u) {
        int16_t *s = (int16_t *)buf.buf;
        for (uint16_t i = 0; i < count / 2; i++) {
            s[i] = (int16_t)(((int64_t)s[i] * gain) >> 16);
        }
    }
    return mp_obj_new_int_from_uint(count);
    #else
    (void)buf_in;
    return MP_OBJ_NEW_SMALL_INT(0);
    #endif
}
static MP_DEFINE_CONST_FUN_OBJ_1(usbif_uac_read_obj, usbif_uac_read);

// The host's own volume and mute, so a board can follow the slider on the PC
// instead of ignoring it. Returned raw (1/256 dB, and a mute flag) because
// mapping decibels onto a particular codec's scale is the board's business.
static mp_obj_t usbif_uac_volume(void) {
    #if defined(CFG_TUD_AUDIO) && CFG_TUD_AUDIO
    mp_obj_t items[2] = {
        mp_obj_new_bool(usbif_uac_is_muted()),
        MP_OBJ_NEW_SMALL_INT(usbif_uac_volume_db256()),
    };
    return mp_obj_new_tuple(2, items);
    #else
    return mp_const_none;
    #endif
}
static MP_DEFINE_CONST_FUN_OBJ_0(usbif_uac_volume_obj, usbif_uac_volume);

// Advertise the audio function to the host, or stop advertising it. The board
// re-enumerates either way -- USB has no way to change identity in place.
static mp_obj_t usbif_uac_enable(size_t n_args, const mp_obj_t *args) {
    #if USBIF_HAVE_FN
    // Compatibility surface over the general one: "the audio function, on
    // or off, leaving every other function as it is."
    if (n_args) {
        uint16_t mask = usbif_fn_get();
        if (mp_obj_is_true(args[0])) {
            mask |= (uint16_t)(USBIF_FN_AUDIO | USBIF_FN_MIDI);
            mask &= usbif_fn_built();
        } else {
            mask &= (uint16_t) ~(USBIF_FN_AUDIO | USBIF_FN_MIDI);
        }
        usbif_fn_set(mask);
    }
    return mp_obj_new_bool((usbif_fn_get() & USBIF_FN_AUDIO) != 0);
    #else
    (void)n_args;
    (void)args;
    return mp_const_false;
    #endif
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(usbif_uac_enable_obj, 0, 1, usbif_uac_enable);

// Start moving received audio to I2S from a C task, taking the interpreter
// out of the isochronous path entirely. Pins and format are supplied by the
// caller: they are board facts, and usbif has no business guessing them.
// The codec itself stays Python's to bring up.
static mp_obj_t usbif_uac_pump_start(size_t n_args, const mp_obj_t *pos_args,
    mp_map_t *kw_args) {
    #if defined(CFG_TUD_AUDIO) && CFG_TUD_AUDIO
    enum { ARG_bclk, ARG_ws, ARG_dout, ARG_mclk, ARG_rate, ARG_bits, ARG_channels,
           ARG_i2s_id };
    static const mp_arg_t allowed[] = {
        { MP_QSTR_bclk, MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = -1 } },
        { MP_QSTR_ws, MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = -1 } },
        { MP_QSTR_dout, MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = -1 } },
        // Give the I2S peripheral the MCLK pin so every clock the codec sees
        // comes from one divider. -1 leaves MCLK to whatever else drives it.
        { MP_QSTR_mclk, MP_ARG_KW_ONLY | MP_ARG_INT, { .u_int = -1 } },
        { MP_QSTR_rate, MP_ARG_KW_ONLY | MP_ARG_INT,
          { .u_int = CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE } },
        { MP_QSTR_bits, MP_ARG_KW_ONLY | MP_ARG_INT, { .u_int = 16 } },
        { MP_QSTR_channels, MP_ARG_KW_ONLY | MP_ARG_INT, { .u_int = 1 } },
        { MP_QSTR_i2s_id, MP_ARG_KW_ONLY | MP_ARG_INT, { .u_int = 0 } },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed), allowed, args);

    int err = usbif_pump_start(args[ARG_i2s_id].u_int, args[ARG_bclk].u_int,
        args[ARG_ws].u_int, args[ARG_dout].u_int, args[ARG_mclk].u_int,
        (uint32_t)args[ARG_rate].u_int, args[ARG_bits].u_int,
        args[ARG_channels].u_int);
    if (err != 0) {
        mp_raise_OSError(err);
    }
    return mp_const_none;
    #else
    (void)n_args;
    (void)pos_args;
    (void)kw_args;
    mp_raise_NotImplementedError(MP_ERROR_TEXT("usbif built without audio"));
    #endif
}
// Minimum 0 positional: MP_ARG_REQUIRED still enforces that bclk/ws/dout are
// supplied, but they may be given by keyword, which reads far better at a call
// site naming three GPIO numbers.
static MP_DEFINE_CONST_FUN_OBJ_KW(usbif_uac_pump_start_obj, 0, usbif_uac_pump_start);

static mp_obj_t usbif_uac_pump_stop(void) {
    #if defined(CFG_TUD_AUDIO) && CFG_TUD_AUDIO
    usbif_pump_stop();
    #endif
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(usbif_uac_pump_stop_obj, usbif_uac_pump_stop);

// (running, bytes moved, idle polls, sink timeouts)
static mp_obj_t usbif_uac_pump_stats(void) {
    #if defined(CFG_TUD_AUDIO) && CFG_TUD_AUDIO
    mp_obj_t items[5] = {
        mp_obj_new_bool(usbif_pump_is_running()),
        mp_obj_new_int_from_uint(usbif_pump_bytes),
        mp_obj_new_int_from_uint(usbif_pump_idle),
        mp_obj_new_int_from_uint(usbif_pump_timeouts),
        mp_obj_new_int_from_uint(usbif_pump_shed),
    };
    return mp_obj_new_tuple(5, items);
    #else
    return mp_const_none;
    #endif
}
static MP_DEFINE_CONST_FUN_OBJ_0(usbif_uac_pump_stats_obj, usbif_uac_pump_stats);

// Diagnostic: (running, attaches, detaches, errors, lib_devices, lib_clients).
static mp_obj_t usbif_host_stats(void) {
    #if USBIF_HAVE_HOST
    int devs = -1, clients = -1;
    usbif_host_lib_counts(&devs, &clients);
    mp_obj_t items[6] = {
        mp_obj_new_bool(usbif_host_is_running()),
        mp_obj_new_int_from_uint(usbif_host_attaches),
        mp_obj_new_int_from_uint(usbif_host_detaches),
        mp_obj_new_int_from_uint(usbif_host_errors),
        MP_OBJ_NEW_SMALL_INT(devs),
        MP_OBJ_NEW_SMALL_INT(clients),
    };
    return mp_obj_new_tuple(6, items);
    #else
    return mp_const_none;
    #endif
}
static MP_DEFINE_CONST_FUN_OBJ_0(usbif_host_stats_obj, usbif_host_stats);

// Software replug: power-cycle the root port to force fresh connect detection.
static mp_obj_t usbif_host_port_cycle_py(void) {
    #if USBIF_HAVE_HOST
    return MP_OBJ_NEW_SMALL_INT(usbif_host_port_cycle());
    #else
    return mp_const_none;
    #endif
}
static MP_DEFINE_CONST_FUN_OBJ_0(usbif_host_port_cycle_obj, usbif_host_port_cycle_py);

static mp_obj_t usbif_host_intr_dump_py(void) {
    #if USBIF_HAVE_HOST
    usbif_host_intr_dump();
    #endif
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(usbif_host_intr_dump_obj, usbif_host_intr_dump_py);

// CDC-ACM session: open by the dev_id an attach event (or host_devices row)
// reported. One session at a time in this first cut.
static mp_obj_t usbif_host_cdc_open(mp_obj_t dev_id_in) {
    #if USBIF_HAVE_HOST
    int err = usbif_cdc_open((uint32_t)mp_obj_get_int(dev_id_in));
    if (err != 0) {
        mp_raise_OSError(MP_EIO);
    }
    #else
    (void)dev_id_in;
    mp_raise_OSError(MP_EOPNOTSUPP);
    #endif
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(usbif_host_cdc_open_obj, usbif_host_cdc_open);

static mp_obj_t usbif_host_cdc_write(mp_obj_t buf_in) {
    #if USBIF_HAVE_HOST
    mp_buffer_info_t buf;
    mp_get_buffer_raise(buf_in, &buf, MP_BUFFER_READ);
    // Write the whole buffer through the single in-flight transfer.
    size_t done = 0;
    while (done < buf.len) {
        int n = usbif_cdc_write((const uint8_t *)buf.buf + done, buf.len - done);
        if (n <= 0) {
            break;
        }
        done += (size_t)n;
    }
    return mp_obj_new_int_from_uint(done);
    #else
    (void)buf_in;
    return MP_OBJ_NEW_SMALL_INT(0);
    #endif
}
static MP_DEFINE_CONST_FUN_OBJ_1(usbif_host_cdc_write_obj, usbif_host_cdc_write);

static mp_obj_t usbif_host_cdc_read(mp_obj_t buf_in) {
    #if USBIF_HAVE_HOST
    mp_buffer_info_t buf;
    mp_get_buffer_raise(buf_in, &buf, MP_BUFFER_WRITE);
    int n = usbif_cdc_read((uint8_t *)buf.buf, buf.len);
    return MP_OBJ_NEW_SMALL_INT(n < 0 ? 0 : n);
    #else
    (void)buf_in;
    return MP_OBJ_NEW_SMALL_INT(0);
    #endif
}
static MP_DEFINE_CONST_FUN_OBJ_1(usbif_host_cdc_read_obj, usbif_host_cdc_read);

static mp_obj_t usbif_host_cdc_close_py(void) {
    #if USBIF_HAVE_HOST
    usbif_cdc_close();
    #endif
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(usbif_host_cdc_close_obj, usbif_host_cdc_close_py);

static mp_obj_t usbif_host_hid_open(mp_obj_t dev_id_in) {
    #if USBIF_HAVE_HOST
    if (usbif_hid_open((uint32_t)mp_obj_get_int(dev_id_in)) != 0) {
        mp_raise_OSError(MP_EIO);
    }
    #else
    (void)dev_id_in;
    mp_raise_OSError(MP_EOPNOTSUPP);
    #endif
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(usbif_host_hid_open_obj, usbif_host_hid_open);

// Returns one input report's length (bytes written into buf), 0 if none.
static mp_obj_t usbif_host_hid_read(mp_obj_t buf_in) {
    #if USBIF_HAVE_HOST
    mp_buffer_info_t buf;
    mp_get_buffer_raise(buf_in, &buf, MP_BUFFER_WRITE);
    int n = usbif_hid_read((uint8_t *)buf.buf, buf.len);
    return MP_OBJ_NEW_SMALL_INT(n < 0 ? 0 : n);
    #else
    (void)buf_in;
    return MP_OBJ_NEW_SMALL_INT(0);
    #endif
}
static MP_DEFINE_CONST_FUN_OBJ_1(usbif_host_hid_read_obj, usbif_host_hid_read);

static mp_obj_t usbif_host_hid_close_py(void) {
    #if USBIF_HAVE_HOST
    usbif_hid_close();
    #endif
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(usbif_host_hid_close_obj, usbif_host_hid_close_py);

static mp_obj_t usbif_host_msc_open(mp_obj_t dev_id_in) {
    #if USBIF_HAVE_HOST
    if (usbif_msc_open((uint32_t)mp_obj_get_int(dev_id_in)) != 0) {
        mp_raise_OSError(MP_EIO);
    }
    #else
    (void)dev_id_in;
    mp_raise_OSError(MP_EOPNOTSUPP);
    #endif
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(usbif_host_msc_open_obj, usbif_host_msc_open);

// (num_blocks, block_size, inquiry_string)
static mp_obj_t usbif_host_msc_info(void) {
    #if USBIF_HAVE_HOST
    uint32_t blocks, bs;
    const char *inq;
    if (usbif_msc_info(&blocks, &bs, &inq) != 0) {
        mp_raise_OSError(MP_EIO);
    }
    mp_obj_t items[3] = {
        mp_obj_new_int_from_uint(blocks),
        mp_obj_new_int_from_uint(bs),
        mp_obj_new_str(inq, strlen(inq)),
    };
    return mp_obj_new_tuple(3, items);
    #else
    return mp_const_none;
    #endif
}
static MP_DEFINE_CONST_FUN_OBJ_0(usbif_host_msc_info_obj, usbif_host_msc_info);

static mp_obj_t usbif_host_msc_read(mp_obj_t lba_in, mp_obj_t buf_in) {
    #if USBIF_HAVE_HOST
    mp_buffer_info_t buf;
    mp_get_buffer_raise(buf_in, &buf, MP_BUFFER_WRITE);
    int n = usbif_msc_read_block((uint32_t)mp_obj_get_int(lba_in), (uint8_t *)buf.buf, buf.len);
    if (n < 0) {
        mp_raise_OSError(MP_EIO);
    }
    return MP_OBJ_NEW_SMALL_INT(n);
    #else
    (void)lba_in;
    (void)buf_in;
    mp_raise_OSError(MP_EOPNOTSUPP);
    #endif
}
static MP_DEFINE_CONST_FUN_OBJ_2(usbif_host_msc_read_obj, usbif_host_msc_read);

static mp_obj_t usbif_host_msc_close_py(void) {
    #if USBIF_HAVE_HOST
    usbif_msc_close();
    #endif
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(usbif_host_msc_close_obj, usbif_host_msc_close_py);

// USB MIDI device: raw MIDI 1.0 byte streams in and out. TinyUSB packs and
// unpacks the USB-MIDI event framing; callers speak plain running-status-free
// MIDI (0x90 0x3C 0x64 is a middle-C note-on).
static mp_obj_t usbif_midi_write(mp_obj_t buf_in) {
    #if defined(CFG_TUD_MIDI) && CFG_TUD_MIDI
    mp_buffer_info_t buf;
    mp_get_buffer_raise(buf_in, &buf, MP_BUFFER_READ);
    uint32_t n = tud_midi_stream_write(0, (const uint8_t *)buf.buf, buf.len);
    return mp_obj_new_int_from_uint(n);
    #else
    (void)buf_in;
    return MP_OBJ_NEW_SMALL_INT(0);
    #endif
}
static MP_DEFINE_CONST_FUN_OBJ_1(usbif_midi_write_obj, usbif_midi_write);

static mp_obj_t usbif_midi_read(mp_obj_t buf_in) {
    #if defined(CFG_TUD_MIDI) && CFG_TUD_MIDI
    mp_buffer_info_t buf;
    mp_get_buffer_raise(buf_in, &buf, MP_BUFFER_WRITE);
    uint32_t n = tud_midi_stream_read(buf.buf, buf.len);
    return mp_obj_new_int_from_uint(n);
    #else
    (void)buf_in;
    return MP_OBJ_NEW_SMALL_INT(0);
    #endif
}
static MP_DEFINE_CONST_FUN_OBJ_1(usbif_midi_read_obj, usbif_midi_read);

// Device-side USB state, straight from TinyUSB: (connected, mounted,
// suspended). connected = a host's bus reset was seen; mounted = the host
// configured us. The first question when nothing enumerates is which of
// those never happened.
static mp_obj_t usbif_dev_state(void) {
    #if MICROPY_HW_ENABLE_USBDEV
    mp_obj_t items[3] = {
        mp_obj_new_bool(tud_connected()),
        mp_obj_new_bool(tud_mounted()),
        mp_obj_new_bool(tud_suspended()),
    };
    return mp_obj_new_tuple(3, items);
    #else
    return mp_const_none;
    #endif
}
static MP_DEFINE_CONST_FUN_OBJ_0(usbif_dev_state_obj, usbif_dev_state);

// (was_inited, init_ok): whether TinyUSB was already up, and what a fresh
// tusb_init() says. Distinguishes "boot init silently failed" from
// "initialised but no bus".
static mp_obj_t usbif_dev_reinit(void) {
    #if MICROPY_HW_ENABLE_USBDEV
    bool was = tud_inited();
    bool ok = tusb_init();
    tud_connect();
    mp_obj_t items[2] = { mp_obj_new_bool(was), mp_obj_new_bool(ok) };
    return mp_obj_new_tuple(2, items);
    #else
    return mp_const_none;
    #endif
}
static MP_DEFINE_CONST_FUN_OBJ_0(usbif_dev_reinit_obj, usbif_dev_reinit);

// The USB identity, as a bitmask of functions. Called with no argument it
// reports what is advertised; with an argument it re-enumerates the board
// wearing exactly that set. Asking for a function this firmware was not
// built with is an error rather than a silent omission.
static mp_obj_t usbif_dev_functions(size_t n_args, const mp_obj_t *args) {
    #if USBIF_HAVE_FN
    if (n_args) {
        mp_int_t mask = mp_obj_get_int(args[0]);
        if (mask == 0) {
            mp_raise_ValueError(MP_ERROR_TEXT("a device must present at least one function"));
        }
        if (mask < 0 || usbif_fn_set((uint16_t)mask) != 0) {
            mp_raise_ValueError(MP_ERROR_TEXT("function not built into this firmware"));
        }
    }
    return mp_obj_new_int_from_uint(usbif_fn_get());
    #else
    (void)n_args;
    (void)args;
    return MP_OBJ_NEW_SMALL_INT(0);
    #endif
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(usbif_dev_functions_obj, 0, 1, usbif_dev_functions);

// Which functions this firmware can advertise at all.
static mp_obj_t usbif_dev_functions_built(void) {
    #if USBIF_HAVE_FN
    return mp_obj_new_int_from_uint(usbif_fn_built());
    #else
    return MP_OBJ_NEW_SMALL_INT(0);
    #endif
}
static MP_DEFINE_CONST_FUN_OBJ_0(usbif_dev_functions_built_obj, usbif_dev_functions_built);

// Submit one HID input report. Returns False when the host is not ready
// for it -- not mounted, or it has not polled since the last one -- which
// a caller should treat as "try again", not as an error.
static mp_obj_t usbif_hid_send_py(mp_obj_t id_in, mp_obj_t buf_in) {
    #if defined(CFG_TUD_HID) && CFG_TUD_HID
    mp_buffer_info_t buf;
    mp_get_buffer_raise(buf_in, &buf, MP_BUFFER_READ);
    extern bool usbif_hid_send(uint8_t report_id, const uint8_t *data, uint16_t len);
    return mp_obj_new_bool(usbif_hid_send((uint8_t)mp_obj_get_int(id_in),
        (const uint8_t *)buf.buf, (uint16_t)buf.len));
    #else
    (void)id_in;
    (void)buf_in;
    return mp_const_false;
    #endif
}
static MP_DEFINE_CONST_FUN_OBJ_2(usbif_hid_send_obj, usbif_hid_send_py);

// The keyboard LED state the host last set (caps lock and friends), for a
// control surface that wants to show it.
static mp_obj_t usbif_hid_leds_py(void) {
    #if defined(CFG_TUD_HID) && CFG_TUD_HID
    extern uint8_t usbif_hid_get_leds(void);
    return MP_OBJ_NEW_SMALL_INT(usbif_hid_get_leds());
    #else
    return MP_OBJ_NEW_SMALL_INT(0);
    #endif
}
static MP_DEFINE_CONST_FUN_OBJ_0(usbif_hid_leds_obj, usbif_hid_leds_py);

// Structural check of the descriptor the current costume produces: 0 when
// sound, or a negative code naming the fault. A host finds these by
// refusing the device; this finds them in microseconds.
static mp_obj_t usbif_dev_desc_check(void) {
    #if USBIF_HAVE_FN
    return MP_OBJ_NEW_SMALL_INT(usbif_desc_check());
    #else
    return MP_OBJ_NEW_SMALL_INT(0);
    #endif
}
static MP_DEFINE_CONST_FUN_OBJ_0(usbif_dev_desc_check_obj, usbif_dev_desc_check);

// The product id the current costume presents. Each combination of
// functions is its own product, so a host's cached driver set always
// matches what it is looking at.
static mp_obj_t usbif_dev_pid(void) {
    #if USBIF_HAVE_FN
    extern uint16_t usbif_desc_pid(void);
    return mp_obj_new_int_from_uint(usbif_desc_pid());
    #else
    return MP_OBJ_NEW_SMALL_INT(0);
    #endif
}
static MP_DEFINE_CONST_FUN_OBJ_0(usbif_dev_pid_obj, usbif_dev_pid);

// Offer a buffer to the host as a removable drive. The buffer's contents
// are whatever the application put there -- usually a FAT image the board
// built and then unmounted.
//
// The object is held in a VM root pointer while attached, so the collector
// cannot free memory the USB task is still serving to a host. Without that
// root, an application that dropped its own reference would leave the C
// side reading freed memory on every host read -- a fault that would
// present as corrupted files rather than as a crash, which is worse.
//
// The rule the firmware cannot enforce: do not mount this locally while a
// host has it. Two writers on one filesystem is a corrupted filesystem.
static mp_obj_t usbif_msc_attach_py(size_t n_args, const mp_obj_t *args) {
    #if defined(CFG_TUD_MSC) && CFG_TUD_MSC
    extern int usbif_msc_attach(uint8_t *buf, size_t len, bool writable);
    mp_buffer_info_t buf;
    mp_get_buffer_raise(args[0], &buf, MP_BUFFER_WRITE);
    const bool writable = (n_args < 2) || mp_obj_is_true(args[1]);
    if (usbif_msc_attach((uint8_t *)buf.buf, buf.len, writable) != 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("buffer smaller than one 512-byte block"));
    }
    MP_STATE_VM(usbif_msc_obj) = args[0];
    return mp_const_none;
    #else
    (void)n_args;
    (void)args;
    mp_raise_OSError(MP_EOPNOTSUPP);
    #endif
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(usbif_msc_attach_obj, 1, 2, usbif_msc_attach_py);

// Real storage instead of a RAM buffer: `obj` is any object implementing
// MicroPython's standard block-device protocol (readblocks/writeblocks,
// e.g. mip's `sdcard.SDCard`) -- num_blocks/block_size come from the
// object's own ioctl(4, 0)/ioctl(5, 0) rather than being asked for, so
// this matches how a board would already open the same object for local
// use (os.mount(obj, ...) reads its size the same way).
static mp_obj_t usbif_msc_attach_blockdev_py(size_t n_args, const mp_obj_t *args) {
    #if defined(CFG_TUD_MSC) && CFG_TUD_MSC
    extern int usbif_msc_attach_blockdev(uint32_t num_blocks, uint32_t block_size, bool writable);
    mp_obj_t dest[4];
    mp_load_method(args[0], MP_QSTR_ioctl, dest);
    dest[2] = MP_OBJ_NEW_SMALL_INT(4);   // IOCTL_BLOCK_COUNT
    dest[3] = MP_OBJ_NEW_SMALL_INT(0);
    mp_int_t num_blocks = mp_obj_get_int(mp_call_method_n_kw(2, 0, dest));
    mp_load_method(args[0], MP_QSTR_ioctl, dest);
    dest[2] = MP_OBJ_NEW_SMALL_INT(5);   // IOCTL_BLOCK_SIZE
    dest[3] = MP_OBJ_NEW_SMALL_INT(0);
    mp_int_t block_size = mp_obj_get_int(mp_call_method_n_kw(2, 0, dest));
    const bool writable = (n_args < 2) || mp_obj_is_true(args[1]);
    int err = usbif_msc_attach_blockdev((uint32_t)num_blocks, (uint32_t)block_size, writable);
    if (err == -4) {
        mp_raise_ValueError(MP_ERROR_TEXT("block device must report a 512-byte block size"));
    } else if (err != 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("MSC already attached; detach first"));
    }
    MP_STATE_VM(usbif_msc_obj) = args[0];
    return mp_const_none;
    #else
    (void)n_args;
    (void)args;
    mp_raise_OSError(MP_EOPNOTSUPP);
    #endif
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(usbif_msc_attach_blockdev_obj, 1, 2, usbif_msc_attach_blockdev_py);

static mp_obj_t usbif_msc_detach_py(void) {
    #if defined(CFG_TUD_MSC) && CFG_TUD_MSC
    extern void usbif_msc_detach(void);
    usbif_msc_detach();
    // Drop the root only after the C side has stopped pointing at it.
    MP_STATE_VM(usbif_msc_obj) = MP_OBJ_NULL;
    #endif
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(usbif_msc_detach_obj, usbif_msc_detach_py);

// (attached, blocks, ejected). `ejected` goes true when the host says it
// has finished with the medium -- the signal to wait for before mounting
// the same buffer locally again.
static mp_obj_t usbif_msc_status(void) {
    #if defined(CFG_TUD_MSC) && CFG_TUD_MSC
    extern bool usbif_msc_is_attached(void);
    extern bool usbif_msc_was_ejected(void);
    extern uint32_t usbif_msc_block_count(void);
    mp_obj_t items[3] = {
        mp_obj_new_bool(usbif_msc_is_attached()),
        mp_obj_new_int_from_uint(usbif_msc_block_count()),
        mp_obj_new_bool(usbif_msc_was_ejected()),
    };
    return mp_obj_new_tuple(3, items);
    #else
    return mp_const_none;
    #endif
}
static MP_DEFINE_CONST_FUN_OBJ_0(usbif_msc_status_obj, usbif_msc_status);

// The buffer currently attached, or None. The VM root keeps it alive
// across anything the application does with its own references -- including
// a soft reset -- so this is how a board gets its drive back after the host
// has finished writing to it, which is the whole point of the workflow:
// attach, let the host write, wait for the eject, take it back and mount it.
static mp_obj_t usbif_msc_buffer(void) {
    #if defined(CFG_TUD_MSC) && CFG_TUD_MSC
    mp_obj_t o = MP_STATE_VM(usbif_msc_obj);
    return o == MP_OBJ_NULL ? mp_const_none : o;
    #else
    return mp_const_none;
    #endif
}
static MP_DEFINE_CONST_FUN_OBJ_0(usbif_msc_buffer_obj, usbif_msc_buffer);

// (calls, errors) for the blockdev path -- whether readblocks/writeblocks
// is being reached at all, and how many of those calls raised a Python
// exception (swallowed at the call site; see usbif_msc_dev.c).
static mp_obj_t usbif_msc_bd_stats_py(void) {
    #if defined(CFG_TUD_MSC) && CFG_TUD_MSC
    extern uint32_t usbif_msc_bd_stats(uint32_t *errors);
    uint32_t errors;
    uint32_t calls = usbif_msc_bd_stats(&errors);
    mp_obj_t items[2] = { mp_obj_new_int_from_uint(calls), mp_obj_new_int_from_uint(errors) };
    return mp_obj_new_tuple(2, items);
    #else
    return mp_const_none;
    #endif
}
static MP_DEFINE_CONST_FUN_OBJ_0(usbif_msc_bd_stats_obj, usbif_msc_bd_stats_py);

static const mp_rom_map_elem_t usbif_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR_msc_attach), MP_ROM_PTR(&usbif_msc_attach_obj) },
    { MP_ROM_QSTR(MP_QSTR_msc_attach_blockdev), MP_ROM_PTR(&usbif_msc_attach_blockdev_obj) },
    { MP_ROM_QSTR(MP_QSTR_msc_bd_stats), MP_ROM_PTR(&usbif_msc_bd_stats_obj) },
    { MP_ROM_QSTR(MP_QSTR_msc_detach), MP_ROM_PTR(&usbif_msc_detach_obj) },
    { MP_ROM_QSTR(MP_QSTR_msc_status), MP_ROM_PTR(&usbif_msc_status_obj) },
    { MP_ROM_QSTR(MP_QSTR_msc_buffer), MP_ROM_PTR(&usbif_msc_buffer_obj) },
    { MP_ROM_QSTR(MP_QSTR_dev_pid), MP_ROM_PTR(&usbif_dev_pid_obj) },
    { MP_ROM_QSTR(MP_QSTR_dev_desc_check), MP_ROM_PTR(&usbif_dev_desc_check_obj) },
    { MP_ROM_QSTR(MP_QSTR_hid_send), MP_ROM_PTR(&usbif_hid_send_obj) },
    { MP_ROM_QSTR(MP_QSTR_hid_leds), MP_ROM_PTR(&usbif_hid_leds_obj) },
    { MP_ROM_QSTR(MP_QSTR_HID_KEYBOARD), MP_ROM_INT(1) },
    { MP_ROM_QSTR(MP_QSTR_HID_MOUSE), MP_ROM_INT(2) },
    { MP_ROM_QSTR(MP_QSTR_dev_functions), MP_ROM_PTR(&usbif_dev_functions_obj) },
    { MP_ROM_QSTR(MP_QSTR_dev_functions_built), MP_ROM_PTR(&usbif_dev_functions_built_obj) },
    { MP_ROM_QSTR(MP_QSTR_FN_CDC), MP_ROM_INT(1) },
    { MP_ROM_QSTR(MP_QSTR_FN_MSC), MP_ROM_INT(2) },
    { MP_ROM_QSTR(MP_QSTR_FN_AUDIO), MP_ROM_INT(4) },
    { MP_ROM_QSTR(MP_QSTR_FN_MIDI), MP_ROM_INT(8) },
    { MP_ROM_QSTR(MP_QSTR_FN_HID), MP_ROM_INT(16) },
    { MP_ROM_QSTR(MP_QSTR_uac_enable), MP_ROM_PTR(&usbif_uac_enable_obj) },
    { MP_ROM_QSTR(MP_QSTR_uac_pump_start), MP_ROM_PTR(&usbif_uac_pump_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_uac_pump_stop), MP_ROM_PTR(&usbif_uac_pump_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_uac_pump_stats), MP_ROM_PTR(&usbif_uac_pump_stats_obj) },
    { MP_ROM_QSTR(MP_QSTR_uac_available), MP_ROM_PTR(&usbif_uac_available_obj) },
    { MP_ROM_QSTR(MP_QSTR_uac_volume), MP_ROM_PTR(&usbif_uac_volume_obj) },
    { MP_ROM_QSTR(MP_QSTR_uac_read), MP_ROM_PTR(&usbif_uac_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_builtin_desc_cfg), MP_ROM_PTR(&usbif_builtin_desc_cfg_obj) },
    { MP_ROM_QSTR(MP_QSTR_uac_stats), MP_ROM_PTR(&usbif_uac_stats_obj) },
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR__usbif) },
    { MP_ROM_QSTR(MP_QSTR_capabilities), MP_ROM_PTR(&usbif_capabilities_obj) },
    { MP_ROM_QSTR(MP_QSTR_host_start), MP_ROM_PTR(&usbif_host_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_host_stop), MP_ROM_PTR(&usbif_host_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_host_devices), MP_ROM_PTR(&usbif_host_devices_obj) },
    { MP_ROM_QSTR(MP_QSTR_host_drain), MP_ROM_PTR(&usbif_host_drain_obj) },
    { MP_ROM_QSTR(MP_QSTR_host_stats), MP_ROM_PTR(&usbif_host_stats_obj) },
    { MP_ROM_QSTR(MP_QSTR_host_port_cycle), MP_ROM_PTR(&usbif_host_port_cycle_obj) },
    { MP_ROM_QSTR(MP_QSTR_host_intr_dump), MP_ROM_PTR(&usbif_host_intr_dump_obj) },
    { MP_ROM_QSTR(MP_QSTR_host_cdc_open), MP_ROM_PTR(&usbif_host_cdc_open_obj) },
    { MP_ROM_QSTR(MP_QSTR_host_cdc_write), MP_ROM_PTR(&usbif_host_cdc_write_obj) },
    { MP_ROM_QSTR(MP_QSTR_host_cdc_read), MP_ROM_PTR(&usbif_host_cdc_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_host_cdc_close), MP_ROM_PTR(&usbif_host_cdc_close_obj) },
    { MP_ROM_QSTR(MP_QSTR_host_hid_open), MP_ROM_PTR(&usbif_host_hid_open_obj) },
    { MP_ROM_QSTR(MP_QSTR_host_hid_read), MP_ROM_PTR(&usbif_host_hid_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_host_hid_close), MP_ROM_PTR(&usbif_host_hid_close_obj) },
    { MP_ROM_QSTR(MP_QSTR_host_msc_open), MP_ROM_PTR(&usbif_host_msc_open_obj) },
    { MP_ROM_QSTR(MP_QSTR_host_msc_info), MP_ROM_PTR(&usbif_host_msc_info_obj) },
    { MP_ROM_QSTR(MP_QSTR_host_msc_read), MP_ROM_PTR(&usbif_host_msc_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_host_msc_close), MP_ROM_PTR(&usbif_host_msc_close_obj) },
    { MP_ROM_QSTR(MP_QSTR_midi_write), MP_ROM_PTR(&usbif_midi_write_obj) },
    { MP_ROM_QSTR(MP_QSTR_midi_read), MP_ROM_PTR(&usbif_midi_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_dev_state), MP_ROM_PTR(&usbif_dev_state_obj) },
    { MP_ROM_QSTR(MP_QSTR_dev_reinit), MP_ROM_PTR(&usbif_dev_reinit_obj) },
};
static MP_DEFINE_CONST_DICT(usbif_module_globals, usbif_module_globals_table);

// Keeps the attached MSC buffer alive for as long as the USB task may read
// it, independently of whether the application still holds a reference.
MP_REGISTER_ROOT_POINTER(mp_obj_t usbif_msc_obj);

#if defined(CFG_TUD_MSC) && CFG_TUD_MSC
// Consulted by usbif_msc_dev.c before every host access. A soft reset
// clears the VM's root pointers, freeing the buffer while the USB task
// still holds its address; without this the host would read reused memory
// and see corrupted files rather than an error. The drive reports "no
// medium" instead, which is both true and something hosts handle.
bool usbif_msc_root_alive(void) {
    return MP_STATE_VM(usbif_msc_obj) != MP_OBJ_NULL;
}

// Consulted by usbif_msc_dev.c's blockdev path (msc_attach_blockdev) to
// call readblocks/writeblocks on whatever object is currently attached.
// Same root, same soft-reset guard as the buffer path -- usbif_msc_live()
// is checked first in every caller, same as it always was.
mp_obj_t usbif_msc_get_obj(void) {
    return MP_STATE_VM(usbif_msc_obj);
}
#endif

const mp_obj_module_t usbif_user_cmodule = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&usbif_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR__usbif, usbif_user_cmodule);
