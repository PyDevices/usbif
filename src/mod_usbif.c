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

#include "shared/usbif_ringbuf.h"

#if MICROPY_HW_ENABLE_USBDEV
#include "tusb.h"
#include "mp_usbd.h"
#endif

// Class bitmask, mirroring the names in the portable Python API. A bitmask
// rather than strings so an event record stays fixed-size and ISR-safe; the
// translation to names happens here, on the consumer side, where allocation
// is allowed.
#define USBIF_CLASS_HID  (1u << 0)
#define USBIF_CLASS_MSC  (1u << 1)
#define USBIF_CLASS_CDC  (1u << 2)
#define USBIF_CLASS_MIDI (1u << 3)
#define USBIF_CLASS_UAC  (1u << 4)
#define USBIF_CLASS_UVC  (1u << 5)

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

// Classes this firmware was actually built with. Zero until Phase 2 registers
// class drivers -- an honest empty set, which the portable API is designed to
// report rather than to hide behind an ImportError.
static uint16_t usbif_supported_classes(void) {
    return 0;
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
    (void)classes_in;  // honoured once class drivers exist (Phase 2)
    usbif_rb_init(&usbif_events, usbif_event_slots, USBIF_EVENT_CAPACITY);
    usbif_host_running = true;
    return usbif_classes_to_set(usbif_supported_classes());
}
static MP_DEFINE_CONST_FUN_OBJ_1(usbif_host_start_obj, usbif_host_start);

static mp_obj_t usbif_host_stop(void) {
    usbif_host_running = false;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(usbif_host_stop_obj, usbif_host_stop);

static mp_obj_t usbif_host_devices(void) {
    return mp_obj_new_tuple(0, NULL);
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
    mp_obj_t items[2] = {
        mp_obj_new_bytes(mp_usbd_builtin_desc_cfg, MP_USBD_BUILTIN_DESC_CFG_LEN),
        MP_OBJ_NEW_SMALL_INT(MP_USBD_BUILTIN_DESC_CFG_LEN),
    };
    return mp_obj_new_tuple(2, items);
}
static MP_DEFINE_CONST_FUN_OBJ_0(usbif_builtin_desc_cfg_obj, usbif_builtin_desc_cfg);

#if CFG_TUD_AUDIO
extern uint32_t usbif_uac_get_reqs, usbif_uac_set_reqs, usbif_uac_itf_sets, usbif_uac_unhandled;
extern uint32_t usbif_uac_overflows;
extern bool usbif_uac_is_streaming(void);
extern uint32_t usbif_uac_current_rate(void);
extern bool usbif_uac_is_muted(void);
extern int usbif_uac_volume_db256(void);
extern bool usbif_uac_is_enabled(void);
extern void usbif_uac_set_enabled(bool enable);
extern void usbif_uac_note_read(void);
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
    #if defined(CFG_TUD_AUDIO) && CFG_TUD_AUDIO
    if (n_args) {
        usbif_uac_set_enabled(mp_obj_is_true(args[0]));
    }
    return mp_obj_new_bool(usbif_uac_is_enabled());
    #else
    (void)n_args;
    (void)args;
    return mp_const_false;
    #endif
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(usbif_uac_enable_obj, 0, 1, usbif_uac_enable);

static const mp_rom_map_elem_t usbif_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR_uac_enable), MP_ROM_PTR(&usbif_uac_enable_obj) },
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
};
static MP_DEFINE_CONST_DICT(usbif_module_globals, usbif_module_globals_table);

const mp_obj_module_t usbif_user_cmodule = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&usbif_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR__usbif, usbif_user_cmodule);
