// SPDX-License-Identifier: MIT
//
// Runtime assembly of the built-in configuration descriptor.
//
// The goal is a single firmware whose USB identity is chosen by Python, not
// by the build: every function (CDC, MSC, audio, MIDI) is compiled in, and
// the application decides which of them the host is allowed to see. A board
// that ships as a sound card, a MIDI instrument, a serial console, or any
// combination, without a rebuild.
//
// That is more than truncating a static descriptor. Dropping a function in
// the middle renumbers every interface after it, and interface numbers are
// referenced from three places: the interface descriptors themselves, the
// interface associations that group them, and a handful of class-specific
// descriptors that point at sibling interfaces (CDC's call-management and
// union blocks, the MIDI 1.0 audio-control header). All of them are
// rewritten here as blocks are emitted.
//
// Two further details are not optional, both learned from hosts refusing us:
//
//   * A composite assembled from interface associations must declare the IAD
//     device class (EF/02/01), and a single-function device must NOT -- the
//     hardware MIDI host that refused every composite we offered accepted a
//     bare MIDI device at interface 0 with device class 00. So the device
//     descriptor is served from RAM too, and the leading IAD of a lone
//     function is stripped.
//
//   * A bulk endpoint is 512 bytes at high speed and at most 64 at full
//     speed. A static descriptor can only be right for one of them, and
//     Windows enforces the rule silently -- it saw 64-byte endpoints at high
//     speed and refused the whole configuration without a word. Endpoint
//     sizes are therefore patched for the speed actually negotiated, which
//     is known by the time the host asks.

#include "py/mpconfig.h"
#include "py/mphal.h"

#if MICROPY_HW_ENABLE_USBDEV && defined(MICROPY_HW_USB_EXT_TUSB_CONFIG)

#include <string.h>

#include "tusb.h"
#include "mp_usbd.h"

// Function identifiers, as a bitmask so Python can name a costume in one
// integer. Values are API: they appear in usbif's Python surface.
#define USBIF_FN_CDC   (1u << 0)
#define USBIF_FN_MSC   (1u << 1)
#define USBIF_FN_AUDIO (1u << 2)
#define USBIF_FN_MIDI  (1u << 3)
#define USBIF_FN_HID   (1u << 4)

// Which functions this firmware was built with. A function that is not
// compiled in can never be advertised, and asking for it is an error rather
// than a silent no-op.
#define USBIF_FN_BUILT ( \
    (CFG_TUD_CDC ? USBIF_FN_CDC : 0) | \
    (CFG_TUD_MSC ? USBIF_FN_MSC : 0) | \
    (USBIF_EXT_AUDIO ? USBIF_FN_AUDIO : 0) | \
    (CFG_TUD_MIDI ? USBIF_FN_MIDI : 0) | \
    (CFG_TUD_HID ? USBIF_FN_HID : 0))

// Where each function's block sits inside the compile-time descriptor, in
// the order mp_usbd_descriptor.c emits them (CDC, MSC) followed by the
// extension block (audio, MIDI). Derived from the same length macros that
// built it, so the two cannot drift.
#define USBIF_OFF_CDC   (TUD_CONFIG_DESC_LEN)
#define USBIF_LEN_CDC   (CFG_TUD_CDC ? TUD_CDC_DESC_LEN : 0)
#define USBIF_OFF_MSC   (USBIF_OFF_CDC + USBIF_LEN_CDC)
#define USBIF_LEN_MSC   (CFG_TUD_MSC ? TUD_MSC_DESC_LEN : 0)
#define USBIF_OFF_AUDIO (USBIF_OFF_MSC + USBIF_LEN_MSC)
#if USBIF_EXT_AUDIO
#define USBIF_LEN_AUDIO (USBIF_AUDIO_SPEAKER_STEREO_FB_DESC_LEN)
#else
#define USBIF_LEN_AUDIO (0)
#endif
#define USBIF_OFF_MIDI  (USBIF_OFF_AUDIO + USBIF_LEN_AUDIO)
#define USBIF_LEN_MIDI  (USBIF_MIDI_IAD_LEN + TUD_MIDI_DESC_LEN)
#define USBIF_OFF_HID   (USBIF_OFF_MIDI + USBIF_LEN_MIDI)
#define USBIF_LEN_HID   (CFG_TUD_HID ? TUD_HID_DESC_LEN : 0)

typedef struct {
    uint16_t bit;
    uint16_t offset;
    uint16_t len;
    uint8_t itf_count;
    // Whether this function's leading interface association may be dropped
    // when it is the only function on the device. MIDI 1.0 is
    // class-compliant as a bare AudioControl+MIDIStreaming pair and the
    // naive hosts that matter expect exactly that -- but UAC2 *requires*
    // its association to group control and streaming, and CDC's host
    // driver is happiest with one too. Getting this wrong is not
    // theoretical: an audio-only costume with its IAD stripped enumerated
    // and then bound no driver at all.
    bool iad_strippable;
    // Whether this function's class *requires* an interface association to
    // be legal. UAC2 and CDC do -- their control and data interfaces are
    // only grouped by it. MIDI 1.0 and single-interface functions do not.
    // A block that requires one and lost it is malformed in a way no
    // internal-consistency check can see, because the device class is
    // derived to match and the result looks self-consistent.
    bool iad_required;
} usbif_fn_block_t;

// Order matters: it is the order the blocks are emitted, and therefore the
// order interfaces are numbered.
static const usbif_fn_block_t usbif_blocks[] = {
    { USBIF_FN_CDC,   USBIF_OFF_CDC,   USBIF_LEN_CDC,   2, false, true  },
    { USBIF_FN_MSC,   USBIF_OFF_MSC,   USBIF_LEN_MSC,   1, false, false },
    { USBIF_FN_AUDIO, USBIF_OFF_AUDIO, USBIF_LEN_AUDIO, 2, false, true  },
    { USBIF_FN_MIDI,  USBIF_OFF_MIDI,  USBIF_LEN_MIDI,  2, true,  false },
    // HID is a single interface and carries no association of its own, so
    // there is nothing to strip and nothing that requires one.
    { USBIF_FN_HID,   USBIF_OFF_HID,   USBIF_LEN_HID,   1, false, false },
};

// The advertised set. Empty at boot by design: a board that always
// enumerates as a sound card or an instrument is not merely untidy -- a host
// will make it the default device and stall its audio engine when nothing
// aboard is draining the stream (observed on Windows, desktop and keyboard
// backed up). MICROPY_HW_USB_EXT_BOOT_FUNCTIONS lets a board opt into a
// different boot identity; the default keeps the built-in classes only.
#ifndef MICROPY_HW_USB_EXT_BOOT_FUNCTIONS
#define MICROPY_HW_USB_EXT_BOOT_FUNCTIONS \
    ((CFG_TUD_CDC ? USBIF_FN_CDC : 0) | (CFG_TUD_MSC ? USBIF_FN_MSC : 0))
#endif

static uint16_t usbif_fn_enabled = MICROPY_HW_USB_EXT_BOOT_FUNCTIONS;
// Where each emitted block landed, recorded by the assembler so the
// validator can check class policy against the bytes rather than
// re-deriving which function produced which interface.
static uint16_t usbif_emit_off[5];
static const usbif_fn_block_t *usbif_emit_blk[5];
static uint8_t usbif_emit_n;
static uint8_t usbif_desc_buf[MP_USBD_BUILTIN_DESC_CFG_LEN];
static uint8_t usbif_itf_count;
static tusb_desc_device_t usbif_desc_dev;

// Audio resets its streaming state when the costume changes; weak so builds
// without the audio function link without it.
void usbif_uac_on_ext_toggled(void) __attribute__((weak));

// Rewrite every interface number in one emitted block, and patch endpoint
// sizes for the negotiated speed. `delta` is how far this block's interfaces
// moved from where the compile-time descriptor put them.
static void usbif_fixup_block(uint8_t *p, uint16_t len, int itf_delta, uint16_t midi_mps) {
    uint16_t off = 0;
    uint8_t cur_class = 0;
    while (off + 2u <= len && p[off] >= 2 && off + p[off] <= len) {
        uint8_t *d = p + off;
        const uint8_t dlen = d[0];
        switch (d[1]) {
            case TUSB_DESC_INTERFACE_ASSOCIATION:
                if (dlen >= 8) {
                    d[2] = (uint8_t)(d[2] + itf_delta);   // bFirstInterface
                }
                break;
            case TUSB_DESC_INTERFACE:
                if (dlen >= 9) {
                    d[2] = (uint8_t)(d[2] + itf_delta);   // bInterfaceNumber
                    cur_class = d[5];
                }
                break;
            case TUSB_DESC_CS_INTERFACE:
                // Class-specific descriptors that point at sibling
                // interfaces. Which ones exist depends on the class of the
                // interface they belong to, so the walk tracks it.
                if (cur_class == TUSB_CLASS_CDC && dlen >= 5) {
                    if (d[2] == CDC_FUNC_DESC_CALL_MANAGEMENT) {
                        d[4] = (uint8_t)(d[4] + itf_delta);   // bDataInterface
                    } else if (d[2] == CDC_FUNC_DESC_UNION) {
                        d[3] = (uint8_t)(d[3] + itf_delta);   // bControlInterface
                        d[4] = (uint8_t)(d[4] + itf_delta);   // bSubordinateInterface0
                    }
                } else if (cur_class == TUSB_CLASS_AUDIO && dlen >= 9 && d[2] == 0x01) {
                    // Audio-control header. MIDI 1.0 (bcdADC 0x0100) carries
                    // bInCollection plus the streaming interface numbers;
                    // UAC2 (0x0200) carries none, so the version selects.
                    if (d[3] == 0x00 && d[4] == 0x01) {
                        const uint8_t n = d[7];
                        for (uint8_t i = 0; i < n && 8u + i < dlen; i++) {
                            d[8 + i] = (uint8_t)(d[8 + i] + itf_delta);
                        }
                    }
                }
                break;
            case TUSB_DESC_ENDPOINT:
                if (dlen >= 7 && midi_mps) {
                    const uint8_t addr = d[2];
                    #ifdef USBD_EP_MIDI_OUT
                    if (addr == USBD_EP_MIDI_OUT || addr == USBD_EP_MIDI_IN) {
                        d[4] = (uint8_t)(midi_mps & 0xFF);
                        d[5] = (uint8_t)(midi_mps >> 8);
                    }
                    #else
                    (void)addr;
                    #endif
                }
                break;
            default:
                break;
        }
        off = (uint16_t)(off + dlen);
    }
}

// Assemble the configuration descriptor for the currently enabled set.
static void usbif_build_desc(void) {
    const uint16_t midi_mps = (tud_speed_get() == TUSB_SPEED_HIGH) ? 512 : 64;
    uint16_t out = TUD_CONFIG_DESC_LEN;
    uint8_t itf = 0;
    uint8_t fn_count = 0;
    const usbif_fn_block_t *only = NULL;
    usbif_emit_n = 0;

    memcpy(usbif_desc_buf, mp_usbd_builtin_desc_cfg, TUD_CONFIG_DESC_LEN);

    for (size_t i = 0; i < MP_ARRAY_SIZE(usbif_blocks); i++) {
        const usbif_fn_block_t *b = &usbif_blocks[i];
        if (b->len == 0 || !(usbif_fn_enabled & b->bit)) {
            continue;
        }
        fn_count++;
        only = b;
        usbif_emit_off[usbif_emit_n] = out;
        usbif_emit_blk[usbif_emit_n] = b;
        usbif_emit_n++;
        memcpy(usbif_desc_buf + out, mp_usbd_builtin_desc_cfg + b->offset, b->len);
        // The compile-time descriptor numbered this block's interfaces from
        // its own position; move them to where it actually landed. The first
        // interface descriptor in a block always carries that base, and an
        // IAD when present repeats it.
        const uint8_t *src = mp_usbd_builtin_desc_cfg + b->offset;
        uint8_t base = 0;
        for (uint16_t o = 0; o + 2u <= b->len; o += src[o]) {
            if (src[o] < 2) {
                break;
            }
            if (src[o + 1] == TUSB_DESC_INTERFACE && src[o] >= 9) {
                base = src[o + 2];
                break;
            }
        }
        usbif_fixup_block(usbif_desc_buf + out, b->len, (int)itf - (int)base, midi_mps);
        out = (uint16_t)(out + b->len);
        itf = (uint8_t)(itf + b->itf_count);
    }

    // A lone function whose class is happy without one sheds its interface
    // association, so the device can present as a plain single-function
    // device rather than a composite of one.
    if (fn_count == 1 && only != NULL && only->iad_strippable
        && out > TUD_CONFIG_DESC_LEN + 8
        && usbif_desc_buf[TUD_CONFIG_DESC_LEN + 1] == TUSB_DESC_INTERFACE_ASSOCIATION) {
        const uint16_t iad = usbif_desc_buf[TUD_CONFIG_DESC_LEN];
        memmove(usbif_desc_buf + TUD_CONFIG_DESC_LEN,
            usbif_desc_buf + TUD_CONFIG_DESC_LEN + iad,
            (size_t)(out - TUD_CONFIG_DESC_LEN - iad));
        out = (uint16_t)(out - iad);
    }

    // wTotalLength and bNumInterfaces, at their fixed offsets (USB 2.0
    // table 9-10).
    usbif_desc_buf[2] = (uint8_t)(out & 0xFF);
    usbif_desc_buf[3] = (uint8_t)(out >> 8);
    usbif_desc_buf[4] = itf;
    usbif_itf_count = itf;

    // The device descriptor must agree with what was actually emitted: a
    // descriptor containing interface associations has to declare the IAD
    // device class, and one containing none must not. Derived by looking,
    // not by counting functions -- MSC alone carries no association, and a
    // lone MIDI function has just shed its own.
    bool has_iad = false;
    for (uint16_t o = TUD_CONFIG_DESC_LEN; o + 2u <= out && usbif_desc_buf[o] >= 2;
         o = (uint16_t)(o + usbif_desc_buf[o])) {
        if (usbif_desc_buf[o + 1] == TUSB_DESC_INTERFACE_ASSOCIATION) {
            has_iad = true;
            break;
        }
    }
    memcpy(&usbif_desc_dev, &mp_usbd_builtin_desc_dev, sizeof(usbif_desc_dev));
    if (!has_iad) {
        usbif_desc_dev.bDeviceClass = 0x00;
        usbif_desc_dev.bDeviceSubClass = 0x00;
        usbif_desc_dev.bDeviceProtocol = 0x00;
    }
}

const uint8_t *mp_usbd_builtin_desc_cfg_get(void) {
    usbif_build_desc();
    return usbif_desc_buf;
}

const void *mp_usbd_builtin_desc_dev_get(void) {
    // Built alongside the configuration; the host asks for this first, so
    // build on demand rather than relying on call order.
    usbif_build_desc();
    return &usbif_desc_dev;
}

// Must track the descriptor: runtime_dev_open() uses this to decide which
// interfaces belong to built-in drivers, and a bound that disagrees with the
// descriptor leaves an interface claimed by nobody.
uint8_t mp_usbd_builtin_itf_max(void) {
    if (usbif_itf_count == 0) {
        usbif_build_desc();
    }
    return usbif_itf_count;
}

// Structural validation of whatever the assembler just produced. A host
// discovers a malformed descriptor by refusing the device, days later and
// with no explanation; this finds the same faults in microseconds and says
// which one. Returns 0 when sound, or a negative code naming the fault --
// the codes are API, since a test script reports them.
//
// Written because the assembler's arithmetic is the kind that looks right:
// the bug that shipped an audio costume binding no driver was a policy
// mistake, but the next one will be an off-by-one in a renumbering walk.
int usbif_desc_check(void) {
    usbif_build_desc();
    const uint16_t total = (uint16_t)(usbif_desc_buf[2] | (usbif_desc_buf[3] << 8));
    if (total < TUD_CONFIG_DESC_LEN || total > MP_USBD_BUILTIN_DESC_CFG_LEN) {
        return -1;      // wTotalLength outside the buffer it describes
    }

    uint8_t seen_itf[32];
    memset(seen_itf, 0, sizeof(seen_itf));
    uint8_t highest_itf = 0;
    bool any_itf = false;
    uint16_t off = TUD_CONFIG_DESC_LEN;
    uint8_t cur_class = 0;

    while (off < total) {
        const uint8_t *d = usbif_desc_buf + off;
        const uint8_t dlen = d[0];
        if (dlen < 2) {
            return -2;                  // zero-length descriptor: walk would hang
        }
        if (off + dlen > total) {
            return -3;                  // descriptor runs past wTotalLength
        }
        switch (d[1]) {
            case TUSB_DESC_INTERFACE:
                if (dlen < 9) {
                    return -4;
                }
                if (d[2] >= sizeof(seen_itf)) {
                    return -5;          // interface number out of any sane range
                }
                seen_itf[d[2]] = 1;
                if (!any_itf || d[2] > highest_itf) {
                    highest_itf = d[2];
                }
                any_itf = true;
                cur_class = d[5];
                break;
            case TUSB_DESC_INTERFACE_ASSOCIATION:
                if (dlen < 8) {
                    return -6;
                }
                if ((uint16_t)d[2] + d[3] > sizeof(seen_itf)) {
                    return -7;          // association spans past the range
                }
                break;
            case TUSB_DESC_CS_INTERFACE:
                // The sibling references the assembler rewrites must land on
                // interfaces that exist.
                if (cur_class == TUSB_CLASS_CDC && dlen >= 5) {
                    if (d[2] == CDC_FUNC_DESC_CALL_MANAGEMENT && d[4] >= sizeof(seen_itf)) {
                        return -8;
                    }
                    if (d[2] == CDC_FUNC_DESC_UNION
                        && (d[3] >= sizeof(seen_itf) || d[4] >= sizeof(seen_itf))) {
                        return -9;
                    }
                } else if (cur_class == TUSB_CLASS_AUDIO && dlen >= 9 && d[2] == 0x01
                           && d[3] == 0x00 && d[4] == 0x01) {
                    const uint8_t n = d[7];
                    for (uint8_t i = 0; i < n && 8u + i < dlen; i++) {
                        if (d[8 + i] >= sizeof(seen_itf)) {
                            return -10;
                        }
                    }
                }
                break;
            default:
                break;
        }
        off = (uint16_t)(off + dlen);
    }

    if (off != total) {
        return -11;                     // descriptors do not tile wTotalLength
    }
    if (!any_itf) {
        return -12;                     // a configuration with no interfaces
    }
    // Interface numbers must be dense from zero: a gap means a block was
    // renumbered against the wrong base, which hosts reject in ways that
    // look like anything but arithmetic.
    for (uint8_t i = 0; i <= highest_itf; i++) {
        if (!seen_itf[i]) {
            return -13;
        }
    }
    if (usbif_desc_buf[4] != (uint8_t)(highest_itf + 1)) {
        return -14;                     // bNumInterfaces disagrees with content
    }
    // The class-specific references above are checked for range; the device
    // descriptor's composite claim must match what was emitted.
    bool has_iad = false;
    for (uint16_t o = TUD_CONFIG_DESC_LEN; o < total; o = (uint16_t)(o + usbif_desc_buf[o])) {
        if (usbif_desc_buf[o + 1] == TUSB_DESC_INTERFACE_ASSOCIATION) {
            has_iad = true;
            break;
        }
    }
    if (has_iad != (usbif_desc_dev.bDeviceClass == TUSB_CLASS_MISC)) {
        return -15;                     // device class disagrees with the IADs
    }
    // Class policy, which internal consistency cannot see: a function whose
    // class requires an interface association must still have one. When such
    // a block loses its association the device class is derived to match, so
    // the result is self-consistent and wrong -- which is exactly how the
    // audio-only costume once enumerated and bound no driver, and exactly
    // what the first version of this validator failed to catch.
    for (uint8_t i = 0; i < usbif_emit_n; i++) {
        if (!usbif_emit_blk[i]->iad_required) {
            continue;
        }
        const uint16_t o = usbif_emit_off[i];
        if (o + 2u > total || usbif_desc_buf[o + 1] != TUSB_DESC_INTERFACE_ASSOCIATION) {
            return -16;                 // required association missing
        }
    }
    return 0;
}

uint16_t usbif_fn_get(void) {
    return usbif_fn_enabled;
}

uint16_t usbif_fn_built(void) {
    return USBIF_FN_BUILT;
}

// Re-enumerate so the host re-reads the descriptors. USB has no way to
// change identity in place; a detach and re-attach is the mechanism.
int usbif_fn_set(uint16_t mask) {
    if (mask & ~(uint16_t)USBIF_FN_BUILT) {
        return -1;      // asked for a function this firmware does not have
    }
    if (mask == 0) {
        // A configuration descriptor with no interfaces is malformed, and
        // hosts say so: Windows enumerates it and marks the device in
        // error. Presenting nothing is a legitimate wish, but detaching is
        // how USB expresses it -- not an empty costume.
        return -2;
    }
    if (mask == usbif_fn_enabled) {
        return 0;
    }
    usbif_fn_enabled = mask;
    usbif_itf_count = 0;
    if (usbif_uac_on_ext_toggled) {
        usbif_uac_on_ext_toggled();
    }
    // A build whose boot set has no CDC/MSC never initialises TinyUSB
    // (mp_usbd_init's need_usb is false), so the port stays electrically
    // dark until someone turns the key. Learned knocking on a synth box
    // that could not hear us.
    if (!tud_inited()) {
        tusb_init();
        mp_hal_delay_ms(50);
    }
    tud_disconnect();
    mp_hal_delay_ms(120);
    tud_connect();
    return 0;
}

#endif // MICROPY_HW_ENABLE_USBDEV && MICROPY_HW_USB_EXT_TUSB_CONFIG
