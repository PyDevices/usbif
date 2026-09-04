// SPDX-License-Identifier: MIT
//
// usbif's contribution to MicroPython's TinyUSB configuration.
//
// Named by MICROPY_HW_USB_EXT_TUSB_CONFIG and included from the tail of
// MicroPython's shared/tinyusb/tusb_config.h (patch 0001). Everything
// volatile lives here rather than in MicroPython, so the downstream diff
// stays at two small hooks across three files.
//
// Why the audio interface must be *built-in* rather than declared at runtime
// from Python: TinyUSB offers each interface to application drivers before
// built-in ones, and MicroPython's runtime driver claims everything numbered
// at or above USBD_ITF_BUILTIN_MAX whatever its class. An audio interface
// declared through machine.USBDevice would therefore be claimed by Python,
// which cannot service isochronous endpoints. Numbering it below the built-in
// maximum is what routes it to TinyUSB's audiod_open() instead.
#ifndef USBIF_TUSB_EXT_H
#define USBIF_TUSB_EXT_H

// The audio function can be excluded to present a slim CDC+MIDI composite:
// purpose-built embedded MIDI hosts (the kind in a hardware synth box) often
// carry enumeration buffers smaller than our full three-function descriptor,
// and have no use for an isochronous audio function anyway. Default off for
// the slim-descriptor experiment; boards that want the sound card set it.
#ifndef USBIF_EXT_AUDIO
#define USBIF_EXT_AUDIO (1)
#endif
#define CFG_TUD_AUDIO (USBIF_EXT_AUDIO)
// And a USB MIDI function beside it: TinyUSB carries the whole class
// (descriptor macro, jack plumbing, tud_midi_* API); this file only has to
// give it numbers and space. Both functions appear and disappear together
// through the same runtime opt-in the audio function uses.
#define CFG_TUD_MIDI (1)
// HID as a device: a keyboard, a mouse, or whatever report descriptor an
// application wants. One interface, one interrupt IN endpoint.
#define CFG_TUD_HID (1)
#define CFG_TUD_HID_EP_BUFSIZE (16)
#define CFG_TUD_MIDI_RX_BUFSIZE (TUD_OPT_HIGH_SPEED ? 512 : 64)
#define CFG_TUD_MIDI_TX_BUFSIZE (TUD_OPT_HIGH_SPEED ? 512 : 64)
// Video as a device: the board *is* the webcam. One VideoControl interface
// and one VideoStreaming interface, with an isochronous IN endpoint on the
// streaming interface's alternate setting 1 -- the same shape usbif.uvc
// reads when hosting somebody else's camera, pointed the other way.
//
// The same argument as the audio function applies with more force: nothing
// declared through machine.USBDevice can service an isochronous endpoint, so
// the video interfaces are numbered below USBD_ITF_BUILTIN_MAX to route them
// to TinyUSB's videod_open() rather than to MicroPython's runtime driver.
#ifndef USBIF_EXT_VIDEO
#define USBIF_EXT_VIDEO (1)
#endif
#define CFG_TUD_VIDEO (USBIF_EXT_VIDEO)
#define CFG_TUD_VIDEO_STREAMING (USBIF_EXT_VIDEO)
// Isochronous, not bulk. Bulk streaming is simpler and is what several
// examples default to, but it competes for bandwidth rather than reserving
// it, and a webcam that drops frames under load is a worse demonstration
// than one that reserves what it needs.
#define CFG_TUD_VIDEO_STREAMING_BULK (0)
// One packet per (micro)frame. 512 is inside the full-speed isochronous
// limit of 1023 and a natural high-speed size, so the same descriptor works
// on the S3 and the P4.
#define CFG_TUD_VIDEO_STREAMING_EP_BUFSIZE (512)

// --- Interface, endpoint and string numbering.
//
// Deliberately derived from CFG_TUD_CDC/CFG_TUD_MSC rather than from
// USBD_ITF_BUILTIN_MAX: a macro expands where it is used, not where it is
// defined, so a definition in terms of a maximum that this file then raises
// would silently evaluate against the new value.
#if CFG_TUD_MSC
#define USBD_ITF_AUDIO (USBD_ITF_MSC + 1)
#define USBIF_EPNUM_AUDIO (EPNUM_MSC_OUT + 1)
#elif CFG_TUD_CDC
#define USBD_ITF_AUDIO (USBD_ITF_CDC + 2)
#define USBIF_EPNUM_AUDIO (3)
#else
#define USBD_ITF_AUDIO (0)
#define USBIF_EPNUM_AUDIO (1)
#endif

#define USBD_EP_AUDIO_OUT (USBIF_EPNUM_AUDIO)
#define USBD_EP_AUDIO_FB (0x80 | USBIF_EPNUM_AUDIO)

// MIDI sits after the audio function: its descriptor also spans two
// interfaces (an AudioControl stub and MIDIStreaming), with one bulk
// endpoint pair.
#if USBIF_EXT_AUDIO
#define USBD_ITF_MIDI (USBD_ITF_AUDIO + 2)
#define USBIF_EPNUM_MIDI (USBIF_EPNUM_AUDIO + 1)
#else
#define USBD_ITF_MIDI (USBD_ITF_AUDIO)
#define USBIF_EPNUM_MIDI (USBIF_EPNUM_AUDIO)
#endif
#define USBD_EP_MIDI_OUT (USBIF_EPNUM_MIDI)
#define USBD_EP_MIDI_IN (0x80 | USBIF_EPNUM_MIDI)

// HID follows MIDI: one interface, one interrupt IN endpoint.
#define USBD_ITF_HID (USBD_ITF_MIDI + 2)
#define USBIF_EPNUM_HID (USBIF_EPNUM_MIDI + 1)
#define USBD_EP_HID_IN (0x80 | USBIF_EPNUM_HID)

// Video last: two interfaces (VideoControl, VideoStreaming) and one
// isochronous IN endpoint.
#define USBD_ITF_VIDEO (USBD_ITF_HID + 1)
#define USBIF_EPNUM_VIDEO (USBIF_EPNUM_HID + 1)
#define USBD_EP_VIDEO_IN (0x80 | USBIF_EPNUM_VIDEO)

// The audio function occupies two interfaces (AudioControl and
// AudioStreaming) and MIDI two more, which the bounds below must cover so
// runtime_dev_open keeps declining everything the built-in descriptor owns.
#undef USBD_ITF_BUILTIN_MAX
#define USBD_ITF_BUILTIN_MAX (USBD_ITF_VIDEO + 2)
#undef USBD_EP_BUILTIN_MAX
#define USBD_EP_BUILTIN_MAX (USBIF_EPNUM_VIDEO + 1)

// --- Audio function sizing, following TinyUSB's own uac2_speaker_fb example.
//
// Mono 48 kHz 16-bit to begin with: it exercises the whole isochronous path,
// including the feedback endpoint that keeps the host's clock and the board's
// in step, without the descriptor bulk of a multi-channel function. Channel
// count is a descriptor change once the path is proven.
#define CFG_TUD_AUDIO_FUNC_1_DESC_LEN USBIF_AUDIO_SPEAKER_STEREO_FB_DESC_LEN
#define CFG_TUD_AUDIO_ENABLE_FEEDBACK_FORMAT_CORRECTION (0)
// Matched to the board's existing I2S/codec path rather than the more usual
// 48 kHz: the ES8311 on this board is already brought up mono 16-bit at
// 24 kHz by board_peripherals, and a rate the two sides agree on removes
// resampling from the first working version entirely. Hosts resample their
// own output happily; a mismatch here would show up as wrong pitch.
// 48 kHz stereo: the format a sound card is expected to present.
//
// The first working version matched the board's own 24 kHz mono I2S path,
// which removed resampling but made the endpoint an unusual target. Windows
// Media Player could not build a conversion chain to it and refused files it
// plays happily on an ordinary device -- the file was fine, the sink was odd.
// The host now sees a conventional endpoint and the conversion to the board's
// hardware happens in usbif's pump, where it belongs.
#define CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE (48000)
#define CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX (2)
#define CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_RX (2)
#define CFG_TUD_AUDIO_FUNC_1_RESOLUTION_RX (16)
#define CFG_TUD_AUDIO_ENABLE_EP_OUT (1)
#define CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ_MAX             \
    TUD_AUDIO_EP_SIZE(CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE, \
    CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_RX,        \
    CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX)
// Software FIFO ahead of the endpoint.
//
// Sized as TinyUSB's own examples size it, and deliberately small. An earlier
// version made this 170 ms because the pump was Python and needed the slack;
// that is actively harmful once the pump is in C, because
// AUDIO_FEEDBACK_METHOD_FIFO_COUNT works by "regulating the FIFO level to half
// fill". A 170 ms FIFO therefore instructs the host to slow down until ~85 ms
// of audio is sitting in it -- measured as the host sending 3.3% slow, heard
// as playback that drags and skips, and felt on the host as an audio engine
// backing up. The buffer was not protecting the stream; it was detuning the
// regulator that governs it.
//
// TinyUSB's guidance is a minimum of four frames to absorb jitter; the
// example's multiplier keeps a little more than that.
#define CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ          \
    ((TUD_OPT_HIGH_SPEED ? 32 : 4) * CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ_MAX)
#define CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP (1)
#define CFG_TUD_AUDIO_FUNC_1_N_AS_INT (1)
#define CFG_TUD_AUDIO_FUNC_1_CTRL_BUF_SZ (64)

// --- A stereo speaker descriptor.
//
// TinyUSB's library ships only a mono speaker macro; the stereo one lives in
// its uac2_speaker_fb example, so it is copied here verbatim (renamed) rather
// than reinvented. Note that the example already sets the feedback endpoint's
// bInterval to 4 at high speed -- independent confirmation of the fix that had
// to be made by hand for the mono macro, whose hardcoded 1 is 125 us on a
// high-speed device and out of spec.
#define USBIF_AUDIO_SPEAKER_STEREO_FB_DESC_LEN (TUD_AUDIO_DESC_IAD_LEN\
  + TUD_AUDIO_DESC_STD_AC_LEN\
  + TUD_AUDIO_DESC_CS_AC_LEN\
  + TUD_AUDIO_DESC_CLK_SRC_LEN\
  + TUD_AUDIO_DESC_INPUT_TERM_LEN\
  + TUD_AUDIO_DESC_OUTPUT_TERM_LEN\
  + TUD_AUDIO_DESC_FEATURE_UNIT_TWO_CHANNEL_LEN\
  + TUD_AUDIO_DESC_STD_AS_INT_LEN\
  + TUD_AUDIO_DESC_STD_AS_INT_LEN\
  + TUD_AUDIO_DESC_CS_AS_INT_LEN\
  + TUD_AUDIO_DESC_TYPE_I_FORMAT_LEN\
  + TUD_AUDIO_DESC_STD_AS_ISO_EP_LEN\
  + TUD_AUDIO_DESC_CS_AS_ISO_EP_LEN\
  + TUD_AUDIO_DESC_STD_AS_ISO_FB_EP_LEN)

#define USBIF_AUDIO_SPEAKER_STEREO_FB_DESCRIPTOR(_itfnum, _stridx, _nBytesPerSample, _nBitsUsedPerSample, _epout, _epoutsize, _epfb, _epfbsize) \
  /* Standard Interface Association Descriptor (IAD) */\
  TUD_AUDIO_DESC_IAD(/*_firstitf*/ _itfnum, /*_nitfs*/ 0x02, /*_stridx*/ 0x00),\
  /* Standard AC Interface Descriptor(4.7.1) */\
  TUD_AUDIO_DESC_STD_AC(/*_itfnum*/ _itfnum, /*_nEPs*/ 0x00, /*_stridx*/ _stridx),\
  /* Class-Specific AC Interface Header Descriptor(4.7.2) */\
  TUD_AUDIO_DESC_CS_AC(/*_bcdADC*/ 0x0200, /*_category*/ AUDIO_FUNC_DESKTOP_SPEAKER, /*_totallen*/ TUD_AUDIO_DESC_CLK_SRC_LEN+TUD_AUDIO_DESC_INPUT_TERM_LEN+TUD_AUDIO_DESC_OUTPUT_TERM_LEN+TUD_AUDIO_DESC_FEATURE_UNIT_TWO_CHANNEL_LEN, /*_ctrl*/ AUDIO_CS_AS_INTERFACE_CTRL_LATENCY_POS),\
  /* Clock Source Descriptor(4.7.2.1) */\
  TUD_AUDIO_DESC_CLK_SRC(/*_clkid*/ 0x04, /*_attr*/ AUDIO_CLOCK_SOURCE_ATT_INT_PRO_CLK, /*_ctrl*/ (AUDIO_CTRL_RW << AUDIO_CLOCK_SOURCE_CTRL_CLK_FRQ_POS), /*_assocTerm*/ 0x01,  /*_stridx*/ 0x00),\
  /* Input Terminal Descriptor(4.7.2.4) */\
  TUD_AUDIO_DESC_INPUT_TERM(/*_termid*/ 0x01, /*_termtype*/ AUDIO_TERM_TYPE_USB_STREAMING, /*_assocTerm*/ 0x00, /*_clkid*/ 0x04, /*_nchannelslogical*/ 0x02, /*_channelcfg*/ AUDIO_CHANNEL_CONFIG_NON_PREDEFINED, /*_idxchannelnames*/ 0x00, /*_ctrl*/ 0 * (AUDIO_CTRL_R << AUDIO_IN_TERM_CTRL_CONNECTOR_POS), /*_stridx*/ 0x00),\
  /* Output Terminal Descriptor(4.7.2.5) */\
  TUD_AUDIO_DESC_OUTPUT_TERM(/*_termid*/ 0x03, /*_termtype*/ AUDIO_TERM_TYPE_OUT_DESKTOP_SPEAKER, /*_assocTerm*/ 0x01, /*_srcid*/ 0x02, /*_clkid*/ 0x04, /*_ctrl*/ 0x0000, /*_stridx*/ 0x00),\
  /* Feature Unit Descriptor(4.7.2.8) */\
  TUD_AUDIO_DESC_FEATURE_UNIT_TWO_CHANNEL(/*_unitid*/ 0x02, /*_srcid*/ 0x01, /*_ctrlch0master*/ AUDIO_CTRL_RW << AUDIO_FEATURE_UNIT_CTRL_MUTE_POS | AUDIO_CTRL_RW << AUDIO_FEATURE_UNIT_CTRL_VOLUME_POS, /*_ctrlch1*/ AUDIO_CTRL_RW << AUDIO_FEATURE_UNIT_CTRL_MUTE_POS | AUDIO_CTRL_RW << AUDIO_FEATURE_UNIT_CTRL_VOLUME_POS, /*_ctrlch2*/ AUDIO_CTRL_RW << AUDIO_FEATURE_UNIT_CTRL_MUTE_POS | AUDIO_CTRL_RW << AUDIO_FEATURE_UNIT_CTRL_VOLUME_POS,/*_stridx*/ 0x00),\
  /* Standard AS Interface Descriptor(4.9.1) */\
  /* Interface 1, Alternate 0 - default alternate setting with 0 bandwidth */\
  TUD_AUDIO_DESC_STD_AS_INT(/*_itfnum*/ (uint8_t)((_itfnum) + 1), /*_altset*/ 0x00, /*_nEPs*/ 0x00, /*_stridx*/ 0x00),\
  /* Standard AS Interface Descriptor(4.9.1) */\
  /* Interface 1, Alternate 1 - alternate interface for data streaming */\
  TUD_AUDIO_DESC_STD_AS_INT(/*_itfnum*/ (uint8_t)((_itfnum) + 1), /*_altset*/ 0x01, /*_nEPs*/ 0x02, /*_stridx*/ 0x00),\
  /* Class-Specific AS Interface Descriptor(4.9.2) */\
  TUD_AUDIO_DESC_CS_AS_INT(/*_termid*/ 0x01, /*_ctrl*/ AUDIO_CTRL_NONE, /*_formattype*/ AUDIO_FORMAT_TYPE_I, /*_formats*/ AUDIO_DATA_FORMAT_TYPE_I_PCM, /*_nchannelsphysical*/ 0x02, /*_channelcfg*/ AUDIO_CHANNEL_CONFIG_NON_PREDEFINED, /*_stridx*/ 0x00),\
  /* Type I Format Type Descriptor(2.3.1.6 - Audio Formats) */\
  TUD_AUDIO_DESC_TYPE_I_FORMAT(_nBytesPerSample, _nBitsUsedPerSample),\
  /* Standard AS Isochronous Audio Data Endpoint Descriptor(4.10.1.1) */\
  TUD_AUDIO_DESC_STD_AS_ISO_EP(/*_ep*/ _epout, /*_attr*/ (uint8_t) ((uint8_t)TUSB_XFER_ISOCHRONOUS | (uint8_t)TUSB_ISO_EP_ATT_ASYNCHRONOUS | (uint8_t)TUSB_ISO_EP_ATT_DATA), /*_maxEPsize*/ _epoutsize, /*_interval*/ 0x01),\
  /* Class-Specific AS Isochronous Audio Data Endpoint Descriptor(4.10.1.2) */\
  TUD_AUDIO_DESC_CS_AS_ISO_EP(/*_attr*/ AUDIO_CS_AS_ISO_DATA_EP_ATT_NON_MAX_PACKETS_OK, /*_ctrl*/ AUDIO_CTRL_NONE, /*_lockdelayunit*/ AUDIO_CS_AS_ISO_DATA_EP_LOCK_DELAY_UNIT_MILLISEC, /*_lockdelay*/ 0x0001),\
  /* Standard AS Isochronous Feedback Endpoint Descriptor(4.10.2.1) */\
  TUD_AUDIO_DESC_STD_AS_ISO_FB_EP(/*_ep*/ _epfb, /*_epsize*/ _epfbsize, /*_interval*/ TUD_OPT_HIGH_SPEED ? 4 : 1)\

// --- The descriptor block appended to MicroPython's built-in configuration.
//
// String index 0: the function is left unnamed so that no entry has to be
// added to MicroPython's string table, which would be a third hook for
// cosmetic benefit. Hosts fall back to the device product string.
// The MIDI function needs its own Interface Association Descriptor: the
// device already announces itself as an IAD composite (CDC's doing), and
// on such devices Windows requires every function grouped by one.
// TinyUSB's TUD_MIDI_DESCRIPTOR ships without it; the audio descriptor
// that worked leads with its own IAD, which is the tell.
// The IAD groups the MIDI function on composite (class EF) devices, where
// Windows requires it. A single-function device must NOT carry one -- the
// classic embedded MIDI host expects a bare AC+MS pair at interface 0 --
// but that is a runtime property now: usbif_desc.c strips this IAD when
// the assembled costume turns out to hold only one function. So it is
// always compiled in, and always the first thing in the MIDI block.
// The HID report descriptor's length is needed by the configuration
// descriptor at compile time. Computed from the same macros that build it
// in usbif_hid_dev.c, so the two cannot drift.
#define USBIF_HID_REPORT_DESC_LEN \
    (sizeof((uint8_t[]){ TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(1)), \
                         TUD_HID_REPORT_DESC_MOUSE(HID_REPORT_ID(2)) }))

// --- Video function: the board *is* the webcam.
//
// 160x120 YUY2 at 10 fps for the first working version. Uncompressed and
// small on purpose: it needs no encoder anywhere, every host has a YUY2
// path, and 38400 bytes a frame at 10 fps is 384 kB/s -- comfortable inside
// a full-speed isochronous budget, so the same descriptor works on the S3
// and the P4. Larger frames and MJPEG are a descriptor change once a real
// sensor is feeding this rather than a generated pattern.
#define USBIF_VIDEO_WIDTH       (160)
#define USBIF_VIDEO_HEIGHT      (120)
#define USBIF_VIDEO_BITS_PER_PX (16)
#define USBIF_VIDEO_FRAME_BYTES (USBIF_VIDEO_WIDTH * USBIF_VIDEO_HEIGHT * 2)
// Frame interval in 100 ns units, which is how UVC counts everywhere.
#define USBIF_VIDEO_INTERVAL    (1000000)
#define USBIF_VIDEO_BITRATE     (USBIF_VIDEO_FRAME_BYTES * 8 * 10)

// The class-specific descriptors the VideoStreaming input header must
// account for. Kept as its own name because the header carries the total as
// a field, and a mismatch there is the kind of error a host reports only as
// "device cannot start".
#define USBIF_VIDEO_VS_PAYLOAD_LEN                     \
    (TUD_VIDEO_DESC_CS_VS_FMT_UNCOMPR_LEN +            \
     TUD_VIDEO_DESC_CS_VS_FRM_UNCOMPR_CONT_LEN +       \
     TUD_VIDEO_DESC_CS_VS_COLOR_MATCHING_LEN)

// The +1s are the variadic tails: one interface number in the VideoControl
// collection, and one control byte per format in the input header.
#define USBIF_VIDEO_DESC_LEN                           \
    (TUD_VIDEO_DESC_IAD_LEN +                          \
     TUD_VIDEO_DESC_STD_VC_LEN +                       \
     TUD_VIDEO_DESC_CS_VC_LEN + 1 +                    \
     TUD_VIDEO_DESC_CAMERA_TERM_LEN +                  \
     TUD_VIDEO_DESC_OUTPUT_TERM_LEN +                  \
     TUD_VIDEO_DESC_STD_VS_LEN +                       \
     TUD_VIDEO_DESC_CS_VS_IN_LEN + 1 +                 \
     USBIF_VIDEO_VS_PAYLOAD_LEN +                      \
     TUD_VIDEO_DESC_STD_VS_LEN +                       \
     7)

// The frame descriptor is the *continuous* variant, not the discrete one,
// and that is a correctness fix rather than a preference. TinyUSB's
// TUD_VIDEO_DESC_CS_VS_FRM_UNCOMPR_DISC computes its length as
// `_LEN + N*4` -- four bytes per interval -- but emits `__VA_ARGS__` raw,
// so a 32-bit interval is truncated to a single byte and the descriptor
// runs three bytes short of what it declares. Everything after it shifts,
// and Windows rejects the whole configuration with "device cannot start"
// (problem code 10). Nothing in TinyUSB uses that macro -- its own video
// examples build descriptors from structs -- so it is effectively
// untested upstream. The CONT variant expands every field properly and
// takes no varargs. A single fixed rate is expressed as a degenerate
// range: min = max = the interval, step 0.
//
// Alt 0 carries no endpoint -- the zero-bandwidth setting a host parks on
// when it is not streaming -- and alt 1 carries the isochronous IN endpoint.
// That is the same shape usbif.uvc reads when hosting somebody else's
// camera, which is a useful thing to have proven from both ends.
#define USBIF_VIDEO_DESCRIPTOR                                              \
    TUD_VIDEO_DESC_IAD(USBD_ITF_VIDEO, 2, 0),                               \
    TUD_VIDEO_DESC_STD_VC(USBD_ITF_VIDEO, 0, 0),                            \
    TUD_VIDEO_DESC_CS_VC(0x0150,                                            \
    /*_totallen*/ TUD_VIDEO_DESC_CAMERA_TERM_LEN                            \
                  + TUD_VIDEO_DESC_OUTPUT_TERM_LEN,                         \
    /*_clkfreq*/ 27000000, (USBD_ITF_VIDEO + 1)),                           \
    TUD_VIDEO_DESC_CAMERA_TERM(/*_tid*/ 1, 0, 0, 0, 0, 0, 0),               \
    TUD_VIDEO_DESC_OUTPUT_TERM(/*_tid*/ 2, VIDEO_TT_STREAMING, 0,           \
    /*_srcid*/ 1, 0),                                                       \
    TUD_VIDEO_DESC_STD_VS(USBD_ITF_VIDEO + 1, /*_alt*/ 0, /*_epn*/ 0, 0),   \
    TUD_VIDEO_DESC_CS_VS_INPUT(/*_numfmt*/ 1, USBIF_VIDEO_VS_PAYLOAD_LEN,   \
    /*_ep*/ USBD_EP_VIDEO_IN, 0, /*_termlnk*/ 2, 0, 0, 0, 0),               \
    TUD_VIDEO_DESC_CS_VS_FMT_UNCOMPR(/*_fmtidx*/ 1, /*_numfrmdesc*/ 1,      \
    TUD_VIDEO_GUID_YUY2, USBIF_VIDEO_BITS_PER_PX, /*_frmidx*/ 1, 0, 0, 0, 0), \
    TUD_VIDEO_DESC_CS_VS_FRM_UNCOMPR_CONT(/*_frmidx*/ 1, 0,                 \
    USBIF_VIDEO_WIDTH, USBIF_VIDEO_HEIGHT,                                  \
    USBIF_VIDEO_BITRATE, USBIF_VIDEO_BITRATE,                               \
    USBIF_VIDEO_FRAME_BYTES, USBIF_VIDEO_INTERVAL,                          \
    /*_min*/ USBIF_VIDEO_INTERVAL, /*_max*/ USBIF_VIDEO_INTERVAL,           \
    /*_step*/ 0),                                                           \
    TUD_VIDEO_DESC_CS_VS_COLOR_MATCHING(1, 1, 4),                           \
    TUD_VIDEO_DESC_STD_VS(USBD_ITF_VIDEO + 1, /*_alt*/ 1, /*_epn*/ 1, 0),   \
    TUD_VIDEO_DESC_EP_ISO(USBD_EP_VIDEO_IN,                                 \
    CFG_TUD_VIDEO_STREAMING_EP_BUFSIZE, 1)

#define USBIF_MIDI_IAD_LEN (8)
#define USBIF_MIDI_IAD \
    8, TUSB_DESC_INTERFACE_ASSOCIATION, USBD_ITF_MIDI, 2, \
    TUSB_CLASS_AUDIO, AUDIO_SUBCLASS_MIDI_STREAMING, \
    AUDIO_FUNC_PROTOCOL_CODE_UNDEF, 0,

#if USBIF_EXT_AUDIO
#define MICROPY_HW_USB_EXT_DESC_CFG_LEN (USBIF_AUDIO_SPEAKER_STEREO_FB_DESC_LEN + USBIF_MIDI_IAD_LEN + TUD_MIDI_DESC_LEN + TUD_HID_DESC_LEN + USBIF_VIDEO_DESC_LEN)
#else
#define MICROPY_HW_USB_EXT_DESC_CFG_LEN (USBIF_MIDI_IAD_LEN + TUD_MIDI_DESC_LEN + TUD_HID_DESC_LEN + USBIF_VIDEO_DESC_LEN)
#endif
// High speed counts in 125 us microframes, so bInterval 4 gives the 1 ms
// feedback refresh the class expects; full speed counts in 1 ms frames, where
// that is bInterval 1.
#define USBIF_AUDIO_FB_INTERVAL (TUD_OPT_HIGH_SPEED ? 4 : 1)

#if !USBIF_EXT_AUDIO
#define MICROPY_HW_USB_EXT_DESC_CFG                    \
    USBIF_MIDI_IAD                                     \
    TUD_MIDI_DESCRIPTOR(                               \
    /*_itfnum*/ USBD_ITF_MIDI,                         \
    /*_stridx*/ 0,                                     \
    /*_epout*/ USBD_EP_MIDI_OUT,                       \
    /*_epin*/ USBD_EP_MIDI_IN,                         \
    /*_epsize*/ 64),                                   \
    TUD_HID_DESCRIPTOR(                                \
    /*_itfnum*/ USBD_ITF_HID,                          \
    /*_stridx*/ 0,                                     \
    /*_boot_protocol*/ HID_ITF_PROTOCOL_NONE,          \
    /*_report_desc_len*/ USBIF_HID_REPORT_DESC_LEN,    \
    /*_epin*/ USBD_EP_HID_IN,                          \
    /*_epsize*/ CFG_TUD_HID_EP_BUFSIZE,                \
    /*_ep_interval*/ 10),                              \
    USBIF_VIDEO_DESCRIPTOR,
#else
#define MICROPY_HW_USB_EXT_DESC_CFG                    \
    USBIF_AUDIO_SPEAKER_STEREO_FB_DESCRIPTOR(          \
    /*_itfnum*/ USBD_ITF_AUDIO,                        \
    /*_stridx*/ 0,                                     \
    /*_nBytesPerSample*/ CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_RX, \
    /*_nBitsUsedPerSample*/ CFG_TUD_AUDIO_FUNC_1_RESOLUTION_RX,      \
    /*_epout*/ USBD_EP_AUDIO_OUT,                      \
    /*_epoutsize*/ CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ_MAX, \
    /*_epfb*/ USBD_EP_AUDIO_FB,                        \
    /*_epfbsize*/ 4),                                  \
    USBIF_MIDI_IAD                                     \
    TUD_MIDI_DESCRIPTOR(                               \
    /*_itfnum*/ USBD_ITF_MIDI,                         \
    /*_stridx*/ 0,                                     \
    /*_epout*/ USBD_EP_MIDI_OUT,                       \
    /*_epin*/ USBD_EP_MIDI_IN,                         \
    /*_epsize*/ 64),                                   \
    TUD_HID_DESCRIPTOR(                                \
    /*_itfnum*/ USBD_ITF_HID,                          \
    /*_stridx*/ 0,                                     \
    /*_boot_protocol*/ HID_ITF_PROTOCOL_NONE,          \
    /*_report_desc_len*/ USBIF_HID_REPORT_DESC_LEN,    \
    /*_epin*/ USBD_EP_HID_IN,                          \
    /*_epsize*/ CFG_TUD_HID_EP_BUFSIZE,                \
    /*_ep_interval*/ 10),                              \
    USBIF_VIDEO_DESCRIPTOR,
#endif // USBIF_EXT_AUDIO

#endif // USBIF_TUSB_EXT_H
