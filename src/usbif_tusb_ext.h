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

#define CFG_TUD_AUDIO (1)
// And a USB MIDI function beside it: TinyUSB carries the whole class
// (descriptor macro, jack plumbing, tud_midi_* API); this file only has to
// give it numbers and space. Both functions appear and disappear together
// through the same runtime opt-in the audio function uses.
#define CFG_TUD_MIDI (1)
#define CFG_TUD_MIDI_RX_BUFSIZE (TUD_OPT_HIGH_SPEED ? 512 : 64)
#define CFG_TUD_MIDI_TX_BUFSIZE (TUD_OPT_HIGH_SPEED ? 512 : 64)

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
#define USBD_ITF_MIDI (USBD_ITF_AUDIO + 2)
#define USBIF_EPNUM_MIDI (USBIF_EPNUM_AUDIO + 1)
#define USBD_EP_MIDI_OUT (USBIF_EPNUM_MIDI)
#define USBD_EP_MIDI_IN (0x80 | USBIF_EPNUM_MIDI)

// The audio function occupies two interfaces (AudioControl and
// AudioStreaming) and MIDI two more, which the bounds below must cover so
// runtime_dev_open keeps declining everything the built-in descriptor owns.
#undef USBD_ITF_BUILTIN_MAX
#define USBD_ITF_BUILTIN_MAX (USBD_ITF_MIDI + 2)
#undef USBD_EP_BUILTIN_MAX
#define USBD_EP_BUILTIN_MAX (USBIF_EPNUM_MIDI + 1)

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
#define MICROPY_HW_USB_EXT_DESC_CFG_LEN (USBIF_AUDIO_SPEAKER_STEREO_FB_DESC_LEN + TUD_MIDI_DESC_LEN)
// High speed counts in 125 us microframes, so bInterval 4 gives the 1 ms
// feedback refresh the class expects; full speed counts in 1 ms frames, where
// that is bInterval 1.
#define USBIF_AUDIO_FB_INTERVAL (TUD_OPT_HIGH_SPEED ? 4 : 1)

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
    TUD_MIDI_DESCRIPTOR(                               \
    /*_itfnum*/ USBD_ITF_MIDI,                         \
    /*_stridx*/ 0,                                     \
    /*_epout*/ USBD_EP_MIDI_OUT,                       \
    /*_epin*/ USBD_EP_MIDI_IN,                         \
    /*_epsize*/ (TUD_OPT_HIGH_SPEED ? 512 : 64)),

#endif // USBIF_TUSB_EXT_H
