// SPDX-License-Identifier: MIT
//
// Class bitmask, mirroring the names in the portable Python API. A bitmask
// rather than strings so an event record stays fixed-size and ISR-safe; the
// translation to names happens on the consumer side, where allocation is
// allowed. Shared between the module surface (mod_usbif.c) and the host
// engine (usbif_host.c), which derives the bits from interface descriptors.
#ifndef USBIF_CLASSES_H
#define USBIF_CLASSES_H

#define USBIF_CLASS_HID  (1u << 0)
#define USBIF_CLASS_MSC  (1u << 1)
#define USBIF_CLASS_CDC  (1u << 2)
#define USBIF_CLASS_MIDI (1u << 3)
#define USBIF_CLASS_UAC  (1u << 4)
#define USBIF_CLASS_UVC  (1u << 5)

// Which hosted UAC stream a call means (usbif_host_uac.c keeps one per
// direction). ANY is the meaning the calls had when there was one stream:
// whichever is open, playback first; for close, both.
#define USBIF_HOST_UAC_ANY (-1)
#define USBIF_HOST_UAC_OUT (0)
#define USBIF_HOST_UAC_IN  (1)

#endif // USBIF_CLASSES_H
