# MicroPython Make-based build glue for usbif (unix, windows).
# For CMake-based ports (esp32, rp2, …), see micropython.cmake in this dir.
#
# Named by c_module() in this repo's manifest.py (MicroPython 1.29+): add
#   include("<path to usbif>/manifest.py")
# to your manifest, or point USER_C_MODULES at this directory.

USBIF_MOD_DIR := $(USERMOD_DIR)
USBIF_SRC_DIR := $(USBIF_MOD_DIR)/src

CFLAGS_USERMOD += -I$(USBIF_SRC_DIR)

SRC_USERMOD_C += \
    $(USBIF_SRC_DIR)/mod_usbif.c \
    $(USBIF_SRC_DIR)/usbif_uac.c \
    $(USBIF_SRC_DIR)/usbif_i2s.c \
    $(USBIF_SRC_DIR)/usbif_meter.c \
    $(USBIF_SRC_DIR)/shared/usbif_ringbuf.c \
    $(USBIF_SRC_DIR)/shared/usbif_midi_packet.c

# --- which usbif this firmware was built from ----------------------
# Computed at build time from this repo's own git, never stored; "unknown" when
# there is no git (a tarball). Read on a target as <module>.__revision__.
USBIF_REVISION := $(shell git -C $(USBIF_MOD_DIR) describe --always --dirty --abbrev=7 2>/dev/null || echo unknown)
CFLAGS_USERMOD += -DUSBIF_REVISION='"$(USBIF_REVISION)"'
