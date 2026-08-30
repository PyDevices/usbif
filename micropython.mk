# MicroPython Make-based build glue for usbif (unix, windows).
# For CMake-based ports (esp32, rp2, …), see micropython.cmake in this dir.
#
# Discovered via USER_C_MODULES pointing at the workspace directory that
# contains this repo (its parent) — see cmods/build_mp.sh.

USBIF_MOD_DIR := $(USERMOD_DIR)
USBIF_SRC_DIR := $(USBIF_MOD_DIR)/src

CFLAGS_USERMOD += -I$(USBIF_SRC_DIR)

SRC_USERMOD_C += \
    $(USBIF_SRC_DIR)/mod_usbif.c \
    $(USBIF_SRC_DIR)/shared/usbif_ringbuf.c
