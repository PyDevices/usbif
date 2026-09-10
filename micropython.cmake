# MicroPython CMake glue for usbif (ESP32-S2/S3 only; see the guard below).
# For Make-based ports (unix, windows), see micropython.mk in this dir.
#
# Point USER_C_MODULES at this repo (or this file) directly, e.g.:
#   idf.py build -DUSER_C_MODULES=<path to usbif>/micropython.cmake
# Or let the workspace aggregator's own micropython.cmake discover it
# alongside other usermods.

set(USBIF_MOD_DIR ${CMAKE_CURRENT_LIST_DIR})
set(USBIF_SRC_DIR ${USBIF_MOD_DIR}/src)

# usbif is TinyUSB all the way down -- every source below needs it -- and it has
# only ever been built for the ESP32-S2 and S3.
#
# Other ESP32 parts (C6, C3, C61, H2, classic) have USB Serial/JTAG at most, so
# the TinyUSB component is absent from their build. rp2 fails differently and
# more quietly: it HAS TinyUSB, but a usermod there does not get its include
# path, so every source fails on `tusb.h: No such file or directory`. Wiring
# that up is plausible work nobody has done -- the host-mode half is S3-specific
# regardless -- so rp2 is skipped too rather than half-supported.
#
# Without this guard the whole firmware fails to configure on those targets:
# idf_component_get_property() below is a hard CMake error for a component that
# does not exist, so the `if(usbif_tusb_lib)` test never gets to run. Skipping
# the module leaves the rest of the firmware buildable, which is what lets us
# target chips beyond the S2/S3. MicroPython draws the same line with
# MICROPY_PY_TINYUSB (ports/esp32/esp32_common.cmake).
if(NOT (ESP_PLATFORM AND (CONFIG_IDF_TARGET_ESP32S2 OR CONFIG_IDF_TARGET_ESP32S3)))
    message(STATUS
        "usbif: skipped -- builds only on the ESP32-S2 and S3. The firmware "
        "builds without the _usbif module.")
    return()
endif()

add_library(usermod_usbif INTERFACE)

target_sources(usermod_usbif INTERFACE
    ${USBIF_SRC_DIR}/mod_usbif.c
    ${USBIF_SRC_DIR}/usbif_uac.c
    ${USBIF_SRC_DIR}/usbif_i2s.c
    ${USBIF_SRC_DIR}/usbif_host.c
    ${USBIF_SRC_DIR}/usbif_host_cdc.c
    ${USBIF_SRC_DIR}/usbif_host_hid.c
    ${USBIF_SRC_DIR}/usbif_host_msc.c
    ${USBIF_SRC_DIR}/usbif_host_midi.c
    ${USBIF_SRC_DIR}/usbif_host_uac.c
    ${USBIF_SRC_DIR}/usbif_host_uvc.c
    ${USBIF_SRC_DIR}/usbif_desc.c
    ${USBIF_SRC_DIR}/usbif_hid_dev.c
    ${USBIF_SRC_DIR}/usbif_msc_dev.c
    ${USBIF_SRC_DIR}/usbif_uvc_dev.c
    ${USBIF_SRC_DIR}/shared/usbif_ringbuf.c
    ${USBIF_SRC_DIR}/shared/usbif_midi_packet.c
)

target_include_directories(usermod_usbif INTERFACE
    ${USBIF_SRC_DIR}
)

# MicroPython compiles the TinyUSB component against its own tusb_config.h,
# whose extension hook includes a header from this repo. The component's
# include path must therefore reach src/ -- the same arrangement the esp32
# port already makes for shared/tinyusb.
if(ESP_PLATFORM)
    idf_component_get_property(usbif_tusb_lib espressif__tinyusb COMPONENT_LIB)
    if(usbif_tusb_lib)
        target_include_directories(${usbif_tusb_lib} PRIVATE ${USBIF_SRC_DIR})
    endif()
endif()

target_link_libraries(usermod INTERFACE usermod_usbif)
