# MicroPython CMake glue for usbif (ESP32-S2/S3/P4; see the guard below).
# For Make-based ports (unix, windows), see micropython.mk in this dir.
#
# Point USER_C_MODULES at this repo (or this file) directly, e.g.:
#   idf.py build -DUSER_C_MODULES=<path to usbif>/micropython.cmake
# Or add one line to your own manifest.py, which names this module via c_module():
#   include("<path to usbif>/manifest.py")

set(USBIF_MOD_DIR ${CMAKE_CURRENT_LIST_DIR})
set(USBIF_SRC_DIR ${USBIF_MOD_DIR}/src)

# usbif is TinyUSB all the way down -- every source below needs it -- and
# TinyUSB is present on parts with a USB OTG controller: ESP32-S2, S3, and P4.
# That is the same line MicroPython draws with MICROPY_PY_TINYUSB
# (ports/esp32/esp32_common.cmake).
#
# Other ESP32 parts (C6, C3, C61, H2, classic) have USB Serial/JTAG at most, so
# the TinyUSB component is absent from their build. rp2 fails differently and
# more quietly: it HAS TinyUSB, but a usermod there does not get its include
# path, so every source fails on `tusb.h: No such file or directory`. Wiring
# that up is plausible work nobody has done, so rp2 is skipped too rather than
# half-supported.
#
# Without this guard the whole firmware fails to configure on those targets:
# idf_component_get_property() below is a hard CMake error for a component that
# does not exist, so the `if(usbif_tusb_lib)` test never gets to run. Skipping
# the module leaves the rest of the firmware buildable.
if(NOT (ESP_PLATFORM AND (CONFIG_IDF_TARGET_ESP32S2 OR CONFIG_IDF_TARGET_ESP32S3 OR CONFIG_IDF_TARGET_ESP32P4)))
    message(STATUS
        "usbif: skipped -- builds only on the ESP32-S2, S3, and P4. The firmware "
        "builds without the _usbif module.")
    return()
endif()

add_library(usermod_usbif INTERFACE)

target_sources(usermod_usbif INTERFACE
    ${USBIF_SRC_DIR}/mod_usbif.c
    ${USBIF_SRC_DIR}/usbif_uac.c
    ${USBIF_SRC_DIR}/usbif_i2s.c
    ${USBIF_SRC_DIR}/usbif_meter.c
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
    ${USBIF_SRC_DIR}/shared/usbif_byte_ring.c
    ${USBIF_SRC_DIR}/shared/usbif_pcm_sink.c
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
        # Spike: the audio receive path in IRAM (src/usbif_isr_iram.lf).
        __ldgen_add_fragment_files("${USBIF_SRC_DIR}/usbif_isr_iram.lf")
    endif()
    # The IDF defines ESP_PLATFORM for its own components but not for a user C
    # module, and FreeRTOS's ESP-IDF trace-macro defaults sit behind it: on the
    # S3 (Xtensa) portYIELD_FROM_ISR in usbif_i2s.c then fails to compile with
    # an implicit traceISR_EXIT_TO_SCHEDULER. Kitchen-sink builds never saw it
    # because another module's glue defines it; usbif built on its own did.
    target_compile_definitions(usermod_usbif INTERFACE ESP_PLATFORM=1)
endif()

target_link_libraries(usermod INTERFACE usermod_usbif)

# --- which usbif this firmware was built from ----------------------
# Computed at build time from this repo's own git, never stored; "unknown" when
# there is no git (a tarball). Read on a target as <module>.__revision__.
execute_process(
    COMMAND git -C ${USBIF_MOD_DIR} describe --always --dirty --abbrev=7
    OUTPUT_VARIABLE USBIF_REVISION
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET)
if(NOT USBIF_REVISION)
    set(USBIF_REVISION "unknown")
endif()
target_compile_definitions(usermod_usbif INTERFACE USBIF_REVISION=\"${USBIF_REVISION}\")
