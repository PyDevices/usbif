# MicroPython CMake glue for usbif (esp32, rp2, …).
# For Make-based ports (unix, windows), see micropython.mk in this dir.
#
# Point USER_C_MODULES at this repo (or this file) directly, e.g.:
#   idf.py build -DUSER_C_MODULES=<path to usbif>/micropython.cmake
# Or let the workspace aggregator (cmods/micropython.cmake) discover it
# alongside other usermods.

set(USBIF_MOD_DIR ${CMAKE_CURRENT_LIST_DIR})
set(USBIF_SRC_DIR ${USBIF_MOD_DIR}/src)

add_library(usermod_usbif INTERFACE)

target_sources(usermod_usbif INTERFACE
    ${USBIF_SRC_DIR}/mod_usbif.c
    ${USBIF_SRC_DIR}/usbif_uac.c
    ${USBIF_SRC_DIR}/shared/usbif_ringbuf.c
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
