// SPDX-License-Identifier: MIT
//
// USB host bring-up: the IDF USB Host Library on the OTG controller, pushing
// enumeration events into the same ring Python already drains. This is the
// Phase 2 milestone the transport was built for: attach and detach arrive as
// fixed-size records that survive whatever the VM is doing, and Python
// collects them with poll() at its own pace.
//
// The controller handoff is the part worth reading twice. At boot the port
// owns the OTG controller as a TinyUSB *device* (the CDC REPL). Host duty is
// opt-in from Python, mirroring uac_enable: usbif_host_start_c() quiesces the
// device stack (tud_disconnect + tud_deinit frees the DWC ISR), releases the
// device-mode PHY through the patch-0004 helpers in ports/esp32/usb.c, and
// only then installs the host library, which creates its own host-mode PHY.
// Stopping reverses the sequence, so a board is a plain CDC device again
// after host_stop() + replug. One controller, one owner at a time; the
// concurrent host+device story waits for hardware with both ports wired.
//
// Class drivers are deliberately absent at this stage. Enumeration alone
// answers the questions that matter first -- does the handoff work, does a
// device attach, what does it claim to be -- and the class bitmask derived
// from interface descriptors is exactly what host_start()'s class filter
// will need when the drivers arrive.

#include "py/mpconfig.h"

// Usermod sources build with MicroPython's define set, not the IDF's, so
// ESP_PLATFORM is NOT defined here -- learned the hard way when this gate
// silently compiled the engine out and the Phase 1 stubs answered in its
// place. sdkconfig.h existing (and naming an OTG controller) is the real
// signal that the IDF host library is available.
#if defined(__has_include)
#if __has_include("sdkconfig.h")
#include "sdkconfig.h"
#endif
#endif

#if defined(CONFIG_SOC_USB_OTG_SUPPORTED) && CONFIG_SOC_USB_OTG_SUPPORTED

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/idf_additions.h"   // xTaskCreatePinnedToCore on IDF 5
#include "esp_intr_alloc.h"
#include "esp_cpu.h"
#include "usb/usb_host.h"

#include "shared/usbif_ringbuf.h"
#include "usbif_classes.h"

#if MICROPY_HW_ENABLE_USBDEV
#include "tusb.h"
// From ports/esp32/usb.c via usbif patch 0004: release the device-mode PHY
// around host duty, and restore it afterwards.
extern void usb_phy_otg_release(void);
extern void usb_phy_otg_device_mode(void);
#endif

// mod_usbif.c owns the event ring; this is its producer hook. Single
// producer holds: only the host task below ever calls it.
extern void usbif_host_emit(const usbif_event_t *event);

#define USBIF_HOST_MAX_DEVS (8)
// Below the pump (10) -- enumeration control transfers are not realtime
// audio -- and above MicroPython's task so events keep flowing while the
// interpreter is busy, which is the transport's whole reason to exist.
#define USBIF_HOST_TASK_PRIO (7)
#define USBIF_HOST_TASK_STACK (8192)

typedef struct {
    bool in_use;
    uint8_t addr;
    uint8_t speed;      // usbif encoding: 1 low, 2 full, 3 high
    uint16_t vid, pid, classes;
    usb_device_handle_t hdl;
} usbif_host_slot_t;

static usbif_host_slot_t usbif_host_devs[USBIF_HOST_MAX_DEVS];
static usb_host_client_handle_t usbif_host_client;
static TaskHandle_t usbif_host_task_handle;
static volatile bool usbif_host_task_running;

// Diagnostic counters, same philosophy as the UAC ones: the first question
// when nothing attaches is whether anything happened at all.
uint32_t usbif_host_attaches, usbif_host_detaches, usbif_host_errors;

static uint16_t usbif_class_bits(const usb_config_desc_t *cfg) {
    // Walk the whole configuration: every interface descriptor contributes
    // its class to the mask. Composite devices (a keyboard with a hub, a
    // CDC+MSC gadget) are the rule, not the exception.
    const uint8_t *p = (const uint8_t *)cfg;
    const uint8_t *end = p + cfg->wTotalLength;
    uint16_t bits = 0;
    p += cfg->bLength;
    while (p + 2 <= end && p[0] >= 2 && p + p[0] <= end) {
        if (p[1] == USB_B_DESCRIPTOR_TYPE_INTERFACE && p[0] >= 9) {
            uint8_t klass = p[5];
            uint8_t subclass = p[6];
            switch (klass) {
                case USB_CLASS_HID:
                    bits |= USBIF_CLASS_HID;
                    break;
                case USB_CLASS_MASS_STORAGE:
                    bits |= USBIF_CLASS_MSC;
                    break;
                case USB_CLASS_COMM:
                case USB_CLASS_CDC_DATA:
                    bits |= USBIF_CLASS_CDC;
                    break;
                case USB_CLASS_AUDIO:
                    // Audio subclass 3 is MIDIStreaming; everything else in
                    // the audio class is, for our purposes, a sound device.
                    bits |= (subclass == 3) ? USBIF_CLASS_MIDI : USBIF_CLASS_UAC;
                    break;
                case USB_CLASS_VIDEO:
                    bits |= USBIF_CLASS_UVC;
                    break;
                default:
                    break;
            }
        }
        p += p[0];
    }
    return bits;
}

static void usbif_host_on_new_dev(uint8_t addr) {
    usbif_host_slot_t *slot = NULL;
    for (int i = 0; i < USBIF_HOST_MAX_DEVS; i++) {
        if (!usbif_host_devs[i].in_use) {
            slot = &usbif_host_devs[i];
            break;
        }
    }
    if (slot == NULL) {
        usbif_host_errors++;
        return;
    }

    usb_device_handle_t hdl;
    if (usb_host_device_open(usbif_host_client, addr, &hdl) != ESP_OK) {
        usbif_host_errors++;
        return;
    }
    usb_device_info_t info;
    const usb_device_desc_t *dev_desc;
    const usb_config_desc_t *cfg_desc;
    if (usb_host_device_info(hdl, &info) != ESP_OK
        || usb_host_get_device_descriptor(hdl, &dev_desc) != ESP_OK
        || usb_host_get_active_config_descriptor(hdl, &cfg_desc) != ESP_OK) {
        usbif_host_errors++;
        usb_host_device_close(usbif_host_client, hdl);
        return;
    }

    slot->in_use = true;
    slot->addr = addr;
    slot->hdl = hdl;
    slot->vid = dev_desc->idVendor;
    slot->pid = dev_desc->idProduct;
    slot->classes = usbif_class_bits(cfg_desc);
    // usb_speed_t counts low/full/high from zero; usbif reserves zero for
    // "unknown", so shift by one.
    slot->speed = (uint8_t)(info.speed + 1);

    usbif_event_t event = {
        .kind = USBIF_EV_ATTACH,
        .speed = slot->speed,
        .vid = slot->vid,
        .pid = slot->pid,
        .classes = slot->classes,
        .dev_id = slot->addr,
    };
    usbif_host_emit(&event);
    usbif_host_attaches++;
}

extern void usbif_cdc_on_dev_gone(usb_device_handle_t dev);

static void usbif_host_on_dev_gone(usb_device_handle_t hdl) {
    usbif_cdc_on_dev_gone(hdl);
    for (int i = 0; i < USBIF_HOST_MAX_DEVS; i++) {
        usbif_host_slot_t *slot = &usbif_host_devs[i];
        if (slot->in_use && slot->hdl == hdl) {
            usbif_event_t event = {
                .kind = USBIF_EV_DETACH,
                .speed = slot->speed,
                .vid = slot->vid,
                .pid = slot->pid,
                .classes = slot->classes,
                .dev_id = slot->addr,
            };
            usbif_host_emit(&event);
            usbif_host_detaches++;
            usb_host_device_close(usbif_host_client, hdl);
            slot->in_use = false;
            return;
        }
    }
    usbif_host_errors++;
}

static void usbif_host_client_cb(const usb_host_client_event_msg_t *msg, void *arg) {
    (void)arg;
    // Runs inside usb_host_client_handle_events, i.e. in the host task --
    // ordinary task context, free to open devices and read descriptors.
    switch (msg->event) {
        case USB_HOST_CLIENT_EVENT_NEW_DEV:
            usbif_host_on_new_dev(msg->new_dev.address);
            break;
        case USB_HOST_CLIENT_EVENT_DEV_GONE:
            usbif_host_on_dev_gone(msg->dev_gone.dev_hdl);
            break;
        default:
            break;
    }
}

// Rendezvous for the install that happens inside the task: the interrupt a
// task allocates lands on the core that task runs on. MicroPython's own task
// -- and therefore TinyUSB's boot-time allocation and everything a Python
// call runs -- lives on core 1 (MP_TASK_COREID), and core 1 answered "No
// free interrupt inputs for USB_OTG" even after tud_deinit freed the device
// line. So the task is pinned to core 0 and does the install itself;
// start_c waits here for the verdict. Teardown, including
// usb_host_uninstall(), also runs in the task, because freeing an interrupt
// belongs to the core that owns it.
#define USBIF_HOST_INSTALL_PENDING (0x7FFFFFFF)
static volatile int usbif_host_install_result = USBIF_HOST_INSTALL_PENDING;

static void usbif_host_task(void *arg) {
    (void)arg;
    printf("usbif_host: task up on core %d\n", esp_cpu_get_core_id());

    usb_host_config_t config = {
        // Levels 1-3: what TinyUSB's own esp32 glue requests for the same
        // controller.
        .intr_flags = ESP_INTR_FLAG_LOWMED,
        // On dual-controller chips (P4): BIT1 = the HS controller,
        // explicitly. The header says a map of 0 defaults to the High-Speed
        // peripheral on HS-capable targets; the code says `map == 0 ? BIT0`
        // -- the FS controller, whose INT PHY belongs to USB-Serial-JTAG
        // there, presenting as "selected PHY is in use". On single-
        // controller chips (S3) BIT1 names hardware that does not exist and
        // install fails with ESP_ERR_INVALID_ARG, so the default stands.
        #if defined(CONFIG_SOC_USB_OTG_PERIPH_NUM) && CONFIG_SOC_USB_OTG_PERIPH_NUM > 1
        .peripheral_map = BIT1,
        #endif
    };
    esp_err_t err = usb_host_install(&config);
    printf("usbif_host: install -> 0x%x\n", (unsigned)err);
    if (err == ESP_OK) {
        usb_host_client_config_t client_config = {
            .is_synchronous = false,
            .max_num_event_msg = 8,
            .async = {
                .client_event_callback = usbif_host_client_cb,
                .callback_arg = NULL,
            },
        };
        err = usb_host_client_register(&client_config, &usbif_host_client);
        printf("usbif_host: client register -> 0x%x\n", (unsigned)err);
        if (err != ESP_OK) {
            usb_host_uninstall();
        }
    }
    usbif_host_install_result = (int)err;
    if (err != ESP_OK) {
        usbif_host_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    // One task services both the library and the client. The library call
    // blocks up to a tick so the loop idles cheaply; the client call then
    // collects whatever that produced without waiting.
    while (usbif_host_task_running) {
        uint32_t flags = 0;
        usb_host_lib_handle_events(pdMS_TO_TICKS(10), &flags);
        usb_host_client_handle_events(usbif_host_client, 0);
    }

    // Teardown, in the order the library requires: close what we opened,
    // walk away as a client, free the devices, and drain events until the
    // library confirms everything is gone.
    for (int i = 0; i < USBIF_HOST_MAX_DEVS; i++) {
        if (usbif_host_devs[i].in_use) {
            usb_host_device_close(usbif_host_client, usbif_host_devs[i].hdl);
            usbif_host_devs[i].in_use = false;
        }
    }
    usb_host_client_deregister(usbif_host_client);
    usbif_host_client = NULL;
    usb_host_device_free_all();
    for (int i = 0; i < 100; i++) {
        uint32_t flags = 0;
        usb_host_lib_handle_events(pdMS_TO_TICKS(10), &flags);
        if (flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            break;
        }
    }
    usb_host_uninstall();
    usbif_host_task_handle = NULL;
    vTaskDelete(NULL);
}

int usbif_host_start_c(void) {
    if (usbif_host_task_handle != NULL) {
        return 0;
    }

    #if MICROPY_HW_ENABLE_USBDEV
    // Quiesce the device stack before touching the controller: disconnect so
    // an attached host sees a clean detach, deinit so the DWC interrupt is
    // freed, then release the device-mode PHY. The host library creates its
    // own PHY in host mode when installed -- from inside the task, on core 1.
    // TUD_OPT_RHPORT, not 0: on the ESP32-P4 TinyUSB numbers the FS
    // controller port 0 and the HS controller -- the one on the connector,
    // the one the device stack actually runs on -- port 1. tud_deinit(0)
    // "succeeds" by early-returning on the uninitialized FS port, leaving
    // the HS device ISR and controller fully alive, which then presents as
    // "selected PHY is in use" and a phantom "No free interrupt inputs"
    // (a claimed interrupt source reports as exhaustion). Cost of learning
    // this: five builds.
    tud_disconnect();
    tud_deinit(TUD_OPT_RHPORT);
    usb_phy_otg_release();
    #endif

    usbif_host_install_result = USBIF_HOST_INSTALL_PENDING;
    usbif_host_task_running = true;
    if (xTaskCreatePinnedToCore(usbif_host_task, "usbif_host",
        USBIF_HOST_TASK_STACK, NULL, USBIF_HOST_TASK_PRIO,
        &usbif_host_task_handle, 0) != pdPASS) {
        usbif_host_task_running = false;
        goto restore_device;
    }
    // Wait for the task's install verdict (it is quick; the timeout is a
    // failsafe, not an expectation). vTaskDelay(1), not pdMS_TO_TICKS(5):
    // this port ticks at 100 Hz, so five milliseconds rounds to ZERO ticks
    // and a "delay" loop of yields burns out in microseconds -- the same
    // trap usbif_i2s.c documents, stepped in again here. One tick is 10 ms;
    // 200 of them bound the wait at two seconds.
    for (int i = 0; i < 200 && usbif_host_install_result == USBIF_HOST_INSTALL_PENDING; i++) {
        vTaskDelay(1);
    }
    if (usbif_host_install_result == ESP_OK) {
        return 0;
    }
    usbif_host_task_running = false;

restore_device:
    #if MICROPY_HW_ENABLE_USBDEV
    usb_phy_otg_device_mode();
    tusb_init();
    #endif
    return usbif_host_install_result == USBIF_HOST_INSTALL_PENDING
        ? -1 : usbif_host_install_result;
}

void usbif_host_stop_c(void) {
    if (usbif_host_task_handle == NULL) {
        return;
    }
    usbif_host_task_running = false;
    for (int i = 0; i < 200 && usbif_host_task_handle != NULL; i++) {
        vTaskDelay(1);   // one 10 ms tick; pdMS_TO_TICKS(5) is ZERO ticks here
    }
    // The task uninstalled the library itself before exiting: the HCD
    // interrupt lives on the task's core and is freed there.

    #if MICROPY_HW_ENABLE_USBDEV
    // Hand the controller back: device-mode PHY, then the TinyUSB device
    // stack. The board is a CDC device again on its next connection.
    usb_phy_otg_device_mode();
    tusb_init();
    #endif
}

// Diagnostic: IDF's own per-core interrupt allocation table, to stdout.
// The first question when "No free interrupt inputs" appears is which core
// is full, of what, and whether a freed line actually returned to the pool.
#include <stdio.h>
void usbif_host_intr_dump(void) {
    esp_intr_dump(stdout);
}

bool usbif_host_is_running(void) {
    return usbif_host_task_handle != NULL;
}

// For class drivers (usbif_host_cdc.c): the client everything is opened
// under, and a device lookup by the dev_id the event transport reported.
usb_host_client_handle_t usbif_host_client_get(void) {
    return usbif_host_client;
}

int usbif_host_dev_lookup(uint32_t dev_id, usb_device_handle_t *out) {
    for (int i = 0; i < USBIF_HOST_MAX_DEVS; i++) {
        if (usbif_host_devs[i].in_use && usbif_host_devs[i].addr == dev_id) {
            *out = usbif_host_devs[i].hdl;
            return 0;
        }
    }
    return -1;
}

// The library's own view: how many devices and clients it currently tracks.
// The first question when nothing attaches is whether the controller saw a
// connection at all -- zero devices here across a replug means the root port
// never detected one, which points at wiring, orientation or VBUS rather
// than at enumeration.
int usbif_host_lib_counts(int *num_devices, int *num_clients) {
    usb_host_lib_info_t info;
    if (usb_host_lib_info(&info) != ESP_OK) {
        return -1;
    }
    *num_devices = info.num_devices;
    *num_clients = info.num_clients;
    return 0;
}

// Software replug: power-cycle the root port so connect detection starts
// fresh, without anyone touching a cable.
int usbif_host_port_cycle(void) {
    if (usbif_host_task_handle == NULL) {
        return -1;
    }
    usb_host_lib_set_root_port_power(false);
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_err_t err = usb_host_lib_set_root_port_power(true);
    return err == ESP_OK ? 0 : (int)err;
}

// Snapshot of currently attached devices, as ATTACH-shaped records. Read
// from the VM task while the host task may write a slot; each field is
// written before in_use is set and after it is cleared, so a torn row is
// bounded to a device changing state mid-snapshot -- which the next
// poll()'s events describe anyway.
int usbif_host_snapshot(usbif_event_t *out, int max) {
    int n = 0;
    for (int i = 0; i < USBIF_HOST_MAX_DEVS && n < max; i++) {
        const usbif_host_slot_t *slot = &usbif_host_devs[i];
        if (slot->in_use) {
            out[n].kind = USBIF_EV_ATTACH;
            out[n].speed = slot->speed;
            out[n].vid = slot->vid;
            out[n].pid = slot->pid;
            out[n].classes = slot->classes;
            out[n].dev_id = slot->addr;
            n++;
        }
    }
    return n;
}

#endif // ESP_PLATFORM && CONFIG_SOC_USB_OTG_SUPPORTED
