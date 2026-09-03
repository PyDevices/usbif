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
// Class drivers (CDC, HID, MSC) now exist alongside this file, each opened
// on demand by Python once it sees an attach whose class bitmask matches.
// The bitmask itself is derived from interface descriptors below, and is
// also what host_start()'s class filter is checked against: a device
// offering none of the requested classes is closed immediately in
// usbif_host_on_new_dev and never occupies a slot, so it never appears in
// host_devices() or as an ATTACH event.

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
// Set once teardown begins, so no device is opened after the close loop has
// already passed over it. See usbif_host_on_new_dev() for why this exists.
static volatile bool usbif_host_tearing_down;
// Set when usbif_host_stop_c()'s wait for the task to exit times out.
// Once set, the task handle is known-zombie: it is stuck inside a blocking
// IDF call (observed: usb_host_lib_handle_events() during the ALL_FREE
// wait, when a device that was genuinely held open -- not merely opened
// and immediately closed as a class-filter rejection -- is torn down) and
// will not clear itself. Reboot is the only recovery currently known.
// Recorded so usbif_host_start_c() can refuse loudly on a later call
// instead of taking the non-NULL handle at face value and silently
// pretending a fresh host actually (re)started.
static volatile bool usbif_host_task_wedged;

// Set by mod_usbif.c's host_start() before usbif_host_start_c() installs the
// library, so it is always in place before the client callback can fire.
// Defaults to "everything" rather than zero, so a device attaching before any
// filter is set (there shouldn't be one, but a static default of zero would
// silently hide every device) is never mistaken for "filter to nothing".
static uint16_t usbif_host_class_filter = 0xFFFF;

void usbif_host_set_class_filter(uint16_t mask) {
    usbif_host_class_filter = mask;
}

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
    if (usbif_host_tearing_down) {
        // Teardown has started. Do NOT open this device.
        //
        // This is the host_stop() race, found 2026-09-02. Teardown closes
        // every tracked device, then drains client events to retire the
        // asynchronous completions those closes produce. But that same pump
        // also delivers queued USB_HOST_CLIENT_EVENT_NEW_DEV messages, and
        // handling one here opens a device *after* the close loop has already
        // passed over it. Nothing closes it afterwards, so
        // usb_host_client_deregister() -- which requires the client to have
        // closed every device it opened -- refuses with ESP_ERR_INVALID_STATE,
        // ALL_FREE never arrives, uninstall fails, and the next host_start()
        // is poisoned.
        //
        // Measured either side of this gate: teardown fired while a second
        // device was still enumerating gave deregister 0x103, an ALL_FREE
        // timeout and 1584 ms; the same teardown after the bus had settled
        // gave deregister 0x0, ALL_FREE in 10 ms and 556 ms.
        //
        // Declining to open is already an understood outcome here -- the
        // full-slot-table path below returns the same way, and the device
        // simply stays unclaimed. It will be enumerated again by whoever
        // starts the host next.
        return;
    }
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

    uint16_t classes = usbif_class_bits(cfg_desc);
    if ((classes & usbif_host_class_filter) == 0) {
        // Offers none of the classes host_start() was asked for: close it
        // without ever marking the slot in_use, so it is invisible to
        // host_devices() and generates no ATTACH event -- the slot search
        // above finds it free again next time, since it was never claimed.
        usb_host_device_close(usbif_host_client, hdl);
        return;
    }

    slot->in_use = true;
    slot->addr = addr;
    slot->hdl = hdl;
    slot->vid = dev_desc->idVendor;
    slot->pid = dev_desc->idProduct;
    slot->classes = classes;
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
extern void usbif_hid_on_dev_gone(usb_device_handle_t dev);
extern void usbif_msc_on_dev_gone(usb_device_handle_t dev);
extern void usbif_host_midi_on_dev_gone(usb_device_handle_t dev);
extern void usbif_cdc_close(void);
extern void usbif_hid_close(void);
extern void usbif_msc_close(void);
extern void usbif_cdc_close_for_host_stop(void);
extern void usbif_hid_close_for_host_stop(void);
extern void usbif_host_midi_close_for_host_stop(void);
extern void usbif_msc_close_for_host_stop(void);

static void usbif_host_on_dev_gone(usb_device_handle_t hdl) {
    usbif_cdc_on_dev_gone(hdl);
    usbif_hid_on_dev_gone(hdl);
    usbif_msc_on_dev_gone(hdl);
    usbif_host_midi_on_dev_gone(hdl);
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
    usbif_host_tearing_down = true;
    printf("usbif_host: teardown starting (new devices now declined)\n");
    for (int i = 0; i < USBIF_HOST_MAX_DEVS; i++) {
        if (usbif_host_devs[i].in_use) {
            // A class driver that is genuinely open (e.g. HID's interrupt-IN
            // transfer, which resubmits itself from its own completion
            // callback until something clears its `open` flag) never learns
            // host_stop() is happening if we go straight to device_close()
            // -- the transfer keeps re-arming forever, the device can never
            // go idle, and usb_host_device_free_all()'s async completion
            // (what the ALL_FREE wait below is blocking on) never arrives.
            //
            // The FIRST fix attempt called usbif_*_on_dev_gone() here, the
            // same hooks the surprise-detach path uses -- wrong choice,
            // caught by testing rather than review: those hooks free the
            // transfer object without first halting/flushing/clearing its
            // endpoint, which is correct for a detach (the device is
            // already physically gone, so endpoint operations on it would
            // be meaningless) but leaves the library's internal state
            // inconsistent when the device is still attached, which then
            // surfaced downstream as usb_host_client_deregister() failing
            // with ESP_ERR_INVALID_STATE (0x103) instead of the hang moving
            // anywhere. The device IS still attached here -- host_stop() is
            // a voluntary teardown, not a detach -- so the correct call is
            // each driver's ordinary _close(), which does halt+flush+clear
            // before freeing, exactly as if Python had closed it itself.
            // SECOND fix attempt: the plain _close() functions still failed
            // identically even with real wall-clock time in the
            // pre-deregister drain below. Root cause, found by reading
            // esp-idf's usb_host.c directly rather than guessing further:
            // usb_host_endpoint_halt()/flush() are themselves asynchronous
            // -- interface_release() refuses (ESP_ERR_INVALID_STATE) until
            // every endpoint's in-flight URB count is back to zero, and
            // that only happens once something pumps
            // usb_host_client_handle_events(). Live, that's always the host
            // task's own main loop running concurrently in the background;
            // here, that loop has already exited, so nothing pumps it
            // between halt/flush and interface_release inside the plain
            // close(). The _for_host_stop() variants pump explicitly in
            // that exact gap -- safe here specifically because nothing else
            // is servicing this client concurrently at this point. MSC was long
            // excluded on the grounds that its transfers are
            // request/response rather than continuously re-armed, so no URB
            // would be in flight at rest for interface_release() to wait on.
            // Measured 2026-09-02, that was wrong: an MSC session left open
            // made this teardown take 1466 ms instead of ~450 ms (the
            // ALL_FREE wait timing out), uninstall then failed, and the next
            // host_start() returned ESP_ERR_INVALID_STATE. A MIDI session
            // left open across the identical path was clean 3/3, and MIDI's
            // only relevant difference was having a variant. MSC has one now.
            // Diagnostic bracketing (phase0-findings.md): an intermittent
            // hang still shows up roughly 1-in-4 with no printf between
            // "teardown starting" and "pre-deregister drain done" at all,
            // meaning it moved somewhere inside these three calls or the
            // device_close() below rather than being eliminated. Bracket
            // each one so the next hang pins an exact call, not a region.
            printf("usbif_host: closing cdc\n");
            usbif_cdc_close_for_host_stop();
            printf("usbif_host: closing hid\n");
            usbif_hid_close_for_host_stop();
            printf("usbif_host: closing midi\n");
            usbif_host_midi_close_for_host_stop();
            printf("usbif_host: closing msc\n");
            usbif_msc_close_for_host_stop();
            printf("usbif_host: device_close\n");
            usb_host_device_close(usbif_host_client, usbif_host_devs[i].hdl);
            printf("usbif_host: device_close returned\n");
            usbif_host_devs[i].in_use = false;
        }
    }
    // usb_host_client_deregister() requires the client to have closed every
    // device it opened (usb_host.h's own doc comment), and a class driver's
    // close() (or usbif_host_on_new_dev()'s open-then-immediately-close of a
    // class-filter reject) issues endpoint halt/flush/clear and interface
    // release calls whose completions are asynchronous -- retired only when
    // usb_host_lib_handle_events()/usb_host_client_handle_events() are
    // pumped. The outer `while (usbif_host_task_running)` loop above has
    // already exited by this point, so nothing pumps them otherwise.
    //
    // This drain's first version used timeout=0 on every call -- a
    // non-blocking poll, 20 times with no delay between them, done in
    // microseconds. That is fast enough to catch a completion already
    // sitting in the queue (the class-filter-reject case, fixed by this
    // version), but gives the hardware no wall-clock time to actually
    // finish halting/flushing/clearing a live, previously-open endpoint --
    // caught by testing with a genuinely-open device, where deregister and
    // device_free_all both came back ESP_ERR_INVALID_STATE (0x103) even
    // after routing through the correct close() calls above. Real waits,
    // not a zero-timeout spin, up to 200 ms total -- generous next to the
    // ALL_FREE wait below, cheap next to a wedge.
    for (int i = 0; i < 20; i++) {
        uint32_t flags = 0;
        usb_host_lib_handle_events(pdMS_TO_TICKS(10), &flags);
        usb_host_client_handle_events(usbif_host_client, pdMS_TO_TICKS(10));
    }
    printf("usbif_host: pre-deregister drain done\n");
    esp_err_t dereg_err = usb_host_client_deregister(usbif_host_client);
    printf("usbif_host: deregister -> 0x%x\n", (unsigned)dereg_err);
    usbif_host_client = NULL;
    esp_err_t free_all_err = usb_host_device_free_all();
    printf("usbif_host: device_free_all -> 0x%x\n", (unsigned)free_all_err);
    // Diagnostic instrumentation for the host_stop() hang investigation
    // (phase0-findings.md): does ALL_FREE actually arrive, or does this
    // loop silently exhaust its 100-tick (1000 ms) bound every time? The
    // original code proceeded to usb_host_uninstall() either way with no
    // record of which happened -- that silence is exactly what made the
    // outer hang unfalsifiable from the Python side.
    bool all_free_seen = false;
    int free_wait_ticks = 0;
    for (int i = 0; i < 100; i++) {
        uint32_t flags = 0;
        usb_host_lib_handle_events(pdMS_TO_TICKS(10), &flags);
        free_wait_ticks = i + 1;
        if (flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            all_free_seen = true;
            break;
        }
    }
    printf("usbif_host: ALL_FREE %s after %d tick(s) (%d ms)\n",
        all_free_seen ? "seen" : "NOT seen -- timed out", free_wait_ticks, free_wait_ticks * 10);
    esp_err_t uninstall_err = usb_host_uninstall();
    printf("usbif_host: uninstall -> 0x%x\n", (unsigned)uninstall_err);
    // A failed uninstall leaves the library installed, so the next
    // usb_host_install() returns ESP_ERR_INVALID_STATE and host_start() fails
    // for a reason that looks nothing like its cause. Printing this and
    // discarding it is what let a broken teardown be reported to Python as a
    // clean stop, and is why this read for weeks as "a narrow race inside
    // esp-idf" rather than as our own unpumped close. Mark the task wedged so
    // host_start() refuses honestly instead of failing mysteriously later.
    if (uninstall_err != ESP_OK) {
        usbif_host_task_wedged = true;
        printf("usbif_host: uninstall FAILED -- marking wedged; a reboot is "
               "required and host_start() will now say so\n");
    }
    usbif_host_task_handle = NULL;
    printf("usbif_host: task exiting, handle cleared, core %d\n", esp_cpu_get_core_id());
    vTaskDelete(NULL);
}

// The active configuration descriptor of a hosted device, as raw bytes.
//
// Handed to Python whole, including every class-specific descriptor, because
// that is where format parsing belongs. UAC and UVC describe their formats in
// class-specific descriptors whose shape varies by device, and iterating a
// parser for those in C -- reflash, reboot, retry -- against iterating one in
// Python is not a close contest. It also matches this module's own division:
// Python configures and observes, C moves the isochronous bytes.
//
// The pointer is into the host library's own cached descriptor, valid while
// the device stays attached; the caller copies it immediately.
int usbif_host_desc_get(uint32_t dev_id, const uint8_t **out, uint16_t *len) {
    #if USBIF_HAVE_HOST
    usb_device_handle_t dev;
    if (usbif_host_dev_lookup(dev_id, &dev) != 0) {
        return -1;
    }
    const usb_config_desc_t *cfg;
    if (usb_host_get_active_config_descriptor(dev, &cfg) != ESP_OK) {
        return -2;
    }
    *out = (const uint8_t *)cfg;
    *len = cfg->wTotalLength;
    return 0;
    #else
    (void)dev_id; (void)out; (void)len;
    return -1;
    #endif
}

int usbif_host_start_c(void) {
    if (usbif_host_task_wedged) {
        // A previous host_stop() never actually finished -- the old task is
        // still alive somewhere inside a blocking IDF call and holding the
        // core-0 USB interrupt the host library needs. Silently returning 0
        // here (the old behaviour) let every later host_start()/host_stop()
        // pair "succeed" while doing nothing at the C level and reporting
        // host_devices() as permanently empty -- a silent lie, not a no-op.
        // Refuse instead: there is no known in-software recovery, only a
        // reboot clears the wedge (verified: hard-reset restores clean
        // host_stats()).
        printf("usbif_host: refusing host_start() -- a previous host_stop() "
               "never completed; the host task is wedged. Reboot required.\n");
        return -1;
    }
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
    usbif_host_tearing_down = false;
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
    int wait_ticks = 0;
    for (int i = 0; i < 200 && usbif_host_task_handle != NULL; i++) {
        vTaskDelay(1);   // one 10 ms tick; pdMS_TO_TICKS(5) is ZERO ticks here
        wait_ticks = i + 1;
    }
    // The task uninstalled the library itself before exiting: the HCD
    // interrupt lives on the task's core and is freed there.
    //
    // Diagnostic instrumentation for the host_stop() hang investigation
    // (phase0-findings.md hypothesis 1): the original code called
    // tusb_init() unconditionally here with no record of whether the wait
    // above actually succeeded or timed out. Log it explicitly so a hang
    // report can be told apart from "the wait timed out and tusb_init()
    // itself is what's stuck" versus "the wait succeeded and something in
    // tusb_init()/the PHY handoff hangs regardless."
    bool task_exited = (usbif_host_task_handle == NULL);
    if (!task_exited) {
        usbif_host_task_wedged = true;
    }
    printf("usbif_host_stop: task %s after %d tick(s) (%d ms)\n",
        task_exited ? "exited cleanly" : "DID NOT EXIT -- timed out (marked wedged)",
        wait_ticks, wait_ticks * 10);

    #if MICROPY_HW_ENABLE_USBDEV
    // Hand the controller back: device-mode PHY, then the TinyUSB device
    // stack. The board is a CDC device again on its next connection.
    printf("usbif_host_stop: calling usb_phy_otg_device_mode()\n");
    usb_phy_otg_device_mode();
    printf("usbif_host_stop: calling tusb_init()\n");
    tusb_init();
    printf("usbif_host_stop: tusb_init() returned\n");
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
