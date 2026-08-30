// SPDX-License-Identifier: MIT
//
// The C pump: move received USB audio to an I2S sink without the interpreter
// in the path.
//
// This is the arrangement the vision asks for -- "Python configures and
// observes; C moves isochronous bytes" -- and the division of labour is
// deliberate. usbif owns the I2S channel because it owns the isochronous
// stream feeding it. It does not own the codec: bringing up an ES8311 over
// I2C, enabling a speaker amplifier, choosing a sample rate are board
// decisions, and they stay in Python beside every other board decision.
//
// MCLK is deliberately not driven here. On this board board_peripherals
// already generates it with a PWM before the codec is enabled; taking it over
// would mean two peripherals fighting for one pin for no gain.
//
// Honest note on necessity: the Python pump reaches 96% of the offered stream
// once the FIFO is sized properly, so this is an optimisation rather than a
// rescue. What it buys is headroom -- the interpreter can be busy elsewhere,
// or garbage collecting, without the audio noticing.

#include "py/mpconfig.h"

#if MICROPY_HW_ENABLE_USBDEV
#include "tusb.h"
#endif

#if defined(CFG_TUD_AUDIO) && CFG_TUD_AUDIO

#include "driver/i2s_std.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "py/runtime.h"

// One 20 ms block at 24 kHz mono 16-bit, the same size the Python pump found
// worked well: large enough that per-write overhead is irrelevant, small
// enough to stay far inside the FIFO.
#define USBIF_PUMP_BLOCK (960)

// Above MicroPython's task so audio is not held up by whatever the
// interpreter is doing, and below TinyUSB's so servicing the endpoint always
// wins over draining it.
#define USBIF_PUMP_TASK_PRIO (10)
#define USBIF_PUMP_TASK_STACK (4096)

static i2s_chan_handle_t usbif_i2s_tx;
static TaskHandle_t usbif_pump_task_handle;
static volatile bool usbif_pump_running;

uint32_t usbif_pump_bytes;
uint32_t usbif_pump_idle;
uint32_t usbif_pump_timeouts;

static void usbif_pump_task(void *arg) {
    (void)arg;
    static uint8_t block[USBIF_PUMP_BLOCK];

    while (usbif_pump_running) {
        uint16_t count = tud_audio_read(block, sizeof(block));
        if (count == 0) {
            // Nothing buffered: the host is not streaming, or has not filled a
            // block yet. Note that a tick here is 10 ms, not 1 -- FreeRTOS runs
            // at 100 Hz on this port -- so this is a real sleep, not a spin.
            // 10 ms is well inside the FIFO's depth at any rate we advertise.
            usbif_pump_idle++;
            vTaskDelay(1);
            continue;
        }
        size_t written = 0;
        esp_err_t err = i2s_channel_write(usbif_i2s_tx, block, count, &written,
            pdMS_TO_TICKS(100));
        if (err == ESP_ERR_TIMEOUT) {
            // The sink is not draining. Dropping is right: this is realtime
            // audio, and stale samples are worth less than the next ones.
            usbif_pump_timeouts++;
        }
        usbif_pump_bytes += written;
    }

    usbif_pump_task_handle = NULL;
    vTaskDelete(NULL);
}

void usbif_pump_stop(void) {
    if (!usbif_pump_running) {
        return;
    }
    usbif_pump_running = false;
    // Let the task observe the flag and exit before the channel goes away.
    for (int i = 0; i < 50 && usbif_pump_task_handle; i++) {
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    if (usbif_i2s_tx) {
        i2s_channel_disable(usbif_i2s_tx);
        i2s_del_channel(usbif_i2s_tx);
        usbif_i2s_tx = NULL;
    }
}

// Returns an esp_err_t; the caller raises. Pins and format come from Python
// because they are board facts, not usbif's to assume.
int usbif_pump_start(int i2s_id, int bclk, int ws, int dout,
    uint32_t rate, int bits, int channels) {
    if (usbif_pump_running) {
        return ESP_ERR_INVALID_STATE;
    }

    i2s_chan_config_t chan_config = I2S_CHANNEL_DEFAULT_CONFIG(i2s_id, I2S_ROLE_MASTER);
    chan_config.auto_clear = true;   // send zeros on underrun rather than stale data
    esp_err_t err = i2s_new_channel(&chan_config, &usbif_i2s_tx, NULL);
    if (err != ESP_OK) {
        return err;
    }

    i2s_std_slot_config_t slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
        bits == 32 ? I2S_DATA_BIT_WIDTH_32BIT :
        (bits == 24 ? I2S_DATA_BIT_WIDTH_24BIT : I2S_DATA_BIT_WIDTH_16BIT),
        channels == 1 ? I2S_SLOT_MODE_MONO : I2S_SLOT_MODE_STEREO);
    // Both slots even in mono, matching machine.I2S: the codec expects a
    // frame, and a mono source is duplicated into it.
    slot_cfg.slot_mask = I2S_STD_SLOT_BOTH;

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(rate),
        .slot_cfg = slot_cfg,
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,   // board_peripherals drives MCLK
            .bclk = bclk,
            .ws = ws,
            .dout = dout,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = { false, false, false },
        },
    };

    err = i2s_channel_init_std_mode(usbif_i2s_tx, &std_cfg);
    if (err == ESP_OK) {
        err = i2s_channel_enable(usbif_i2s_tx);
    }
    if (err != ESP_OK) {
        i2s_del_channel(usbif_i2s_tx);
        usbif_i2s_tx = NULL;
        return err;
    }

    usbif_pump_bytes = 0;
    usbif_pump_idle = 0;
    usbif_pump_timeouts = 0;
    usbif_pump_running = true;

    if (xTaskCreate(usbif_pump_task, "usbif_uac", USBIF_PUMP_TASK_STACK, NULL,
        USBIF_PUMP_TASK_PRIO, &usbif_pump_task_handle) != pdPASS) {
        usbif_pump_running = false;
        i2s_channel_disable(usbif_i2s_tx);
        i2s_del_channel(usbif_i2s_tx);
        usbif_i2s_tx = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

bool usbif_pump_is_running(void) {
    return usbif_pump_running;
}

#endif // CFG_TUD_AUDIO
