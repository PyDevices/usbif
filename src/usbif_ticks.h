// Waits in milliseconds, whatever the board's FreeRTOS tick rate.
//
// A loop of vTaskDelay(1) calls bounded by a count is a wait in ticks, and a
// tick is 10 ms only at CONFIG_FREERTOS_HZ=100. Boards that run at 1000 Hz
// (the Waveshare ESP32-S3-Touch-LCD-7's firmware does, so sleep_ms() under
// 10 ms stops busy-waiting) cut every such wait to a tenth. host_stop() gave
// teardown 200 ms instead of 2 s, and an ordinary stop was reported as a
// wedge (usbif#29). Bound waits with USBIF_MS_TICKS() instead.
#ifndef USBIF_TICKS_H
#define USBIF_TICKS_H

#include "freertos/FreeRTOS.h"

// At least one tick: pdMS_TO_TICKS() truncates, and vTaskDelay(0) doesn't
// block at all.
#define USBIF_MS_TICKS(ms) ((pdMS_TO_TICKS(ms) > 0) ? pdMS_TO_TICKS(ms) : 1)

// Ticks back to milliseconds, for logs.
#define USBIF_TICKS_MS(ticks) ((int)((ticks) * portTICK_PERIOD_MS))

#endif // USBIF_TICKS_H
