#include "Watchdog.h"
#include <esp_task_wdt.h>

bool Watchdog::begin(uint32_t timeoutSec) {
    // Arduino-ESP32 may already have initialised TWDT — treat that as OK.
    esp_err_t err = esp_task_wdt_init(timeoutSec, true);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        _initialised = false;
        return false;
    }
    _initialised = true;
    return true;
}

void Watchdog::subscribeCurrentTask() {
    if (!_initialised) return;
    esp_task_wdt_add(NULL);   // NULL = current task
}

void Watchdog::feed() {
    if (!_initialised) return;
    esp_task_wdt_reset();
}
