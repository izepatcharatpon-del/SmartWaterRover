#include "Watchdog.h"
#include <esp_task_wdt.h>

void Watchdog::begin(uint32_t timeoutSec) {
    // Arduino-ESP32 core 2.x uses esp_task_wdt_init directly.
    esp_task_wdt_init(timeoutSec, true);
    _initialised = true;
}

void Watchdog::subscribeCurrentTask() {
    if (!_initialised) return;
    esp_task_wdt_add(NULL);   // NULL = current task
}

void Watchdog::feed() {
    esp_task_wdt_reset();
}
