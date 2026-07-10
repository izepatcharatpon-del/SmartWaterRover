#ifndef USV_WATCHDOG_H
#define USV_WATCHDOG_H

#include <Arduino.h>

// Thin wrapper over the ESP-IDF task watchdog (esp_task_wdt). Each RTOS task
// that could hang subscribes itself and must feed() within the timeout, or the
// chip resets. See project_prerequisites.md 6.1.
class Watchdog {
public:
    // Initialise the TWDT once (panic reset on timeout).
    void begin(uint32_t timeoutSec = 3);

    // Register the CALLING task with the watchdog.
    void subscribeCurrentTask();

    // Kick the watchdog for the calling task (call each loop iteration).
    void feed();

private:
    bool _initialised = false;
};

#endif // USV_WATCHDOG_H
