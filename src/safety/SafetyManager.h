#ifndef USV_SAFETY_MANAGER_H
#define USV_SAFETY_MANAGER_H

#include <Arduino.h>

// Priority-ordered safety states. Higher value = more severe.
enum class SafetyState {
    NORMAL           = 0,
    LOW_BATTERY_RTH  = 1,   // battery <= VOLTAGE_RTH -> return home
    OVERCURRENT_STOP = 2,   // motor stall / short -> cut thrust
    LEAK_STOP        = 3,   // hull water ingress detected
    CRITICAL_STOP    = 4    // battery <= VOLTAGE_CRITICAL -> hard stop
};

// Aggregates hardware guards (battery voltage, ACS712 current, leak sensor)
// into a single actionable state with hysteresis on the battery threshold.
class SafetyManager {
public:
    SafetyManager();

    // currentA: measured motor current (A). leak: true if water detected.
    SafetyState evaluate(float batteryV, float currentA, bool leak);

    SafetyState state() const { return _state; }
    bool isEmergency() const {
        return _state == SafetyState::OVERCURRENT_STOP ||
               _state == SafetyState::LEAK_STOP ||
               _state == SafetyState::CRITICAL_STOP;
    }
    bool needsReturnHome() const { return _state == SafetyState::LOW_BATTERY_RTH; }

private:
    SafetyState   _state;
    unsigned long _overcurrentSince;   // debounce transient current spikes
};

#endif // USV_SAFETY_MANAGER_H
