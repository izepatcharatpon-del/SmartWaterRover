#include "SafetyManager.h"
#include "config.h"

static const unsigned long OVERCURRENT_DEBOUNCE_MS = 500;

SafetyManager::SafetyManager()
    : _state(SafetyState::NORMAL), _overcurrentSince(0) {}

SafetyState SafetyManager::evaluate(float batteryV, float currentA, bool leak) {
    // Evaluate from most to least severe; first match wins.

    // 1) Critical battery -> hard stop (latching, most severe).
    if (batteryV <= VOLTAGE_CRITICAL) {
        _state = SafetyState::CRITICAL_STOP;
        return _state;
    }

    // 2) Hull leak -> stop.
    if (leak) {
        _state = SafetyState::LEAK_STOP;
        return _state;
    }

    // 3) Overcurrent with debounce (ignore momentary inrush).
    if (currentA >= OVERCURRENT_LIMIT_A) {
        if (_overcurrentSince == 0) _overcurrentSince = millis();
        if (millis() - _overcurrentSince >= OVERCURRENT_DEBOUNCE_MS) {
            _state = SafetyState::OVERCURRENT_STOP;
            return _state;
        }
    } else {
        _overcurrentSince = 0;
    }

    // 4) Low battery -> return home (with small hysteresis to avoid chatter).
    if (batteryV <= VOLTAGE_RTH) {
        _state = SafetyState::LOW_BATTERY_RTH;
        return _state;
    }
    if (_state == SafetyState::LOW_BATTERY_RTH && batteryV < VOLTAGE_RTH + 0.2f) {
        return _state;   // stay in RTH until recovered above threshold+hysteresis
    }

    _state = SafetyState::NORMAL;
    return _state;
}
