#include "SafetyManager.h"
#include "config.h"

SafetyManager::SafetyManager()
    : _state(SafetyState::NORMAL) {}

SafetyState SafetyManager::evaluate(float batteryV, bool underPropulsionLoad) {
    const float rthThresh =
        underPropulsionLoad ? VOLTAGE_RTH_LOADED : VOLTAGE_RTH;

    if (batteryV <= VOLTAGE_CRITICAL) {
        _state = SafetyState::CRITICAL_STOP;
        return _state;
    }

    // CRITICAL latches until pack recovers above idle RTH + hysteresis.
    if (_state == SafetyState::CRITICAL_STOP) {
        if (batteryV < VOLTAGE_RTH + VOLTAGE_RECOVER_HYST) {
            return _state;
        }
    }

    if (batteryV <= rthThresh) {
        _state = SafetyState::LOW_BATTERY_RTH;
        return _state;
    }
    if (_state == SafetyState::LOW_BATTERY_RTH &&
        batteryV < VOLTAGE_RTH + VOLTAGE_RECOVER_HYST) {
        return _state;
    }

    _state = SafetyState::NORMAL;
    return _state;
}
