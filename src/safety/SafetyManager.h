#ifndef USV_SAFETY_MANAGER_H
#define USV_SAFETY_MANAGER_H

#include <stdint.h>

// Voltage-based safety only (AA118). Overcurrent/leak reserved for future HW.
enum class SafetyState {
    NORMAL           = 0,
    LOW_BATTERY_RTH  = 1,
    OVERCURRENT_STOP = 2,   // reserved — not produced
    LEAK_STOP        = 3,   // reserved — not produced
    CRITICAL_STOP    = 4
};

class SafetyManager {
public:
    SafetyManager();

    // underPropulsionLoad: pack sag expected — use VOLTAGE_RTH_LOADED.
    SafetyState evaluate(float batteryV, bool underPropulsionLoad = false);

private:
    SafetyState _state;
};

#endif // USV_SAFETY_MANAGER_H
