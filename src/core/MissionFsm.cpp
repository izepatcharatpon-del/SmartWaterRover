#include "MissionFsm.h"

MissionFsm::MissionFsm() : _state(STATE_STANDBY) {}

void MissionFsm::begin() { _state = STATE_STANDBY; }

BoatState MissionFsm::tick(const FsmInputs& in) {
    // --- Highest priority: emergency / critical safety from ANY state ---
    if (in.emergencyCommand ||
        in.safety == SafetyState::CRITICAL_STOP ||
        in.safety == SafetyState::OVERCURRENT_STOP ||
        in.safety == SafetyState::LEAK_STOP) {
        _state = STATE_EMERGENCY_STOP;
        return _state;
    }

    // --- Force RTH / low-battery RTH from any active state ---
    if ((in.forceRthCommand || in.safety == SafetyState::LOW_BATTERY_RTH) &&
        _state != STATE_STANDBY && _state != STATE_EMERGENCY_STOP) {
        _state = STATE_RTH;
        return _state;
    }

    // --- Normal transition table ---
    switch (_state) {
        case STATE_STANDBY:
            if (in.startCommand) _state = STATE_GRID_PATROL;
            break;

        case STATE_GRID_PATROL:
            if (in.anomalyDetected)   _state = STATE_SPIRAL_MAPPING;
            else if (in.gridComplete) _state = STATE_RTH;
            break;

        case STATE_SPIRAL_MAPPING:
            if (in.spiralComplete) _state = STATE_GRID_PATROL;  // resume patrol
            break;

        case STATE_RTH:
            if (in.arrivedHome) _state = STATE_STANDBY;
            break;

        case STATE_EMERGENCY_STOP:
            // Latching: only a fresh START from a cleared state re-arms.
            if (in.startCommand && in.safety == SafetyState::NORMAL)
                _state = STATE_STANDBY;
            break;
    }
    return _state;
}
