#include "MissionFsm.h"

MissionFsm::MissionFsm() : _state(STATE_STANDBY) {}

void MissionFsm::begin() { _state = STATE_STANDBY; }

BoatState MissionFsm::tick(const FsmInputs& in) {
    // --- Highest priority: emergency / critical safety from ANY state ---
    if (in.emergencyCommand || in.gpsFault ||
        in.safety == SafetyState::CRITICAL_STOP ||
        in.safety == SafetyState::OVERCURRENT_STOP ||
        in.safety == SafetyState::LEAK_STOP) {
        _state = STATE_EMERGENCY_STOP;
        return _state;
    }

    // --- RTH arrival must be checked BEFORE re-asserting RTH triggers ---
    // Otherwise low-battery / latched Force-RTH would trap the boat forever.
    if (_state == STATE_RTH && in.arrivedHome) {
        _state = STATE_STANDBY;
        return _state;
    }

    // --- Force RTH / low-battery RTH from active non-RTH states only ---
    if ((in.forceRthCommand || in.safety == SafetyState::LOW_BATTERY_RTH) &&
        _state != STATE_STANDBY &&
        _state != STATE_EMERGENCY_STOP &&
        _state != STATE_RTH) {
        if (in.homeSet) {
            _state = STATE_RTH;
        } else {
            // No home known — safer to stop than sail toward (0,0).
            _state = STATE_EMERGENCY_STOP;
        }
        return _state;
    }

    // --- Normal transition table ---
    switch (_state) {
        case STATE_STANDBY:
            // Require explicit START_ARM + GPS + compass + loaded waypoints.
            if (in.startCommand &&
                in.safety == SafetyState::NORMAL &&
                in.gpsOk && in.compassOk && in.hasWaypoints)
                _state = STATE_GRID_PATROL;
            break;

        case STATE_GRID_PATROL:
            if (in.anomalyDetected)   _state = STATE_SPIRAL_MAPPING;
            else if (in.gridComplete) _state = in.homeSet ? STATE_RTH : STATE_STANDBY;
            break;

        case STATE_SPIRAL_MAPPING:
            if (in.spiralComplete) _state = STATE_GRID_PATROL;
            break;

        case STATE_RTH:
            // arrivedHome handled above; stay in RTH until home reached
            break;

        case STATE_EMERGENCY_STOP:
            // Latching: only explicit CLEAR_ESTOP with NORMAL safety re-arms.
            // startCommand alone must NOT clear E-Stop.
            if (in.clearEstop && !in.gpsFault &&
                in.safety == SafetyState::NORMAL)
                _state = STATE_STANDBY;
            break;
    }
    return _state;
}
