#include "MissionFsm.h"

MissionFsm::MissionFsm() : _state(STATE_STANDBY) {}

void MissionFsm::begin() { _state = STATE_STANDBY; }

BoatState MissionFsm::tick(const FsmInputs& in) {
    if (in.emergencyCommand || in.gpsFault ||
        in.safety == SafetyState::CRITICAL_STOP ||
        in.safety == SafetyState::OVERCURRENT_STOP ||
        in.safety == SafetyState::LEAK_STOP) {
        _state = STATE_EMERGENCY_STOP;
        return _state;
    }

    // Check arrival before re-applying RTH, or low battery loops forever.
    if (_state == STATE_RTH && in.arrivedHome) {
        _state = STATE_STANDBY;
        return _state;
    }

    if ((in.forceRthCommand || in.safety == SafetyState::LOW_BATTERY_RTH) &&
        _state != STATE_STANDBY &&
        _state != STATE_EMERGENCY_STOP &&
        _state != STATE_RTH) {
        if (in.homeSet) {
            _state = STATE_RTH;
        } else {
            _state = STATE_EMERGENCY_STOP;
        }
        return _state;
    }

    switch (_state) {
        case STATE_STANDBY:
            if (in.startCommand &&
                in.safety == SafetyState::NORMAL &&
                in.gpsOk && in.compassOk && in.hasWaypoints)
                _state = STATE_GRID_PATROL;
            break;

        case STATE_GRID_PATROL:
            if (in.anomalyDetected || in.forceSpiralCommand)
                _state = STATE_SPIRAL_MAPPING;
            else if (in.gridComplete) _state = in.homeSet ? STATE_RTH : STATE_STANDBY;
            break;

        case STATE_SPIRAL_MAPPING:
            if (in.spiralComplete) _state = STATE_GRID_PATROL;
            break;

        case STATE_RTH:
            break;

        case STATE_EMERGENCY_STOP:
            if (in.clearEstop && !in.gpsFault &&
                in.safety == SafetyState::NORMAL)
                _state = STATE_STANDBY;
            break;
    }
    return _state;
}
