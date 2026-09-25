#include "MissionFsm.h"
#include "navigation/Navigator.h"
#include "config.h"
#include <math.h>

MissionFsm::MissionFsm()
    : _state(STATE_STANDBY),
      _forceSpiralCooldownStartMs(0),
      _forceSpiralCooldownStarted(false) {}

bool MissionFsm::latchDeparted(bool alreadyLeft, float distHomeM,
                               uint32_t& outsideAccumMs, uint32_t dtMs) {
    if (alreadyLeft) {
        return true;
    }
    if (!isfinite(distHomeM) || distHomeM < RTH_DEPART_RADIUS_M) {
        outsideAccumMs = 0;
        return false;
    }
    outsideAccumMs += dtMs;
    return outsideAccumMs >= RTH_DEPART_DWELL_MS;
}

bool MissionFsm::isArrivedHome(bool homeSet, bool fixValid, bool departed,
                               float distHomeM, float rthEntryDistM,
                               uint32_t& insideAccumMs, uint32_t dtMs) {
    if (!homeSet || !fixValid || !departed) {
        insideAccumMs = 0;
        return false;
    }
    if (!isfinite(distHomeM) || !isfinite(rthEntryDistM)) {
        insideAccumMs = 0;
        return false;
    }
    // Do not finish an RTH that started on the SET_HOME GPS fix.
    if (rthEntryDistM <= RTH_ARRIVE_RADIUS_M) {
        insideAccumMs = 0;
        return false;
    }
    if (distHomeM > RTH_ARRIVE_RADIUS_M) {
        insideAccumMs = 0;
        return false;
    }
    insideAccumMs += dtMs;
    return insideAccumMs >= RTH_MIN_HOLD_MS;
}

void MissionFsm::homeCaptureReset(HomeCapture& cap) {
    cap.sumLat = 0.0;
    cap.sumLon = 0.0;
    cap.minLat = 0.0;
    cap.maxLat = 0.0;
    cap.minLon = 0.0;
    cap.maxLon = 0.0;
    cap.count = 0;
}

void MissionFsm::homeCaptureAdd(HomeCapture& cap, double lat, double lon) {
    if (cap.count == 0) {
        cap.minLat = cap.maxLat = lat;
        cap.minLon = cap.maxLon = lon;
    } else {
        if (lat < cap.minLat) cap.minLat = lat;
        if (lat > cap.maxLat) cap.maxLat = lat;
        if (lon < cap.minLon) cap.minLon = lon;
        if (lon > cap.maxLon) cap.maxLon = lon;
    }
    cap.sumLat += lat;
    cap.sumLon += lon;
    cap.count++;
}

bool MissionFsm::homeCaptureMean(const HomeCapture& cap, double& lat, double& lon) {
    if (cap.count == 0) return false;
    lat = cap.sumLat / (double)cap.count;
    lon = cap.sumLon / (double)cap.count;
    return true;
}

float MissionFsm::homeCaptureSpreadM(const HomeCapture& cap) {
    if (cap.count < 2) return 0.0f;
    return (float)Navigator::calculateDistance(
        cap.minLat, cap.minLon, cap.maxLat, cap.maxLon);
}

void MissionFsm::begin() {
    _state = STATE_STANDBY;
    _forceSpiralCooldownStartMs = 0;
    _forceSpiralCooldownStarted = false;
}

bool MissionFsm::forceSpiralCooldownActive(uint32_t nowMs) const {
    return _forceSpiralCooldownStarted &&
           (uint32_t)(nowMs - _forceSpiralCooldownStartMs) <
               FORCE_SPIRAL_COOLDOWN_MS;
}

void MissionFsm::startForceSpiralCooldown(uint32_t nowMs) {
    _forceSpiralCooldownStartMs = nowMs;
    _forceSpiralCooldownStarted = true;
}

BoatState MissionFsm::tick(const FsmInputs& in) {
    if (in.emergencyCommand || in.gpsFault ||
        in.safety == SafetyState::CRITICAL_STOP ||
        in.safety == SafetyState::OVERCURRENT_STOP ||
        in.safety == SafetyState::LEAK_STOP) {
        if (_state == STATE_SPIRAL_MAPPING)
            startForceSpiralCooldown(in.nowMs);
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
        if (_state == STATE_SPIRAL_MAPPING)
            startForceSpiralCooldown(in.nowMs);
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
            if ((in.anomalyDetected ||
                 (in.forceSpiralCommand &&
                  !forceSpiralCooldownActive(in.nowMs))) &&
                in.passedFirstWaypoint)
                _state = STATE_SPIRAL_MAPPING;
            else if (in.gridComplete) _state = in.homeSet ? STATE_RTH : STATE_STANDBY;
            break;

        case STATE_SPIRAL_MAPPING:
            if (in.spiralComplete || in.spiralAborted) {
                startForceSpiralCooldown(in.nowMs);
                _state = STATE_GRID_PATROL;
            }
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
