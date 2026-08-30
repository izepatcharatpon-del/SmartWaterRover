#ifndef USV_MISSION_FSM_H
#define USV_MISSION_FSM_H

#include <stdint.h>
#include "../safety/SafetyManager.h"

// Mission states (value doubles as TelemetryPacket.current_mode).
enum BoatState : uint8_t {
    STATE_STANDBY        = 0,
    STATE_GRID_PATROL    = 1,
    STATE_SPIRAL_MAPPING = 2,
    STATE_RTH            = 3,
    STATE_EMERGENCY_STOP = 4
};

// Decoupled inputs so the FSM has no hardware dependencies (unit-testable).
struct FsmInputs {
    bool        startCommand;     // CMD_START_ARM
    bool        clearEstop;       // CMD_CLEAR_ESTOP
    bool        emergencyCommand; // CMD_EMERGENCY_STOP
    bool        forceRthCommand;  // CMD_FORCE_RTH
    bool        forceSpiralCommand; // CMD_FORCE_SPIRAL (GRID only, one-shot)
    bool        anomalyDetected;  // WaterAnomalyMonitor (after hysteresis)
    bool        spiralComplete;   // SpiralPlanner finished all turns
    bool        gridComplete;     // Navigator ran out of patrol waypoints
    bool        arrivedHome;      // reached home during RTH
    bool        gpsOk;            // GPS usable for navigation
    bool        gpsFault;         // prolonged, latched GPS loss
    bool        homeSet;          // home waypoint captured this mission
    bool        compassOk;        // heading usable
    bool        hasWaypoints;     // Navigator mission ready (upload sealed)
    SafetyState safety;           // aggregated safety verdict
};

// 5-state mission finite state machine.
class MissionFsm {
public:
    MissionFsm();

    void      begin();
    BoatState tick(const FsmInputs& in);
    BoatState state() const { return _state; }

private:
    BoatState _state;
};

#endif // USV_MISSION_FSM_H
