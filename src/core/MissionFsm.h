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
    bool        startCommand;    // operator START
    bool        emergencyCommand;// operator Emergency Stop
    bool        forceRthCommand; // operator Force RTH
    bool        anomalyDetected; // WaterAnomalyMonitor::shouldTriggerSpiral
    bool        spiralComplete;  // SpiralPlanner finished all turns
    bool        gridComplete;    // Navigator ran out of patrol waypoints
    bool        arrivedHome;     // reached the home waypoint during RTH
    SafetyState safety;          // aggregated safety verdict
};

// 5-state mission finite state machine (codebase_mathematical_models.md).
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
