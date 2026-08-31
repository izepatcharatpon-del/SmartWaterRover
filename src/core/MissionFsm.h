#ifndef USV_MISSION_FSM_H
#define USV_MISSION_FSM_H

#include <stdint.h>
#include "../safety/SafetyManager.h"

enum BoatState : uint8_t {
    STATE_STANDBY        = 0,
    STATE_GRID_PATROL    = 1,
    STATE_SPIRAL_MAPPING = 2,
    STATE_RTH            = 3,
    STATE_EMERGENCY_STOP = 4
};

struct FsmInputs {
    bool        startCommand;
    bool        clearEstop;
    bool        emergencyCommand;
    bool        forceRthCommand;
    bool        forceSpiralCommand;
    bool        anomalyDetected;
    bool        spiralComplete;
    bool        gridComplete;
    bool        arrivedHome;
    bool        gpsOk;
    bool        gpsFault;
    bool        homeSet;
    bool        compassOk;
    bool        hasWaypoints;
    SafetyState safety;
};

class MissionFsm {
public:
    MissionFsm();

    void      begin();
    BoatState tick(const FsmInputs& in);
    BoatState state() const { return _state; }

    // Latch "left home" only after a dwell outside RTH_DEPART — one GPS spike must not count.
    static bool latchDeparted(bool alreadyLeft, float distHomeM,
                              uint32_t& outsideAccumMs, uint32_t dtMs);
    // True only after a dwell inside RTH_ARRIVE of the stored SET_HOME fix.
    static bool isArrivedHome(bool homeSet, bool fixValid, bool departed,
                              float distHomeM, float rthEntryDistM,
                              uint32_t& insideAccumMs, uint32_t dtMs);

private:
    BoatState _state;
};

#endif // USV_MISSION_FSM_H
