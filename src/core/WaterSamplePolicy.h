#ifndef USV_WATER_SAMPLE_POLICY_H
#define USV_WATER_SAMPLE_POLICY_H

class WaterSamplePolicy {
public:
    static bool crossedBoundary(int beforeIndex, int afterIndex,
                                int waypointCount, int everyNWaypoints);

    // A final waypoint that is also a sample boundary must finish its quiet
    // window before MissionFsm is allowed to leave GRID for RTH/STANDBY.
    static bool missionCompleteForFsm(bool missionComplete,
                                      bool sampleActive);
};

#endif // USV_WATER_SAMPLE_POLICY_H
