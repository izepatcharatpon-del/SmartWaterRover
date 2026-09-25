#include "WaterSamplePolicy.h"

bool WaterSamplePolicy::crossedBoundary(int beforeIndex, int afterIndex,
                                        int waypointCount,
                                        int everyNWaypoints) {
    if (everyNWaypoints <= 0 || waypointCount <= 0 || beforeIndex < 0 ||
        afterIndex <= beforeIndex || afterIndex > waypointCount) {
        return false;
    }
    const int nextBoundary =
        ((beforeIndex / everyNWaypoints) + 1) * everyNWaypoints;
    return nextBoundary <= afterIndex && nextBoundary <= waypointCount;
}

bool WaterSamplePolicy::missionCompleteForFsm(bool missionComplete,
                                              bool sampleActive) {
    return missionComplete && !sampleActive;
}
