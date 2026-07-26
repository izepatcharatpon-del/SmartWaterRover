#ifndef USV_SPIRAL_PLANNER_H
#define USV_SPIRAL_PLANNER_H

#include "Packets.h"

// Archimedean spiral waypoints (r = a + b*theta) around a GPS center.
class SpiralPlanner {
public:
    SpiralPlanner();

    void begin(double centerLat, double centerLon);
    bool nextSpiralPoint(Waypoint& out);  // false when complete

    static Waypoint calculateSpiralOffset(double centerLat, double centerLon,
                                          double radiusM, double thetaRad);

private:
    double _centerLat, _centerLon;
    double _theta;
    bool   _complete;
};

#endif // USV_SPIRAL_PLANNER_H
