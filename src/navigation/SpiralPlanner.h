#ifndef USV_SPIRAL_PLANNER_H
#define USV_SPIRAL_PLANNER_H

#include "Packets.h"

// Generates Archimedean spiral waypoints (r = a + b*theta) around a center
// GPS coordinate, converting local polar offsets to lat/lon. Pure math.
// Bounded by SPIRAL_MAX_RADIUS_M / SPIRAL_MAX_THETA (config.h).
class SpiralPlanner {
public:
    SpiralPlanner();

    void begin(double centerLat, double centerLon);
    void reset();

    // Fetch the next spiral waypoint; returns false when spiral is complete.
    bool nextSpiralPoint(Waypoint& out);

    // Convert a local (radius, theta) polar offset to an absolute waypoint.
    static Waypoint calculateSpiralOffset(double centerLat, double centerLon,
                                          double radiusM, double thetaRad);

    bool complete() const { return _complete; }

private:
    double _centerLat, _centerLon;
    double _theta;
    bool   _complete;
};

#endif // USV_SPIRAL_PLANNER_H
