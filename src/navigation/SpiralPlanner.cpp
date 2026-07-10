#include "SpiralPlanner.h"
#include "config.h"
#include <math.h>

static const double DEG2RAD_SP = M_PI / 180.0;
static const double METERS_PER_DEG_LAT = 111320.0;

SpiralPlanner::SpiralPlanner()
    : _centerLat(0.0), _centerLon(0.0), _theta(0.0), _complete(false) {}

void SpiralPlanner::begin(double centerLat, double centerLon) {
    _centerLat = centerLat;
    _centerLon = centerLon;
    _theta     = 0.0;
    _complete  = false;
}

void SpiralPlanner::reset() {
    _theta    = 0.0;
    _complete = false;
}

Waypoint SpiralPlanner::calculateSpiralOffset(double centerLat, double centerLon,
                                              double radiusM, double thetaRad) {
    double dxEast  = radiusM * cos(thetaRad);   // meters east
    double dyNorth = radiusM * sin(thetaRad);   // meters north

    double dLat = dyNorth / METERS_PER_DEG_LAT;
    double dLon = dxEast / (METERS_PER_DEG_LAT * cos(centerLat * DEG2RAD_SP));

    Waypoint wp;
    wp.lat = centerLat + dLat;
    wp.lon = centerLon + dLon;
    return wp;
}

bool SpiralPlanner::nextSpiralPoint(Waypoint& out) {
    if (_complete) return false;

    double r = SPIRAL_A_M + SPIRAL_B_M * _theta;
    if (r > SPIRAL_MAX_RADIUS_M || _theta > SPIRAL_MAX_THETA) {
        _complete = true;
        return false;
    }

    out = calculateSpiralOffset(_centerLat, _centerLon, r, _theta);
    _theta += SPIRAL_DTHETA;
    return true;
}
