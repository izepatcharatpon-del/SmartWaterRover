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

Waypoint SpiralPlanner::calculateSpiralOffset(double centerLat, double centerLon,
                                              double radiusM, double thetaRad) {
    double dxEast  = radiusM * cos(thetaRad);
    double dyNorth = radiusM * sin(thetaRad);

    double dLat = dyNorth / METERS_PER_DEG_LAT;
    double cosLat = cos(centerLat * DEG2RAD_SP);
    if (fabs(cosLat) < 1e-6) cosLat = (cosLat >= 0.0) ? 1e-6 : -1e-6;
    double dLon = dxEast / (METERS_PER_DEG_LAT * cosLat);

    Waypoint wp;
    wp.lat = centerLat + dLat;
    wp.lon = centerLon + dLon;
    while (wp.lon > 180.0)  wp.lon -= 360.0;
    while (wp.lon < -180.0) wp.lon += 360.0;
    return wp;
}

bool SpiralPlanner::nextSpiralPoint(Waypoint& out) {
    if (_complete) return false;

    const double eps = 1e-3;
    if (_theta > SPIRAL_MAX_THETA + eps) {
        _complete = true;
        return false;
    }

    double theta = _theta > SPIRAL_MAX_THETA ? SPIRAL_MAX_THETA : _theta;
    double r = SPIRAL_A_M + SPIRAL_B_M * theta;
    if (r > SPIRAL_MAX_RADIUS_M) r = SPIRAL_MAX_RADIUS_M;

    out = calculateSpiralOffset(_centerLat, _centerLon, r, theta);
    _theta += SPIRAL_DTHETA;
    return true;
}
