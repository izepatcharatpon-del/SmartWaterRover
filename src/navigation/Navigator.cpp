#include "Navigator.h"
#include <math.h>

static const double EARTH_RADIUS_M = 6371000.0;
static const double DEG2RAD = M_PI / 180.0;
static const double RAD2DEG = 180.0 / M_PI;

Navigator::Navigator() : _count(0), _index(0) {}

double Navigator::calculateDistance(double lat1, double lon1,
                                    double lat2, double lon2) {
    double dLat = (lat2 - lat1) * DEG2RAD;
    double dLon = (lon2 - lon1) * DEG2RAD;
    double a = sin(dLat / 2.0) * sin(dLat / 2.0) +
               cos(lat1 * DEG2RAD) * cos(lat2 * DEG2RAD) *
               sin(dLon / 2.0) * sin(dLon / 2.0);
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    return EARTH_RADIUS_M * c;
}

double Navigator::calculateBearing(double lat1, double lon1,
                                   double lat2, double lon2) {
    double phi1 = lat1 * DEG2RAD;
    double phi2 = lat2 * DEG2RAD;
    double dLon = (lon2 - lon1) * DEG2RAD;
    double y = sin(dLon) * cos(phi2);
    double x = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(dLon);
    double bearing = atan2(y, x) * RAD2DEG;
    if (bearing < 0.0) bearing += 360.0;
    return bearing;
}

void Navigator::clear() {
    _count = 0;
    _index = 0;
}

void Navigator::loadWaypoints(const WaypointArrayPacket& pkt) {
    clear();
    appendBatch(pkt);
}

bool Navigator::appendBatch(const WaypointArrayPacket& pkt) {
    uint8_t n = pkt.waypoint_count;
    if (n > 12) n = 12;
    for (uint8_t i = 0; i < n; i++) {
        if (_count >= MAX_WAYPOINTS) return false;   // overflow guard
        _wps[_count++] = pkt.waypoints[i];
    }
    return true;
}

Waypoint Navigator::currentTarget() const {
    if (_index < _count) return _wps[_index];
    Waypoint empty = {0.0, 0.0};
    return empty;
}

bool Navigator::isWaypointReached(double lat, double lon, double radiusM) const {
    if (_index >= _count) return false;
    double d = calculateDistance(lat, lon, _wps[_index].lat, _wps[_index].lon);
    return d <= radiusM;
}

void Navigator::advanceWaypoint() {
    if (_index < _count) _index++;
}
