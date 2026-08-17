#include "Navigator.h"
#include <math.h>
#include <cmath>
#include <stddef.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const double EARTH_RADIUS_M = 6371000.0;
static const double DEG2RAD = M_PI / 180.0;
static const double RAD2DEG = 180.0 / M_PI;
static const double METERS_PER_DEG_LAT = 111320.0;

static double wrapLongitudeDelta(double deltaDeg) {
    while (deltaDeg > 180.0) deltaDeg -= 360.0;
    while (deltaDeg < -180.0) deltaDeg += 360.0;
    return deltaDeg;
}

static double wrapLongitude(double lonDeg) {
    while (lonDeg > 180.0) lonDeg -= 360.0;
    while (lonDeg < -180.0) lonDeg += 360.0;
    return lonDeg;
}

Navigator::Navigator()
    : _count(0), _index(0),
      _originLat(0.0), _originLon(0.0), _originSet(false),
      _expectedPacketIndex(0), _lastAcceptedPacketIndex(0),
      _lastAcceptedPacketHash(0),
      _haveAcceptedPacket(false),
      _missionUploadComplete(false) {}

double Navigator::calculateDistance(double lat1, double lon1,
                                    double lat2, double lon2) {
    double dLat = (lat2 - lat1) * DEG2RAD;
    double dLon = wrapLongitudeDelta(lon2 - lon1) * DEG2RAD;
    double a = sin(dLat / 2.0) * sin(dLat / 2.0) +
               cos(lat1 * DEG2RAD) * cos(lat2 * DEG2RAD) *
               sin(dLon / 2.0) * sin(dLon / 2.0);
    // Clamp against floating-point overshoot that would NaN sqrt(1-a).
    if (a < 0.0) a = 0.0;
    if (a > 1.0) a = 1.0;
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    return EARTH_RADIUS_M * c;
}

double Navigator::calculateBearing(double lat1, double lon1,
                                   double lat2, double lon2) {
    double phi1 = lat1 * DEG2RAD;
    double phi2 = lat2 * DEG2RAD;
    double dLon = wrapLongitudeDelta(lon2 - lon1) * DEG2RAD;
    double y = sin(dLon) * cos(phi2);
    double x = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(dLon);
    double bearing = atan2(y, x) * RAD2DEG;
    if (bearing < 0.0) bearing += 360.0;
    return bearing;
}

void Navigator::toLocalEnu(double refLat, double refLon,
                           double lat, double lon,
                           double& eastM, double& northM) {
    double cosLat = cos(refLat * DEG2RAD);
    if (fabs(cosLat) < 1e-6) cosLat = (cosLat >= 0.0) ? 1e-6 : -1e-6;
    northM = (lat - refLat) * METERS_PER_DEG_LAT;
    eastM  = wrapLongitudeDelta(lon - refLon) * METERS_PER_DEG_LAT * cosLat;
}

void Navigator::fromLocalEnu(double refLat, double refLon,
                             double eastM, double northM,
                             double& lat, double& lon) {
    double cosLat = cos(refLat * DEG2RAD);
    if (fabs(cosLat) < 1e-6) cosLat = (cosLat >= 0.0) ? 1e-6 : -1e-6;
    lat = refLat + northM / METERS_PER_DEG_LAT;
    lon = wrapLongitude(refLon + eastM / (METERS_PER_DEG_LAT * cosLat));
}

bool Navigator::isValidWaypoint(const Waypoint& wp) {
    if (!std::isfinite(wp.lat) || !std::isfinite(wp.lon)) return false;
    if (wp.lat < -90.0 || wp.lat > 90.0) return false;
    if (wp.lon < -180.0 || wp.lon > 180.0) return false;
    return true;
}

void Navigator::clear() {
    _count = 0;
    _index = 0;
    _originSet = false;
    _expectedPacketIndex = 0;
    _lastAcceptedPacketIndex = 0;
    _lastAcceptedPacketHash = 0;
    _haveAcceptedPacket = false;
    _missionUploadComplete = false;
}

void Navigator::setMissionOrigin(double lat, double lon) {
    _originLat = lat;
    _originLon = lon;
    _originSet = true;
}

bool Navigator::loadWaypoints(const WaypointArrayPacket& pkt) {
    // Validate before mutating: a corrupt batch 0 must not wipe a valid mission.
    if (!validateBatch(pkt, MAX_WAYPOINTS)) return false;

    clear();
    for (uint8_t i = 0; i < pkt.waypoint_count; i++) {
        _wps[_count++] = pkt.waypoints[i];
    }
    return true;
}

bool Navigator::appendBatch(const WaypointArrayPacket& pkt) {
    // Validate the whole packet and capacity first; append atomically afterwards.
    if (!validateBatch(pkt, MAX_WAYPOINTS - _count)) return false;
    for (uint8_t i = 0; i < pkt.waypoint_count; i++) {
        _wps[_count++] = pkt.waypoints[i];
    }
    return true;
}

uint32_t Navigator::packetHash(const WaypointArrayPacket& pkt) {
    // FNV-1a over semantic fields only. Ignore unused waypoint slots because
    // gateway retries may not zero-fill them identically.
    uint32_t hash = 2166136261u;
    auto addBytes = [&hash](const void* ptr, size_t len) {
        const uint8_t* bytes = reinterpret_cast<const uint8_t*>(ptr);
        for (size_t i = 0; i < len; i++) {
            hash ^= bytes[i];
            hash *= 16777619u;
        }
    };
    addBytes(&pkt.packet_index, sizeof(pkt.packet_index));
    addBytes(&pkt.waypoint_count, sizeof(pkt.waypoint_count));
    const uint8_t n = pkt.waypoint_count <= 12 ? pkt.waypoint_count : 12;
    addBytes(pkt.waypoints, (size_t)n * sizeof(Waypoint));
    return hash;
}

bool Navigator::validateBatch(const WaypointArrayPacket& pkt, int availableSlots) {
    if (pkt.waypoint_count == 0 || pkt.waypoint_count > 12) return false;
    if ((int)pkt.waypoint_count > availableSlots) return false;
    for (uint8_t i = 0; i < pkt.waypoint_count; i++) {
        if (!isValidWaypoint(pkt.waypoints[i])) return false;
    }
    return true;
}

bool Navigator::acceptBatch(const WaypointArrayPacket& pkt, bool allowNewMission) {
    const uint32_t hash = packetHash(pkt);

    // Duplicate of the last accepted packet: ACK only in STANDBY. Outside
    // STANDBY a retry of the old last batch must NACK so shore does not treat
    // a rejected re-upload (new idx 0/1, coincidental idx 2) as success.
    if (_haveAcceptedPacket &&
        pkt.packet_index == _lastAcceptedPacketIndex &&
        hash == _lastAcceptedPacketHash) {
        if (!allowNewMission) return false;
        if (pkt.packet_index == 0 && _index > 0) {
            if (!loadWaypoints(pkt)) return false;
            _lastAcceptedPacketIndex = 0;
            _lastAcceptedPacketHash = hash;
            _expectedPacketIndex = 1;
            _haveAcceptedPacket = true;
            _missionUploadComplete = (pkt.waypoint_count < 12);
        }
        return true;
    }

    // Mid-mission / E-Stop: reject ALL batches, including non-duplicates.
    if (!allowNewMission) return false;

    if (pkt.packet_index == 0) {
        if (!loadWaypoints(pkt)) return false;
        _lastAcceptedPacketIndex = pkt.packet_index;
        _lastAcceptedPacketHash = hash;
        _expectedPacketIndex = (uint8_t)(pkt.packet_index + 1);
        _haveAcceptedPacket = true;
        _missionUploadComplete = (pkt.waypoint_count < 12);
        return true;
    }

    if (pkt.packet_index != _expectedPacketIndex) return false;

    if (!appendBatch(pkt)) return false;
    _lastAcceptedPacketIndex = pkt.packet_index;
    _lastAcceptedPacketHash = hash;
    _expectedPacketIndex = (uint8_t)(pkt.packet_index + 1);
    _haveAcceptedPacket = true;
    _missionUploadComplete = (pkt.waypoint_count < 12);
    return true;
}

void Navigator::sealMissionUpload() {
    if (_count > 0) {
        _missionUploadComplete = true;
    }
}

Waypoint Navigator::currentTarget() const {
    if (_index < _count) return _wps[_index];
    Waypoint empty = {0.0, 0.0};
    return empty;
}

Waypoint Navigator::segmentStart() const {
    if (_index > 0 && _index <= _count) return _wps[_index - 1];
    if (_originSet) {
        Waypoint o = {_originLat, _originLon};
        return o;
    }
    // Fallback: use current target as both ends (degenerate segment).
    return currentTarget();
}

bool Navigator::isWaypointReached(double lat, double lon, double radiusM) const {
    if (_index >= _count) return false;
    double d = calculateDistance(lat, lon, _wps[_index].lat, _wps[_index].lon);
    return d <= radiusM;
}

void Navigator::advanceWaypoint() {
    if (_index < _count) _index++;
}

LosGuidance Navigator::computeLos(double lat, double lon,
                                  double lookaheadM, double radiusM,
                                  double passMarginM) const {
    LosGuidance g;
    g.bearingDeg = 0.0f;
    g.crossTrackM = 0.0f;
    g.alongTrackM = 0.0f;
    g.distToEndM = 0.0f;
    g.lookAheadDistM = 0.0f;
    g.reached = false;
    g.passed = false;
    g.valid = false;

    if (_index >= _count) return g;

    Waypoint end = _wps[_index];
    Waypoint start = segmentStart();

    g.distToEndM = (float)calculateDistance(lat, lon, end.lat, end.lon);
    g.reached = (g.distToEndM <= (float)radiusM);

    double se, sn, ee, en, pe, pn;
    toLocalEnu(start.lat, start.lon, start.lat, start.lon, se, sn); // 0,0
    (void)se; (void)sn;
    toLocalEnu(start.lat, start.lon, end.lat, end.lon, ee, en);
    toLocalEnu(start.lat, start.lon, lat, lon, pe, pn);

    double segEx = ee - 0.0;
    double segNy = en - 0.0;
    double segLen2 = segEx * segEx + segNy * segNy;
    double segLen = sqrt(segLen2);

    if (segLen < 0.05) {
        // Degenerate: point-chase the end.
        g.bearingDeg = (float)calculateBearing(lat, lon, end.lat, end.lon);
        g.lookAheadDistM = g.distToEndM;
        g.passed = false;
        g.valid = true;
        return g;
    }

    // Projection of boat onto segment: t in [0, 1+] (allow past end).
    double t = ((pe - 0.0) * segEx + (pn - 0.0) * segNy) / segLen2;
    double projE = 0.0 + t * segEx;
    double projN = 0.0 + t * segNy;

    // Cross-track: signed left(+)/right(-) of track direction via 2D cross.
    g.crossTrackM = (float)(((pe - 0.0) * segNy - (pn - 0.0) * segEx) / segLen);
    g.alongTrackM = (float)(t * segLen);

    g.passed = (t >= 1.0) && (g.alongTrackM >= (float)(segLen + passMarginM));

    // Look-ahead along segment from projection (clamped to end).
    double alongFromStart = t * segLen + lookaheadM;
    if (alongFromStart > segLen) alongFromStart = segLen;
    if (alongFromStart < 0.0) alongFromStart = 0.0;
    double laE = (alongFromStart / segLen) * segEx;
    double laN = (alongFromStart / segLen) * segNy;

    double laLat, laLon;
    fromLocalEnu(start.lat, start.lon, laE, laN, laLat, laLon);

    g.bearingDeg = (float)calculateBearing(lat, lon, laLat, laLon);
    g.lookAheadDistM = (float)calculateDistance(lat, lon, laLat, laLon);
    g.valid = true;
    return g;
}
