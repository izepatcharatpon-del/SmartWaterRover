#ifndef USV_NAVIGATOR_H
#define USV_NAVIGATOR_H

#include <stdint.h>
#include "Packets.h"

// Result of LOS guidance on the current segment (previous -> current waypoint).
struct LosGuidance {
    float bearingDeg;      // heading to look-ahead point (0-360)
    float crossTrackM;     // signed cross-track error (m, + = right of track)
    float alongTrackM;     // progress along segment (m)
    float distToEndM;      // geodesic distance to segment end (m)
    float lookAheadDistM;  // distance to look-ahead point (m)
    bool  reached;         // inside acceptance radius of end
    bool  passed;          // projected past end + margin
    bool  valid;
};

// Waypoint management + geodesic helpers + LOS segment tracking.
// Pure math (no Arduino deps) so it is unit-testable on native.
class Navigator {
public:
    static const int MAX_WAYPOINTS = 64;

    Navigator();

    // --- Static geodesy helpers (degrees in, meters / degrees out) ---
    static double calculateDistance(double lat1, double lon1,
                                    double lat2, double lon2);      // meters
    static double calculateBearing(double lat1, double lon1,
                                   double lat2, double lon2);       // 0-360 deg

    // Local ENU offsets (metres) relative to a reference lat/lon.
    static void toLocalEnu(double refLat, double refLon,
                           double lat, double lon,
                           double& eastM, double& northM);
    static void fromLocalEnu(double refLat, double refLon,
                             double eastM, double northM,
                             double& lat, double& lon);

    static bool isValidWaypoint(const Waypoint& wp);

    // Replace list (tests / batch-0 path). Production uploads use acceptBatch.
    bool     loadWaypoints(const WaypointArrayPacket& pkt);

    bool     hasWaypoints() const { return _count > 0; }
    bool     isMissionReady() const {
        return _count > 0 && _missionUploadComplete && !missionComplete();
    }
    int      count() const { return _count; }
    int      index() const { return _index; }

    // Idempotent multi-batch upload. Mid-mission (!allowNewMission) rejects
    // all new batches; identical last-packet retries still ACK.
    bool     acceptBatch(const WaypointArrayPacket& pkt, bool allowNewMission);

    void     sealMissionUpload();

    Waypoint currentTarget() const;
    Waypoint segmentStart() const;

    void     setMissionOrigin(double lat, double lon);

    bool     isWaypointReached(double lat, double lon, double radiusM) const;
    void     advanceWaypoint();
    bool     missionComplete() const { return _count > 0 && _index >= _count; }

    LosGuidance computeLos(double lat, double lon,
                           double lookaheadM, double radiusM,
                           double passMarginM) const;

private:
    Waypoint _wps[MAX_WAYPOINTS];
    int      _count;
    int      _index;
    double   _originLat, _originLon;
    bool     _originSet;
    uint8_t  _expectedPacketIndex;
    uint8_t  _lastAcceptedPacketIndex;
    uint32_t _lastAcceptedPacketHash;
    bool     _haveAcceptedPacket;
    bool     _missionUploadComplete;

    void     clear();
    bool     appendBatch(const WaypointArrayPacket& pkt);
    static uint32_t packetHash(const WaypointArrayPacket& pkt);
    static bool validateBatch(const WaypointArrayPacket& pkt, int availableSlots);
};

#endif // USV_NAVIGATOR_H
