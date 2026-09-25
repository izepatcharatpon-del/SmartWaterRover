#ifndef USV_NAVIGATOR_H
#define USV_NAVIGATOR_H

#include <stdint.h>
#include "Packets.h"

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

class Navigator {
public:
    static const int MAX_WAYPOINTS = 64;

    Navigator();

    static double calculateDistance(double lat1, double lon1,
                                    double lat2, double lon2);
    static double calculateBearing(double lat1, double lon1,
                                   double lat2, double lon2);

    static void toLocalEnu(double refLat, double refLon,
                           double lat, double lon,
                           double& eastM, double& northM);
    static void fromLocalEnu(double refLat, double refLon,
                             double eastM, double northM,
                             double& lat, double& lon);

    static bool isValidWaypoint(const Waypoint& wp);

    bool     loadWaypoints(const WaypointArrayPacket& pkt);

    bool     hasWaypoints() const { return _count > 0; }
    bool     isMissionReady() const {
        return _count > 0 && _missionUploadComplete && !missionComplete();
    }
    int      count() const { return _count; }
    int      index() const { return _index; }
    bool     uploadComplete() const { return _missionUploadComplete; }
    uint8_t  expectedPacketIndex() const { return _expectedPacketIndex; }

    bool     acceptBatch(const WaypointArrayPacket& pkt, bool allowNewMission);

    void     sealMissionUpload();

    Waypoint currentTarget() const;
    Waypoint segmentStart() const;

    void     setMissionOrigin(double lat, double lon);

    bool     isWaypointReached(double lat, double lon, double radiusM) const;
    void     advanceWaypoint();
    // Advance one reached WP per tick, after segment progress or a near-track pass.
    int      skipReached(double lat, double lon,
                         double lookaheadM, double radiusM, double passMarginM);
    bool     missionComplete() const { return _count > 0 && _index >= _count; }

    LosGuidance computeLos(double lat, double lon,
                           double lookaheadM, double radiusM,
                           double passMarginM) const;

    // Same LOS as computeLos, for a segment that is not the grid list.
    static LosGuidance computeSegmentLos(double lat, double lon,
                                         const Waypoint& start,
                                         const Waypoint& end,
                                         double lookaheadM, double radiusM,
                                         double passMarginM);

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
