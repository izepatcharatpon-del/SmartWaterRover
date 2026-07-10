#ifndef USV_NAVIGATOR_H
#define USV_NAVIGATOR_H

#include <stdint.h>
#include "Packets.h"

// Waypoint management + geodesic helpers (Haversine distance / great-circle
// bearing). Pure math (no Arduino deps) so it is unit-testable on native.
// See codebase_mathematical_models.md.
class Navigator {
public:
    static const int MAX_WAYPOINTS = 64;

    Navigator();

    // --- Static geodesy helpers (degrees in, meters / degrees out) ---
    static double calculateDistance(double lat1, double lon1,
                                    double lat2, double lon2);      // meters
    static double calculateBearing(double lat1, double lon1,
                                   double lat2, double lon2);       // 0-360 deg

    // --- Waypoint list management ---
    void     loadWaypoints(const WaypointArrayPacket& pkt);         // replace list
    bool     appendBatch(const WaypointArrayPacket& pkt);           // multi-batch
    void     clear();
    bool     hasWaypoints() const { return _count > 0; }
    int      count() const { return _count; }

    Waypoint currentTarget() const;
    bool     isWaypointReached(double lat, double lon,
                               double radiusM) const;
    void     advanceWaypoint();
    bool     missionComplete() const { return _index >= _count; }

private:
    Waypoint _wps[MAX_WAYPOINTS];
    int      _count;
    int      _index;
};

#endif // USV_NAVIGATOR_H
