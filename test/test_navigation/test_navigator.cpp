// Native unit tests for the pure-math Navigator helpers (run: pio test -e native)
#include <unity.h>
#include <math.h>
#include "navigation/Navigator.h"
#include "navigation/SpiralPlanner.h"

// Two points ~1.11 km apart in latitude (0.01 deg ~ 1113 m).
void test_distance_latitude_step(void) {
    double d = Navigator::calculateDistance(13.7563, 100.5018, 13.7663, 100.5018);
    TEST_ASSERT_FLOAT_WITHIN(5.0, 1113.0, (float)d);
}

void test_distance_zero(void) {
    double d = Navigator::calculateDistance(13.75, 100.50, 13.75, 100.50);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 0.0, (float)d);
}

void test_bearing_due_north(void) {
    double b = Navigator::calculateBearing(13.75, 100.50, 13.80, 100.50);
    TEST_ASSERT_FLOAT_WITHIN(1.0, 0.0, (float)b);
}

void test_bearing_due_east(void) {
    double b = Navigator::calculateBearing(13.75, 100.50, 13.75, 100.55);
    TEST_ASSERT_FLOAT_WITHIN(1.0, 90.0, (float)b);
}

void test_waypoint_advance_and_complete(void) {
    WaypointArrayPacket pkt;
    pkt.packet_index = 0;
    pkt.waypoint_count = 2;
    pkt.waypoints[0] = {13.7500, 100.5000};
    pkt.waypoints[1] = {13.7510, 100.5000};

    Navigator nav;
    nav.loadWaypoints(pkt);
    TEST_ASSERT_TRUE(nav.hasWaypoints());
    TEST_ASSERT_FALSE(nav.missionComplete());

    // At the first waypoint -> reached within radius.
    TEST_ASSERT_TRUE(nav.isWaypointReached(13.7500, 100.5000, 2.5));
    nav.advanceWaypoint();
    nav.advanceWaypoint();
    TEST_ASSERT_TRUE(nav.missionComplete());
}

void test_spiral_offset_grows(void) {
    Waypoint p1 = SpiralPlanner::calculateSpiralOffset(13.75, 100.50, 2.0, 0.0);
    Waypoint p2 = SpiralPlanner::calculateSpiralOffset(13.75, 100.50, 10.0, 0.0);
    double d1 = Navigator::calculateDistance(13.75, 100.50, p1.lat, p1.lon);
    double d2 = Navigator::calculateDistance(13.75, 100.50, p2.lat, p2.lon);
    TEST_ASSERT_TRUE(d2 > d1);
    TEST_ASSERT_FLOAT_WITHIN(1.0, 2.0, (float)d1);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_distance_latitude_step);
    RUN_TEST(test_distance_zero);
    RUN_TEST(test_bearing_due_north);
    RUN_TEST(test_bearing_due_east);
    RUN_TEST(test_waypoint_advance_and_complete);
    RUN_TEST(test_spiral_offset_grows);
    return UNITY_END();
}
