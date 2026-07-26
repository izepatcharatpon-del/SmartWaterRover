// Native unit tests: Navigator LOS, FSM, PID, Safety, Spiral
// Run: pio test -e native
#include <unity.h>
#include <math.h>
#include <string.h>

#include "navigation/Navigator.h"
#include "navigation/SpiralPlanner.h"
#include "control/PidController.h"
#include "core/MissionFsm.h"
#include "safety/SafetyManager.h"
#include "config.h"

void setUp(void) {}
void tearDown(void) {}

// ---------------------------------------------------------------------------
// Geodesy
// ---------------------------------------------------------------------------
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

void test_haversine_clamp_no_nan(void) {
    // Identical antipodal-ish edge: should still be finite.
    double d = Navigator::calculateDistance(0.0, 0.0, 0.0, 179.999999);
    TEST_ASSERT_TRUE(isfinite(d));
}

void test_dateline_distance_uses_short_arc(void) {
    double d = Navigator::calculateDistance(0.0, 179.999, 0.0, -179.999);
    TEST_ASSERT_TRUE(d < 300.0);
}

void test_invalid_waypoint_rejected(void) {
    TEST_ASSERT_FALSE(Navigator::isValidWaypoint(Waypoint{NAN, 100.0}));
    TEST_ASSERT_FALSE(Navigator::isValidWaypoint(Waypoint{13.0, INFINITY}));
    TEST_ASSERT_FALSE(Navigator::isValidWaypoint(Waypoint{91.0, 100.0}));
    TEST_ASSERT_FALSE(Navigator::isValidWaypoint(Waypoint{13.0, 181.0}));
    TEST_ASSERT_TRUE(Navigator::isValidWaypoint(Waypoint{13.75, 100.50}));
}

// ---------------------------------------------------------------------------
// Waypoints / batching
// ---------------------------------------------------------------------------
void test_waypoint_advance_and_complete(void) {
    WaypointArrayPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.packet_index = 0;
    pkt.waypoint_count = 2;
    pkt.waypoints[0] = {13.7500, 100.5000};
    pkt.waypoints[1] = {13.7510, 100.5000};

    Navigator nav;
    TEST_ASSERT_TRUE(nav.loadWaypoints(pkt));
    TEST_ASSERT_TRUE(nav.hasWaypoints());
    TEST_ASSERT_FALSE(nav.missionComplete());

    TEST_ASSERT_TRUE(nav.isWaypointReached(13.7500, 100.5000, 2.5));
    nav.advanceWaypoint();
    nav.advanceWaypoint();
    TEST_ASSERT_TRUE(nav.missionComplete());
}

void test_accept_batch_idempotent_and_order(void) {
    WaypointArrayPacket p0, p1, p1dup, p2skip;
    memset(&p0, 0, sizeof(p0));
    memset(&p1, 0, sizeof(p1));
    memset(&p1dup, 0, sizeof(p1dup));
    memset(&p2skip, 0, sizeof(p2skip));

    p0.packet_index = 0; p0.waypoint_count = 1;
    p0.waypoints[0] = {13.75, 100.50};

    p1.packet_index = 1; p1.waypoint_count = 1;
    p1.waypoints[0] = {13.76, 100.50};

    p1dup = p1;

    p2skip.packet_index = 3; p2skip.waypoint_count = 1;
    p2skip.waypoints[0] = {13.77, 100.50};

    Navigator nav;
    TEST_ASSERT_TRUE(nav.acceptBatch(p0, true));
    TEST_ASSERT_EQUAL(1, nav.count());
    // Lost ACK -> identical batch 0 retry must not reset/reload the mission.
    TEST_ASSERT_TRUE(nav.acceptBatch(p0, false));
    TEST_ASSERT_EQUAL(1, nav.count());
    // Multi-batch upload happens in STANDBY (allowNewMission=true).
    TEST_ASSERT_TRUE(nav.acceptBatch(p1, true));
    TEST_ASSERT_EQUAL(2, nav.count());
    // Duplicate of last -> OK, no append
    TEST_ASSERT_TRUE(nav.acceptBatch(p1dup, false));
    TEST_ASSERT_EQUAL(2, nav.count());
    // Skip index 2 -> NACK
    TEST_ASSERT_FALSE(nav.acceptBatch(p2skip, true));
    TEST_ASSERT_EQUAL(2, nav.count());
}

void test_mid_mission_append_rejected(void) {
    WaypointArrayPacket p0, p1;
    memset(&p0, 0, sizeof(p0));
    memset(&p1, 0, sizeof(p1));
    p0.packet_index = 0; p0.waypoint_count = 1;
    p0.waypoints[0] = {13.75, 100.50};
    p1.packet_index = 1; p1.waypoint_count = 1;
    p1.waypoints[0] = {13.76, 100.50};

    Navigator nav;
    TEST_ASSERT_TRUE(nav.acceptBatch(p0, true));
    // Simulate active mission: no new batches allowed.
    TEST_ASSERT_FALSE(nav.acceptBatch(p1, false));
    TEST_ASSERT_EQUAL(1, nav.count());
}

void test_changed_batch_zero_rejected_mid_mission(void) {
    WaypointArrayPacket p0, changed;
    memset(&p0, 0, sizeof(p0));
    memset(&changed, 0, sizeof(changed));
    p0.packet_index = 0; p0.waypoint_count = 1;
    p0.waypoints[0] = {13.75, 100.50};
    changed = p0;
    changed.waypoints[0] = {13.76, 100.51};

    Navigator nav;
    TEST_ASSERT_TRUE(nav.acceptBatch(p0, true));
    nav.setMissionOrigin(13.74, 100.49);
    nav.advanceWaypoint();
    TEST_ASSERT_TRUE(nav.acceptBatch(p0, false));
    TEST_ASSERT_EQUAL(1, nav.index());
    TEST_ASSERT_FALSE(nav.acceptBatch(changed, false));
    TEST_ASSERT_EQUAL(1, nav.count());
    TEST_ASSERT_EQUAL(1, nav.index());
}

void test_same_one_batch_mission_can_restart_in_standby(void) {
    WaypointArrayPacket p0;
    memset(&p0, 0, sizeof(p0));
    p0.packet_index = 0; p0.waypoint_count = 1;
    p0.waypoints[0] = {13.75, 100.50};

    Navigator nav;
    TEST_ASSERT_TRUE(nav.acceptBatch(p0, true));
    nav.advanceWaypoint();
    TEST_ASSERT_TRUE(nav.missionComplete());

    TEST_ASSERT_TRUE(nav.acceptBatch(p0, true));
    TEST_ASSERT_EQUAL(0, nav.index());
    TEST_ASSERT_FALSE(nav.missionComplete());
}

void test_invalid_append_is_transactional(void) {
    WaypointArrayPacket p0, bad;
    memset(&p0, 0, sizeof(p0));
    memset(&bad, 0, sizeof(bad));
    p0.packet_index = 0; p0.waypoint_count = 1;
    p0.waypoints[0] = {13.75, 100.50};
    bad.packet_index = 1; bad.waypoint_count = 2;
    bad.waypoints[0] = {13.76, 100.50};
    bad.waypoints[1] = {NAN, 100.50};

    Navigator nav;
    TEST_ASSERT_TRUE(nav.acceptBatch(p0, true));
    TEST_ASSERT_FALSE(nav.acceptBatch(bad, true));
    TEST_ASSERT_EQUAL(1, nav.count());
}

void test_reject_nan_batch(void) {
    WaypointArrayPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.packet_index = 0;
    pkt.waypoint_count = 1;
    pkt.waypoints[0] = {NAN, 100.50};
    Navigator nav;
    TEST_ASSERT_FALSE(nav.acceptBatch(pkt, true));
    TEST_ASSERT_FALSE(nav.hasWaypoints());
}

void test_short_batch_seals_mission_ready(void) {
    WaypointArrayPacket p0;
    memset(&p0, 0, sizeof(p0));
    p0.packet_index = 0;
    p0.waypoint_count = 2;
    p0.waypoints[0] = {13.75, 100.50};
    p0.waypoints[1] = {13.76, 100.51};

    Navigator nav;
    TEST_ASSERT_TRUE(nav.acceptBatch(p0, true));
    TEST_ASSERT_TRUE(nav.hasWaypoints());
    TEST_ASSERT_TRUE(nav.isMissionReady());
}

void test_full_batch_needs_seal_before_start(void) {
    WaypointArrayPacket p0, p1;
    memset(&p0, 0, sizeof(p0));
    memset(&p1, 0, sizeof(p1));
    p0.packet_index = 0;
    p0.waypoint_count = 12;
    for (int i = 0; i < 12; i++) {
        p0.waypoints[i] = {13.75 + i * 0.001, 100.50};
    }
    p1.packet_index = 1;
    p1.waypoint_count = 3;
    p1.waypoints[0] = {13.80, 100.50};
    p1.waypoints[1] = {13.81, 100.50};
    p1.waypoints[2] = {13.82, 100.50};

    Navigator nav;
    TEST_ASSERT_TRUE(nav.acceptBatch(p0, true));
    TEST_ASSERT_TRUE(nav.hasWaypoints());
    TEST_ASSERT_FALSE(nav.isMissionReady());  // full batch → more may follow

    nav.sealMissionUpload();
    TEST_ASSERT_TRUE(nav.isMissionReady());

    // Fresh upload: full then short final batch auto-seals without CMD.
    Navigator nav2;
    TEST_ASSERT_TRUE(nav2.acceptBatch(p0, true));
    TEST_ASSERT_FALSE(nav2.isMissionReady());
    TEST_ASSERT_TRUE(nav2.acceptBatch(p1, true));
    TEST_ASSERT_EQUAL(15, nav2.count());
    TEST_ASSERT_TRUE(nav2.isMissionReady());
}

void test_completed_mission_blocks_restart_until_reupload(void) {
    WaypointArrayPacket p0;
    memset(&p0, 0, sizeof(p0));
    p0.packet_index = 0;
    p0.waypoint_count = 1;
    p0.waypoints[0] = {13.75, 100.50};

    Navigator nav;
    TEST_ASSERT_TRUE(nav.acceptBatch(p0, true));
    TEST_ASSERT_TRUE(nav.isMissionReady());
    nav.advanceWaypoint();
    TEST_ASSERT_TRUE(nav.missionComplete());
    TEST_ASSERT_FALSE(nav.isMissionReady());

    TEST_ASSERT_TRUE(nav.acceptBatch(p0, true));
    TEST_ASSERT_TRUE(nav.isMissionReady());
}

// ---------------------------------------------------------------------------
// LOS
// ---------------------------------------------------------------------------
void test_los_on_track_bearing_east(void) {
    Navigator nav;
    WaypointArrayPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.packet_index = 0;
    pkt.waypoint_count = 1;
    // ~111 m east of origin
    pkt.waypoints[0] = {13.7500, 100.5010};
    TEST_ASSERT_TRUE(nav.loadWaypoints(pkt));
    nav.setMissionOrigin(13.7500, 100.5000);

    LosGuidance g = nav.computeLos(13.7500, 100.5000, 4.0, 2.5, 0.5);
    TEST_ASSERT_TRUE(g.valid);
    TEST_ASSERT_FLOAT_WITHIN(15.0, 90.0, g.bearingDeg);
    TEST_ASSERT_FALSE(g.passed);
}

void test_los_passed_end(void) {
    Navigator nav;
    WaypointArrayPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.packet_index = 0;
    pkt.waypoint_count = 1;
    pkt.waypoints[0] = {13.7500, 100.5005};
    TEST_ASSERT_TRUE(nav.loadWaypoints(pkt));
    nav.setMissionOrigin(13.7500, 100.5000);

    // Place boat past the end waypoint along longitude.
    LosGuidance g = nav.computeLos(13.7500, 100.5015, 4.0, 2.5, 0.5);
    TEST_ASSERT_TRUE(g.valid);
    TEST_ASSERT_TRUE(g.passed || g.reached);
}

void test_los_cross_track_nonzero(void) {
    Navigator nav;
    WaypointArrayPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.packet_index = 0;
    pkt.waypoint_count = 1;
    pkt.waypoints[0] = {13.7500, 100.5010}; // east
    TEST_ASSERT_TRUE(nav.loadWaypoints(pkt));
    nav.setMissionOrigin(13.7500, 100.5000);

    // Offset north of the eastbound track.
    LosGuidance g = nav.computeLos(13.7505, 100.5005, 4.0, 2.5, 0.5);
    TEST_ASSERT_TRUE(g.valid);
    TEST_ASSERT_TRUE(fabsf(g.crossTrackM) > 1.0f);
}

// ---------------------------------------------------------------------------
// Spiral
// ---------------------------------------------------------------------------
void test_spiral_offset_grows(void) {
    Waypoint p1 = SpiralPlanner::calculateSpiralOffset(13.75, 100.50, 2.0, 0.0);
    Waypoint p2 = SpiralPlanner::calculateSpiralOffset(13.75, 100.50, 10.0, 0.0);
    double d1 = Navigator::calculateDistance(13.75, 100.50, p1.lat, p1.lon);
    double d2 = Navigator::calculateDistance(13.75, 100.50, p2.lat, p2.lon);
    TEST_ASSERT_TRUE(d2 > d1);
    TEST_ASSERT_FLOAT_WITHIN(1.0, 2.0, (float)d1);
}

void test_spiral_offset_wraps_longitude(void) {
    // East offset from near +179.9° must wrap into negative lon, not >180.
    Waypoint wp = SpiralPlanner::calculateSpiralOffset(0.0, 179.9, 50000.0, 0.0);
    TEST_ASSERT_TRUE(wp.lon >= -180.0 && wp.lon <= 180.0);
    TEST_ASSERT_TRUE(wp.lon < 0.0);
}

void test_spiral_accept_radius_below_inner_chord(void) {
    // chord ≈ 2*r*sin(dθ/2) at r=SPIRAL_A_M must exceed acceptance radius.
    const float half = SPIRAL_DTHETA * 0.5f;
    const float chord = 2.0f * SPIRAL_A_M * sinf(half);
    TEST_ASSERT_TRUE(chord > SPIRAL_WAYPOINT_RADIUS_M * 1.5f);
}

void test_spiral_reaches_three_turns_inside_radius(void) {
    SpiralPlanner sp;
    sp.begin(13.75, 100.50);
    int count = 0;
    Waypoint wp;
    double maxR = 0.0;
    while (sp.nextSpiralPoint(wp)) {
        double r = Navigator::calculateDistance(13.75, 100.50, wp.lat, wp.lon);
        if (r > maxR) maxR = r;
        count++;
        TEST_ASSERT_TRUE(count < 1000); // safety
    }
    // With b≈0.69 and dtheta≈0.52, ~36 points for 3 turns.
    TEST_ASSERT_TRUE(count >= 30);
    TEST_ASSERT_TRUE(maxR <= SPIRAL_MAX_RADIUS_M + 0.5);
    TEST_ASSERT_TRUE(maxR > SPIRAL_MAX_RADIUS_M - 0.2);
}

// ---------------------------------------------------------------------------
// PID angular wrap
// ---------------------------------------------------------------------------
void test_pid_angular_wrap_no_spike(void) {
    // Kp=0 so we only observe derivative; wrap must turn ±358° into ~±2°.
    PidController pid(0.0f, 0.0f, 1.0f, -1000.0f, 1000.0f, 1.0f);
    pid.setAngular(true);
    (void)pid.update(179.0f, 0.05f);
    float out = pid.update(-179.0f, 0.05f);
    // Wrapped delta ≈ +2° / 0.05s = 40. Unwrapped would be ~7160.
    TEST_ASSERT_TRUE(fabsf(out) < 100.0f);
}

void test_pid_reset_clears_integral(void) {
    PidController pid(0.0f, 1.0f, 0.0f, -100.0f, 100.0f);
    (void)pid.update(10.0f, 0.1f);
    pid.reset();
    float out = pid.update(0.0f, 0.1f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, out);
}

// ---------------------------------------------------------------------------
// SafetyManager (AA118 voltage only)
// ---------------------------------------------------------------------------
void test_safety_critical_and_rth(void) {
    SafetyManager s;
    TEST_ASSERT_EQUAL((int)SafetyState::CRITICAL_STOP, (int)s.evaluate(9.9f));
    // CRITICAL latches until recovered above RTH + hysteresis.
    TEST_ASSERT_EQUAL((int)SafetyState::CRITICAL_STOP, (int)s.evaluate(10.4f));
    TEST_ASSERT_EQUAL((int)SafetyState::CRITICAL_STOP, (int)s.evaluate(10.6f));
    TEST_ASSERT_EQUAL((int)SafetyState::NORMAL, (int)s.evaluate(10.8f));

    s = SafetyManager();
    TEST_ASSERT_EQUAL((int)SafetyState::LOW_BATTERY_RTH, (int)s.evaluate(10.4f));
    TEST_ASSERT_EQUAL((int)SafetyState::LOW_BATTERY_RTH, (int)s.evaluate(10.6f));
    TEST_ASSERT_EQUAL((int)SafetyState::NORMAL, (int)s.evaluate(10.8f));
}

// ---------------------------------------------------------------------------
// Mission FSM
// ---------------------------------------------------------------------------
static FsmInputs baseInputs() {
    FsmInputs in;
    memset(&in, 0, sizeof(in));
    in.gpsOk = true;
    in.compassOk = true;
    in.hasWaypoints = true;
    in.homeSet = true;
    in.safety = SafetyState::NORMAL;
    return in;
}

void test_fsm_rth_arrives_home_despite_low_battery(void) {
    MissionFsm fsm;
    fsm.begin();
    FsmInputs in = baseInputs();

    in.startCommand = true;
    TEST_ASSERT_EQUAL(STATE_GRID_PATROL, fsm.tick(in));
    in.startCommand = false;

    in.safety = SafetyState::LOW_BATTERY_RTH;
    TEST_ASSERT_EQUAL(STATE_RTH, fsm.tick(in));

    // Still low battery but arrived home -> must leave RTH.
    in.arrivedHome = true;
    TEST_ASSERT_EQUAL(STATE_STANDBY, fsm.tick(in));
}

void test_fsm_estop_requires_clear(void) {
    MissionFsm fsm;
    fsm.begin();
    FsmInputs in = baseInputs();
    in.startCommand = true;
    TEST_ASSERT_EQUAL(STATE_GRID_PATROL, fsm.tick(in));
    in.startCommand = false;

    in.emergencyCommand = true;
    TEST_ASSERT_EQUAL(STATE_EMERGENCY_STOP, fsm.tick(in));
    in.emergencyCommand = false;

    // START alone must NOT clear E-Stop.
    in.startCommand = true;
    TEST_ASSERT_EQUAL(STATE_EMERGENCY_STOP, fsm.tick(in));
    in.startCommand = false;

    in.clearEstop = true;
    TEST_ASSERT_EQUAL(STATE_STANDBY, fsm.tick(in));
}

void test_fsm_clear_ignored_outside_estop(void) {
    MissionFsm fsm;
    fsm.begin();
    FsmInputs in = baseInputs();
    in.clearEstop = true;
    TEST_ASSERT_EQUAL(STATE_STANDBY, fsm.tick(in));
    in.clearEstop = false;
    in.startCommand = true;
    TEST_ASSERT_EQUAL(STATE_GRID_PATROL, fsm.tick(in));
    in.startCommand = false;
    in.clearEstop = true;  // CLEAR while GRID must not change state by itself
    TEST_ASSERT_EQUAL(STATE_GRID_PATROL, fsm.tick(in));
}

void test_fsm_force_rth_without_home_is_estop(void) {
    MissionFsm fsm;
    fsm.begin();
    FsmInputs in = baseInputs();
    in.startCommand = true;
    TEST_ASSERT_EQUAL(STATE_GRID_PATROL, fsm.tick(in));
    in.startCommand = false;

    in.homeSet = false;
    in.forceRthCommand = true;
    TEST_ASSERT_EQUAL(STATE_EMERGENCY_STOP, fsm.tick(in));
}

void test_fsm_start_needs_waypoints(void) {
    MissionFsm fsm;
    fsm.begin();
    FsmInputs in = baseInputs();
    in.hasWaypoints = false;
    in.startCommand = true;
    TEST_ASSERT_EQUAL(STATE_STANDBY, fsm.tick(in));
}

void test_fsm_start_blocked_on_low_battery(void) {
    MissionFsm fsm;
    fsm.begin();
    FsmInputs in = baseInputs();
    in.startCommand = true;
    in.safety = SafetyState::LOW_BATTERY_RTH;
    TEST_ASSERT_EQUAL(STATE_STANDBY, fsm.tick(in));
}

void test_fsm_gps_fault_latches_estop(void) {
    MissionFsm fsm;
    fsm.begin();
    FsmInputs in = baseInputs();
    in.startCommand = true;
    TEST_ASSERT_EQUAL(STATE_GRID_PATROL, fsm.tick(in));

    in.startCommand = false;
    in.gpsFault = true;
    TEST_ASSERT_EQUAL(STATE_EMERGENCY_STOP, fsm.tick(in));

    in.clearEstop = true;
    TEST_ASSERT_EQUAL(STATE_EMERGENCY_STOP, fsm.tick(in));

    in.gpsFault = false;
    TEST_ASSERT_EQUAL(STATE_STANDBY, fsm.tick(in));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_distance_latitude_step);
    RUN_TEST(test_distance_zero);
    RUN_TEST(test_bearing_due_north);
    RUN_TEST(test_bearing_due_east);
    RUN_TEST(test_haversine_clamp_no_nan);
    RUN_TEST(test_dateline_distance_uses_short_arc);
    RUN_TEST(test_invalid_waypoint_rejected);
    RUN_TEST(test_waypoint_advance_and_complete);
    RUN_TEST(test_accept_batch_idempotent_and_order);
    RUN_TEST(test_mid_mission_append_rejected);
    RUN_TEST(test_changed_batch_zero_rejected_mid_mission);
    RUN_TEST(test_same_one_batch_mission_can_restart_in_standby);
    RUN_TEST(test_invalid_append_is_transactional);
    RUN_TEST(test_reject_nan_batch);
    RUN_TEST(test_short_batch_seals_mission_ready);
    RUN_TEST(test_full_batch_needs_seal_before_start);
    RUN_TEST(test_completed_mission_blocks_restart_until_reupload);
    RUN_TEST(test_los_on_track_bearing_east);
    RUN_TEST(test_los_passed_end);
    RUN_TEST(test_los_cross_track_nonzero);
    RUN_TEST(test_spiral_offset_grows);
    RUN_TEST(test_spiral_offset_wraps_longitude);
    RUN_TEST(test_spiral_accept_radius_below_inner_chord);
    RUN_TEST(test_spiral_reaches_three_turns_inside_radius);
    RUN_TEST(test_pid_angular_wrap_no_spike);
    RUN_TEST(test_pid_reset_clears_integral);
    RUN_TEST(test_safety_critical_and_rth);
    RUN_TEST(test_fsm_rth_arrives_home_despite_low_battery);
    RUN_TEST(test_fsm_estop_requires_clear);
    RUN_TEST(test_fsm_clear_ignored_outside_estop);
    RUN_TEST(test_fsm_force_rth_without_home_is_estop);
    RUN_TEST(test_fsm_start_needs_waypoints);
    RUN_TEST(test_fsm_start_blocked_on_low_battery);
    RUN_TEST(test_fsm_gps_fault_latches_estop);
    return UNITY_END();
}
