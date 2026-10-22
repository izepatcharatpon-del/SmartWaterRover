#include <unity.h>
#include <math.h>
#include <initializer_list>
#include "control/GridCaptureController.h"
#include "navigation/Navigator.h"
#include "config.h"

void setUp() {}
void tearDown() {}

struct GridRig {
    GridCaptureController grid;
    DriveController drive{MOTOR_PWM_LIMIT, MOTOR_MIN_PWM, MOTOR_PWM_SLEW_PER_SEC};
    DriveInterlocks locks{STATE_GRID_PATROL, SafetyState::NORMAL, false, true, false};
    DriveRequest request{};
    uint32_t now = 1000000;
    uint32_t targetId = 0;

    DriveDecision tick(float target, float heading, uint32_t stepUs = 50000,
                       bool fresh = true, bool automatic = true,
                       int base = MOTOR_CRUISE_PWM) {
        now += stepUs;
        request = grid.update(target, heading, base, now, fresh, targetId, locks, automatic);
        return drive.resolve(request, locks, stepUs / 1000000.0f);
    }
};

void test_grid_fast_release_angle_cannot_cruise_but_settled_can() {
    GridRig fast, settled;
    fast.tick(90, 40);
    settled.tick(90, 40);
    fast.tick(90, 65);
    settled.tick(90, 75);
    for (int i = 0; i < 20; ++i) settled.tick(90, 75);
    // Both end at error +15 degrees, inside the legacy TURN release band.
    fast.tick(90, 75);
    TEST_ASSERT_FALSE(fast.grid.phase() == GridCaptureController::Phase::TRACK);
    TEST_ASSERT_TRUE(settled.grid.phase() == GridCaptureController::Phase::TRACK);
    TEST_ASSERT_TRUE(fast.request.leftPwm <= 0 || fast.request.rightPwm <= 0);
    TEST_ASSERT_TRUE(settled.request.leftPwm > 0 && settled.request.rightPwm > 0);
}

void test_grid_los_target_step_has_no_measurement_d_kick() {
    GridRig stepped, steady;
    for (int i = 0; i < 20; ++i) {
        stepped.tick(0, 0);
        steady.tick(15, 0);
    }
    stepped.tick(15, 0);
    steady.tick(15, 0);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0, stepped.grid.headingRateDegPerSec());
    // Same measured heading and final LOS error: only the small I history
    // may differ, never a D transient from the target step.
    TEST_ASSERT_INT_WITHIN(1, steady.request.leftPwm, stepped.request.leftPwm);
    TEST_ASSERT_INT_WITHIN(1, steady.request.rightPwm, stepped.request.rightPwm);
}

static void assertEffectiveBounded(const DriveDecision& out) {
    TEST_ASSERT_TRUE(abs(out.leftPwm) <= MOTOR_PWM_LIMIT);
    TEST_ASSERT_TRUE(abs(out.rightPwm) <= MOTOR_PWM_LIMIT);
    TEST_ASSERT_TRUE(out.leftPwm == 0 || abs(out.leftPwm) >= MOTOR_MIN_PWM);
    TEST_ASSERT_TRUE(out.rightPwm == 0 || abs(out.rightPwm) >= MOTOR_MIN_PWM);
}

void test_grid_signed_counterturn_and_capture_reach_drive_rules() {
    GridRig positive;
    auto before = positive.tick(90, 0);
    auto neutral = positive.tick(90, 88);
    // Still moving clockwise near alignment: counterturn must have the
    // opposite sign, bounded authority, and neutral before motor reversal.
    TEST_ASSERT_TRUE(positive.request.leftPwm < 0);
    TEST_ASSERT_TRUE(positive.request.rightPwm > 0);
    TEST_ASSERT_TRUE(abs(positive.request.leftPwm) <= 100);
    TEST_ASSERT_EQUAL_INT(0, neutral.leftPwm);
    TEST_ASSERT_EQUAL_INT(0, neutral.rightPwm);
    auto reversed = positive.tick(90, 89);
    TEST_ASSERT_TRUE(reversed.leftPwm < 0 && reversed.rightPwm > 0);
    assertEffectiveBounded(before);
    assertEffectiveBounded(neutral);
    assertEffectiveBounded(reversed);

    GridRig negative;
    negative.tick(270, 0);
    negative.tick(270, 272);
    TEST_ASSERT_TRUE(negative.request.leftPwm > 0 && negative.request.rightPwm < 0);

    GridRig capture;
    capture.tick(15, 0);
    auto previous = capture.tick(15, 0);
    TEST_ASSERT_TRUE(capture.grid.phase() == GridCaptureController::Phase::CAPTURE);
    TEST_ASSERT_TRUE(capture.request.leftPwm <= 100 && capture.request.rightPwm <= 100);
    for (int i = 0; i < 20; ++i) {
        auto next = capture.tick(15, 0);
        TEST_ASSERT_TRUE(next.leftPwm >= 0 && next.rightPwm >= 0);
        // DriveController deliberately lifts a neutral axis directly to 80;
        // its 30-PWM/tick slew applies between nonzero requests.
        TEST_ASSERT_TRUE(previous.leftPwm == 0 ? next.leftPwm <= MOTOR_MIN_PWM :
                         abs(next.leftPwm - previous.leftPwm) <= 30);
        TEST_ASSERT_TRUE(previous.rightPwm == 0 ? next.rightPwm <= MOTOR_MIN_PWM :
                         abs(next.rightPwm - previous.rightPwm) <= 30);
        assertEffectiveBounded(next);
        previous = next;
    }
    TEST_ASSERT_TRUE(previous.leftPwm > 100);

    GridRig saturated;
    saturated.tick(180, 0);
    TEST_ASSERT_EQUAL_INT(MOTOR_PWM_LIMIT, saturated.request.leftPwm);
    TEST_ASSERT_EQUAL_INT(-MOTOR_PWM_LIMIT, saturated.request.rightPwm);
}

void test_grid_invalid_stale_and_bad_timing_stop_and_break_dwell() {
    GridRig rig;
    for (int i = 0; i < 9; ++i) rig.tick(15, 0);
    auto stopped = rig.tick(15, 0, 50000, false);
    TEST_ASSERT_FALSE(rig.request.valid);
    TEST_ASSERT_EQUAL_INT(0, stopped.leftPwm);
    TEST_ASSERT_EQUAL_INT(0, stopped.rightPwm);
    TEST_ASSERT_TRUE(rig.grid.phase() == GridCaptureController::Phase::TURN);
    rig.tick(15, 0);
    TEST_ASSERT_FALSE(rig.grid.phase() == GridCaptureController::Phase::TRACK);

    const float bad[][2] = {{NAN, 0}, {INFINITY, 0}, {0, NAN},
                            {0, -INFINITY}, {1e30f, 0}, {0, 1e30f}};
    for (const auto& pair : bad) {
        stopped = rig.tick(pair[0], pair[1]);
        TEST_ASSERT_FALSE(rig.request.valid);
        TEST_ASSERT_EQUAL_INT(0, stopped.leftPwm);
        TEST_ASSERT_EQUAL_INT(0, stopped.rightPwm);
    }
    const uint32_t badSteps[] = {0, 500, 300000};
    for (uint32_t step : badSteps) {
        rig.tick(15, 0); // Seed a fresh first sample after the invalid input.
        stopped = rig.tick(15, 1, step);
        TEST_ASSERT_FALSE(rig.request.valid);
        TEST_ASSERT_EQUAL_INT(0, stopped.leftPwm);
        TEST_ASSERT_EQUAL_INT(0, stopped.rightPwm);
    }
    rig.grid.reset();
    rig.tick(15, 0);
    TEST_ASSERT_TRUE(rig.grid.phase() == GridCaptureController::Phase::TURN);
}

void test_grid_compass_rate_wrap_filter_actual_time_and_noise() {
    GridRig wrap;
    wrap.now = UINT32_MAX - 150000;
    wrap.tick(30, 358);
    wrap.tick(30, 0, 100000);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 20, wrap.grid.headingRateDegPerSec());
    wrap.tick(30, 0); // Filter must retain and decay measured motion.
    TEST_ASSERT_TRUE(wrap.grid.headingRateDegPerSec() > 0);
    TEST_ASSERT_TRUE(wrap.grid.headingRateDegPerSec() < 20);
    TEST_ASSERT_TRUE(wrap.now < 200000); // micros() unsigned wrap was exercised.

    GridRig opposite;
    opposite.tick(330, 2);
    opposite.tick(330, 0, 200000);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -10, opposite.grid.headingRateDegPerSec());
    TEST_ASSERT_TRUE(opposite.request.leftPwm < 0);

    GridRig noise;
    for (int i = 0; i < 30; ++i) noise.tick(15, i % 2 ? 0.05f : 0.0f);
    TEST_ASSERT_TRUE(noise.grid.phase() == GridCaptureController::Phase::TRACK);
    noise.tick(15, 8); // A fresh rate spike breaks readiness immediately.
    TEST_ASSERT_TRUE(noise.grid.phase() == GridCaptureController::Phase::TURN);
    TEST_ASSERT_TRUE(noise.request.leftPwm <= 0 || noise.request.rightPwm <= 0);
}

static void reachTrack(GridRig& rig, float target = 15, float heading = 0) {
    for (int i = 0; i < 30; ++i) rig.tick(target, heading);
    TEST_ASSERT_TRUE(rig.grid.phase() == GridCaptureController::Phase::TRACK);
}

void test_grid_target_mode_manual_and_hold_resume_reacquire() {
    GridRig rig;
    reachTrack(rig);
    ++rig.targetId;
    rig.tick(15, 0);
    TEST_ASSERT_TRUE(rig.grid.phase() == GridCaptureController::Phase::TURN);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0, rig.grid.headingRateDegPerSec());
    GridRig fresh;
    fresh.tick(15, 0);
    TEST_ASSERT_EQUAL_INT(fresh.request.leftPwm, rig.request.leftPwm);

    reachTrack(rig);
    rig.locks.waterSampleHold = true;
    auto out = rig.tick(15, 0);
    TEST_ASSERT_EQUAL_INT((int)DriveStopReason::WATER_SAMPLE, (int)out.stopReason);
    TEST_ASSERT_EQUAL_INT(0, out.leftPwm);
    rig.locks.waterSampleHold = false;
    rig.tick(15, 0);
    TEST_ASSERT_TRUE(rig.grid.phase() == GridCaptureController::Phase::TURN);

    reachTrack(rig);
    rig.locks.state = STATE_SPIRAL_MAPPING;
    rig.tick(15, 0);
    rig.locks.state = STATE_GRID_PATROL;
    rig.tick(15, 0);
    TEST_ASSERT_TRUE(rig.grid.phase() == GridCaptureController::Phase::TURN);

    reachTrack(rig);
    rig.tick(15, 0, 50000, true, false); // Manual owner preempts GRID policy.
    TEST_ASSERT_FALSE(rig.request.valid);
    rig.tick(15, 0);
    TEST_ASSERT_TRUE(rig.grid.phase() == GridCaptureController::Phase::TURN);
    reachTrack(rig);
    rig.locks.state = STATE_STANDBY;
    rig.tick(15, 0);
    rig.locks.state = STATE_GRID_PATROL;
    rig.tick(15, 0);
    TEST_ASSERT_TRUE(rig.grid.phase() == GridCaptureController::Phase::TURN);
}

void test_grid_safety_and_navigation_preempt_and_reacquire() {
    for (int event = 0; event < 6; ++event) {
        GridRig rig;
        reachTrack(rig);
        if (event == 0) rig.locks.emergencyPending = true;
        if (event == 1) rig.locks.safety = SafetyState::CRITICAL_STOP;
        if (event == 2) rig.locks.safety = SafetyState::OVERCURRENT_STOP;
        if (event == 3) rig.locks.safety = SafetyState::LEAK_STOP;
        if (event == 4) rig.locks.navigationAvailable = false;
        if (event == 5) rig.locks.state = STATE_EMERGENCY_STOP;
        const auto stopped = rig.tick(15, 0);
        TEST_ASSERT_EQUAL_INT(0, stopped.leftPwm);
        TEST_ASSERT_EQUAL_INT(0, stopped.rightPwm);
        TEST_ASSERT_EQUAL_INT((int)(event == 4 ? DriveStopReason::NAVIGATION_UNAVAILABLE :
                                   DriveStopReason::HARD_STOP), (int)stopped.stopReason);
        TEST_ASSERT_FALSE(rig.request.valid);
        TEST_ASSERT_TRUE(rig.grid.phase() == GridCaptureController::Phase::TURN);
        rig.locks = {STATE_GRID_PATROL, SafetyState::NORMAL, false, true, false};
        rig.tick(15, 0);
        TEST_ASSERT_TRUE(rig.grid.phase() == GridCaptureController::Phase::TURN);
    }
}

void test_grid_timeout_stops_latches_and_never_grants_cruise() {
    GridRig noisy;
    DriveDecision out{};
    // Above-rate jitter keeps rotation unsettled. Intermittent unavailable
    // navigation also cannot restart the acquisition deadline indefinitely.
    for (int i = 0; i < 270; ++i) {
        noisy.locks.navigationAvailable = i % 7 != 0;
        out = noisy.tick(15, i % 2 ? 8 : 0);
    }
    TEST_ASSERT_TRUE(noisy.grid.timedOut());
    TEST_ASSERT_EQUAL_INT(0, out.leftPwm);
    TEST_ASSERT_EQUAL_INT(0, out.rightPwm);
    noisy.locks.navigationAvailable = true;
    for (int i = 0; i < 40; ++i) {
        out = noisy.tick(15, 0);
        TEST_ASSERT_EQUAL_INT(0, out.leftPwm);
        TEST_ASSERT_EQUAL_INT(0, out.rightPwm);
        TEST_ASSERT_FALSE(noisy.grid.phase() == GridCaptureController::Phase::TRACK);
    }
    ++noisy.targetId; // Explicit new acquisition, still starts without readiness.
    noisy.tick(15, 0);
    TEST_ASSERT_FALSE(noisy.grid.timedOut());
    TEST_ASSERT_TRUE(noisy.grid.phase() == GridCaptureController::Phase::TURN);

    GridRig blockedTurn;
    blockedTurn.now = UINT32_MAX - 500000;
    for (int i = 0; i < 270; ++i) out = blockedTurn.tick(180, 0);
    TEST_ASSERT_TRUE(blockedTurn.grid.timedOut());
    TEST_ASSERT_EQUAL_INT(0, out.leftPwm);
    TEST_ASSERT_EQUAL_INT(0, out.rightPwm);

    GridRig delayed;
    delayed.tick(180, 0);
    out = delayed.tick(180, 0, 13000000);
    TEST_ASSERT_TRUE(delayed.grid.timedOut());
    TEST_ASSERT_EQUAL_INT(0, out.leftPwm);
    TEST_ASSERT_EQUAL_INT(0, out.rightPwm);
}

void test_grid_tracking_hysteresis_long_runtime_and_fresh_reacquisition() {
    GridRig rig;
    reachTrack(rig);
    rig.tick(25, 0); // Existing 30/20 angle hysteresis after earned readiness.
    TEST_ASSERT_TRUE(rig.grid.phase() == GridCaptureController::Phase::TRACK);
    for (int i = 0; i < 90000; ++i) {
        rig.tick(15, 0);
        TEST_ASSERT_TRUE(rig.grid.phase() == GridCaptureController::Phase::TRACK);
        TEST_ASSERT_FALSE(rig.grid.timedOut());
    }
    rig.tick(35, 0);
    TEST_ASSERT_TRUE(rig.grid.phase() == GridCaptureController::Phase::TURN);
    TEST_ASSERT_FALSE(rig.grid.timedOut());
    reachTrack(rig);
}

void test_grid_dwell_requires_continuous_fresh_timed_samples() {
    GridRig rig;
    rig.tick(15, 0);
    TEST_ASSERT_TRUE(rig.grid.phase() == GridCaptureController::Phase::TURN);
    for (int i = 0; i < 9; ++i) rig.tick(15, 0);
    TEST_ASSERT_TRUE(rig.grid.phase() == GridCaptureController::Phase::CAPTURE);
    rig.tick(15, 0, 50000, false);
    rig.tick(15, 0); // First sample after the gap cannot prove settled rotation.
    for (int i = 0; i < 9; ++i) rig.tick(15, 0);
    TEST_ASSERT_TRUE(rig.grid.phase() == GridCaptureController::Phase::CAPTURE);
    rig.tick(15, 0); // Only 450 ms since the first qualifying timed pair.
    TEST_ASSERT_TRUE(rig.grid.phase() == GridCaptureController::Phase::CAPTURE);
    rig.tick(15, 0); // Full 500 ms after that qualifying sample.
    TEST_ASSERT_TRUE(rig.grid.phase() == GridCaptureController::Phase::TRACK);
}

// The caller just supplied the first qualifying timed sample. Its preceding
// interval is not evidence of continuous alignment AND settled rate.
static void assertFullDwellFromHere(GridRig& rig, float target, float heading,
                                    const uint32_t (&steps)[3]) {
    const uint32_t firstQualifiedUs = rig.now;
    TEST_ASSERT_TRUE(rig.grid.phase() == GridCaptureController::Phase::CAPTURE);
    unsigned stepIndex = 0;
    const uint32_t justBefore = GRID_CAPTURE_DWELL_US - GRID_CAPTURE_SAMPLE_MIN_US;
    while (rig.now - firstQualifiedUs < justBefore) {
        const uint32_t remaining = justBefore - (rig.now - firstQualifiedUs);
        const uint32_t step = steps[stepIndex++ % 3];
        rig.tick(target, heading, step < remaining ? step : remaining);
        TEST_ASSERT_TRUE_MESSAGE(rig.grid.phase() == GridCaptureController::Phase::CAPTURE,
                                 "TRACK before full dwell since qualifying evidence");
        TEST_ASSERT_TRUE(rig.request.valid);
        TEST_ASSERT_TRUE(rig.request.leftPwm <= GRID_CAPTURE_PWM_LIMIT);
        TEST_ASSERT_TRUE(rig.request.rightPwm <= GRID_CAPTURE_PWM_LIMIT);
    }
    TEST_ASSERT_EQUAL_UINT32(justBefore, rig.now - firstQualifiedUs);
    rig.tick(target, heading, GRID_CAPTURE_SAMPLE_MIN_US);
    TEST_ASSERT_EQUAL_UINT32(GRID_CAPTURE_DWELL_US, rig.now - firstQualifiedUs);
    TEST_ASSERT_TRUE(rig.grid.phase() == GridCaptureController::Phase::TRACK);
    TEST_ASSERT_TRUE(rig.request.leftPwm > GRID_CAPTURE_PWM_LIMIT ||
                     rig.request.rightPwm > GRID_CAPTURE_PWM_LIMIT);
}

static const uint32_t dwellSchedules[][3] = {
    {50000, 50000, 50000},       // nominal
    {70000, 180000, 250000},     // variable, including maximum accepted dt
    {250000, 250000, 250000}    // maximum dt exposes the review reproduction
};

void test_grid_dwell_starts_at_aligned_sample_not_previous_interval() {
    for (const auto& steps : dwellSchedules) {
        for (int direction : {1, -1}) {
            GridRig rig;
            const float outside = direction > 0 ? 35 : 325;
            const float inside = direction > 0 ? 15 : 345;
            // Repeat after TRACK: losing alignment must discard earned dwell.
            for (int acquisition = 0; acquisition < 2; ++acquisition) {
                rig.tick(outside, 0, steps[0]);
                rig.tick(outside, 0, steps[1]);
                TEST_ASSERT_TRUE(rig.grid.phase() == GridCaptureController::Phase::TURN);
                rig.tick(inside, 0, steps[2]); // LOS changes; targetId does not.
                assertFullDwellFromHere(rig, inside, 0, steps);
            }
        }
    }
}

void test_grid_dwell_starts_at_settled_rate_not_previous_interval() {
    for (const auto& steps : dwellSchedules) {
        for (int direction : {1, -1}) {
            GridRig rig;
            float heading = 180;
            rig.tick(heading + 15, heading, steps[0]);
            // Both first acquisition and rate-spike reacquisition from TRACK.
            for (int acquisition = 0; acquisition < 2; ++acquisition) {
                heading += direction * 10;
                rig.tick(heading + 15, heading, steps[1]);
                TEST_ASSERT_TRUE(fabsf(rig.grid.headingRateDegPerSec()) > GRID_CAPTURE_RATE_MAX_DPS);
                TEST_ASSERT_TRUE(rig.grid.phase() == GridCaptureController::Phase::TURN);
                bool qualified = false;
                for (int i = 0; i < 30; ++i) {
                    rig.tick(heading + 15, heading, steps[i % 3]);
                    if (fabsf(rig.grid.headingRateDegPerSec()) <= GRID_CAPTURE_RATE_MAX_DPS) {
                        qualified = true; // Raw rate is zero on these samples.
                        break;
                    }
                    TEST_ASSERT_TRUE(rig.grid.phase() == GridCaptureController::Phase::TURN);
                }
                TEST_ASSERT_TRUE(qualified);
                assertFullDwellFromHere(rig, heading + 15, heading, steps);
            }
        }
    }
}

void test_grid_reacquisition_requires_full_dwell_after_first_timed_pair() {
    for (const auto& steps : dwellSchedules) {
        for (int event = 0; event < 7; ++event) {
            GridRig rig;
            rig.now = UINT32_MAX - GRID_CAPTURE_DWELL_US;
            reachTrack(rig);
            if (event == 0) rig.grid.reset();
            if (event == 1) ++rig.targetId;
            if (event == 2) {
                rig.locks.waterSampleHold = true;
                rig.tick(15, 0, steps[0]);
                rig.locks.waterSampleHold = false;
            }
            if (event == 3) rig.tick(15, 0, steps[0], false);
            if (event == 4) {
                rig.locks.state = STATE_RTH;
                rig.tick(15, 0, steps[0]);
                rig.locks.state = STATE_GRID_PATROL;
            }
            if (event == 5) rig.tick(15, 0, steps[0], true, false);
            if (event == 6) rig.tick(15, 0, 0);
            rig.tick(15, 0, steps[0]); // First sample proves no rate.
            TEST_ASSERT_TRUE(rig.grid.phase() == GridCaptureController::Phase::TURN);
            rig.tick(15, 0, steps[1]); // First qualifying timed pair starts dwell.
            assertFullDwellFromHere(rig, 15, 0, steps);
        }
    }
}

void test_grid_drive_source_change_and_late_emergency_keep_authority() {
    GridRig rig;
    reachTrack(rig);
    rig.grid.update(15, 0, MOTOR_CRUISE_PWM, rig.now, true, 0, rig.locks, false);
    const DriveRequest manual{DriveSource::MANUAL, 130, 130, true};
    auto out = rig.drive.resolve(manual, rig.locks, 0.05f);
    TEST_ASSERT_EQUAL_INT((int)DriveStopReason::SOURCE_CHANGE, (int)out.stopReason);
    TEST_ASSERT_EQUAL_INT(0, out.leftPwm);
    out = rig.drive.resolve(manual, rig.locks, 0.05f);
    TEST_ASSERT_EQUAL_INT(MOTOR_MIN_PWM, out.leftPwm);
    out = rig.tick(15, 0);
    TEST_ASSERT_EQUAL_INT((int)DriveStopReason::SOURCE_CHANGE, (int)out.stopReason);
    TEST_ASSERT_EQUAL_INT(0, out.leftPwm);
    TEST_ASSERT_EQUAL_INT(0, out.rightPwm);
    TEST_ASSERT_FALSE(rig.grid.phase() == GridCaptureController::Phase::TRACK);
    rig.tick(15, 0);
    rig.locks.emergencyPending = true; // Changed after guidance was computed.
    out = rig.drive.resolve(rig.request, rig.locks, 0.05f);
    TEST_ASSERT_EQUAL_INT((int)DriveStopReason::HARD_STOP, (int)out.stopReason);
    TEST_ASSERT_EQUAL_INT(0, out.leftPwm);
    TEST_ASSERT_EQUAL_INT(0, out.rightPwm);
}

static void assertLegacyPath(DriveSource source, BoatState state, int base) {
    HeadingController heading(HEADING_KP, HEADING_KI, HEADING_KD);
    DriveController drive(MOTOR_PWM_LIMIT, MOTOR_MIN_PWM, MOTOR_PWM_SLEW_PER_SEC);
    DriveInterlocks locks{state, SafetyState::NORMAL, false, true, false};
    int left = 0, right = 0;
    heading.calculateThrusterOutputs(90, 0, base, 0.05f, left, right);
    TEST_ASSERT_TRUE(heading.isTurningInPlace());
    TEST_ASSERT_EQUAL_INT(178, left);
    TEST_ASSERT_EQUAL_INT(-178, right);
    auto out = drive.resolve({source, left, right, true}, locks, 0.05f);
    TEST_ASSERT_EQUAL_INT(80, out.leftPwm);
    TEST_ASSERT_EQUAL_INT(-80, out.rightPwm);
    // Legacy angle-only release and D-on-error remain executable behaviors.
    heading.calculateThrusterOutputs(20, 0, base, 0.05f, left, right);
    TEST_ASSERT_FALSE(heading.isTurningInPlace());
    TEST_ASSERT_EQUAL_INT(0, left);
    TEST_ASSERT_EQUAL_INT(178, right);
    out = drive.resolve({source, left, right, true}, locks, 0.05f);
    TEST_ASSERT_EQUAL_INT(0, out.leftPwm);
    TEST_ASSERT_EQUAL_INT(0, out.rightPwm);
    out = drive.resolve({source, left, right, true}, locks, 0.05f);
    TEST_ASSERT_EQUAL_INT(0, out.leftPwm);
    TEST_ASSERT_EQUAL_INT(80, out.rightPwm);
    heading.reset();
    heading.calculateThrusterOutputs(25, 0, base, 0.05f, left, right);
    TEST_ASSERT_FALSE(heading.isTurningInPlace());
    TEST_ASSERT_TRUE(left >= 0 && right >= 0);
    heading.calculateThrusterOutputs(35, 0, base, 0.05f, left, right);
    TEST_ASSERT_TRUE(heading.isTurningInPlace());
    TEST_ASSERT_TRUE(left > 0 && right < 0);
}

void test_legacy_rth_heading_drive_behavior_unchanged() {
    assertLegacyPath(DriveSource::RTH, STATE_RTH, MOTOR_CRUISE_PWM);
}

void test_legacy_spiral_heading_drive_behavior_unchanged() {
    assertLegacyPath(DriveSource::SPIRAL, STATE_SPIRAL_MAPPING, MOTOR_APPROACH_PWM);
}

void test_grid_real_navigator_los_to_capture_to_drive() {
    Navigator nav;
    WaypointArrayPacket packet{};
    packet.waypoint_count = 1;
    packet.waypoints[0] = {13.75, 100.501};
    TEST_ASSERT_TRUE(nav.loadWaypoints(packet));
    nav.setMissionOrigin(13.75, 100.50);
    const auto los = nav.computeLos(13.75, 100.50, GRID_LOS_LOOKAHEAD_M,
                                    WAYPOINT_RADIUS_M, LOS_PASS_MARGIN_M);
    TEST_ASSERT_TRUE(los.valid);
    GridRig rig;
    rig.targetId = nav.index();
    auto out = rig.tick(los.bearingDeg, los.bearingDeg - 15);
    TEST_ASSERT_FALSE(rig.grid.phase() == GridCaptureController::Phase::TRACK);
    for (int i = 0; i < 30; ++i) out = rig.tick(los.bearingDeg, los.bearingDeg - 15);
    TEST_ASSERT_TRUE(rig.grid.phase() == GridCaptureController::Phase::TRACK);
    TEST_ASSERT_EQUAL_INT((int)DriveSource::GRID, (int)out.source);
    TEST_ASSERT_TRUE(out.leftPwm > 100 && out.rightPwm >= MOTOR_MIN_PWM);
    assertEffectiveBounded(out);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_grid_fast_release_angle_cannot_cruise_but_settled_can);
    RUN_TEST(test_grid_los_target_step_has_no_measurement_d_kick);
    RUN_TEST(test_grid_signed_counterturn_and_capture_reach_drive_rules);
    RUN_TEST(test_grid_invalid_stale_and_bad_timing_stop_and_break_dwell);
    RUN_TEST(test_grid_compass_rate_wrap_filter_actual_time_and_noise);
    RUN_TEST(test_grid_target_mode_manual_and_hold_resume_reacquire);
    RUN_TEST(test_grid_safety_and_navigation_preempt_and_reacquire);
    RUN_TEST(test_grid_timeout_stops_latches_and_never_grants_cruise);
    RUN_TEST(test_grid_tracking_hysteresis_long_runtime_and_fresh_reacquisition);
    RUN_TEST(test_grid_dwell_requires_continuous_fresh_timed_samples);
    RUN_TEST(test_grid_dwell_starts_at_aligned_sample_not_previous_interval);
    RUN_TEST(test_grid_dwell_starts_at_settled_rate_not_previous_interval);
    RUN_TEST(test_grid_reacquisition_requires_full_dwell_after_first_timed_pair);
    RUN_TEST(test_grid_drive_source_change_and_late_emergency_keep_authority);
    RUN_TEST(test_legacy_rth_heading_drive_behavior_unchanged);
    RUN_TEST(test_legacy_spiral_heading_drive_behavior_unchanged);
    RUN_TEST(test_grid_real_navigator_los_to_capture_to_drive);
    return UNITY_END();
}
