#include <unity.h>
#include <cmath>
#include "control/HeadingController.h"
#include "control/DriveController.h"
#include "config.h"

void setUp() {}
void tearDown() {}

struct LegacyGridRig {
    HeadingController heading{HEADING_KP, HEADING_KI, HEADING_KD};
    DriveController drive{MOTOR_PWM_LIMIT, MOTOR_MIN_PWM, MOTOR_PWM_SLEW_PER_SEC};
    DriveInterlocks locks{STATE_GRID_PATROL, SafetyState::NORMAL, false, true, false};
    DriveDecision tick(float measured) {
        int left = 0, right = 0;
        const float error = HeadingController::computeHeadingError(90, measured);
        const int base = std::fabs(error) > LOS_TURN_FIRST_DEG ? 0 : MOTOR_CRUISE_PWM;
        heading.calculateThrusterOutputs(90, measured, base, .05f, left, right);
        const auto result = drive.resolve({DriveSource::GRID, left, right, true}, locks, .05f);
        if (result.stopReason != DriveStopReason::NONE) heading.reset();
        return result;
    }
};

void test_legacy_grid_keeps_forward_thrust_with_small_heading_jitter_beyond_twelve_seconds() {
    LegacyGridRig rig;
    for (int i = 0; i < 320; ++i) {
        const float measured = i < 60 ? i * 1.5f : 90.0f + (i % 2 ? .2f : -.2f);
        const auto out = rig.tick(measured);
        if (i >= 100) {
            TEST_ASSERT_TRUE(out.leftPwm > 0 && out.rightPwm > 0);
            TEST_ASSERT_TRUE(out.stopReason == DriveStopReason::NONE);
        }
    }
}

void test_legacy_grid_still_obeys_navigation_water_and_emergency_stops() {
    LegacyGridRig rig;
    rig.tick(90);
    rig.locks.navigationAvailable = false;
    auto out = rig.tick(90);
    TEST_ASSERT_TRUE(out.stopReason == DriveStopReason::NAVIGATION_UNAVAILABLE);
    TEST_ASSERT_EQUAL_INT(0, out.leftPwm);
    TEST_ASSERT_EQUAL_INT(0, out.rightPwm);
    rig.locks.navigationAvailable = true;
    rig.locks.waterSampleHold = true;
    out = rig.tick(90);
    TEST_ASSERT_TRUE(out.stopReason == DriveStopReason::WATER_SAMPLE);
    TEST_ASSERT_EQUAL_INT(0, out.leftPwm);
    TEST_ASSERT_EQUAL_INT(0, out.rightPwm);
    rig.locks.waterSampleHold = false;
    rig.locks.emergencyPending = true;
    out = rig.tick(90);
    TEST_ASSERT_TRUE(out.stopReason == DriveStopReason::HARD_STOP);
    TEST_ASSERT_EQUAL_INT(0, out.leftPwm);
    TEST_ASSERT_EQUAL_INT(0, out.rightPwm);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_legacy_grid_keeps_forward_thrust_with_small_heading_jitter_beyond_twelve_seconds);
    RUN_TEST(test_legacy_grid_still_obeys_navigation_water_and_emergency_stops);
    return UNITY_END();
}
