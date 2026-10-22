#include <unity.h>
#include <cmath>
#include "control/GridHeadingController.h"
#include "control/HeadingController.h"
#include "config.h"

void setUp() {}
void tearDown() {}

void test_target_step_does_not_change_measured_rate_or_kick_steering() {
    GridHeadingController h;
    h.update(0, 0, 110, 50000, true);
    const auto r = h.update(160, 0, 0, 100000, true);
    TEST_ASSERT_TRUE(r.valid);
    TEST_ASSERT_FLOAT_WITHIN(.001f, 0, h.headingRateDegPerSec());
    TEST_ASSERT_FLOAT_WITHIN(.001f, 10, h.steeringPwm());
    TEST_ASSERT_TRUE(r.leftPwm > 0 && r.rightPwm < 0);
}

void test_signed_damping_brakes_before_heading_error_changes_sign() {
    GridHeadingController h;
    h.update(100, 0, 0, 50000, true);
    h.update(100, 70, 110, 100000, true);
    const auto r = h.update(100, 75, 110, 150000, true);
    TEST_ASSERT_TRUE(h.isTurningInPlace());
    TEST_ASSERT_TRUE(h.headingRateDegPerSec() > 0);
    TEST_ASSERT_TRUE(h.steeringPwm() < 0);
    TEST_ASSERT_TRUE(r.leftPwm < 0 && r.rightPwm > 0);
    // Equivalent legacy input proves why preserving the sign matters.
    HeadingController legacy(HEADING_KP, HEADING_KI, HEADING_KD);
    int l, rr;
    legacy.calculateThrusterOutputs(100, 0, 0, .05f, l, rr);
    legacy.calculateThrusterOutputs(100, 70, 110, .05f, l, rr);
    legacy.calculateThrusterOutputs(100, 75, 110, .05f, l, rr);
    TEST_ASSERT_TRUE(l > 0 && rr < 0);
}

void test_jitter_keeps_forward_thrust_beyond_twelve_seconds() {
    GridHeadingController h;
    for (unsigned i = 1; i <= 800; ++i) {
        const auto r = h.update(90, 90 + (i % 2 ? .2f : -.2f), 110, i * 50000, true);
        TEST_ASSERT_TRUE(r.valid && r.leftPwm > 0 && r.rightPwm > 0);
        TEST_ASSERT_FALSE(h.isTurningInPlace());
        TEST_ASSERT_TRUE(std::fabs(h.headingRateDegPerSec()) < 2.0f);
    }
}

void test_wrap_and_turn_direction_are_symmetric() {
    GridHeadingController cw, ccw;
    const float headings[] = {0, 70, 75, 80, 85, 90};
    for (unsigned i = 0; i < 6; ++i) {
        const auto a = cw.update(100, headings[i], 110, (i + 1) * 50000, true);
        const auto b = ccw.update(260, headings[i] == 0 ? 0 : 360 - headings[i], 110,
                                  (i + 1) * 50000, true);
        TEST_ASSERT_EQUAL_INT(a.leftPwm, b.rightPwm);
        TEST_ASSERT_EQUAL_INT(a.rightPwm, b.leftPwm);
    }
    GridHeadingController wrap;
    wrap.update(10, 359, 110, 50000, true);
    wrap.update(10, 1, 110, 100000, true);
    TEST_ASSERT_FLOAT_WITHIN(.01f, 8.0f, wrap.headingRateDegPerSec());
}

void test_invalid_measurement_resets_and_recovers_without_latch() {
    GridHeadingController h;
    h.update(100, 0, 0, 50000, true);
    const auto invalid = h.update(100, NAN, 0, 100000, true);
    TEST_ASSERT_FALSE(invalid.valid);
    TEST_ASSERT_EQUAL_INT(0, invalid.leftPwm);
    TEST_ASSERT_FALSE(h.isTurningInPlace());
    const auto resumed = h.update(90, 90, 110, 150000, true);
    TEST_ASSERT_TRUE(resumed.valid && resumed.leftPwm == 110 && resumed.rightPwm == 110);
    TEST_ASSERT_FALSE(h.update(90, 90, 110, 200000, false).valid);
    TEST_ASSERT_FALSE(h.update(361, 90, 110, 250000, true).valid);
    TEST_ASSERT_FALSE(h.update(90, 90, 179, 300000, true).valid);
}

void test_timestamp_wrap_gap_and_duplicate_sample_are_bounded() {
    GridHeadingController h;
    h.update(90, 90, 110, 0xfffffff0U, true);
    auto r = h.update(90, 91, 110, 49984, true);
    TEST_ASSERT_TRUE(r.valid);
    TEST_ASSERT_FLOAT_WITHIN(.001f, 4, h.headingRateDegPerSec());
    r = h.update(90, 90, 110, 1049984, true);
    TEST_ASSERT_TRUE(r.valid && r.leftPwm == 110 && r.rightPwm == 110);
    TEST_ASSERT_FLOAT_WITHIN(.001f, 0, h.headingRateDegPerSec());
    TEST_ASSERT_FALSE(h.update(90, 90, 110, 1049984, true).valid);
    TEST_ASSERT_TRUE(h.update(90, 90, 110, 1099984, true).valid);
}

void test_small_error_runs_forward_and_integral_cannot_grow_unbounded() {
    GridHeadingController h;
    for (unsigned i = 1; i <= 20000; ++i) {
        const auto r = h.update(10, 0, 110, i * 50000, true);
        TEST_ASSERT_TRUE(r.valid && r.leftPwm >= MOTOR_MIN_PWM && r.rightPwm >= MOTOR_MIN_PWM);
        TEST_ASSERT_TRUE(r.leftPwm <= MOTOR_PWM_LIMIT && r.rightPwm <= MOTOR_PWM_LIMIT);
        TEST_ASSERT_TRUE(h.steeringPwm() <= 40.001f);
    }
}

void test_correction_slew_bound_and_forward_never_reverses() {
    GridHeadingController h;
    float previous = 0;
    for (unsigned i = 1; i <= 500; ++i) {
        const float target = i % 20 < 10 ? 20 : 340;
        const auto r = h.update(target, 0, 110, i * 50000, true);
        TEST_ASSERT_TRUE(std::fabs(h.steeringPwm() - previous) <= 10.001f);
        previous = h.steeringPwm();
        TEST_ASSERT_TRUE(r.leftPwm > 0 && r.rightPwm > 0);
    }
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_target_step_does_not_change_measured_rate_or_kick_steering);
    RUN_TEST(test_signed_damping_brakes_before_heading_error_changes_sign);
    RUN_TEST(test_jitter_keeps_forward_thrust_beyond_twelve_seconds);
    RUN_TEST(test_wrap_and_turn_direction_are_symmetric);
    RUN_TEST(test_invalid_measurement_resets_and_recovers_without_latch);
    RUN_TEST(test_timestamp_wrap_gap_and_duplicate_sample_are_bounded);
    RUN_TEST(test_small_error_runs_forward_and_integral_cannot_grow_unbounded);
    RUN_TEST(test_correction_slew_bound_and_forward_never_reverses);
    return UNITY_END();
}
