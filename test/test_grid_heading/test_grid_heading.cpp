#include <unity.h>
#include <cmath>
#include <initializer_list>
#include "control/GridHeadingController.h"
#include "control/HeadingController.h"
#include "control/GridMotorMixer.h"
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
    for (unsigned i = 3; i <= 10; ++i) h.update(160, 0, 110, i * 50000, true);
    TEST_ASSERT_TRUE(h.isTurningInPlace());
}

void test_signed_damping_brakes_before_heading_error_changes_sign() {
    GridHeadingController h;
    h.update(100, 0, 0, 50000, true);
    h.update(100, 70, 110, 100000, true);
    auto r = h.update(100, 75, 110, 150000, true);
    TEST_ASSERT_TRUE(h.headingRateDegPerSec() > 0);
    TEST_ASSERT_TRUE(h.steeringPwm() < 0);
    for (unsigned i = 4; i <= 6; ++i) r = h.update(100, 75, 110, i * 50000, true);
    TEST_ASSERT_TRUE(r.leftPwm < r.rightPwm);
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
    TEST_ASSERT_TRUE(resumed.valid && resumed.leftPwm == MOTOR_MIN_PWM && resumed.rightPwm == MOTOR_MIN_PWM);
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
    TEST_ASSERT_TRUE(r.valid && r.leftPwm == MOTOR_MIN_PWM && r.rightPwm == MOTOR_MIN_PWM);
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

void test_new_torque_builds_gradually_and_allocation_never_requests_net_reverse() {
    GridHeadingController h;
    float previous = 0;
    for (unsigned i = 1; i <= 500; ++i) {
        const float target = i % 20 < 10 ? 20 : 340;
        const auto r = h.update(target, 0, 110, i * 50000, true);
        if (h.steeringPwm() * previous < 0)
            TEST_ASSERT_TRUE(std::fabs(h.steeringPwm()) <= 10.001f);
        else if (std::fabs(h.steeringPwm()) > std::fabs(previous))
            TEST_ASSERT_TRUE(std::fabs(h.steeringPwm()) - std::fabs(previous) <= 10.001f);
        previous = h.steeringPwm();
        TEST_ASSERT_TRUE(r.leftPwm + r.rightPwm >= 0);
    }
}

void test_joint_mixer_preserves_yaw_when_an_inner_motor_would_hit_the_floor() {
    const auto a = GridMotorMixer::allocate(80, 75, 110);
    TEST_ASSERT_TRUE((a.leftPwm - a.rightPwm) * .5f >= 70);
    TEST_ASSERT_TRUE(a.leftPwm + a.rightPwm <= 220);
    const auto b = GridMotorMixer::allocate(80, -75, 110);
    TEST_ASSERT_EQUAL_INT(a.leftPwm, b.rightPwm);
    TEST_ASSERT_EQUAL_INT(a.rightPwm, b.leftPwm);
    // Old independent flooring:155/80 gives only37.5PWM of requested75 yaw.
}

void test_mixer_matches_independent_bruteforce_and_never_exceeds_limits() {
    for (bool active : {false, true})
    for (int cap : {0, 80, 110, MOTOR_PWM_LIMIT})
    for (float surge : {0.f, 40.f, 79.6f, 80.f, 90.2f, 110.f})
    for (float yaw : {-178.f,-100.f,-75.f,-40.f,-10.f,0.f,.5f,10.f,17.25f,40.f,75.f,100.f,178.f}) {
        const auto out = GridMotorMixer::allocate(surge, yaw, cap, active);
        auto score = [&](int l, int r) {
            float a = .5f * (l - r) - yaw, b = .5f * (l + r) - surge;
            return GRID_MIX_YAW_WEIGHT * a * a + b * b;
        };
        float best = INFINITY;
        for (int l = -MOTOR_PWM_LIMIT; l <= MOTOR_PWM_LIMIT; ++l)
        for (int r = -MOTOR_PWM_LIMIT; r <= MOTOR_PWM_LIMIT; ++r) {
            if (active && l == 0 && r == 0) continue;
            if ((l && std::abs(l) < MOTOR_MIN_PWM) || (r && std::abs(r) < MOTOR_MIN_PWM) ||
                l + r < 0 || l + r > 2 * cap) continue;
            best = std::fmin(best, score(l, r));
        }
        TEST_ASSERT_FLOAT_WITHIN(.001f, best, score(out.leftPwm, out.rightPwm));
        for (int pwm : {out.leftPwm, out.rightPwm}) {
            TEST_ASSERT_TRUE(pwm == 0 || std::abs(pwm) >= MOTOR_MIN_PWM);
            TEST_ASSERT_TRUE(std::abs(pwm) <= MOTOR_PWM_LIMIT);
        }
    }
}

void test_surge_depends_on_error_and_remaining_rotation_and_rises_gradually() {
    GridHeadingController straight, angled, rotating;
    auto a = straight.update(90, 90, 110, 50000, true);
    TEST_ASSERT_TRUE(a.leftPwm > 0 && a.rightPwm > 0);
    float previous = straight.surgePwm();
    for (unsigned i = 2; i < 30; ++i) {
        straight.update(90, 90, 110, i * 50000, true);
        TEST_ASSERT_TRUE(straight.surgePwm() - previous <= GRID_SURGE_RISE_PWM_S * .05f + .001f);
        previous = straight.surgePwm();
    }
    angled.update(90, 70, 110, 50000, true);
    for (unsigned i = 2; i < 30; ++i) angled.update(90, 70, 110, i * 50000, true);
    TEST_ASSERT_TRUE(angled.surgePwm() < straight.surgePwm());
    rotating.update(90, 69, 110, 50000, true);
    rotating.update(90, 70, 110, 100000, true);
    TEST_ASSERT_TRUE(rotating.surgePwm() < angled.surgePwm());
}

void test_corner_brakes_before_error_crosses_zero_in_both_directions() {
    for (int direction : {-1, 1}) {
        GridHeadingController h; bool braked = false;
        for (unsigned i = 0; i < 120; ++i) {
            float heading = std::fmod(360.f + direction * (float)i, 360.f);
            float target = direction > 0 ? 100 : 260;
            auto request = h.update(target, heading, 110, (i + 1) * 50000, true);
            float e = HeadingController::computeHeadingError(target, heading);
            if (direction * e > 0 && direction * h.steeringPwm() < 0 &&
                direction * (request.leftPwm - request.rightPwm) < 0) braked = true;
        }
        TEST_ASSERT_TRUE(braked);
    }
}

void test_obsolete_torque_is_withdrawn_before_new_torque_builds() {
    GridHeadingController h;
    for (unsigned i = 1; i <= 100; ++i) h.update(100, 0, 110, i * 50000, true);
    TEST_ASSERT_TRUE(h.steeringPwm() > 170);
    h.update(100, 110, 110, 101 * 50000, true);
    TEST_ASSERT_TRUE(h.steeringPwm() <= 0);
    TEST_ASSERT_TRUE(h.steeringPwm() >= -10.001f);
}

void test_every_static_heading_recovers_and_keeps_thrust_beyond_twelve_seconds() {
    for (int e = -180; e <= 180; e += 15) {
        GridHeadingController h;
        for (unsigned i = 1; i <= 400; ++i) {
            auto out = h.update(std::fmod(360.f + e, 360.f), 0, 110, i * 50000, true);
            TEST_ASSERT_TRUE(out.valid);
            if (i > 350) TEST_ASSERT_TRUE(out.leftPwm || out.rightPwm);
        }
        h.reset();
        auto resumed = h.update(0, 0, 110, 401 * 50000, true);
        TEST_ASSERT_TRUE(resumed.leftPwm > 0 && resumed.rightPwm > 0);
    }
}

void test_small_compass_rate_uses_nominal_damping_without_corner_boost() {
    GridHeadingController h;
    for (unsigned i = 1; i <= 400; ++i) {
        h.update(90, 90 + (i % 2 ? .2f : -.2f), 110, i * 50000, true);
        TEST_ASSERT_TRUE(std::fabs(h.headingRateDegPerSec()) < GRID_BRAKE_BLEND_START_DPS);
        TEST_ASSERT_FLOAT_WITHIN(.001f, -GRID_HEADING_KD * h.headingRateDegPerSec(), h.steeringPwm());
    }
}

void test_cruise_is_not_a_hard_mean_cap_that_removes_moderate_yaw_authority() {
    const auto limited = GridMotorMixer::allocate(110, 40, 110);
    const auto actual = GridMotorMixer::allocate(110, 40, MOTOR_PWM_LIMIT);
    TEST_ASSERT_TRUE((limited.leftPwm - limited.rightPwm) * .5f <= 30);
    TEST_ASSERT_TRUE((actual.leftPwm - actual.rightPwm) * .5f >= 38);
    TEST_ASSERT_TRUE(actual.leftPwm + actual.rightPwm > 220);
}

void test_mixer_cannot_creep_forward_outside_the_alignment_envelope() {
    GridHeadingController h;
    for (unsigned i = 0; i < 200; ++i) {
        float head = i * .5f;
        auto out = h.update(100, head, GRID_CRUISE_PWM, (i + 1) * 50000, true);
        if (100 - head >= GRID_SURGE_ALIGN_DEG) {
            TEST_ASSERT_EQUAL_INT(0, out.leftPwm + out.rightPwm);
            TEST_ASSERT_FLOAT_WITHIN(.001f, 0, h.surgePwm());
        }
    }
}

void test_integral_does_not_wind_up_in_pivot_and_learns_underway_without_reset() {
    GridHeadingController h;
    for (unsigned i = 1; i <= 400; ++i) h.update(45, 0, 110, i * 50000, true);
    TEST_ASSERT_FLOAT_WITHIN(.001f, 0, h.integralPwm());
    TEST_ASSERT_FLOAT_WITHIN(.001f, GRID_HEADING_KP * (45 - GRID_HEADING_DEADZONE_DEG), h.steeringPwm());
    for (unsigned i = 401; i <= 1000; ++i) h.update(45, 40, 110, i * 50000, true);
    TEST_ASSERT_TRUE(h.integralPwm() > 0);
    TEST_ASSERT_TRUE(h.integralPwm() <= GRID_INTEGRAL_LIMIT_PWM);
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
    RUN_TEST(test_new_torque_builds_gradually_and_allocation_never_requests_net_reverse);
    RUN_TEST(test_joint_mixer_preserves_yaw_when_an_inner_motor_would_hit_the_floor);
    RUN_TEST(test_mixer_matches_independent_bruteforce_and_never_exceeds_limits);
    RUN_TEST(test_surge_depends_on_error_and_remaining_rotation_and_rises_gradually);
    RUN_TEST(test_corner_brakes_before_error_crosses_zero_in_both_directions);
    RUN_TEST(test_obsolete_torque_is_withdrawn_before_new_torque_builds);
    RUN_TEST(test_every_static_heading_recovers_and_keeps_thrust_beyond_twelve_seconds);
    RUN_TEST(test_small_compass_rate_uses_nominal_damping_without_corner_boost);
    RUN_TEST(test_cruise_is_not_a_hard_mean_cap_that_removes_moderate_yaw_authority);
    RUN_TEST(test_mixer_cannot_creep_forward_outside_the_alignment_envelope);
    RUN_TEST(test_integral_does_not_wind_up_in_pivot_and_learns_underway_without_reset);
    return UNITY_END();
}
