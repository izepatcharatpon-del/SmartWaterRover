#include "GridHeadingController.h"
#include "HeadingController.h"
#include "GridMotorMixer.h"
#include "config.h"
#include <math.h>

namespace {
static_assert(GRID_BRAKE_HORIZON_S > 0 && GRID_BRAKE_WINDOW_DEG > 0,
              "GRID braking scales must be positive");
static_assert(GRID_BRAKE_BLEND_FULL_DPS > GRID_BRAKE_BLEND_START_DPS,
              "GRID rate blend must have positive width");
static_assert(GRID_SURGE_ALIGN_DEG > 0 && GRID_SURGE_ALIGN_DEG <= 90 &&
              GRID_SURGE_CURVE_DEG >= GRID_SURGE_ALIGN_DEG && GRID_SURGE_CURVE_DEG <= 90 &&
              GRID_SURGE_ROTATION_DEG > 0 && GRID_SURGE_RISE_PWM_S > 0 &&
              GRID_MIX_YAW_WEIGHT > 0, "invalid GRID surge/mixer scales");
float clamp(float value, float low, float high) {
    return value < low ? low : (value > high ? high : value);
}
}

GridHeadingController::GridHeadingController() { reset(); }

void GridHeadingController::reset() {
    _haveSample = false;
    _turnInPlace = false;
    _lastSampleUs = 0;
    _lastHeading = 0;
    _rate = 0;
    _integralPwm = 0;
    _steering = 0;
    _surge = 0;
}

DriveRequest GridHeadingController::update(float targetDeg, float headingDeg,
                                          int basePwm, uint32_t sampleUs,
                                          bool fresh) {
    DriveRequest out{DriveSource::GRID, 0, 0, false};
    if (!fresh || !isfinite(targetDeg) || !isfinite(headingDeg) ||
        targetDeg < 0 || targetDeg > 360 || headingDeg < 0 || headingDeg > 360 ||
        basePwm < 0 || basePwm > MOTOR_PWM_LIMIT) {
        reset();
        return out;
    }

    float dt = NAV_LOOP_PERIOD_MS / 1000.0f;
    if (_haveSample) {
        const uint32_t elapsedUs = sampleUs - _lastSampleUs;
        if (elapsedUs < 1000) {
            reset();
            return out;
        }
        if (elapsedUs > 250000) {
            // A measurement gap starts a new estimate; it never latches off.
            reset();
        } else {
            dt = elapsedUs / 1000000.0f;
            const float measuredRate =
                HeadingController::computeHeadingError(headingDeg, _lastHeading) / dt;
            const float alpha = dt / (GRID_RATE_FILTER_TAU_S + dt);
            _rate += alpha * (measuredRate - _rate);
        }
    }
    const bool acquiring = !_haveSample;
    _lastHeading = headingDeg;
    _lastSampleUs = sampleUs;
    _haveSample = true;

    const float error = HeadingController::computeHeadingError(targetDeg, headingDeg);
    const float absError = fabsf(error);
    // No TURN-exit readiness gate: heading and remaining rotation continuously
    // shape both requests, including the first seconds after a corner.
    const float brakeWeight = clamp(1 - absError / GRID_BRAKE_WINDOW_DEG, 0, 1);
    const float rotationWeight = clamp((fabsf(_rate) - GRID_BRAKE_BLEND_START_DPS) /
        (GRID_BRAKE_BLEND_FULL_DPS - GRID_BRAKE_BLEND_START_DPS), 0, 1);
    const float damping = GRID_HEADING_KD +
        GRID_HEADING_KP * GRID_BRAKE_HORIZON_S * brakeWeight * rotationWeight;

    // Continuous dead zone avoids chasing tiny compass fluctuations. D uses
    // measured heading only, so moving the LOS target cannot inject a D kick.
    const float controlError = error > GRID_HEADING_DEADZONE_DEG
        ? error - GRID_HEADING_DEADZONE_DEG
        : (error < -GRID_HEADING_DEADZONE_DEG
            ? error + GRID_HEADING_DEADZONE_DEG : 0.0f);
    const float integralWeight = basePwm > 0
        ? clamp(_surge / basePwm, 0, 1) * (1 - rotationWeight) : 0;
    const float nextIntegral = clamp(_integralPwm +
        GRID_HEADING_KI * controlError * dt * integralWeight,
                                    -GRID_INTEGRAL_LIMIT_PWM, GRID_INTEGRAL_LIMIT_PWM);
    float desired = GRID_HEADING_KP * controlError + nextIntegral - damping * _rate;
    if (!((desired > MOTOR_PWM_LIMIT && controlError > 0) ||
          (desired < -MOTOR_PWM_LIMIT && controlError < 0))) {
        _integralPwm = nextIntegral;
    }
    desired = clamp(GRID_HEADING_KP * controlError + _integralPwm - damping * _rate,
                    -MOTOR_PWM_LIMIT, MOTOR_PWM_LIMIT);
    const float maxStep = GRID_STEERING_SLEW_PWM_PER_SEC * dt;
    // Withdraw obsolete torque immediately; new torque still builds at the
    // GRID slew limit. Final Drive owns axis slew and reversal-neutral timing.
    if (desired * _steering < 0) _steering = 0;
    else if (fabsf(desired) < fabsf(_steering)) _steering = desired;
    _steering += clamp(desired - _steering, -maxStep, maxStep);

    const float cosine = absError < GRID_SURGE_ALIGN_DEG
        ? cosf(absError / GRID_SURGE_CURVE_DEG * 1.570796326794897f) : 0;
    const float alignment = cosine * cosine;
    const float rotation = _rate * GRID_BRAKE_HORIZON_S / GRID_SURGE_ROTATION_DEG;
    // Rate reduces surge without imposing a rate threshold or a zero latch.
    const float goal = basePwm * alignment / (1 + rotation * rotation);
    const float floor = fminf(basePwm, MOTOR_MIN_PWM) * alignment;
    const float surgeGoal = fmaxf(goal, floor);
    if (acquiring) _surge = fminf(surgeGoal, MOTOR_MIN_PWM);
    else if (surgeGoal < _surge) _surge = surgeGoal;
    else _surge = fminf(surgeGoal, _surge + GRID_SURGE_RISE_PWM_S * dt);
    // Cruise is a requested common component, not a new mean-PWM cap that
    // would sacrifice moderate yaw authority at the inner motor's floor.
    out = GridMotorMixer::allocate(_surge, _steering,
        absError >= GRID_SURGE_ALIGN_DEG || basePwm == 0 ? 0 : MOTOR_PWM_LIMIT,
        fabsf(_steering) >= .5f || _surge > 0);
    _turnInPlace = out.leftPwm + out.rightPwm == 0 &&
        (out.leftPwm != 0 || out.rightPwm != 0);
    return out;
}
