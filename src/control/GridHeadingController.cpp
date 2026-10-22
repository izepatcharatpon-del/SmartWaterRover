#include "GridHeadingController.h"
#include "HeadingController.h"
#include "config.h"
#include <math.h>

namespace {
float clamp(float value, float low, float high) {
    return value < low ? low : (value > high ? high : value);
}
int forwardPwm(float value) {
    const int pwm = (int)lroundf(clamp(value, 0, MOTOR_PWM_LIMIT));
    return pwm > 0 && pwm < MOTOR_MIN_PWM ? MOTOR_MIN_PWM : pwm;
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
    _lastHeading = headingDeg;
    _lastSampleUs = sampleUs;
    _haveSample = true;

    const float error = HeadingController::computeHeadingError(targetDeg, headingDeg);
    const float absError = fabsf(error);
    if (_turnInPlace) {
        if (absError <= LOS_TURN_EXIT_DEG) _turnInPlace = false;
    } else if (absError >= LOS_TURN_FIRST_DEG) {
        _turnInPlace = true;
    }

    // Continuous dead zone avoids chasing tiny compass fluctuations. D uses
    // measured heading only, so moving the LOS target cannot inject a D kick.
    const float controlError = error > GRID_HEADING_DEADZONE_DEG
        ? error - GRID_HEADING_DEADZONE_DEG
        : (error < -GRID_HEADING_DEADZONE_DEG
            ? error + GRID_HEADING_DEADZONE_DEG : 0.0f);
    const float nextIntegral = clamp(_integralPwm + GRID_HEADING_KI * controlError * dt,
                                    -GRID_INTEGRAL_LIMIT_PWM, GRID_INTEGRAL_LIMIT_PWM);
    float desired = GRID_HEADING_KP * controlError + nextIntegral - GRID_HEADING_KD * _rate;
    if (!((desired > MOTOR_PWM_LIMIT && controlError > 0) ||
          (desired < -MOTOR_PWM_LIMIT && controlError < 0))) {
        _integralPwm = nextIntegral;
    }
    desired = clamp(GRID_HEADING_KP * controlError + _integralPwm - GRID_HEADING_KD * _rate,
                    -MOTOR_PWM_LIMIT, MOTOR_PWM_LIMIT);
    const float maxStep = GRID_STEERING_SLEW_PWM_PER_SEC * dt;
    _steering += clamp(desired - _steering, -maxStep, maxStep);

    out.valid = true;
    if (_turnInPlace || basePwm == 0) {
        // Preserve the damping sign: negative correction can brake a positive
        // turn before it overshoots. Final drive enforces neutral on reversal.
        if (fabsf(_steering) >= 0.5f) {
            const int magnitude = (int)lroundf(clamp(fabsf(_steering),
                                                      MOTOR_MIN_PWM, MOTOR_PWM_LIMIT));
            out.leftPwm = _steering > 0 ? magnitude : -magnitude;
            out.rightPwm = -out.leftPwm;
        }
    } else {
        out.leftPwm = forwardPwm(basePwm + _steering);
        out.rightPwm = forwardPwm(basePwm - _steering);
    }
    return out;
}
