#include "HeadingController.h"
#include "config.h"
#include <math.h>

static int clampPwm(int pwm, int low, int high) {
    if (pwm < low) return low;
    if (pwm > high) return high;
    return pwm;
}

HeadingController::HeadingController(float kp, float ki, float kd)
    : _pid(kp, ki, kd, -MOTOR_PWM_LIMIT, MOTOR_PWM_LIMIT, PID_DERIV_ALPHA),
      _turnInPlace(false) {
    _pid.setAngular(true);
}

float HeadingController::computeHeadingError(float targetDeg, float currentDeg) {
    float error = targetDeg - currentDeg;
    while (error > 180.0f)  error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    return error;
}

void HeadingController::calculateThrusterOutputs(float targetBearing, float currentHeading,
                                                 int basePwm, float dt,
                                                 int& leftPwm, int& rightPwm) {
    float error = computeHeadingError(targetBearing, currentHeading);
    float correction = _pid.update(error, dt);

    const float absError = fabsf(error);
    if (_turnInPlace) {
        if (absError <= LOS_TURN_EXIT_DEG) {
            _turnInPlace = false;
        }
    } else if (absError >= LOS_TURN_FIRST_DEG) {
        _turnInPlace = true;
    }

    if (_turnInPlace || basePwm <= 0) {
        if (absError <= HEADING_STOP_TOL_DEG) {
            leftPwm = 0;
            rightPwm = 0;
        } else {
            int turn = (int)fabsf(correction);
            turn = clampPwm(turn, MOTOR_MIN_PWM, MOTOR_PWM_LIMIT);
            leftPwm  = (error > 0.0f) ? turn : -turn;
            rightPwm = -leftPwm;
        }
    } else {
        // Forward guidance may slow either side to zero, but must not command
        // an unplanned reverse kick. Reverse is reserved for turn-in-place.
        leftPwm  = clampPwm(basePwm + (int)correction, 0, MOTOR_PWM_LIMIT);
        rightPwm = clampPwm(basePwm - (int)correction, 0, MOTOR_PWM_LIMIT);

        auto liftDeadband = [](int v) -> int {
            if (v == 0) return 0;
            if (v > 0 && v < MOTOR_MIN_PWM) return MOTOR_MIN_PWM;
            return v;
        };
        leftPwm  = liftDeadband(leftPwm);
        rightPwm = liftDeadband(rightPwm);
    }

    leftPwm  = clampPwm(leftPwm,  -MOTOR_PWM_LIMIT, MOTOR_PWM_LIMIT);
    rightPwm = clampPwm(rightPwm, -MOTOR_PWM_LIMIT, MOTOR_PWM_LIMIT);
}
