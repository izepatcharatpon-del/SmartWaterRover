#include "HeadingController.h"
#include "config.h"
#include <Arduino.h>
#include <math.h>

HeadingController::HeadingController(float kp, float ki, float kd)
    : _pid(kp, ki, kd, -MOTOR_PWM_LIMIT, MOTOR_PWM_LIMIT, PID_DERIV_ALPHA) {
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

    // Turn-first: when heading error is large, cancel forward thrust.
    int effectiveBase = basePwm;
    if (fabsf(error) > LOS_TURN_FIRST_DEG) {
        effectiveBase = 0;
    }

    if (effectiveBase == 0) {
        // Turn in place. Avoid mapping a tiny 1-degree PID output to ±80,
        // which causes limit-cycle oscillation around the target heading.
        if (fabsf(error) <= HEADING_STOP_TOL_DEG) {
            leftPwm = 0;
            rightPwm = 0;
        } else {
            int turn = (int)fabsf(correction);
            turn = constrain(turn, MOTOR_MIN_PWM, MOTOR_PWM_LIMIT);
            leftPwm  = (error > 0.0f) ? turn : -turn;
            rightPwm = -leftPwm;
        }
    } else {
        // Base PWM is chosen above motor deadband; preserve proportional
        // differential instead of independently snapping each side to ±80.
        leftPwm  = effectiveBase + (int)correction;
        rightPwm = effectiveBase - (int)correction;

        // After ±correction a side can land in (0, MOTOR_MIN_PWM) where the
        // driver produces no motion — lift out of the hardware deadband.
        auto liftDeadband = [](int v) -> int {
            if (v == 0) return 0;
            if (v > 0 && v < MOTOR_MIN_PWM) return MOTOR_MIN_PWM;
            if (v < 0 && v > -MOTOR_MIN_PWM) return -MOTOR_MIN_PWM;
            return v;
        };
        leftPwm  = liftDeadband(leftPwm);
        rightPwm = liftDeadband(rightPwm);
    }

    leftPwm  = constrain(leftPwm,  -MOTOR_PWM_LIMIT, MOTOR_PWM_LIMIT);
    rightPwm = constrain(rightPwm, -MOTOR_PWM_LIMIT, MOTOR_PWM_LIMIT);
}
