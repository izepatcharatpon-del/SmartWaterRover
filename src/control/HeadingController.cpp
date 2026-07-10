#include "HeadingController.h"
#include "config.h"
#include <Arduino.h>

HeadingController::HeadingController(float kp, float ki, float kd)
    // Correction limited to full PWM span so a hard turn can reverse one side.
    : _pid(kp, ki, kd, -MOTOR_PWM_LIMIT, MOTOR_PWM_LIMIT, PID_DERIV_ALPHA) {}

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

    leftPwm  = basePwm + (int)correction;
    rightPwm = basePwm - (int)correction;

    leftPwm  = constrain(leftPwm,  -MOTOR_PWM_LIMIT, MOTOR_PWM_LIMIT);
    rightPwm = constrain(rightPwm, -MOTOR_PWM_LIMIT, MOTOR_PWM_LIMIT);
}
