#include "EllipseTracker.h"
#include "config.h"
#include <Arduino.h>

float EllipseTracker::crossTrackError(float x, float y, float a, float b) {
    if (a <= 0.0f || b <= 0.0f) return 0.0f;
    return (x * x) / (a * a) + (y * y) / (b * b) - 1.0f;
}

void EllipseTracker::calculateEllipticalThrust(float x, float y, float a, float b,
                                               int basePwm, int& leftPwm, int& rightPwm) {
    float E = crossTrackError(x, y, a, b);

    if (fabs(E) < 0.01f) {          // on track: straight cruise
        leftPwm  = basePwm;
        rightPwm = basePwm;
        return;
    }

    // E > 0 : outside ellipse -> steer inward; E < 0 : inside -> steer outward.
    int correction = (int)(_gain * E);
    correction = constrain(correction, -MOTOR_PWM_LIMIT, MOTOR_PWM_LIMIT);

    leftPwm  = constrain(basePwm + correction, -MOTOR_PWM_LIMIT, MOTOR_PWM_LIMIT);
    rightPwm = constrain(basePwm - correction, -MOTOR_PWM_LIMIT, MOTOR_PWM_LIMIT);
}
