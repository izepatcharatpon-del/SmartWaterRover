#ifndef USV_MOTOR_DRIVER_H
#define USV_MOTOR_DRIVER_H

#include <Arduino.h>

// Dual BTS7960 H-bridges. Signed PWM: positive = forward, negative = reverse.
class MotorDriver {
public:
    void begin();
    void writeThrust(int leftPwm, int rightPwm);  // [-MOTOR_PWM_LIMIT, +…]
    void stop();

private:
    void driveSide(int pwm, uint8_t fwdChannel, uint8_t revChannel);
};

#endif // USV_MOTOR_DRIVER_H
