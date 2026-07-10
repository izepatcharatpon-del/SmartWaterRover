#ifndef USV_MOTOR_DRIVER_H
#define USV_MOTOR_DRIVER_H

#include <Arduino.h>

// Dual BTS7960 H-bridges driving two thrusters. Uses ESP32 LEDC at 20 kHz,
// 8-bit resolution. Signed PWM: positive = forward, negative = reverse.
// See USV_Power_Management_Guide.md and USV_Hardware_GPIO_Mapping.md.
class MotorDriver {
public:
    void begin();

    // leftPwm / rightPwm in [-255, 255]; internally capped to MOTOR_PWM_LIMIT.
    void writeThrust(int leftPwm, int rightPwm);

    // Map a commanded thrust force (N) to an 8-bit PWM magnitude.
    int  forceToPwm(float forceN, float maxThrustN);

    void stop();

private:
    void driveSide(int pwm, uint8_t fwdChannel, uint8_t revChannel);
};

#endif // USV_MOTOR_DRIVER_H
