#include "MotorDriver.h"
#include "config.h"

void MotorDriver::begin() {
    // Arduino-ESP32 core 2.x requires ledcSetup and ledcAttachPin.
    ledcSetup(LEFT_LEDC_CH_IN1,  MOTOR_PWM_FREQ, MOTOR_PWM_RES);
    ledcAttachPin(LEFT_MOTOR_IN1, LEFT_LEDC_CH_IN1);
    ledcSetup(LEFT_LEDC_CH_IN2,  MOTOR_PWM_FREQ, MOTOR_PWM_RES);
    ledcAttachPin(LEFT_MOTOR_IN2, LEFT_LEDC_CH_IN2);
    ledcSetup(RIGHT_LEDC_CH_IN3, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
    ledcAttachPin(RIGHT_MOTOR_IN3, RIGHT_LEDC_CH_IN3);
    ledcSetup(RIGHT_LEDC_CH_IN4, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
    ledcAttachPin(RIGHT_MOTOR_IN4, RIGHT_LEDC_CH_IN4);
    stop();
}

void MotorDriver::driveSide(int pwm, uint8_t fwdChannel, uint8_t revChannel) {
    if (pwm >= 0) {
        ledcWrite(fwdChannel, pwm);
        ledcWrite(revChannel, 0);
    } else {
        ledcWrite(fwdChannel, 0);
        ledcWrite(revChannel, -pwm);
    }
}

void MotorDriver::writeThrust(int leftPwm, int rightPwm) {
    leftPwm  = constrain(leftPwm,  -MOTOR_PWM_LIMIT, MOTOR_PWM_LIMIT);
    rightPwm = constrain(rightPwm, -MOTOR_PWM_LIMIT, MOTOR_PWM_LIMIT);
    driveSide(leftPwm,  LEFT_LEDC_CH_IN1,  LEFT_LEDC_CH_IN2);
    driveSide(rightPwm, RIGHT_LEDC_CH_IN3, RIGHT_LEDC_CH_IN4);
}

int MotorDriver::forceToPwm(float forceN, float maxThrustN) {
    if (maxThrustN <= 0.0f) return 0;
    float ratio = forceN / maxThrustN;                 // -1..1
    ratio = constrain(ratio, -1.0f, 1.0f);
    int pwm = (int)(ratio * MOTOR_PWM_LIMIT);
    // Deadband compensation: below MOTOR_MIN_PWM the thruster does not move.
    if (pwm > 0 && pwm < MOTOR_MIN_PWM) pwm = MOTOR_MIN_PWM;
    if (pwm < 0 && pwm > -MOTOR_MIN_PWM) pwm = -MOTOR_MIN_PWM;
    return pwm;
}

void MotorDriver::stop() {
    ledcWrite(LEFT_LEDC_CH_IN1,  0);
    ledcWrite(LEFT_LEDC_CH_IN2,  0);
    ledcWrite(RIGHT_LEDC_CH_IN3, 0);
    ledcWrite(RIGHT_LEDC_CH_IN4, 0);
}
