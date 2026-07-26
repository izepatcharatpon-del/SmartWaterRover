#include "MotorDriver.h"
#include "config.h"

void MotorDriver::begin() {
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
    ledcWrite(fwdChannel, 0);
    ledcWrite(revChannel, 0);
    if (pwm > 0) {
        ledcWrite(fwdChannel, pwm);
    } else if (pwm < 0) {
        ledcWrite(revChannel, -pwm);
    }
}

void MotorDriver::writeThrust(int leftPwm, int rightPwm) {
    leftPwm  = constrain(leftPwm,  -MOTOR_PWM_LIMIT, MOTOR_PWM_LIMIT);
    rightPwm = constrain(rightPwm, -MOTOR_PWM_LIMIT, MOTOR_PWM_LIMIT);
    driveSide(leftPwm,  LEFT_LEDC_CH_IN1,  LEFT_LEDC_CH_IN2);
    driveSide(rightPwm, RIGHT_LEDC_CH_IN3, RIGHT_LEDC_CH_IN4);
}

void MotorDriver::stop() {
    ledcWrite(LEFT_LEDC_CH_IN1,  0);
    ledcWrite(LEFT_LEDC_CH_IN2,  0);
    ledcWrite(RIGHT_LEDC_CH_IN3, 0);
    ledcWrite(RIGHT_LEDC_CH_IN4, 0);
}
