#include "PidController.h"

PidController::PidController(float kp, float ki, float kd,
                             float outMin, float outMax, float derivAlpha)
    : _kp(kp), _ki(ki), _kd(kd),
      _outMin(outMin), _outMax(outMax),
      _alpha(derivAlpha),
      _integral(0.0f), _prevError(0.0f), _prevDeriv(0.0f),
      _hasPrev(false) {}

void PidController::setTunings(float kp, float ki, float kd) {
    _kp = kp; _ki = ki; _kd = kd;
}

void PidController::reset() {
    _integral  = 0.0f;
    _prevError = 0.0f;
    _prevDeriv = 0.0f;
    _hasPrev   = false;
}

float PidController::update(float error, float dt) {
    if (dt <= 0.0f) dt = 1e-3f;

    // --- Derivative on error with first-order low-pass filter ---
    float rawDeriv = _hasPrev ? (error - _prevError) / dt : 0.0f;
    float deriv    = _alpha * rawDeriv + (1.0f - _alpha) * _prevDeriv;

    // --- Tentative integral ---
    _integral += error * dt;

    float output = _kp * error + _ki * _integral + _kd * deriv;

    // --- Clamping anti-windup: if saturated, undo the integral step ---
    if (output > _outMax) {
        output = _outMax;
        _integral -= error * dt;
    } else if (output < _outMin) {
        output = _outMin;
        _integral -= error * dt;
    }

    _prevError = error;
    _prevDeriv = deriv;
    _hasPrev   = true;
    return output;
}
