#include "PidController.h"
#include <math.h>

PidController::PidController(float kp, float ki, float kd,
                             float outMin, float outMax, float derivAlpha)
    : _kp(kp), _ki(ki), _kd(kd),
      _outMin(outMin), _outMax(outMax),
      _alpha(derivAlpha),
      _integral(0.0f), _prevError(0.0f), _prevDeriv(0.0f),
      _hasPrev(false),
      _angular(false) {}

void PidController::setAngular(bool angular) { _angular = angular; }

void PidController::reset() {
    _integral  = 0.0f;
    _prevError = 0.0f;
    _prevDeriv = 0.0f;
    _hasPrev   = false;
}

float PidController::update(float error, float dt) {
    if (dt <= 0.0f) dt = 1e-3f;
    if (dt > 0.25f) dt = 0.25f;

    float delta = error - _prevError;
    if (_angular) {
        while (delta >  180.0f) delta -= 360.0f;
        while (delta < -180.0f) delta += 360.0f;
    }
    float rawDeriv = _hasPrev ? (delta / dt) : 0.0f;
    float deriv    = _alpha * rawDeriv + (1.0f - _alpha) * _prevDeriv;

    _integral += error * dt;

    float output = _kp * error + _ki * _integral + _kd * deriv;

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
