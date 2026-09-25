#include "DriveController.h"

#include <math.h>

DriveController::DriveController(int pwmLimit, int minEffectivePwm,
                                 float slewPwmPerSecond)
    : _pwmLimit(pwmLimit > 0 ? pwmLimit : 0),
      _minEffectivePwm(
          minEffectivePwm > 0 && minEffectivePwm <= pwmLimit
              ? minEffectivePwm : 0),
      _slewPwmPerSecond(slewPwmPerSecond > 0.0f ? slewPwmPerSecond : 0.0f),
      _leftPwm(0.0f),
      _rightPwm(0.0f),
      _lastSource(DriveSource::NONE) {}

void DriveController::reset() {
    _leftPwm = 0.0f;
    _rightPwm = 0.0f;
    _lastSource = DriveSource::NONE;
}

bool DriveController::sourceAllowed(DriveSource source, BoatState state) const {
    switch (state) {
        case STATE_GRID_PATROL:
            return source == DriveSource::GRID || source == DriveSource::MANUAL;
        case STATE_SPIRAL_MAPPING:
            return source == DriveSource::SPIRAL;
        case STATE_RTH:
            return source == DriveSource::RTH;
        case STATE_STANDBY:
        case STATE_EMERGENCY_STOP:
            return false;
    }
    return false;
}

int DriveController::clampPwm(int pwm) const {
    if (pwm > _pwmLimit) return _pwmLimit;
    if (pwm < -_pwmLimit) return -_pwmLimit;
    return pwm;
}

float DriveController::slewAxis(float current, int target, float maxStep) const {
    float effectiveTarget = (float)target;
    const int targetMagnitude = target < 0 ? -target : target;
    if (target != 0 && targetMagnitude < _minEffectivePwm) {
        effectiveTarget = target > 0 ? (float)_minEffectivePwm
                                     : (float)-_minEffectivePwm;
    }

    if (effectiveTarget == 0.0f) {
        const float nextMagnitude = fabsf(current) - maxStep;
        if (nextMagnitude < (float)_minEffectivePwm) return 0.0f;
        return current > 0.0f ? nextMagnitude : -nextMagnitude;
    }
    if (current == 0.0f) {
        if (_minEffectivePwm > 0) {
            return effectiveTarget > 0.0f ? (float)_minEffectivePwm
                                          : (float)-_minEffectivePwm;
        }
        if (effectiveTarget > maxStep) return maxStep;
        if (effectiveTarget < -maxStep) return -maxStep;
        return effectiveTarget;
    }
    if ((current > 0.0f) != (effectiveTarget > 0.0f)) {
        // Direction changes must spend at least one complete navigation tick
        // at neutral. Ramp down on the old sign; never jump through zero.
        const float nextMagnitude = fabsf(current) - maxStep;
        if (nextMagnitude < (float)_minEffectivePwm) return 0.0f;
        return current > 0.0f ? nextMagnitude : -nextMagnitude;
    }

    const float delta = effectiveTarget - current;
    if (delta > maxStep) return current + maxStep;
    if (delta < -maxStep) return current - maxStep;
    return effectiveTarget;
}

DriveDecision DriveController::stop(DriveStopReason reason) {
    reset();
    DriveDecision out = {};
    out.source = DriveSource::NONE;
    out.stopReason = reason;
    return out;
}

DriveDecision DriveController::resolve(const DriveRequest& request,
                                       const DriveInterlocks& interlocks,
                                       float dtSeconds) {
    const bool fatalSafety =
        interlocks.safety == SafetyState::CRITICAL_STOP ||
        interlocks.safety == SafetyState::OVERCURRENT_STOP ||
        interlocks.safety == SafetyState::LEAK_STOP;
    if (interlocks.emergencyPending || fatalSafety ||
        interlocks.state == STATE_EMERGENCY_STOP) {
        return stop(DriveStopReason::HARD_STOP);
    }
    if (interlocks.state == STATE_STANDBY) {
        return stop(DriveStopReason::INACTIVE_STATE);
    }
    if (!interlocks.navigationAvailable) {
        return stop(DriveStopReason::NAVIGATION_UNAVAILABLE);
    }
    if (interlocks.waterSampleHold) {
        return stop(DriveStopReason::WATER_SAMPLE);
    }
    if (!request.valid || !sourceAllowed(request.source, interlocks.state)) {
        return stop(DriveStopReason::INVALID_REQUEST);
    }

    if (_lastSource != DriveSource::NONE &&
        _lastSource != request.source) {
        DriveDecision out = stop(DriveStopReason::SOURCE_CHANGE);
        // Remember the accepted new owner. The current tick is neutral; the
        // following tick may start that source from the effective deadband.
        _lastSource = request.source;
        return out;
    }
    _lastSource = request.source;

    const int targetLeft = clampPwm(request.leftPwm);
    const int targetRight = clampPwm(request.rightPwm);
    if (targetLeft == 0 && targetRight == 0) {
        return stop(DriveStopReason::REQUESTED_STOP);
    }

    if (dtSeconds < 0.0f) dtSeconds = 0.0f;
    if (dtSeconds > 0.25f) dtSeconds = 0.25f;
    float maxStep = _slewPwmPerSecond * dtSeconds;
    if (_slewPwmPerSecond <= 0.0f) {
        maxStep = (float)(_pwmLimit * 2);
    } else if (maxStep < 1.0f) {
        maxStep = 1.0f;
    }

    _leftPwm = slewAxis(_leftPwm, targetLeft, maxStep);
    _rightPwm = slewAxis(_rightPwm, targetRight, maxStep);

    DriveDecision out = {};
    out.source = request.source;
    out.stopReason = DriveStopReason::NONE;
    out.leftPwm = clampPwm((int)lroundf(_leftPwm));
    out.rightPwm = clampPwm((int)lroundf(_rightPwm));
    return out;
}
