#include "GridCaptureController.h"
#include "config.h"
#include <math.h>

static_assert(MOTOR_MIN_PWM <= GRID_CAPTURE_FORWARD_PWM &&
              GRID_CAPTURE_FORWARD_PWM <= GRID_CAPTURE_PWM_LIMIT &&
              GRID_CAPTURE_PWM_LIMIT <= MOTOR_PWM_LIMIT,
              "GRID capture bounds must include effective motor deadband");
static_assert(GRID_CAPTURE_SAMPLE_MIN_US > 0 &&
              GRID_CAPTURE_SAMPLE_MIN_US <= GRID_CAPTURE_SAMPLE_MAX_US &&
              GRID_CAPTURE_SAMPLE_MAX_US < GRID_CAPTURE_DWELL_US &&
              GRID_CAPTURE_DWELL_US < GRID_CAPTURE_TIMEOUT_US &&
              GRID_CAPTURE_TIMEOUT_US < 0x80000000UL,
              "GRID capture timing must be finite and wrap-safe");
static_assert(GRID_CAPTURE_RATE_TAU_S > 0.0f && GRID_CAPTURE_RATE_MAX_DPS > 0.0f,
              "GRID rate filter and readiness bound must be positive");

GridCaptureController::GridCaptureController()
    : _pi(HEADING_KP, HEADING_KI, 0.0f,
          -MOTOR_PWM_LIMIT, MOTOR_PWM_LIMIT, PID_DERIV_ALPHA) { reset(); }

void GridCaptureController::reset() {
    discardMeasurement();
    _timedOut = false;
    _haveTarget = false;
    _targetId = 0;
    _acquiring = false;
    _acquireStartUs = 0;
}

void GridCaptureController::startAcquisition(uint32_t sampleUs) {
    if (!_acquiring) {
        _acquiring = true;
        _acquireStartUs = sampleUs;
    }
}

void GridCaptureController::discardMeasurement() {
    _pi.reset();
    _phase = Phase::TURN;
    _rate = 0.0f;
    _haveSample = false;
    _haveRate = false;
    _lastSampleUs = 0;
    _lastHeading = 0.0f;
    _settledUs = 0;
}

DriveRequest GridCaptureController::update(
        float targetDeg, float headingDeg, int basePwm,
        uint32_t sampleUs, bool fresh, uint32_t targetId,
        const DriveInterlocks& interlocks, bool automatic) {
    DriveRequest out = {DriveSource::GRID, 0, 0, true};
    const bool fatalSafety = interlocks.safety == SafetyState::CRITICAL_STOP ||
        interlocks.safety == SafetyState::OVERCURRENT_STOP ||
        interlocks.safety == SafetyState::LEAK_STOP;
    if (!automatic || interlocks.state != STATE_GRID_PATROL ||
        interlocks.waterSampleHold || interlocks.emergencyPending ||
        fatalSafety) {
        reset();
        out.valid = false;
        return out;
    }
    if (!_haveTarget || targetId != _targetId) {
        reset();
        _haveTarget = true;
        _targetId = targetId;
    }
    if (_phase != Phase::TRACK) startAcquisition(sampleUs);
    if (_timedOut || (_acquiring &&
                     sampleUs - _acquireStartUs >= GRID_CAPTURE_TIMEOUT_US)) {
        // Explicit latched coast/stop, never automatic cruise on timeout.
        // Intermittent invalid samples cannot renew this deadline. A new
        // target, owner/mode, hold or explicit reset starts a new acquisition.
        _timedOut = true;
        discardMeasurement();
        return out;
    }
    // Reject before angular arithmetic or float-to-int conversion. Compass
    // and LOS APIs provide normalized degrees; enormous finite values are
    // not plausible measurements and must not enter wrap loops.
    if (!interlocks.navigationAvailable || !fresh ||
        !isfinite(targetDeg) || !isfinite(headingDeg) ||
        targetDeg < 0.0f || targetDeg > 360.0f ||
        headingDeg < 0.0f || headingDeg > 360.0f) {
        discardMeasurement();
        startAcquisition(sampleUs);
        out.valid = false;
        return out;
    }
    const float error = HeadingController::computeHeadingError(targetDeg, headingDeg);
    const uint32_t elapsed = sampleUs - _lastSampleUs;
    if (_haveSample && (elapsed < GRID_CAPTURE_SAMPLE_MIN_US ||
                        elapsed > GRID_CAPTURE_SAMPLE_MAX_US)) {
        discardMeasurement();
        startAcquisition(sampleUs);
        out.valid = false;
        return out;
    }
    const bool timedPair = _haveSample;
    const float dt = timedPair ? elapsed / 1000000.0f : 0.0f;
    const float rawRate = timedPair ? HeadingController::computeHeadingError(headingDeg, _lastHeading)
                                      / dt : 0.0f;
    if (timedPair) {
        const float alpha = dt / (GRID_CAPTURE_RATE_TAU_S + dt);
        // Seed from an actual timed pair; do not attenuate unknown initial
        // rotation toward a fictitious zero rate.
        _rate = _haveRate ? _rate + alpha * (rawRate - _rate) : rawRate;
        _haveRate = true;
    }
    const bool rateSettled = timedPair && fabsf(_rate) <= GRID_CAPTURE_RATE_MAX_DPS &&
                             fabsf(rawRate) <= GRID_CAPTURE_RATE_MAX_DPS;
    const bool settled = rateSettled && fabsf(error) <= LOS_TURN_EXIT_DEG;
    if (_phase == Phase::TRACK && rateSettled && fabsf(error) < LOS_TURN_FIRST_DEG) {
        // Retain angle hysteresis after readiness has been earned. Fresh
        // excessive rate still removes TRACK immediately at any angle.
    } else if (!settled) {
        _settledUs = 0;
        _phase = Phase::TURN;
    } else {
        // CAPTURE means the previous sample also qualified. On entry from
        // TURN, start at this sample, not at the preceding unproven interval.
        if (_phase != Phase::CAPTURE) _settledUs = 0;
        else if (_settledUs < GRID_CAPTURE_DWELL_US) _settledUs += elapsed;
        _phase = _settledUs >= GRID_CAPTURE_DWELL_US ? Phase::TRACK : Phase::CAPTURE;
    }
    if (_phase == Phase::TRACK) _acquiring = false;
    else startAcquisition(sampleUs);
    _haveSample = true;
    _lastSampleUs = sampleUs;
    _lastHeading = headingDeg;
    const int base = _phase == Phase::TURN ? 0 :
        (_phase == Phase::CAPTURE && basePwm > GRID_CAPTURE_FORWARD_PWM
            ? GRID_CAPTURE_FORWARD_PWM : basePwm);
    // Retain P/I gains. D is on the compass measurement, never LOS error.
    const float correction = (timedPair ? _pi.update(error, dt) : HEADING_KP * error)
                             - HEADING_KD * _rate;
    auto clamp = [](int v, int low, int high) {
        return v < low ? low : (v > high ? high : v);
    };
    auto lift = [](int v) {
        return v > 0 && v < MOTOR_MIN_PWM ? MOTOR_MIN_PWM : v;
    };
    if (_phase == Phase::TURN || base <= 0) {
        const bool near = fabsf(error) <= LOS_TURN_EXIT_DEG;
        if ((!near || fabsf(_rate) > GRID_CAPTURE_RATE_MAX_DPS ||
             fabsf(error) > HEADING_STOP_TOL_DEG) &&
            fabsf(correction) > GRID_CAPTURE_CORR_DEADBAND_PWM) {
            // Deliberate signed pivot/counterturn only in TURN. The Drive
            // gate enforces deadband, slew and a neutral tick on reversal.
            const int limit = near ? GRID_CAPTURE_PWM_LIMIT : MOTOR_PWM_LIMIT;
            const int turn = clamp((int)fabsf(correction), MOTOR_MIN_PWM, limit);
            out.leftPwm = correction > 0 ? turn : -turn;
            out.rightPwm = -out.leftPwm;
        }
    } else {
        const int limit = _phase == Phase::CAPTURE ? GRID_CAPTURE_PWM_LIMIT : MOTOR_PWM_LIMIT;
        // Forward capture/tracking cannot introduce reverse thrust.
        out.leftPwm = lift(clamp(base + (int)correction, 0, limit));
        out.rightPwm = lift(clamp(base - (int)correction, 0, limit));
    }
    return out;
}
