#include "WaterAnomalyMonitor.h"
#include "PhPolicy.h"
#include "config.h"
#include <math.h>

WaterAnomalyMonitor::WaterAnomalyMonitor()
    : _bufIdx(0), _bufCount(0),
      _turbBaselineEma(0.0f), _lastTurb(0.0f),
      _lastSampleTime(0), _lastRocMs(0),
      _haveLastTurb(false),
      _haveLastPh(false),
      _lastAcceptedPh(7.0f),
      _stepCandidatePh(7.0f),
      _stepCandidateCount(0),
      _phOutActive(false),
      _phOutSince(0),
      _rawAnomaly(false),
      _triggerSpiral(false), _anomalyScore(0.0f), _turbRoc(0.0f),
      _cooldownUntil(0),
      _baselineHighSince(0),
      _baselineHighActive(false) {
    for (int i = 0; i < SMA_WINDOW; i++) { _phBuf[i] = 7.0f; _turbBuf[i] = 0.0f; }
}

float WaterAnomalyMonitor::sma(const float* buf) const {
    int n = _bufCount < SMA_WINDOW ? _bufCount : SMA_WINDOW;
    if (n == 0) return buf[0];
    float sum = 0.0f;
    for (int i = 0; i < n; i++) sum += buf[i];
    return sum / n;
}

float WaterAnomalyMonitor::phHighLimitForHour(int hour) const {
    if (hour < 0) return PH_HIGH_LIMIT_DAY;
    if (hour >= 12 && hour < 17) return PH_HIGH_LIMIT_DAY;
    return PH_HIGH_LIMIT_NIGHT;
}

bool WaterAnomalyMonitor::acceptPhSample(float ph) {
    if (!usvPhPlausible(ph)) {
        _stepCandidateCount = 0;
        return false;
    }

    if (!_haveLastPh) {
        _lastAcceptedPh = ph;
        _haveLastPh = true;
        _stepCandidateCount = 0;
        return true;
    }

    if (fabsf(ph - _lastAcceptedPh) <= PH_GLITCH_MAX_STEP) {
        _lastAcceptedPh = ph;
        _stepCandidateCount = 0;
        return true;
    }

    if (_stepCandidateCount > 0 &&
        fabsf(ph - _stepCandidatePh) <= PH_GLITCH_MAX_STEP) {
        _stepCandidateCount++;
        _stepCandidatePh = ph;
    } else {
        _stepCandidatePh = ph;
        _stepCandidateCount = 1;
    }

    if (_stepCandidateCount >= PH_GLITCH_CONFIRM_SAMPLES) {
        _lastAcceptedPh = ph;
        _stepCandidateCount = 0;
        return true;
    }
    return false;
}

float WaterAnomalyMonitor::phToStore(float ph) {
    if (acceptPhSample(ph)) return ph;
    return _haveLastPh ? _lastAcceptedPh : 7.0f;
}

bool WaterAnomalyMonitor::updateReadings(float ph, float turbidity, float temp,
                                         bool sensorsValid, int hour,
                                         bool motionDisturbance,
                                         uint32_t nowMs) {
    if (!sensorsValid) {
        _triggerSpiral = false;
        return false;
    }

    _lastSampleTime = nowMs;

    _phBuf[_bufIdx]   = phToStore(ph);
    _turbBuf[_bufIdx] = turbidity;
    _bufIdx = (_bufIdx + 1) % SMA_WINDOW;
    if (_bufCount < SMA_WINDOW) _bufCount++;

    float phAvg   = sma(_phBuf);
    float turbAvg = sma(_turbBuf);

    if (!_haveLastTurb) {
        _lastTurb = turbAvg;
        _lastRocMs = nowMs;
        _haveLastTurb = true;
        _turbRoc = 0.0f;
    } else if (nowMs - _lastRocMs >= ANOMALY_ROC_WINDOW_MS) {
        float dtMin = (nowMs - _lastRocMs) / 60000.0f;
        if (dtMin > 1e-4f && dtMin < 10.0f) {
            _turbRoc = (turbAvg - _lastTurb) / dtMin;
        } else {
            _turbRoc = 0.0f;
        }
        _lastTurb = turbAvg;
        _lastRocMs = nowMs;
    }

    if (_turbBaselineEma <= 0.0f) _turbBaselineEma = turbAvg;
    _turbBaselineEma = 0.98f * _turbBaselineEma + 0.02f * turbAvg;
    float baselineDev = turbAvg - _turbBaselineEma;

    float phHigh = phHighLimitForHour(hour);
    bool phOutOfRange = _haveLastPh &&
                        ((phAvg < PH_LOW_LIMIT) || (phAvg > phHigh));
    if (phOutOfRange) {
        if (!_phOutActive) {
            _phOutActive = true;
            _phOutSince = nowMs;
        }
    } else {
        _phOutActive = false;
    }
    bool phOut = _phOutActive &&
                 (nowMs - _phOutSince >= PH_ANOMALY_DWELL_MS);

    bool turbHigh = (turbAvg > TURBIDITY_WASTEWATER_NTU);
    bool turbSpike = (_turbRoc > TURBIDITY_ROC_LIMIT);
    if (motionDisturbance) {
        turbSpike = false;
    }

    bool baselineRising = (baselineDev > ANOMALY_BASELINE_DELTA_NTU);
    if (baselineRising) {
        if (!_baselineHighActive) {
            _baselineHighActive = true;
            _baselineHighSince = nowMs;
        }
    } else {
        _baselineHighActive = false;
    }
    bool baselineDwell =
        _baselineHighActive &&
        (nowMs - _baselineHighSince >= ANOMALY_BASELINE_DWELL_MS);

    (void)temp;
    float score = 0.0f;
    if (turbAvg > TURBIDITY_WARNING_NTU)
        score += (turbAvg - TURBIDITY_WARNING_NTU) /
                 (TURBIDITY_WASTEWATER_NTU - TURBIDITY_WARNING_NTU) * 0.4f;
    if (baselineDev > 0.0f) score += fminf(baselineDev / 50.0f, 0.3f);
    if (phOut)     score += 0.3f;
    if (turbSpike) score += 0.3f;
    if (baselineDwell) score += 0.2f;
    _anomalyScore = fminf(score, 1.5f);

    const float turbClearLevel =
        TURBIDITY_WASTEWATER_NTU - ANOMALY_CLEAR_HYST_NTU;
    if (_rawAnomaly) {
        const bool turbStillHigh = (turbAvg > turbClearLevel);
        _rawAnomaly = turbStillHigh || turbSpike || phOut || baselineDwell;
    } else {
        _rawAnomaly = turbHigh || turbSpike || phOut || baselineDwell;
    }

    if (_rawAnomaly) {
        if (nowMs >= _cooldownUntil) {
            _triggerSpiral = true;
        }
    } else {
        _triggerSpiral = false;
    }

    return _triggerSpiral;
}

void WaterAnomalyMonitor::notifySpiralDone(uint32_t nowMs) {
    _triggerSpiral = false;
    _cooldownUntil = nowMs + ANOMALY_COOLDOWN_MS;
    _baselineHighActive = false;
}
