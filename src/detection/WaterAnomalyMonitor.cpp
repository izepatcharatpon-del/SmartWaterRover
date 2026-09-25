#include "WaterAnomalyMonitor.h"
#include "config.h"
#include <math.h>

WaterAnomalyMonitor::WaterAnomalyMonitor()
    : _bufIdx(0), _bufCount(0),
      _turbBaselineEma(0.0f), _lastTurb(0.0f),
      _lastRocMs(0),
      _haveLastTurb(false),
      _lastValidMs(0),
      _haveLastValid(false),
      _rawAnomaly(false),
      _triggerSpiral(false), _anomalyScore(0.0f), _turbRoc(0.0f),
      _cooldownStartMs(0),
      _cooldownActive(false),
      _baselineHighSince(0),
      _baselineHighActive(false) {
    for (int i = 0; i < SMA_WINDOW; i++) _turbBuf[i] = 0.0f;
}

float WaterAnomalyMonitor::sma() const {
    int n = _bufCount < SMA_WINDOW ? _bufCount : SMA_WINDOW;
    if (n == 0) return 0.0f;
    float sum = 0.0f;
    for (int i = 0; i < n; i++) sum += _turbBuf[i];
    return sum / n;
}

bool WaterAnomalyMonitor::updateTurbidity(float turbidity,
                                          bool motionDisturbance,
                                          uint32_t now) {
    if (!isfinite(turbidity)) {
        notifyTurbidityInvalid(now);
        return _triggerSpiral;
    }
    if (_haveLastValid &&
        (uint32_t)(now - _lastValidMs) >= ANOMALY_SENSOR_STALE_MS) {
        resetPendingEvidence(false);
    }
    _lastValidMs = now;
    _haveLastValid = true;

    _turbBuf[_bufIdx] = turbidity;
    _bufIdx = (_bufIdx + 1) % SMA_WINDOW;
    if (_bufCount < SMA_WINDOW) _bufCount++;

    const bool bufferReady = _bufCount >= SMA_WINDOW;
    float turbAvg = sma();

    if (!_haveLastTurb) {
        _lastTurb = turbAvg;
        _lastRocMs = now;
        _haveLastTurb = true;
        _turbRoc = 0.0f;
    } else if (motionDisturbance) {
        // Do not let a pre-thrust ROC survive until the gate expires.
        _turbRoc = 0.0f;
        _lastTurb = turbAvg;
        _lastRocMs = now;
    } else if (now - _lastRocMs >= ANOMALY_ROC_WINDOW_MS) {
        float dtMin = (now - _lastRocMs) / 60000.0f;
        if (dtMin > 1e-4f && dtMin < 10.0f) {
            _turbRoc = (turbAvg - _lastTurb) / dtMin;
        } else {
            _turbRoc = 0.0f;
        }
        _lastTurb = turbAvg;
        _lastRocMs = now;
    }

    if (_turbBaselineEma <= 0.0f) _turbBaselineEma = turbAvg;
    float baselineDev = turbAvg - _turbBaselineEma;

    // Strong events still need a complete 8-sample average; one ADC spike must
    // not redirect the boat. ROC is diagnostic/candidate-only.
    bool turbHigh = bufferReady &&
                    (turbAvg > TURBIDITY_WASTEWATER_NTU);
    const bool turbDirtyEnough = (turbAvg > TURBIDITY_WARNING_NTU);
    // The baseline represents clean local water. Once readings exceed the
    // warning level, freeze it so a mild plume is not absorbed during dwell.
    if (!turbDirtyEnough) {
        _turbBaselineEma = 0.98f * _turbBaselineEma + 0.02f * turbAvg;
        baselineDev = turbAvg - _turbBaselineEma;
    }
    bool turbSpike = bufferReady && !motionDisturbance && turbDirtyEnough &&
                     (_turbRoc > TURBIDITY_ROC_LIMIT);

    bool baselineRising =
        bufferReady && turbDirtyEnough &&
        (baselineDev > ANOMALY_BASELINE_DELTA_NTU);
    if (baselineRising) {
        if (!_baselineHighActive) {
            _baselineHighActive = true;
            _baselineHighSince = now;
        }
    } else {
        _baselineHighActive = false;
    }
    bool baselineDwell =
        _baselineHighActive &&
        (now - _baselineHighSince >= ANOMALY_BASELINE_DWELL_MS);

    float score = 0.0f;
    if (turbAvg > TURBIDITY_WARNING_NTU)
        score += (turbAvg - TURBIDITY_WARNING_NTU) /
                 (TURBIDITY_WASTEWATER_NTU - TURBIDITY_WARNING_NTU) * 0.4f;
    if (baselineDev > 0.0f) score += fminf(baselineDev / 50.0f, 0.3f);
    if (turbSpike) score += 0.3f;
    if (baselineDwell) score += 0.2f;
    _anomalyScore = fminf(score, 1.5f);

    const float turbClearLevel =
        TURBIDITY_WASTEWATER_NTU - ANOMALY_CLEAR_HYST_NTU;
    if (_rawAnomaly) {
        const bool turbStillHigh = (turbAvg > turbClearLevel);
        _rawAnomaly = turbStillHigh || baselineDwell;
    } else {
        _rawAnomaly = turbHigh || baselineDwell;
    }

    if (_cooldownActive &&
        (uint32_t)(now - _cooldownStartMs) >= ANOMALY_COOLDOWN_MS) {
        _cooldownActive = false;
    }
    if (_rawAnomaly) {
        if (!_cooldownActive) {
            _triggerSpiral = true;
        }
    } else {
        _triggerSpiral = false;
    }

    return _triggerSpiral;
}

void WaterAnomalyMonitor::notifyTurbidityInvalid(uint32_t nowMs) {
    if (_haveLastValid &&
        (uint32_t)(nowMs - _lastValidMs) >= ANOMALY_SENSOR_STALE_MS) {
        // Preserve an already-issued trigger and cooldown, but never allow a
        // long sensor outage to complete an SMA or dwell started beforehand.
        resetPendingEvidence(false);
        _haveLastValid = false;
    }
}

void WaterAnomalyMonitor::notifySpiralDone(uint32_t nowMs) {
    _triggerSpiral = false;
    _cooldownStartMs = nowMs;
    _cooldownActive = true;
    _baselineHighActive = false;
}

void WaterAnomalyMonitor::clearPendingTrigger() {
    _triggerSpiral = false;
    resetPendingEvidence(true);
    _haveLastValid = false;
}

void WaterAnomalyMonitor::resetPendingEvidence(bool resetBaseline) {
    _rawAnomaly = false;
    _baselineHighActive = false;
    _haveLastTurb = false;
    _turbRoc = 0.0f;
    if (resetBaseline) _turbBaselineEma = 0.0f;
    _bufIdx = 0;
    _bufCount = 0;
    for (int i = 0; i < SMA_WINDOW; i++) _turbBuf[i] = 0.0f;
}
