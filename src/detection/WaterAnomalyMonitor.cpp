#include "WaterAnomalyMonitor.h"
#include "config.h"
#include <math.h>

WaterAnomalyMonitor::WaterAnomalyMonitor()
    : _bufIdx(0), _bufCount(0),
      _turbBaselineEma(0.0f), _lastTurb(0.0f),
      _lastSampleTime(0), _lastRocMs(0),
      _haveLastTurb(false),
      _rawAnomaly(false),
      _triggerSpiral(false), _anomalyScore(0.0f), _turbRoc(0.0f),
      _cooldownUntil(0),
      _baselineHighSince(0),
      _baselineHighActive(false) {
    for (int i = 0; i < SMA_WINDOW; i++) { _phBuf[i] = 7.0f; _turbBuf[i] = 0.0f; }
    _lastSampleTime = millis();
    _lastRocMs = _lastSampleTime;
}

float WaterAnomalyMonitor::sma(const float* buf) const {
    int n = _bufCount < SMA_WINDOW ? _bufCount : SMA_WINDOW;
    if (n == 0) return buf[0];
    float sum = 0.0f;
    for (int i = 0; i < n; i++) sum += buf[i];
    return sum / n;
}

float WaterAnomalyMonitor::phHighLimitForHour(int hour) const {
    // Unknown GPS time: use day (looser) ceiling to avoid dawn false spirals.
    if (hour < 0) return PH_HIGH_LIMIT_DAY;
    if (hour >= 12 && hour < 17) return PH_HIGH_LIMIT_DAY;
    return PH_HIGH_LIMIT_NIGHT;
}

bool WaterAnomalyMonitor::updateReadings(float ph, float turbidity, float temp,
                                         bool sensorsValid, int hour,
                                         bool motionDisturbance) {
    // Sensor fault: suppress trigger but keep Schmitt latch (avoid glitch reset).
    if (!sensorsValid) {
        _triggerSpiral = false;
        return false;
    }

    unsigned long now = millis();
    _lastSampleTime = now;

    _phBuf[_bufIdx]   = ph;
    _turbBuf[_bufIdx] = turbidity;
    _bufIdx = (_bufIdx + 1) % SMA_WINDOW;
    if (_bufCount < SMA_WINDOW) _bufCount++;

    float phAvg   = sma(_phBuf);
    float turbAvg = sma(_turbBuf);

    // ROC over a fixed ≥2 s window so ~4 Hz ADS noise cannot inflate NTU/min.
    if (!_haveLastTurb) {
        _lastTurb = turbAvg;
        _lastRocMs = now;
        _haveLastTurb = true;
        _turbRoc = 0.0f;
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
    _turbBaselineEma = 0.98f * _turbBaselineEma + 0.02f * turbAvg;
    float baselineDev = turbAvg - _turbBaselineEma;

    float phHigh = phHighLimitForHour(hour);
    bool phOut   = (phAvg < PH_LOW_LIMIT) || (phAvg > phHigh);
    bool turbHigh = (turbAvg > TURBIDITY_WASTEWATER_NTU);
    bool turbSpike = (_turbRoc > TURBIDITY_ROC_LIMIT);
    if (motionDisturbance) {
        // Prop wash / hard turns often spike ROC without real wastewater.
        turbSpike = false;
    }

    // Sustained rise above slow baseline (catches slow pollution < absolute NTU).
    bool baselineRising = (baselineDev > ANOMALY_BASELINE_DELTA_NTU);
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

    // Score is telemetry/debug only — ammonia equilibrium fraction is NOT a
    // standalone trigger (needs real TAN; pH alone already covers chemistry).
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
        if (now >= _cooldownUntil) {
            _triggerSpiral = true;
        }
    } else {
        _triggerSpiral = false;
    }

    return _triggerSpiral;
}

void WaterAnomalyMonitor::notifySpiralDone() {
    _triggerSpiral = false;
    _cooldownUntil = millis() + ANOMALY_COOLDOWN_MS;
    _baselineHighActive = false;
}
