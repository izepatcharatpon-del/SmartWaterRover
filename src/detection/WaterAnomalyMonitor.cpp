#include "WaterAnomalyMonitor.h"
#include "config.h"
#include <math.h>

WaterAnomalyMonitor::WaterAnomalyMonitor()
    : _bufIdx(0), _bufCount(0),
      _turbBaselineEma(0.0f), _lastTurb(0.0f),
      _lastSampleTime(millis()),      // fix: init in ctor, not at first sample
      _triggerSpiral(false), _anomalyScore(0.0f), _turbRoc(0.0f) {
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
    // Daytime photosynthesis raises pH; allow a higher ceiling 12:00-17:00.
    if (hour >= 12 && hour < 17) return PH_HIGH_LIMIT_DAY;   // 9.2
    return PH_HIGH_LIMIT_NIGHT;                              // 8.8
}

float WaterAnomalyMonitor::calculateAmmoniaToxicity(float ph, float temp) {
    // Fraction of un-ionized NH3 = 1 / (1 + 10^(pKa - pH)).
    float pKa = 0.09018f + 2729.92f / (temp + 273.15f);
    float fraction = 1.0f / (1.0f + powf(10.0f, pKa - ph));
    return fraction;   // 0..1
}

bool WaterAnomalyMonitor::updateReadings(float ph, float turbidity, float temp, int hour) {
    unsigned long now = millis();
    float dtMin = (now - _lastSampleTime) / 60000.0f;   // minutes
    _lastSampleTime = now;

    // --- Ring-buffer for SMA smoothing ---
    _phBuf[_bufIdx]   = ph;
    _turbBuf[_bufIdx] = turbidity;
    _bufIdx = (_bufIdx + 1) % SMA_WINDOW;
    if (_bufCount < SMA_WINDOW) _bufCount++;

    float phAvg   = sma(_phBuf);
    float turbAvg = sma(_turbBuf);

    // --- Turbidity rate-of-change (NTU per minute) ---
    if (dtMin > 1e-4f && dtMin < 10.0f) {
        _turbRoc = (turbidity - _lastTurb) / dtMin;
    } else {
        _turbRoc = 0.0f;
    }
    _lastTurb = turbidity;

    // --- EMA baseline (slow) + deviation ---
    if (_turbBaselineEma <= 0.0f) _turbBaselineEma = turbAvg;
    _turbBaselineEma = 0.98f * _turbBaselineEma + 0.02f * turbAvg;
    float baselineDev = turbAvg - _turbBaselineEma;

    // --- Individual anomaly conditions (Decision #5 unified OR rule) ---
    float phHigh = phHighLimitForHour(hour);
    bool phOut   = (phAvg < PH_LOW_LIMIT) || (phAvg > phHigh);
    bool turbHigh = (turbAvg > TURBIDITY_WASTEWATER_NTU);
    bool turbSpike = (_turbRoc > TURBIDITY_ROC_LIMIT);
    float toxicity = calculateAmmoniaToxicity(phAvg, temp);
    bool ammoniaBad = (toxicity > AMMONIA_TOXICITY_LIMIT);

    // --- Continuous anomaly score (0..1-ish) for telemetry / tuning ---
    float score = 0.0f;
    if (turbAvg > TURBIDITY_WARNING_NTU)
        score += (turbAvg - TURBIDITY_WARNING_NTU) /
                 (TURBIDITY_WASTEWATER_NTU - TURBIDITY_WARNING_NTU) * 0.4f;
    if (baselineDev > 0.0f) score += fminf(baselineDev / 50.0f, 0.3f);
    if (phOut)      score += 0.3f;
    if (ammoniaBad) score += 0.4f;
    if (turbSpike)  score += 0.3f;
    _anomalyScore = fminf(score, 1.5f);

    _triggerSpiral = turbHigh || turbSpike || phOut || ammoniaBad;
    return _triggerSpiral;
}
