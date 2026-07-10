#ifndef USV_WATER_ANOMALY_MONITOR_H
#define USV_WATER_ANOMALY_MONITOR_H

#include <Arduino.h>

// Detects wastewater/pollution anomalies from pH, turbidity and temperature.
// Techniques: simple moving average (SMA) smoothing, turbidity rate-of-change
// (ROC), EMA baseline deviation, un-ionized ammonia toxicity, and a
// time-of-day pH envelope. See outdoor_pond_wastewater_insights.md.
//
// Decision #5: spiral is triggered by a single OR rule (see shouldTriggerSpiral).
class WaterAnomalyMonitor {
public:
    WaterAnomalyMonitor();

    // Feed the newest sample. hour = local hour 0-23 (for pH envelope).
    // Returns true if an anomaly is currently flagged.
    bool  updateReadings(float ph, float turbidity, float temp, int hour = 12);

    // Fraction of toxic un-ionized ammonia (NH3) given pH & temperature.
    float calculateAmmoniaToxicity(float ph, float temp);

    bool  shouldTriggerSpiral() const { return _triggerSpiral; }
    float lastAnomalyScore()    const { return _anomalyScore; }
    float turbidityRoc()        const { return _turbRoc; }

private:
    static const int SMA_WINDOW = 8;

    float _phBuf[SMA_WINDOW];
    float _turbBuf[SMA_WINDOW];
    int   _bufIdx;
    int   _bufCount;

    float _turbBaselineEma;   // slow EMA baseline
    float _lastTurb;
    unsigned long _lastSampleTime;

    bool  _triggerSpiral;
    float _anomalyScore;
    float _turbRoc;

    float sma(const float* buf) const;
    float phHighLimitForHour(int hour) const;
};

#endif // USV_WATER_ANOMALY_MONITOR_H
