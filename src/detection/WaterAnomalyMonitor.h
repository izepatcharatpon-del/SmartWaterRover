#ifndef USV_WATER_ANOMALY_MONITOR_H
#define USV_WATER_ANOMALY_MONITOR_H

#include <Arduino.h>

class WaterAnomalyMonitor {
public:
    WaterAnomalyMonitor();

    // Returns true if spiral should trigger (after hysteresis / cooldown).
    // motionDisturbance: recent hard thrust / turn — ROC-only spikes ignored.
    bool updateReadings(float ph, float turbidity, float temp,
                        bool sensorsValid, int hour = -1,
                        bool motionDisturbance = false);

    bool shouldTriggerSpiral() const { return _triggerSpiral; }

    // Call when leaving SPIRAL — starts cooldown to avoid chatter.
    void notifySpiralDone();

private:
    static const int SMA_WINDOW = 8;

    float _phBuf[SMA_WINDOW];
    float _turbBuf[SMA_WINDOW];
    int   _bufIdx;
    int   _bufCount;

    float _turbBaselineEma;
    float _lastTurb;
    unsigned long _lastSampleTime;
    unsigned long _lastRocMs;
    bool  _haveLastTurb;

    bool  _rawAnomaly;
    bool  _triggerSpiral;
    float _anomalyScore;
    float _turbRoc;
    unsigned long _cooldownUntil;
    unsigned long _baselineHighSince;
    bool  _baselineHighActive;

    float sma(const float* buf) const;
    float phHighLimitForHour(int hour) const;
};

#endif // USV_WATER_ANOMALY_MONITOR_H
