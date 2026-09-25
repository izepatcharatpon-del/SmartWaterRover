#ifndef USV_WATER_ANOMALY_MONITOR_H
#define USV_WATER_ANOMALY_MONITOR_H

#include <stdint.h>

class WaterAnomalyMonitor {
public:
    WaterAnomalyMonitor();

    // Turbidity is the sole automatic spiral trigger. pH remains telemetry and
    // heatmap data, so an invalid pH sample cannot block this state machine.
    // motionDisturbance invalidates ROC history but not absolute/baseline tests.
    bool updateTurbidity(float turbidity, bool motionDisturbance,
                         uint32_t nowMs);
    void notifyTurbidityInvalid(uint32_t nowMs);

    bool shouldTriggerSpiral() const { return _triggerSpiral; }

    // Call when leaving SPIRAL — starts cooldown to avoid chatter.
    void notifySpiralDone(uint32_t nowMs);

    // New GRID mission — drop a STANDBY/dock latch. Keep cooldown.
    void clearPendingTrigger();

private:
    static const int SMA_WINDOW = 8;

    float _turbBuf[SMA_WINDOW];
    int   _bufIdx;
    int   _bufCount;

    float _turbBaselineEma;
    float _lastTurb;
    uint32_t _lastRocMs;
    bool  _haveLastTurb;
    uint32_t _lastValidMs;
    bool  _haveLastValid;

    bool  _rawAnomaly;
    bool  _triggerSpiral;
    float _anomalyScore;
    float _turbRoc;
    uint32_t _cooldownStartMs;
    bool  _cooldownActive;
    uint32_t _baselineHighSince;
    bool  _baselineHighActive;

    float sma() const;
    void resetPendingEvidence(bool resetBaseline);
};

#endif // USV_WATER_ANOMALY_MONITOR_H
