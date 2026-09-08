#ifndef USV_WATER_ANOMALY_MONITOR_H
#define USV_WATER_ANOMALY_MONITOR_H

#include <stdint.h>

class WaterAnomalyMonitor {
public:
    WaterAnomalyMonitor();

    // Returns true if spiral should trigger (after hysteresis / cooldown).
    // motionDisturbance: recent hard thrust / turn — ROC-only spikes ignored.
    // nowMs: caller clock (millis on device) so this stays host-testable.
    // Invalid / implausible pH must not be written into the SMA; pass NAN.
    bool updateReadings(float ph, float turbidity, float temp,
                        bool sensorsValid, int hour,
                        bool motionDisturbance,
                        uint32_t nowMs);

    bool shouldTriggerSpiral() const { return _triggerSpiral; }

    // Call when leaving SPIRAL — starts cooldown to avoid chatter.
    void notifySpiralDone(uint32_t nowMs);

private:
    static const int SMA_WINDOW = 8;

    float _phBuf[SMA_WINDOW];
    float _turbBuf[SMA_WINDOW];
    int   _bufIdx;
    int   _bufCount;

    float _turbBaselineEma;
    float _lastTurb;
    uint32_t _lastSampleTime;
    uint32_t _lastRocMs;
    bool  _haveLastTurb;

    bool  _haveLastPh;
    float _lastAcceptedPh;
    float _stepCandidatePh;
    int   _stepCandidateCount;
    bool  _phOutActive;
    uint32_t _phOutSince;

    bool  _rawAnomaly;
    bool  _triggerSpiral;
    float _anomalyScore;
    float _turbRoc;
    uint32_t _cooldownUntil;
    uint32_t _baselineHighSince;
    bool  _baselineHighActive;

    float sma(const float* buf) const;
    float phHighLimitForHour(int hour) const;
    bool  acceptPhSample(float ph);
    float phToStore(float ph);
};

#endif // USV_WATER_ANOMALY_MONITOR_H
