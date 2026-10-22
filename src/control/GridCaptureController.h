#ifndef USV_GRID_CAPTURE_CONTROLLER_H
#define USV_GRID_CAPTURE_CONTROLLER_H

#include "DriveController.h"
#include "HeadingController.h"

// GRID guidance proposes thrust; DriveController remains the final authority.
// Rate is compass-derived heading change, not independently measured hull yaw.
class GridCaptureController {
public:
    enum class Phase { TURN, CAPTURE, TRACK };
    GridCaptureController();
    // sampleUs is the successful compass-read timestamp (unclamped micros),
    // fresh describes that read; targetId is waypoint identity, not LOS angle.
    // Call every navigation tick, even while held/inactive. reset() explicitly
    // re-arms a timed-out acquisition; Drive requested-stop must not reset it.
    DriveRequest update(float targetDeg, float headingDeg, int basePwm,
                        uint32_t sampleUs, bool fresh, uint32_t targetId,
                        const DriveInterlocks& interlocks, bool automatic = true);
    void reset();
    Phase phase() const { return _phase; }
    float headingRateDegPerSec() const { return _rate; }
    bool timedOut() const { return _timedOut; }

private:
    PidController _pi;
    Phase _phase;
    float _rate;
    bool _timedOut;
    bool _haveSample;
    bool _haveRate;
    uint32_t _lastSampleUs;
    float _lastHeading;
    uint32_t _settledUs;
    bool _haveTarget;
    uint32_t _targetId;
    bool _acquiring;
    uint32_t _acquireStartUs;
    void discardMeasurement();
    void startAcquisition(uint32_t sampleUs);
};

#endif
