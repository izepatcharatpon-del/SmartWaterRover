#ifndef USV_GRID_HEADING_CONTROLLER_H
#define USV_GRID_HEADING_CONTROLLER_H

#include "DriveController.h"

// GRID-only steering. Compass-derived rate is a damping estimate, never a
// readiness gate. DriveController still owns interlocks and motor slew.
class GridHeadingController {
public:
    GridHeadingController();
    DriveRequest update(float targetDeg, float headingDeg, int basePwm,
                        uint32_t sampleUs, bool fresh);
    void reset();
    bool isTurningInPlace() const { return _turnInPlace; }
    float headingRateDegPerSec() const { return _rate; }
    float steeringPwm() const { return _steering; }

private:
    bool _haveSample;
    bool _turnInPlace;
    uint32_t _lastSampleUs;
    float _lastHeading;
    float _rate;
    float _integralPwm;
    float _steering;
};

#endif // USV_GRID_HEADING_CONTROLLER_H
