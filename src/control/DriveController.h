#ifndef USV_DRIVE_CONTROLLER_H
#define USV_DRIVE_CONTROLLER_H

#include <stdint.h>

#include "core/MissionFsm.h"

enum class DriveSource : uint8_t {
    NONE = 0,
    MANUAL,
    GRID,
    SPIRAL,
    RTH
};

enum class DriveStopReason : uint8_t {
    NONE = 0,
    HARD_STOP,
    INACTIVE_STATE,
    NAVIGATION_UNAVAILABLE,
    WATER_SAMPLE,
    INVALID_REQUEST,
    SOURCE_CHANGE,
    REQUESTED_STOP
};

struct DriveRequest {
    DriveSource source;
    int leftPwm;
    int rightPwm;
    bool valid;
};

struct DriveInterlocks {
    BoatState state;
    SafetyState safety;
    bool emergencyPending;
    bool navigationAvailable;
    bool waterSampleHold;
};

struct DriveDecision {
    DriveSource source;
    DriveStopReason stopReason;
    int leftPwm;
    int rightPwm;
};

// The only policy gate between mission guidance and MotorDriver. Mission modes
// may propose thrust, but this class owns interlock priority, output limits and
// slew. Every stop is immediate; slew applies only between non-zero requests.
class DriveController {
public:
    DriveController(int pwmLimit, int minEffectivePwm,
                    float slewPwmPerSecond);

    DriveDecision resolve(const DriveRequest& request,
                          const DriveInterlocks& interlocks,
                          float dtSeconds);
    void reset();

private:
    int _pwmLimit;
    int _minEffectivePwm;
    float _slewPwmPerSecond;
    float _leftPwm;
    float _rightPwm;
    DriveSource _lastSource;

    bool sourceAllowed(DriveSource source, BoatState state) const;
    int clampPwm(int pwm) const;
    float slewAxis(float current, int target, float maxStep) const;
    DriveDecision stop(DriveStopReason reason);
};

#endif // USV_DRIVE_CONTROLLER_H
