#ifndef USV_HEADING_CONTROLLER_H
#define USV_HEADING_CONTROLLER_H

#include "PidController.h"

// Turns a desired bearing + current heading into differential thruster PWM.
// Uses shortest-turn error and a PID correction around a base cruise speed.
class HeadingController {
public:
    HeadingController(float kp, float ki, float kd);

    // Shortest signed turn (-180..180) from current to target heading.
    static float computeHeadingError(float targetDeg, float currentDeg);

    // Fills left/right PWM: left = base + corr, right = base - corr.
    void calculateThrusterOutputs(float targetBearing, float currentHeading,
                                  int basePwm, float dt,
                                  int& leftPwm, int& rightPwm);

    void reset() { _pid.reset(); }

private:
    PidController _pid;
};

#endif // USV_HEADING_CONTROLLER_H
