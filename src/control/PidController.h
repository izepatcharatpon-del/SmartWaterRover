#ifndef USV_PID_CONTROLLER_H
#define USV_PID_CONTROLLER_H

// Generic PID with low-pass filtered derivative and clamping anti-windup.
// See PID_Control_Guide.md and codebase_mathematical_models.md.
class PidController {
public:
    PidController(float kp, float ki, float kd,
                  float outMin, float outMax,
                  float derivAlpha = 0.30f);

    float update(float error, float dt);
    void  reset();

    void  setTunings(float kp, float ki, float kd);

private:
    float _kp, _ki, _kd;
    float _outMin, _outMax;
    float _alpha;          // derivative low-pass factor
    float _integral;
    float _prevError;
    float _prevDeriv;
    bool  _hasPrev;
};

#endif // USV_PID_CONTROLLER_H
