#ifndef USV_PID_CONTROLLER_H
#define USV_PID_CONTROLLER_H

class PidController {
public:
    PidController(float kp, float ki, float kd,
                  float outMin, float outMax,
                  float derivAlpha = 0.30f);

    float update(float error, float dt);
    void  reset();
    void  setAngular(bool angular);

private:
    float _kp, _ki, _kd;
    float _outMin, _outMax;
    float _alpha;
    float _integral;
    float _prevError;
    float _prevDeriv;
    bool  _hasPrev;
    bool  _angular;
};

#endif // USV_PID_CONTROLLER_H
