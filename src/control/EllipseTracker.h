#ifndef USV_ELLIPSE_TRACKER_H
#define USV_ELLIPSE_TRACKER_H

// Cross-track controller for an elliptical reference path. Computes the
// normalized ellipse error E = (x/a)^2 + (y/b)^2 - 1 and steers to drive
// |E| -> 0. Position is relative to the ellipse center (meters).
class EllipseTracker {
public:
    EllipseTracker(float gain = 120.0f) : _gain(gain) {}

    // x,y: boat position relative to ellipse center.
    // a,b: semi-axes. basePwm: forward cruise PWM.
    void calculateEllipticalThrust(float x, float y, float a, float b,
                                   int basePwm, int& leftPwm, int& rightPwm);

    // Signed cross-track error metric.
    static float crossTrackError(float x, float y, float a, float b);

private:
    float _gain;
};

#endif // USV_ELLIPSE_TRACKER_H
