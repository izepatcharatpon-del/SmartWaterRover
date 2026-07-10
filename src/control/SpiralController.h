#ifndef USV_SPIRAL_CONTROLLER_H
#define USV_SPIRAL_CONTROLLER_H

// Super-Twisting Sliding Mode Control (STSMC) for a twin-motor USV tracking a
// spiral path. Port of TwinMotorUSVController (USV_Spiral_Tracking_Summary.md).

// Vehicle state in the local ENU frame.
struct UsvState {
    double x, y;     // position (m)
    double psi;      // heading (rad)
    double u, v;     // surge, sway velocity (m/s)
    double r;        // yaw rate (rad/s)
};

// Guidance target (from LOS over the spiral reference).
struct UsvTarget {
    double x_ref, y_ref;   // reference point on the spiral
    double u_d;            // desired surge speed (m/s)
};

struct ControlOutput {
    double tau_u;     // surge force command (N)
    double tau_r;     // yaw moment command (N*m)
    double T_port;    // port thruster force (N)
    double T_stbd;    // starboard thruster force (N)
    int    pwm_port;  // -255..255
    int    pwm_stbd;  // -255..255
};

class SpiralController {
public:
    SpiralController(double halfBeam = 0.375,   // B/2, thruster moment arm (m)
                     double maxThrust = 30.0,   // per-thruster saturation (N)
                     double losLookahead = 3.0);// LOS lookahead distance (m)

    ControlOutput computeControlInputs(const UsvState& s, const UsvTarget& t, double dt);
    void reset();

private:
    // Rigid-body + added-mass + drag model parameters.
    double _m_u, _m_r;      // effective surge/yaw inertia
    double _d_u, _d_r;      // linear drag surge/yaw
    double _B_half;         // moment arm
    double _Tmax;
    double _losLookahead;

    // STSMC gains (surge and yaw channels).
    double _k1_u, _k2_u;
    double _k1_r, _k2_r;

    // Super-twisting integral states.
    double _sigma_u_int;
    double _sigma_r_int;

    int forceToPwm(double force) const;
};

#endif // USV_SPIRAL_CONTROLLER_H
