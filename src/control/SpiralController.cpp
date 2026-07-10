#include "SpiralController.h"
#include "config.h"
#include <math.h>

static inline double sgn(double x) {
    return (x > 0.0) - (x < 0.0);
}

static inline double wrapPi(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

SpiralController::SpiralController(double halfBeam, double maxThrust, double losLookahead)
    : _m_u(25.0), _m_r(8.0),
      _d_u(12.0), _d_r(6.0),
      _B_half(halfBeam),
      _Tmax(maxThrust),
      _losLookahead(losLookahead),
      _k1_u(6.0), _k2_u(4.0),
      _k1_r(5.0), _k2_r(3.0),
      _sigma_u_int(0.0), _sigma_r_int(0.0) {}

void SpiralController::reset() {
    _sigma_u_int = 0.0;
    _sigma_r_int = 0.0;
}

int SpiralController::forceToPwm(double force) const {
    double ratio = force / _Tmax;
    if (ratio >  1.0) ratio =  1.0;
    if (ratio < -1.0) ratio = -1.0;
    int pwm = (int)(ratio * MOTOR_PWM_LIMIT);
    if (pwm > 0 && pwm < MOTOR_MIN_PWM)  pwm = MOTOR_MIN_PWM;
    if (pwm < 0 && pwm > -MOTOR_MIN_PWM) pwm = -MOTOR_MIN_PWM;
    return pwm;
}

ControlOutput SpiralController::computeControlInputs(const UsvState& s,
                                                     const UsvTarget& t, double dt) {
    if (dt <= 0.0) dt = 1e-3;
    ControlOutput out;

    // --- LOS guidance: desired heading toward the (look-ahead) reference ---
    double dx = t.x_ref - s.x;
    double dy = t.y_ref - s.y;
    double psi_d = atan2(dy, dx);

    // --- Sliding surfaces ---
    double e_u = s.u - t.u_d;                 // surge speed error
    double e_psi = wrapPi(s.psi - psi_d);     // heading error
    double sigma_u = e_u;
    double sigma_r = s.r + 1.5 * e_psi;       // yaw sliding surface (rate + heading)

    // --- Super-twisting algorithm (surge channel) ---
    _sigma_u_int += _k2_u * sgn(sigma_u) * dt;
    double stsmc_u = -_k1_u * sqrt(fabs(sigma_u)) * sgn(sigma_u) - _sigma_u_int;

    // --- Super-twisting algorithm (yaw channel) ---
    _sigma_r_int += _k2_r * sgn(sigma_r) * dt;
    double stsmc_r = -_k1_r * sqrt(fabs(sigma_r)) * sgn(sigma_r) - _sigma_r_int;

    // --- Model-based equivalent control + STSMC robustifying term ---
    out.tau_u = _d_u * s.u + _m_u * stsmc_u;
    out.tau_r = _d_r * s.r + _m_r * stsmc_r;

    // --- Actuator allocation (differential thrust) ---
    // tau_u = T_port + T_stbd ; tau_r = (T_stbd - T_port) * B_half
    double diff = (2.0 * _B_half > 0.0) ? out.tau_r / (2.0 * _B_half) : 0.0;
    out.T_port = 0.5 * out.tau_u - diff;
    out.T_stbd = 0.5 * out.tau_u + diff;

    // --- Thrust saturation ---
    if (out.T_port >  _Tmax) out.T_port =  _Tmax;
    if (out.T_port < -_Tmax) out.T_port = -_Tmax;
    if (out.T_stbd >  _Tmax) out.T_stbd =  _Tmax;
    if (out.T_stbd < -_Tmax) out.T_stbd = -_Tmax;

    out.pwm_port = forceToPwm(out.T_port);
    out.pwm_stbd = forceToPwm(out.T_stbd);
    return out;
}
