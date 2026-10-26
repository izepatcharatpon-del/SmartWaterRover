#ifndef USV_GRID_MOTOR_MIXER_H
#define USV_GRID_MOTOR_MIXER_H

#include <math.h>
#include "DriveController.h"
#include "config.h"

// Joint allocation in PWM coordinates, not a calibrated thrust model. Nonzero
// axes respect the configured floor; net reverse travel is never requested.
namespace GridMotorMixer {
inline DriveRequest allocate(float surge, float yaw, int ceiling, bool requireMotion = false) {
    DriveRequest out{DriveSource::GRID, 0, 0, true};
    const bool mirror = yaw < 0;
    yaw = fabsf(yaw);
    float bestScore = INFINITY;
    const int low[] = {-MOTOR_PWM_LIMIT, 0, MOTOR_MIN_PWM};
    const int high[] = {-MOTOR_MIN_PWM, 0, MOTOR_PWM_LIMIT};
    const float weight = GRID_MIX_YAW_WEIGHT;
    auto bound = [](float x, float a, float b) { return fmaxf(a, fminf(b, x)); };
    auto score = [&](float l, float r) {
        const float y = .5f * (l - r) - yaw;
        const float s = .5f * (l + r) - surge;
        return weight * y * y + s * s;
    };
    for (int a = 0; a < 3; ++a) for (int b = 0; b < 3; ++b) {
        if (low[a] + low[b] > 2 * ceiling || high[a] + high[b] < 0) continue;
        float bandScore = INFINITY, bestL = 0, bestR = 0;
        auto consider = [&](float l, float r) {
            if (requireMotion && l == 0 && r == 0) return;
            if (l < low[a] || l > high[a] || r < low[b] || r > high[b] ||
                l + r < 0 || l + r > 2 * ceiling) return;
            const float cost = score(l, r);
            if (cost < bandScore) { bandScore = cost; bestL = l; bestR = r; }
        };
        consider(surge + yaw, surge - yaw);
        // The convex optimum is interior or on one of these six edges.
        for (int k = 0; k < 2; ++k) {
            float l = k ? high[a] : low[a];
            float lo = fmaxf(low[b], -l), hi = fminf(high[b], 2 * ceiling - l);
            if (lo <= hi) consider(l, bound(((weight - 1) * l + 2 * surge -
                                           2 * weight * yaw) / (weight + 1), lo, hi));
            float r = k ? high[b] : low[b];
            lo = fmaxf(low[a], -r); hi = fminf(high[a], 2 * ceiling - r);
            if (lo <= hi) consider(bound(((weight - 1) * r + 2 * surge +
                                         2 * weight * yaw) / (weight + 1), lo, hi), r);
            const float common = k ? ceiling : 0;
            lo = fmaxf(low[a], 2 * common - high[b]);
            hi = fminf(high[a], 2 * common - low[b]);
            if (lo <= hi) {
                l = bound(common + yaw, lo, hi);
                consider(l, 2 * common - l);
            }
        }
        if (!isfinite(bandScore)) continue;
        for (int x = 0; x < 2; ++x) for (int y = 0; y < 2; ++y) {
            const int l = (int)(x ? ceilf(bestL) : floorf(bestL));
            const int r = (int)(y ? ceilf(bestR) : floorf(bestR));
            if (requireMotion && l == 0 && r == 0) continue;
            if (l < low[a] || l > high[a] || r < low[b] || r > high[b] ||
                l + r < 0 || l + r > 2 * ceiling) continue;
            const float cost = score(l, r);
            if (cost < bestScore || (cost == bestScore &&
                l + r > out.leftPwm + out.rightPwm)) {
                bestScore = cost; out.leftPwm = l; out.rightPwm = r;
            }
        }
    }
    if (mirror) { const int l = out.leftPwm; out.leftPwm = out.rightPwm; out.rightPwm = l; }
    return out;
}
}
#endif
