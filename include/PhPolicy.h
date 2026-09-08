#ifndef USV_PH_POLICY_H
#define USV_PH_POLICY_H

#include "config.h"
#include <math.h>

// Host-testable pH usability rule. Pond chemistry does not produce pH 0–2
// dropouts; those are dry-probe / ADC / clamp artifacts and must not arm spiral.
inline bool usvPhPlausible(float ph) {
    return isfinite(ph) &&
           ph >= PH_PLAUSIBLE_MIN &&
           ph <= PH_PLAUSIBLE_MAX;
}

#endif // USV_PH_POLICY_H
