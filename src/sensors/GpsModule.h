#ifndef USV_GPS_MODULE_H
#define USV_GPS_MODULE_H

#include <Arduino.h>
#include <TinyGPSPlus.h>

struct GpsFix {
    double  lat;
    double  lon;
    float   hdop;
    uint8_t sats;
    bool    valid;
};

// Tiered response to GPS degradation (gps_drift_solutions.md).
enum class GpsLossTier {
    NONE,            // fresh valid fix
    DEAD_RECKONING,  // < 10s: coast on last heading/speed
    SAFE_FLOAT,      // 10-60s: stop motors, hold position
    EMERGENCY_RTH    // > 60s: escalate to safety layer
};

// NEO-8M over UART2 with fix-quality gate, low-pass position filter and
// "sea mode" dynamic model configuration (UBX-CFG-NAV5).
class GpsModule {
public:
    GpsModule();

    void        begin();
    void        configureSeaMode();   // UBX dynamic model = Sea (5)
    bool        update();             // pump serial bytes into parser
    bool        isFixValid();         // sats>=6 && hdop<2.0
    GpsFix      getFilteredFix();
    GpsLossTier getLossTier();
    unsigned long timeSinceFix() const;

private:
    TinyGPSPlus   _gps;
    double        _filtLat, _filtLon;
    bool          _filterInit;
    unsigned long _lastValidFixMs;

    void applyLowPassFilter(double lat, double lon);
    void sendUbx(const uint8_t* msg, size_t len);
};

#endif // USV_GPS_MODULE_H
