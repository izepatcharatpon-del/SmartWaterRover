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

enum class GpsLossTier {
    OK,
    SAFE_STOP,
    GPS_FAULT
};

class GpsModule {
public:
    GpsModule();

    void        begin();
    void        configureSeaMode();
    bool        update();
    bool        isFixValid();
    bool        hasFreshFix();
    bool        clearFaultIfFresh();
    GpsFix      getFilteredFix();
    GpsLossTier getLossTier();
    unsigned long timeSinceFix() const;
    bool        hasTime() const;
    int         localHour() const;

private:
    TinyGPSPlus   _gps;
    double        _filtLat, _filtLon;
    bool          _filterInit;
    unsigned long _lastValidFixMs;
    bool          _everHadFix;
    bool          _faultLatched;

    void applyLowPassFilter(double lat, double lon);
    double filterAlpha() const;
    void sendUbx(const uint8_t* msg, size_t len);
};

#endif // USV_GPS_MODULE_H
