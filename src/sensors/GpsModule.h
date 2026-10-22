#ifndef USV_GPS_MODULE_H
#define USV_GPS_MODULE_H

#include <Arduino.h>
#include <TinyGPSPlus.h>

struct GpsFix {
    double  lat;
    double  lon;
    float   hdop;
    float   speedKmh;
    uint8_t sats;
    bool    valid;
};

struct GpsDateTime {
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  minute;
    uint8_t  second;
    bool     valid;
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
    bool        getRawPosition(double& lat, double& lon);
    float       courseDeg();
    uint32_t    locationAgeMs();
    GpsLossTier getLossTier();
    unsigned long timeSinceFix() const;
    bool        hasTime() const;
    uint32_t    utcAgeMs() const;
    int         localHour() const;
    GpsDateTime utcNow() const;

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
