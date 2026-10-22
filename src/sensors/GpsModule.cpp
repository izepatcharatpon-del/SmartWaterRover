#include "GpsModule.h"
#include "config.h"
#include <math.h>

GpsModule::GpsModule()
    : _filtLat(0.0), _filtLon(0.0),
      _filterInit(false),
      _lastValidFixMs(0),
      _everHadFix(false),
      _faultLatched(false) {}

void GpsModule::begin() {
    Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    // Do not stamp a fake "last fix" at boot — that made getLossTier() report OK
    // for ~GPS_LOSS_HOLD_MS with no satellites.
    _lastValidFixMs = 0;
    _everHadFix = false;
    _faultLatched = false;
}

void GpsModule::sendUbx(const uint8_t* msg, size_t len) {
    for (size_t i = 0; i < len; i++) Serial2.write(msg[i]);
    Serial2.flush();
}

void GpsModule::configureSeaMode() {
    uint8_t cfg[] = {
        0xB5, 0x62, 0x06, 0x24, 0x24, 0x00,
        0x01, 0x00,
        0x05,
        0x03,
        0x00, 0x00, 0x00, 0x00,
        0x10, 0x27, 0x00, 0x00,
        0x05,
        0x00,
        0xFA, 0x00,
        0xFA, 0x00,
        0x64, 0x00,
        0x2C, 0x01,
        0x00,
        0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00,
        0x00, 0x00
    };
    uint8_t ckA = 0, ckB = 0;
    for (size_t i = 2; i < sizeof(cfg) - 2; i++) {
        ckA += cfg[i];
        ckB += ckA;
    }
    cfg[sizeof(cfg) - 2] = ckA;
    cfg[sizeof(cfg) - 1] = ckB;
    sendUbx(cfg, sizeof(cfg));
}

bool GpsModule::update() {
    bool gotSentence = false;
    int bytes = 0;
    while (Serial2.available() > 0 && bytes < GPS_MAX_BYTES_PER_UPDATE) {
        if (_gps.encode(Serial2.read())) gotSentence = true;
        bytes++;
    }
    if (gotSentence && isFixValid()) {
        applyLowPassFilter(_gps.location.lat(), _gps.location.lng());
        _lastValidFixMs = millis();
        _everHadFix = true;
    }
    return gotSentence;
}

bool GpsModule::isFixValid() {
    if (!_gps.location.isValid()) return false;
    if (_gps.location.age() > GPS_FRESH_FIX_MS) return false;
    uint8_t sats = _gps.satellites.isValid() ? _gps.satellites.value() : 0;
    float   hdop = _gps.hdop.isValid() ? _gps.hdop.hdop() : 99.0f;
    return (sats >= GPS_MIN_SATS) && (hdop < GPS_MAX_HDOP);
}

bool GpsModule::hasFreshFix() {
    return isFixValid() && timeSinceFix() <= GPS_FRESH_FIX_MS;
}

bool GpsModule::clearFaultIfFresh() {
    if (!hasFreshFix()) return false;
    _faultLatched = false;
    return true;
}

double GpsModule::filterAlpha() const {
    float hdop = 99.0f;
    if (_gps.hdop.isValid()) {
        hdop = (float)const_cast<TinyGPSHDOP&>(_gps.hdop).hdop();
    }
    float speedKmh = 0.0f;
    if (_gps.speed.isValid() &&
        const_cast<TinyGPSSpeed&>(_gps.speed).age() < GPS_FRESH_FIX_MS) {
        speedKmh = (float)const_cast<TinyGPSSpeed&>(_gps.speed).kmph();
    }
    if (speedKmh >= GPS_FILTER_SPEED_KMH && hdop <= GPS_FILTER_HDOP_GOOD) {
        return (double)GPS_FILTER_ALPHA_FAST;
    }
    return (double)GPS_FILTER_ALPHA;
}

void GpsModule::applyLowPassFilter(double lat, double lon) {
    if (!_filterInit) {
        _filtLat = lat;
        _filtLon = lon;
        _filterInit = true;
        return;
    }
    const double alpha = filterAlpha();
    _filtLat = alpha * lat + (1.0 - alpha) * _filtLat;

    double dLon = lon - _filtLon;
    while (dLon > 180.0)  dLon -= 360.0;
    while (dLon < -180.0) dLon += 360.0;
    _filtLon = _filtLon + alpha * dLon;
    while (_filtLon > 180.0)  _filtLon -= 360.0;
    while (_filtLon < -180.0) _filtLon += 360.0;
}

GpsFix GpsModule::getFilteredFix() {
    GpsFix fix;
    fix.lat   = _filtLat;
    fix.lon   = _filtLon;
    fix.hdop  = _gps.hdop.isValid() ? _gps.hdop.hdop() : 99.0f;
    fix.speedKmh = 0.0f;
    if (_gps.speed.isValid() &&
        const_cast<TinyGPSSpeed&>(_gps.speed).age() < GPS_FRESH_FIX_MS) {
        fix.speedKmh = (float)const_cast<TinyGPSSpeed&>(_gps.speed).kmph();
    }
    fix.sats  = _gps.satellites.isValid() ? _gps.satellites.value() : 0;
    fix.valid = _filterInit && !_faultLatched && hasFreshFix();
    return fix;
}

bool GpsModule::getRawPosition(double& lat, double& lon) {
    if (!_gps.location.isValid() ||
        _gps.location.age() > GPS_FRESH_FIX_MS) return false;
    lat = _gps.location.lat();
    lon = _gps.location.lng();
    return true;
}

float GpsModule::courseDeg() {
    if (!_gps.course.isValid() ||
        _gps.course.age() > GPS_FRESH_FIX_MS) return NAN;
    // Course over ground is diagnostic only; it can be noisy at low speed.
    return (float)_gps.course.deg();
}

uint32_t GpsModule::locationAgeMs() {
    if (!_gps.location.isValid()) return UINT32_MAX;
    return (uint32_t)_gps.location.age();
}

unsigned long GpsModule::timeSinceFix() const {
    if (!_everHadFix || _lastValidFixMs == 0) return 0;
    return millis() - _lastValidFixMs;
}

GpsLossTier GpsModule::getLossTier() {
    if (_faultLatched) return GpsLossTier::GPS_FAULT;

    if (!_everHadFix) return GpsLossTier::SAFE_STOP;

    unsigned long elapsed = timeSinceFix();
    if (elapsed < GPS_LOSS_HOLD_MS) return GpsLossTier::OK;
    if (elapsed < GPS_LOSS_SAFE_STOP_MS) return GpsLossTier::SAFE_STOP;
    if (elapsed < GPS_LOSS_FAULT_MS) return GpsLossTier::SAFE_STOP;

    _faultLatched = true;
    return GpsLossTier::GPS_FAULT;
}

bool GpsModule::hasTime() const {
    return _gps.time.isValid() && const_cast<TinyGPSTime&>(_gps.time).age() < 5000;
}

uint32_t GpsModule::utcAgeMs() const {
    TinyGPSDate& date = const_cast<TinyGPSDate&>(_gps.date);
    TinyGPSTime& time = const_cast<TinyGPSTime&>(_gps.time);
    if (!date.isValid() || !time.isValid()) return UINT32_MAX;
    const uint32_t dateAge = (uint32_t)date.age();
    const uint32_t timeAge = (uint32_t)time.age();
    return dateAge > timeAge ? dateAge : timeAge;
}

int GpsModule::localHour() const {
    if (!hasTime()) return -1;
    // TinyGPSPlus hour() is non-const; cast is safe (read-only use).
    int hour = (int)const_cast<TinyGPSTime&>(_gps.time).hour();
    hour = (hour + GPS_TIMEZONE_OFFSET_H) % 24;
    if (hour < 0) hour += 24;
    return hour;
}

GpsDateTime GpsModule::utcNow() const {
    GpsDateTime t = {};
    TinyGPSDate& date = const_cast<TinyGPSDate&>(_gps.date);
    TinyGPSTime& time = const_cast<TinyGPSTime&>(_gps.time);
    if (!date.isValid() || !time.isValid()) return t;
    if (date.age() > 5000 || time.age() > 5000) return t;
    uint16_t year = (uint16_t)date.year();
    if (year < 100) year = (uint16_t)(year + 2000);
    const uint8_t month = (uint8_t)date.month();
    const uint8_t day = (uint8_t)date.day();
    if (year < 2020 || year > 2099) return t;
    if (month < 1 || month > 12 || day < 1 || day > 31) return t;
    t.year = year;
    t.month = month;
    t.day = day;
    t.hour = (uint8_t)time.hour();
    t.minute = (uint8_t)time.minute();
    t.second = (uint8_t)time.second();
    t.valid = true;
    return t;
}
