#include "GpsModule.h"
#include "config.h"

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
    // Prefer age-aware checks when TinyGPSPlus provides them.
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

void GpsModule::applyLowPassFilter(double lat, double lon) {
    if (!_filterInit) {
        _filtLat = lat;
        _filtLon = lon;
        _filterInit = true;
        return;
    }
    _filtLat = GPS_FILTER_ALPHA * lat + (1.0 - GPS_FILTER_ALPHA) * _filtLat;

    // Wrap longitude error across ±180° before blending.
    double dLon = lon - _filtLon;
    while (dLon > 180.0)  dLon -= 360.0;
    while (dLon < -180.0) dLon += 360.0;
    _filtLon = _filtLon + GPS_FILTER_ALPHA * dLon;
    while (_filtLon > 180.0)  _filtLon -= 360.0;
    while (_filtLon < -180.0) _filtLon += 360.0;
}

GpsFix GpsModule::getFilteredFix() {
    GpsFix fix;
    fix.lat   = _filtLat;
    fix.lon   = _filtLon;
    fix.hdop  = _gps.hdop.isValid() ? _gps.hdop.hdop() : 99.0f;
    fix.sats  = _gps.satellites.isValid() ? _gps.satellites.value() : 0;
    // Never navigate or declare arrival from held/stale coordinates.
    fix.valid = _filterInit && !_faultLatched && hasFreshFix();
    return fix;
}

unsigned long GpsModule::timeSinceFix() const {
    if (!_everHadFix || _lastValidFixMs == 0) return 0;
    return millis() - _lastValidFixMs;
}

GpsLossTier GpsModule::getLossTier() {
    if (_faultLatched) return GpsLossTier::GPS_FAULT;

    // No quality fix since boot: not OK, but do not latch FAULT until we have
    // acquired then lost a fix for GPS_LOSS_FAULT_MS.
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

int GpsModule::localHour() const {
    if (!hasTime()) return -1;
    // TinyGPSPlus hour() is non-const; cast is safe (read-only use).
    int hour = (int)const_cast<TinyGPSTime&>(_gps.time).hour();
    hour = (hour + GPS_TIMEZONE_OFFSET_H) % 24;
    if (hour < 0) hour += 24;
    return hour;
}
