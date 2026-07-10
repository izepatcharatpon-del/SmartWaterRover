#include "GpsModule.h"
#include "config.h"

GpsModule::GpsModule()
    : _filtLat(0.0), _filtLon(0.0),
      _filterInit(false),
      _lastValidFixMs(0) {}

void GpsModule::begin() {
    Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    _lastValidFixMs = millis();
}

void GpsModule::sendUbx(const uint8_t* msg, size_t len) {
    for (size_t i = 0; i < len; i++) Serial2.write(msg[i]);
    Serial2.flush();
}

void GpsModule::configureSeaMode() {
    // UBX-CFG-NAV5: set dynamic platform model = 5 (Sea). Only dynModel masked.
    uint8_t cfg[] = {
        0xB5, 0x62, 0x06, 0x24, 0x24, 0x00,
        0x01, 0x00,             // mask: apply dynModel only
        0x05,                   // dynModel = Sea
        0x03,                   // fixMode = auto 2D/3D
        0x00, 0x00, 0x00, 0x00, // fixedAlt
        0x10, 0x27, 0x00, 0x00, // fixedAltVar
        0x05,                   // minElev
        0x00,                   // drLimit
        0xFA, 0x00,             // pDop
        0xFA, 0x00,             // tDop
        0x64, 0x00,             // pAcc
        0x2C, 0x01,             // tAcc
        0x00,                   // staticHoldThresh
        0x00,                   // dgnssTimeout
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00,
        0x00, 0x00              // checksum placeholder (computed below)
    };
    // Compute Fletcher checksum over class..payload (bytes 2..end-2).
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
    while (Serial2.available() > 0) {
        if (_gps.encode(Serial2.read())) gotSentence = true;
    }
    if (gotSentence && isFixValid()) {
        applyLowPassFilter(_gps.location.lat(), _gps.location.lng());
        _lastValidFixMs = millis();
    }
    return gotSentence;
}

bool GpsModule::isFixValid() {
    if (!_gps.location.isValid()) return false;
    uint8_t sats = _gps.satellites.isValid() ? _gps.satellites.value() : 0;
    float   hdop = _gps.hdop.isValid() ? _gps.hdop.hdop() : 99.0f;
    return (sats >= GPS_MIN_SATS) && (hdop < GPS_MAX_HDOP);
}

void GpsModule::applyLowPassFilter(double lat, double lon) {
    if (!_filterInit) {
        _filtLat = lat;
        _filtLon = lon;
        _filterInit = true;
        return;
    }
    _filtLat = GPS_FILTER_ALPHA * lat + (1.0f - GPS_FILTER_ALPHA) * _filtLat;
    _filtLon = GPS_FILTER_ALPHA * lon + (1.0f - GPS_FILTER_ALPHA) * _filtLon;
}

GpsFix GpsModule::getFilteredFix() {
    GpsFix fix;
    fix.lat   = _filtLat;
    fix.lon   = _filtLon;
    fix.hdop  = _gps.hdop.isValid() ? _gps.hdop.hdop() : 99.0f;
    fix.sats  = _gps.satellites.isValid() ? _gps.satellites.value() : 0;
    fix.valid = _filterInit && (getLossTier() == GpsLossTier::NONE);
    return fix;
}

unsigned long GpsModule::timeSinceFix() const {
    return millis() - _lastValidFixMs;
}

GpsLossTier GpsModule::getLossTier() {
    unsigned long elapsed = timeSinceFix();
    if (elapsed < GPS_LOSS_T1_MS) return GpsLossTier::NONE;
    if (elapsed < GPS_LOSS_T2_MS) return GpsLossTier::DEAD_RECKONING;
    if (elapsed < 2 * GPS_LOSS_T2_MS) return GpsLossTier::SAFE_FLOAT;
    return GpsLossTier::EMERGENCY_RTH;
}
