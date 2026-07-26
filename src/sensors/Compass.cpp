#include "Compass.h"
#include "config.h"
#include <Wire.h>
#include <math.h>

static const uint8_t P_REG_CHIPID = 0x00;
static const uint8_t P_REG_DATA   = 0x01;
static const uint8_t P_REG_CTRL1  = 0x0A;
static const uint8_t P_REG_CTRL2  = 0x0B;

Compass::Compass()
    : _rawX(0), _rawY(0), _rawZ(0),
      _declination(0.0f),
      _addr(0),
      _fresh(false),
      _failCount(0) {}

bool Compass::writeRegister(uint8_t reg, uint8_t value) {
    if (_addr == 0) return false;
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

bool Compass::readRegisters(uint8_t reg, uint8_t* buf, uint8_t len) {
    if (_addr == 0) return false;
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    if (Wire.endTransmission(true) != 0) return false;
    delayMicroseconds(200);
    if (Wire.requestFrom((int)_addr, (int)len) != (int)len) return false;
    for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
    return true;
}

bool Compass::begin() {
    _addr = QMC5883P_I2C_ADDR;
    _fresh = false;
    _failCount = 0;

    Wire.beginTransmission(_addr);
    if (Wire.endTransmission() != 0) {
        _addr = 0;
        return false;
    }

    uint8_t id = 0;
    if (!readRegisters(P_REG_CHIPID, &id, 1) || id != QMC5883P_CHIP_ID) {
        _addr = 0;
        return false;
    }

    if (!writeRegister(P_REG_CTRL2, 0x80)) {
        _addr = 0;
        return false;
    }
    delay(10);
    if (!writeRegister(P_REG_CTRL1, 0xCF)) {
        _addr = 0;
        return false;
    }
    return true;
}

bool Compass::update() {
    if (_addr == 0) {
        _fresh = false;
        return false;
    }

    uint8_t buf[6];
    if (!readRegisters(P_REG_DATA, buf, 6)) {
        if (_failCount < 255) _failCount++;
        _fresh = false;
        return false;
    }

    _rawX = (int16_t)(buf[0] | (buf[1] << 8));
    _rawY = (int16_t)(buf[2] | (buf[3] << 8));
    _rawZ = (int16_t)(buf[4] | (buf[5] << 8));
    _fresh = true;
    _failCount = 0;
    return true;
}

void Compass::setDeclination(float degrees) { _declination = degrees; }

float Compass::applyDeclination(float headingDeg) {
    float h = headingDeg + _declination;
    while (h < 0.0f)    h += 360.0f;
    while (h >= 360.0f) h -= 360.0f;
    return h;
}

float Compass::getHeadingDeg() {
    if (!_fresh) return NAN;

    float x = (float)_rawX;
    float y = (float)_rawY;
    if ((x * x + y * y) < 1.0f) return NAN;

    float heading = atan2f(y, x) * 180.0f / (float)M_PI;
    if (heading < 0.0f) heading += 360.0f;
    return applyDeclination(heading);
}
