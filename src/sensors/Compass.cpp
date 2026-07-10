#include "Compass.h"
#include "config.h"
#include <Wire.h>
#include <math.h>

// QMC5883L register map
static const uint8_t REG_DATA_X_LSB = 0x00;
static const uint8_t REG_STATUS     = 0x06;
static const uint8_t REG_CONTROL_1  = 0x09;
static const uint8_t REG_SET_RESET  = 0x0B;

Compass::Compass()
    : _rawX(0), _rawY(0), _rawZ(0),
      _offX(0), _offY(0), _offZ(0),
      _scaleX(1.0f), _scaleY(1.0f), _scaleZ(1.0f),
      _declination(0.0f) {}

bool Compass::writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(QMC5883L_I2C_ADDR);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

bool Compass::begin() {
    // Assumes Wire.begin(SDA, SCL) already called by the system init.
    if (!writeRegister(REG_SET_RESET, 0x01)) return false;
    // OSR=512, RNG=8G, ODR=200Hz, MODE=continuous  -> 0b0001 1101 = 0x1D
    return writeRegister(REG_CONTROL_1, 0x1D);
}

void Compass::update() {
    Wire.beginTransmission(QMC5883L_I2C_ADDR);
    Wire.write(REG_DATA_X_LSB);
    if (Wire.endTransmission(false) != 0) return;

    if (Wire.requestFrom(QMC5883L_I2C_ADDR, 6) != 6) return;
    _rawX = (int16_t)(Wire.read() | (Wire.read() << 8));
    _rawY = (int16_t)(Wire.read() | (Wire.read() << 8));
    _rawZ = (int16_t)(Wire.read() | (Wire.read() << 8));
}

void Compass::setHardIron(float x, float y, float z) { _offX = x; _offY = y; _offZ = z; }
void Compass::setSoftIron(float sx, float sy, float sz) { _scaleX = sx; _scaleY = sy; _scaleZ = sz; }
void Compass::setDeclination(float degrees) { _declination = degrees; }

float Compass::applyDeclination(float headingDeg) {
    float h = headingDeg + _declination;
    while (h < 0.0f)    h += 360.0f;
    while (h >= 360.0f) h -= 360.0f;
    return h;
}

float Compass::getHeadingDeg() {
    float x = (_rawX - _offX) * _scaleX;
    float y = (_rawY - _offY) * _scaleY;

    float heading = atan2f(y, x) * 180.0f / (float)M_PI;
    if (heading < 0.0f) heading += 360.0f;
    return applyDeclination(heading);
}

void Compass::runCalibration(unsigned long durationMs) {
    int16_t minX = 32767, minY = 32767, maxX = -32768, maxY = -32768;
    unsigned long start = millis();
    while (millis() - start < durationMs) {
        update();
        if (_rawX < minX) minX = _rawX;
        if (_rawX > maxX) maxX = _rawX;
        if (_rawY < minY) minY = _rawY;
        if (_rawY > maxY) maxY = _rawY;
        delay(20);
    }
    _offX = (maxX + minX) / 2.0f;
    _offY = (maxY + minY) / 2.0f;
    float rangeX = (maxX - minX) / 2.0f;
    float rangeY = (maxY - minY) / 2.0f;
    float avgRange = (rangeX + rangeY) / 2.0f;
    if (rangeX > 1.0f) _scaleX = avgRange / rangeX;
    if (rangeY > 1.0f) _scaleY = avgRange / rangeY;
}
