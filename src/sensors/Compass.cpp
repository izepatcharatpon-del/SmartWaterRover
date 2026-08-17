#include "Compass.h"
#include "config.h"
#include <Wire.h>
#include <Preferences.h>
#include <math.h>

static const uint8_t P_REG_CHIPID    = 0x00;
static const uint8_t P_REG_DATA      = 0x01;
static const uint8_t P_REG_CTRL1     = 0x0A;
static const uint8_t P_REG_CTRL2     = 0x0B;
static const uint8_t P_REG_AXIS_SIGN = 0x29;
// Datasheet 7.2: sign XYZ, then CTRL2 set/reset+8G, then CTRL1 continuous.
static const uint8_t P_AXIS_SIGN_XYZ     = 0x06;
static const uint8_t P_CTRL2_SETRESET_8G = 0x08;
static const uint8_t P_CTRL1_CONT_200HZ  = 0xCF;

static const char* MAG_NVS_NS   = "usv_mag";
static const char* MAG_NVS_XMIN = "xmin";
static const char* MAG_NVS_XMAX = "xmax";
static const char* MAG_NVS_YMIN = "ymin";
static const char* MAG_NVS_YMAX = "ymax";
static const char* MAG_NVS_OK   = "ok";

Compass::Compass()
    : _rawX(0), _rawY(0), _rawZ(0),
      _declination(0.0f),
      _xMin(COMPASS_X_MIN), _xMax(COMPASS_X_MAX),
      _yMin(COMPASS_Y_MIN), _yMax(COMPASS_Y_MAX),
      _addr(0),
      _fresh(false),
      _failCount(0),
      _nvsLoaded(false),
      _calibrating(false),
      _calStartMs(0),
      _calSamples(0),
      _calMinX(0.0f), _calMaxX(0.0f),
      _calMinY(0.0f), _calMaxY(0.0f) {}

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
    // Datasheet 8.2.4: repeated START between register write and data read.
    if (Wire.endTransmission(false) != 0) return false;
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
    // Soft reset leaves suspend (no measurements). CTRL2 must be rewritten
    // with set/reset + range or XYZ stay 0 while I2C still ACKs.
    (void)writeRegister(P_REG_AXIS_SIGN, P_AXIS_SIGN_XYZ);
    if (!writeRegister(P_REG_CTRL2, P_CTRL2_SETRESET_8G)) {
        _addr = 0;
        return false;
    }
    if (!writeRegister(P_REG_CTRL1, P_CTRL1_CONT_200HZ)) {
        _addr = 0;
        return false;
    }
    uint8_t ctrl1 = 0;
    if (!readRegisters(P_REG_CTRL1, &ctrl1, 1) || ctrl1 != P_CTRL1_CONT_200HZ) {
        _addr = 0;
        return false;
    }
    loadCalibration();
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
    if (_calibrating) {
        collectCalibrationSample();
    }
    return true;
}

void Compass::setDeclination(float degrees) { _declination = degrees; }

float Compass::applyDeclination(float headingDeg) {
    float h = headingDeg + _declination;
    while (h < 0.0f)    h += 360.0f;
    while (h >= 360.0f) h -= 360.0f;
    return h;
}

void Compass::applyHardIron(float rawX, float rawY, float& outX, float& outY) const {
    const float xSpan = _xMax - _xMin;
    const float ySpan = _yMax - _yMin;
    // Default equal ±2048 → centered unit circle (identity-ish scale).
    if (xSpan < 1.0f || ySpan < 1.0f) {
        outX = rawX;
        outY = rawY;
        return;
    }
    const float xOff = 0.5f * (_xMin + _xMax);
    const float yOff = 0.5f * (_yMin + _yMax);
    outX = (rawX - xOff) / (0.5f * xSpan);
    outY = (rawY - yOff) / (0.5f * ySpan);
}

float Compass::getHeadingDeg() {
    if (!_fresh) return NAN;

    float x = 0.0f, y = 0.0f;
    applyHardIron((float)_rawX, (float)_rawY, x, y);
    if ((x * x + y * y) < 1e-6f) return NAN;

    float heading = atan2f(y, x) * 180.0f / (float)M_PI;
    if (heading < 0.0f) heading += 360.0f;
    return applyDeclination(heading);
}

void Compass::setFactoryHardIron() {
    _xMin = COMPASS_X_MIN;
    _xMax = COMPASS_X_MAX;
    _yMin = COMPASS_Y_MIN;
    _yMax = COMPASS_Y_MAX;
    _nvsLoaded = false;
}

bool Compass::hardIronSpansOk(float xMin, float xMax, float yMin, float yMax) const {
    if (!isfinite(xMin) || !isfinite(xMax) || !isfinite(yMin) || !isfinite(yMax)) {
        return false;
    }
    return ((xMax - xMin) >= COMPASS_CAL_MIN_SPAN) &&
           ((yMax - yMin) >= COMPASS_CAL_MIN_SPAN);
}

void Compass::collectCalibrationSample() {
    const float x = (float)_rawX;
    const float y = (float)_rawY;
    if (x < _calMinX) _calMinX = x;
    if (x > _calMaxX) _calMaxX = x;
    if (y < _calMinY) _calMinY = y;
    if (y > _calMaxY) _calMaxY = y;
    if (_calSamples < 65535) _calSamples++;
}

bool Compass::calibrationReady() const {
    if (!_calibrating) return false;
    return (_calSamples >= COMPASS_CAL_MIN_SAMPLES) &&
           hardIronSpansOk(_calMinX, _calMaxX, _calMinY, _calMaxY);
}

bool Compass::startCalibration() {
    if (_addr == 0) return false;
    _calibrating = true;
    _calStartMs = millis();
    _calSamples = 0;
    _calMinX =  1.0e9f;
    _calMaxX = -1.0e9f;
    _calMinY =  1.0e9f;
    _calMaxY = -1.0e9f;
    if (_fresh) {
        collectCalibrationSample();
    }
    return true;
}

void Compass::abortCalibration() {
    _calibrating = false;
}

bool Compass::finishCalibration() {
    if (!calibrationReady()) {
        return false;
    }
    _calibrating = false;
    _xMin = _calMinX;
    _xMax = _calMaxX;
    _yMin = _calMinY;
    _yMax = _calMaxY;
    if (!saveCalibration()) {
        return false;
    }
    _nvsLoaded = true;
    return true;
}

bool Compass::clearStoredCalibration() {
    abortCalibration();
    setFactoryHardIron();
    Preferences prefs;
    if (!prefs.begin(MAG_NVS_NS, false)) return false;
    prefs.clear();
    prefs.end();
    return true;
}

void Compass::getHardIron(float& xMin, float& xMax, float& yMin, float& yMax) const {
    xMin = _xMin;
    xMax = _xMax;
    yMin = _yMin;
    yMax = _yMax;
}

void Compass::getCalProgress(float& xMin, float& xMax, float& yMin, float& yMax,
                             uint16_t& samples, uint32_t& elapsedMs) const {
    xMin = _calMinX;
    xMax = _calMaxX;
    yMin = _calMinY;
    yMax = _calMaxY;
    samples = _calSamples;
    elapsedMs = millis() - _calStartMs;
}

bool Compass::saveCalibration() {
    if (!hardIronSpansOk(_xMin, _xMax, _yMin, _yMax)) return false;
    Preferences prefs;
    if (!prefs.begin(MAG_NVS_NS, false)) return false;
    prefs.putFloat(MAG_NVS_XMIN, _xMin);
    prefs.putFloat(MAG_NVS_XMAX, _xMax);
    prefs.putFloat(MAG_NVS_YMIN, _yMin);
    prefs.putFloat(MAG_NVS_YMAX, _yMax);
    prefs.putUChar(MAG_NVS_OK, 1);
    prefs.end();
    return true;
}

bool Compass::loadCalibration() {
    Preferences prefs;
    if (prefs.begin(MAG_NVS_NS, true)) {
        if (prefs.getUChar(MAG_NVS_OK, 0) != 0) {
            const float xMin = prefs.getFloat(MAG_NVS_XMIN, NAN);
            const float xMax = prefs.getFloat(MAG_NVS_XMAX, NAN);
            const float yMin = prefs.getFloat(MAG_NVS_YMIN, NAN);
            const float yMax = prefs.getFloat(MAG_NVS_YMAX, NAN);
            prefs.end();
            if (hardIronSpansOk(xMin, xMax, yMin, yMax)) {
                _xMin = xMin;
                _xMax = xMax;
                _yMin = yMin;
                _yMax = yMax;
                _nvsLoaded = true;
                return true;
            }
        } else {
            prefs.end();
        }
    }
    setFactoryHardIron();
    return false;
}
