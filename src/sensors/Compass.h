#ifndef USV_COMPASS_H
#define USV_COMPASS_H

#include <Arduino.h>

// QMC5883P only (I2C 0x2C, chip ID 0x80).
class Compass {
public:
    Compass();

    bool  begin();
    bool  update();
    float getHeadingDeg();          // 0-360 + declination; NAN if not fresh
    void  setDeclination(float degrees);
    uint8_t i2cAddress() const { return _addr; }
    bool    isOk() const { return _addr != 0 && _fresh && _failCount < 5; }
    bool    isFresh() const { return _fresh; }
    uint8_t failCount() const { return _failCount; }
    int16_t rawX() const { return _rawX; }
    int16_t rawY() const { return _rawY; }

    bool nvsLoaded() const { return _nvsLoaded; }
    bool isCalibrating() const { return _calibrating; }
    bool calibrationReady() const;
    bool startCalibration();
    void abortCalibration();
    bool finishCalibration();
    bool clearStoredCalibration();
    void getHardIron(float& xMin, float& xMax, float& yMin, float& yMax) const;
    void getCalProgress(float& xMin, float& xMax, float& yMin, float& yMax,
                        uint16_t& samples, uint32_t& elapsedMs) const;

private:
    int16_t _rawX, _rawY, _rawZ;
    float   _declination;
    float   _xMin, _xMax, _yMin, _yMax;
    uint8_t _addr;
    bool    _fresh;
    uint8_t _failCount;
    bool    _nvsLoaded;
    bool    _calibrating;
    uint32_t _calStartMs;
    uint16_t _calSamples;
    float   _calMinX, _calMaxX, _calMinY, _calMaxY;

    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegisters(uint8_t reg, uint8_t* buf, uint8_t len);
    float applyDeclination(float headingDeg);
    void  applyHardIron(float rawX, float rawY, float& outX, float& outY) const;
    void  setFactoryHardIron();
    void  collectCalibrationSample();
    bool  hardIronSpansOk(float xMin, float xMax, float yMin, float yMax) const;
    bool  saveCalibration();
    bool  loadCalibration();
};

#endif // USV_COMPASS_H
