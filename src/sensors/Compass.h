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

private:
    int16_t _rawX, _rawY, _rawZ;
    float   _declination;
    uint8_t _addr;
    bool    _fresh;
    uint8_t _failCount;

    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegisters(uint8_t reg, uint8_t* buf, uint8_t len);
    float applyDeclination(float headingDeg);
    void  applyHardIron(float rawX, float rawY, float& outX, float& outY) const;
};

#endif // USV_COMPASS_H
