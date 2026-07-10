#ifndef USV_COMPASS_H
#define USV_COMPASS_H

#include <Arduino.h>

// QMC5883L 3-axis magnetometer (raw I2C register access at 0x0D) with
// hard-iron offset, soft-iron scale, and magnetic declination correction.
// See project_prerequisites.md 6.2.
class Compass {
public:
    Compass();

    bool  begin();
    void  update();               // read raw XYZ from the sensor
    float getHeadingDeg();        // 0-360, north-referenced + declination

    void  setHardIron(float x, float y, float z);
    void  setSoftIron(float sx, float sy, float sz);
    void  setDeclination(float degrees);
    float applyDeclination(float headingDeg);

    // Blocking figure-8 calibration helper (call from bench routine).
    void  runCalibration(unsigned long durationMs);

private:
    int16_t _rawX, _rawY, _rawZ;
    float   _offX, _offY, _offZ;
    float   _scaleX, _scaleY, _scaleZ;
    float   _declination;

    bool    writeRegister(uint8_t reg, uint8_t value);
};

#endif // USV_COMPASS_H
