#ifndef USV_WATER_SENSORS_H
#define USV_WATER_SENSORS_H

#include <Arduino.h>
#include <Adafruit_ADS1X15.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Reads pH + turbidity via ADS1115 (0-5V on A0/A1; I2C to ESP32 via NA019) and
// via DS18B20 (non-blocking). See codebase_mathematical_models.md.
class WaterSensors {
public:
    WaterSensors();

    bool begin();

    // Non-blocking temperature: call every loop; refreshes at DS18B20 conversion rate.
    void  updateTemperatureNonBlocking();
    float getWaterTemp() const { return _waterTemp; }

    // pH with Nernst temperature compensation (tempC from getWaterTemp()).
    float readPh(float tempC);

    // Turbidity in NTU via quadratic regression with voltage clamps.
    float readTurbidityNtu();

private:
    Adafruit_ADS1115  _ads;
    OneWire           _oneWire;
    DallasTemperature _temp;
    unsigned long     _lastTempRequest;
    float             _waterTemp;
    bool              _adsReady;

    float readChannelVoltage(uint8_t channel);
};

#endif // USV_WATER_SENSORS_H
