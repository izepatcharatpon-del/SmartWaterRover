#ifndef USV_WATER_SENSORS_H
#define USV_WATER_SENSORS_H

#include <Arduino.h>
#include <Adafruit_ADS1X15.h>
#include <OneWire.h>
#include <DallasTemperature.h>

struct WaterReading {
    float value;
    bool  valid;
};

class WaterSensors {
public:
    WaterSensors();

    bool begin();

    void  updateTemperatureNonBlocking();
    float getWaterTemp() const { return _waterTemp; }
    bool  tempValid() const { return _tempValid; }

    WaterReading readPh(float tempC);
    WaterReading readTurbidityNtu();

private:
    Adafruit_ADS1115  _ads;
    OneWire           _oneWire;
    DallasTemperature _temp;
    unsigned long     _lastTempRequest;
    float             _waterTemp;
    bool              _tempValid;
    bool              _adsReady;

    bool readChannelVoltage(uint8_t channel, float& outV);
};

#endif // USV_WATER_SENSORS_H
