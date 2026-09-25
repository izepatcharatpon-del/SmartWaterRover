#ifndef USV_WATER_SENSORS_H
#define USV_WATER_SENSORS_H

#include <Arduino.h>
#include <Adafruit_ADS1X15.h>
#include <OneWire.h>
#include <DallasTemperature.h>

struct WaterReading {
    float value;  // computed reading; NAN when ADS/calculation produced nothing
    bool  valid;  // quality flag (pH is diagnostic-only, not a spiral trigger)
};

// 3-point pH cal @ 25 °C — buffers 4.01 / 6.86 / 9.18
// Persisted in NVS namespace "usv_ph".
struct PhCalibration {
    float v401;      // voltage in pH 4.01 buffer
    float v686;      // voltage in pH 6.86 buffer (neutral ref)
    float v918;      // voltage in pH 9.18 buffer
    bool  valid;
};

class WaterSensors {
public:
    WaterSensors();

    bool begin();
    bool adsReady() const { return _adsReady; }
    // Retry ADS begin() if the first attempt failed. No delay().
    bool pollAds(unsigned long nowMs);

    void  updateTemperatureNonBlocking();
    float getWaterTemp() const { return _waterTemp; }
    bool  tempValid() const { return _tempValid; }

    WaterReading readPh(float tempC);
    WaterReading readTurbidityNtu();
    // Last ADS A0/A1 volts (kept even when pH/NTU are invalid).
    float lastPhVoltage() const { return _lastPhVoltage; }
    float lastTurbVoltage() const { return _lastTurbVoltage; }

    // --- pH calibration (buffers @ 25 °C) ---
    bool  samplePhVoltage(float& outV, int samples = 30);
    float readPhVoltage();

    // Capture one buffer voltage (probe immersed & stable).
    bool captureBuffer401(float& outV);  // pH 4.01
    bool captureBuffer686(float& outV);  // pH 6.86
    bool captureBuffer918(float& outV);  // pH 9.18

    // Apply 3 voltages → piecewise slope cal. Expects V401 > V686 > V918.
    bool calibrateThreePoint(float v401, float v686, float v918);

    // Convenience: use last captured voltages (all three must be set).
    bool applyCapturedCalibration();

    void setCalibration(float v401, float v686, float v918);
    void resetCalibration();
    PhCalibration getCalibration() const { return _phCal; }
    bool hasCaptured401() const { return _cap401; }
    bool hasCaptured686() const { return _cap686; }
    bool hasCaptured918() const { return _cap918; }
    float captured401() const { return _vCap401; }
    float captured686() const { return _vCap686; }
    float captured918() const { return _vCap918; }

    bool saveCalibration();
    bool loadCalibration();

private:
    Adafruit_ADS1115  _ads;
    OneWire           _oneWire;
    DallasTemperature _temp;
    unsigned long     _lastTempRequest;
    float             _waterTemp;
    bool              _tempValid;
    bool              _adsReady;
    unsigned long     _lastAdsRetryMs;
    PhCalibration     _phCal;

    float _vCap401, _vCap686, _vCap918;
    bool  _cap401, _cap686, _cap918;
    float _lastPhVoltage;
    float _lastTurbVoltage;

    bool readChannelVoltage(uint8_t channel, float& outV);
    float phFromVoltageCalibrated(float volts, float tempC) const;
};

#endif // USV_WATER_SENSORS_H
