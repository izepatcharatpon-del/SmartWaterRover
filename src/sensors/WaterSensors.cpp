#include "WaterSensors.h"
#include "config.h"
#include <math.h>

static const int   TEMP_CONVERSION_MS = 750;
static const uint8_t ADS_PH_CHANNEL   = 0;
static const uint8_t ADS_TURB_CHANNEL = 1;

WaterSensors::WaterSensors()
    : _oneWire(TEMP_SENSOR_PIN),
      _temp(&_oneWire),
      _lastTempRequest(0),
      _waterTemp(25.0f),
      _tempValid(false),
      _adsReady(false) {}

bool WaterSensors::begin() {
    _adsReady = _ads.begin(ADS1115_I2C_ADDR);
    if (_adsReady) {
        _ads.setGain(GAIN_TWOTHIRDS);
    }

    _temp.begin();
    _temp.setWaitForConversion(false);
    _temp.requestTemperatures();
    _lastTempRequest = millis();
    return _adsReady;
}

bool WaterSensors::readChannelVoltage(uint8_t channel, float& outV) {
    if (!_adsReady) return false;
    int16_t raw = _ads.readADC_SingleEnded(channel);
    outV = _ads.computeVolts(raw);
    // pH/turbidity modules are powered 0-5 V and should not rail at zero or
    // exceed the configured ±6.144 V range. Reject common I2C/ADC fault values.
    if (!isfinite(outV) || outV < 0.01f || outV > 5.25f) return false;
    return true;
}

void WaterSensors::updateTemperatureNonBlocking() {
    if (millis() - _lastTempRequest >= (unsigned long)TEMP_CONVERSION_MS) {
        float t = _temp.getTempCByIndex(0);
        if (t > -55.0f && t < 125.0f) {
            _waterTemp = t;
            _tempValid = true;
        } else {
            _tempValid = false;
        }
        _temp.requestTemperatures();
        _lastTempRequest = millis();
    }
}

WaterReading WaterSensors::readPh(float tempC) {
    WaterReading r = {7.0f, false};
    // Nernst slope needs a real temperature. Stale/missing DS18B20 must not
    // produce a "valid" pH that shore or anomaly logic would trust.
    if (!_tempValid || !isfinite(tempC) || tempC < -5.0f || tempC > 60.0f) {
        return r;
    }

    float rawVoltage = 0.0f;
    if (!readChannelVoltage(ADS_PH_CHANNEL, rawVoltage)) return r;

    float slopeAtTemp = 0.05916f * ((tempC + 273.15f) / 298.15f);
    if (slopeAtTemp < 1e-4f) return r;
    float neutralVoltage = 2.0f;
    float phValue = 7.0f + ((neutralVoltage - rawVoltage) / slopeAtTemp);
    if (phValue < 0.0f)  phValue = 0.0f;
    if (phValue > 14.0f) phValue = 14.0f;
    r.value = phValue;
    r.valid = true;
    return r;
}

WaterReading WaterSensors::readTurbidityNtu() {
    WaterReading r = {0.0f, false};
    float voltage = 0.0f;
    if (!readChannelVoltage(ADS_TURB_CHANNEL, voltage)) return r;

    // Module output below ~2.5 V is open-wire / brownout / fault — not "3000 NTU".
    // Reporting max pollution here falsely triggers spiral mapping.
    if (voltage < 2.5f) return r;

    float ntu;
    if (voltage > 4.2f) ntu = 0.0f;
    else {
        ntu = -1120.4f * (voltage * voltage) + 5742.3f * voltage - 4353.8f;
        if (ntu < 0.0f) ntu = 0.0f;
    }
    r.value = ntu;
    r.valid = true;
    return r;
}
