#include "WaterSensors.h"
#include "config.h"

static const int   TEMP_CONVERSION_MS = 750;  // DS18B20 12-bit conversion time
static const uint8_t ADS_PH_CHANNEL   = 0;
static const uint8_t ADS_TURB_CHANNEL = 1;

WaterSensors::WaterSensors()
    : _oneWire(TEMP_SENSOR_PIN),
      _temp(&_oneWire),
      _lastTempRequest(0),
      _waterTemp(25.0f),
      _adsReady(false) {}

bool WaterSensors::begin() {
    _adsReady = _ads.begin(ADS1115_I2C_ADDR);
    if (_adsReady) {
        // +/-6.144V range: pH/turbidity 0-5V on A0/A1 (NA019 is on I2C only).
        _ads.setGain(GAIN_TWOTHIRDS);
    }

    _temp.begin();
    _temp.setWaitForConversion(false);   // non-blocking mode
    _temp.requestTemperatures();
    _lastTempRequest = millis();
    return _adsReady;
}

float WaterSensors::readChannelVoltage(uint8_t channel) {
    if (!_adsReady) return 0.0f;
    int16_t raw = _ads.readADC_SingleEnded(channel);
    return _ads.computeVolts(raw);
}

void WaterSensors::updateTemperatureNonBlocking() {
    if (millis() - _lastTempRequest >= (unsigned long)TEMP_CONVERSION_MS) {
        float t = _temp.getTempCByIndex(0);
        if (t > -55.0f && t < 125.0f) {   // ignore DEVICE_DISCONNECTED_C etc.
            _waterTemp = t;
        }
        _temp.requestTemperatures();      // kick off next conversion
        _lastTempRequest = millis();
    }
}

float WaterSensors::readPh(float tempC) {
    float rawVoltage = readChannelVoltage(ADS_PH_CHANNEL);
    // Nernst slope factor (V/pH) scaled by absolute temperature.
    float slopeAtTemp   = 0.05916f * ((tempC + 273.15f) / 298.15f);
    float neutralVoltage = 2.0f;   // pH 7.00 reference voltage assumption
    float phValue = 7.0f + ((neutralVoltage - rawVoltage) / slopeAtTemp);
    if (phValue < 0.0f)  phValue = 0.0f;
    if (phValue > 14.0f) phValue = 14.0f;
    return phValue;
}

float WaterSensors::readTurbidityNtu() {
    float voltage = readChannelVoltage(ADS_TURB_CHANNEL);
    if (voltage < 2.5f) return 3000.0f;  // very turbid
    if (voltage > 4.2f) return 0.0f;     // clear water
    float ntu = -1120.4f * (voltage * voltage) + 5742.3f * voltage - 4353.8f;
    return ntu < 0.0f ? 0.0f : ntu;
}
