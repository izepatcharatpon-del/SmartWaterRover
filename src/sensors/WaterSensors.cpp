#include "WaterSensors.h"
#include "config.h"
#include <Preferences.h>
#include <math.h>

static const int   TEMP_CONVERSION_MS = 750;
static const uint8_t ADS_PH_CHANNEL   = 0;  // A0 pH
static const uint8_t ADS_TURB_CHANNEL = 1;  // A1 turbidity

static const char* PH_NVS_NS    = "usv_ph";
static const char* PH_NVS_V401  = "v401";
static const char* PH_NVS_V686  = "v686";
static const char* PH_NVS_V918  = "v918";
static const char* PH_NVS_OK    = "ok";

WaterSensors::WaterSensors()
    : _oneWire(TEMP_SENSOR_PIN),
      _temp(&_oneWire),
      _lastTempRequest(0),
      _waterTemp(25.0f),
      _tempValid(false),
      _adsReady(false),
      _vCap401(NAN), _vCap686(NAN), _vCap918(NAN),
      _cap401(false), _cap686(false), _cap918(false),
      _lastTurbVoltage(NAN) {
    resetCalibration();
}

bool WaterSensors::begin() {
    _adsReady = _ads.begin(ADS1115_I2C_ADDR);
    if (_adsReady) {
        _ads.setGain(GAIN_TWOTHIRDS);
    }

    _temp.begin();
    _temp.setWaitForConversion(false);
    _temp.requestTemperatures();
    _lastTempRequest = millis();

    loadCalibration();
    return _adsReady;
}

bool WaterSensors::readChannelVoltage(uint8_t channel, float& outV) {
    if (!_adsReady) return false;
    int16_t raw = _ads.readADC_SingleEnded(channel);
    outV = _ads.computeVolts(raw);
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

float WaterSensors::readPhVoltage() {
    float v = NAN;
    if (!readChannelVoltage(ADS_PH_CHANNEL, v)) return NAN;
    return v;
}

bool WaterSensors::samplePhVoltage(float& outV, int samples) {
    if (!_adsReady || samples < 1) return false;
    double sum = 0.0;
    int ok = 0;
    for (int i = 0; i < samples; i++) {
        float v = 0.0f;
        if (readChannelVoltage(ADS_PH_CHANNEL, v)) {
            sum += v;
            ok++;
        }
        delay(20);
    }
    if (ok < (samples / 2 + 1)) return false;
    outV = (float)(sum / (double)ok);
    return true;
}

bool WaterSensors::captureBuffer401(float& outV) {
    if (!samplePhVoltage(outV, 30)) return false;
    _vCap401 = outV;
    _cap401 = true;
    return true;
}

bool WaterSensors::captureBuffer686(float& outV) {
    if (!samplePhVoltage(outV, 30)) return false;
    _vCap686 = outV;
    _cap686 = true;
    return true;
}

bool WaterSensors::captureBuffer918(float& outV) {
    if (!samplePhVoltage(outV, 30)) return false;
    _vCap918 = outV;
    _cap918 = true;
    return true;
}

bool WaterSensors::calibrateThreePoint(float v401, float v686, float v918) {
    // Expect acid voltage > mid > alkali (typical glass electrode)
    if (!isfinite(v401) || !isfinite(v686) || !isfinite(v918)) return false;
    if (v401 <= v686 || v686 <= v918) return false;
    if (v401 > 5.0f || v918 < 0.2f) return false;

    float slopeAcid = fabsf((v401 - v686) / (PH_BUF_401 - PH_BUF_686));
    float slopeAlk  = fabsf((v686 - v918) / (PH_BUF_686 - PH_BUF_918));
    // Ideal ~0.059 V/pH @ 25 °C — allow clone-board gain / aging band
    if (slopeAcid < 0.020f || slopeAcid > 0.250f) return false;
    if (slopeAlk  < 0.020f || slopeAlk  > 0.250f) return false;

    setCalibration(v401, v686, v918);
    return true;
}

bool WaterSensors::applyCapturedCalibration() {
    if (!_cap401 || !_cap686 || !_cap918) return false;
    return calibrateThreePoint(_vCap401, _vCap686, _vCap918);
}

void WaterSensors::setCalibration(float v401, float v686, float v918) {
    _phCal.v401  = v401;
    _phCal.v686  = v686;
    _phCal.v918  = v918;
    _phCal.valid = true;
}

void WaterSensors::resetCalibration() {
    _phCal.v401  = NAN;
    _phCal.v686  = PH_NEUTRAL_VOLTAGE_V;  // fallback single-point mid
    _phCal.v918  = NAN;
    _phCal.valid = false;
    _cap401 = _cap686 = _cap918 = false;
    _vCap401 = _vCap686 = _vCap918 = NAN;
}

bool WaterSensors::saveCalibration() {
    if (!_phCal.valid) return false;
    Preferences prefs;
    if (!prefs.begin(PH_NVS_NS, false)) return false;
    prefs.putFloat(PH_NVS_V401, _phCal.v401);
    prefs.putFloat(PH_NVS_V686, _phCal.v686);
    prefs.putFloat(PH_NVS_V918, _phCal.v918);
    prefs.putUChar(PH_NVS_OK, 1);
    prefs.end();
    return true;
}

bool WaterSensors::loadCalibration() {
    Preferences prefs;
    if (prefs.begin(PH_NVS_NS, true)) {
        if (prefs.getUChar(PH_NVS_OK, 0) != 0) {
            float v401 = prefs.getFloat(PH_NVS_V401, NAN);
            float v686 = prefs.getFloat(PH_NVS_V686, NAN);
            float v918 = prefs.getFloat(PH_NVS_V918, NAN);
            prefs.end();
            if (calibrateThreePoint(v401, v686, v918)) return true;
        } else {
            prefs.end();
        }
    }

    // No valid NVS yet — apply bench factory defaults, then persist them.
    if (!calibrateThreePoint(PH_CAL_V401, PH_CAL_V686, PH_CAL_V918)) {
        resetCalibration();
        return false;
    }
    saveCalibration();
    return true;
}

float WaterSensors::phFromVoltageCalibrated(float volts, float tempC) const {
    float t = tempC;
    if (!isfinite(t)) t = 25.0f;
    float kelvinRatio = (t + 273.15f) / 298.15f;

    if (_phCal.valid) {
        float slopeAcid25 = fabsf((_phCal.v401 - _phCal.v686) /
                                  (PH_BUF_401 - PH_BUF_686));
        float slopeAlk25  = fabsf((_phCal.v686 - _phCal.v918) /
                                  (PH_BUF_686 - PH_BUF_918));
        float slopeAcid = slopeAcid25 * kelvinRatio;
        float slopeAlk  = slopeAlk25 * kelvinRatio;

        float ph;
        if (volts >= _phCal.v686) {
            // Acidic side (higher V → lower pH)
            ph = PH_BUF_686 + ((_phCal.v686 - volts) / slopeAcid);
        } else {
            // Alkaline side (lower V → higher pH)
            ph = PH_BUF_686 + ((_phCal.v686 - volts) / slopeAlk);
        }
        return ph;
    }

    // Fallback: single-point Nernst around config mid voltage
    float slopeAtTemp =
        PH_SLOPE_25C_V * PH_SLOPE_SCALE * kelvinRatio;
    if (slopeAtTemp < 1e-4f) return NAN;
    return 7.0f + ((PH_NEUTRAL_VOLTAGE_V - volts) / slopeAtTemp);
}

WaterReading WaterSensors::readPh(float tempC) {
    WaterReading r = {7.0f, false};
    if (!_tempValid || !isfinite(tempC) || tempC < -5.0f || tempC > 60.0f) {
        return r;
    }

    float rawVoltage = 0.0f;
    if (!readChannelVoltage(ADS_PH_CHANNEL, rawVoltage)) return r;

    float phValue = phFromVoltageCalibrated(rawVoltage, tempC);
    if (!isfinite(phValue)) return r;
    if (phValue < 0.0f)  phValue = 0.0f;
    if (phValue > 14.0f) phValue = 14.0f;
    r.value = phValue;
    r.valid = true;
    return r;
}

WaterReading WaterSensors::readTurbidityNtu() {
    WaterReading r = {0.0f, false};
    _lastTurbVoltage = NAN;
    if (!_adsReady) return r;

    // Keep raw volts even below TURBIDITY_FAULT_V so Serial can show trim/open-wire.
    const int16_t raw = _ads.readADC_SingleEnded(ADS_TURB_CHANNEL);
    const float voltage = _ads.computeVolts(raw);
    _lastTurbVoltage = voltage;
    if (!isfinite(voltage) || voltage < 0.01f || voltage > 5.25f) return r;
    if (voltage < TURBIDITY_FAULT_V) return r;
#if !TURBIDITY_OPTICS_OK
    // Voltage is logged via lastTurbVoltage(); do not publish NTU or arm spiral.
    return r;
#endif

    float ntu;
    if (voltage > TURBIDITY_CLEAR_V) ntu = 0.0f;
    else {
        ntu = TURBIDITY_A * (voltage * voltage) + TURBIDITY_B * voltage + TURBIDITY_C;
        ntu = ntu * TURBIDITY_NTU_SCALE + TURBIDITY_NTU_OFFSET;
        if (ntu < 0.0f) ntu = 0.0f;
    }
    r.value = ntu;
    r.valid = true;
    return r;
}
