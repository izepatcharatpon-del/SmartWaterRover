#include "BatteryMonitor.h"
#include "config.h"
#include <math.h>

BatteryMonitor::BatteryMonitor()
    : _lastVoltage(VOLTAGE_FULL),
      _filtVoltage(VOLTAGE_FULL),
      _emaInit(false),
      _saneFailCount(0),
      _outOfRangeLogged(false) {}

void BatteryMonitor::begin() {
    analogSetPinAttenuation(BATTERY_ADC_PIN, ADC_11db);
    _saneFailCount = 0;
    _emaInit = false;
    _outOfRangeLogged = false;
}

float BatteryMonitor::readVoltage() {
    uint32_t mv = analogReadMilliVolts(BATTERY_ADC_PIN);
    float v = (mv / 1000.0f) * BATTERY_DIVIDER_RATIO;
    if (isfinite(v) && v >= VOLTAGE_SANE_MIN && v <= VOLTAGE_SANE_MAX) {
        if (_outOfRangeLogged) {
            Serial.println(F("[USV] battery ADC recovered — voltage in sane range"));
            _outOfRangeLogged = false;
        }
        _saneFailCount = 0;
        if (!_emaInit) {
            _filtVoltage = v;
            _emaInit = true;
        } else {
            _filtVoltage = BATTERY_VOLTAGE_EMA_ALPHA * v +
                           (1.0f - BATTERY_VOLTAGE_EMA_ALPHA) * _filtVoltage;
        }
        _lastVoltage = _filtVoltage;
    } else {
        if (_saneFailCount < 255) _saneFailCount++;
        // Open-wire must not freeze at boot-time FULL.
        if (_saneFailCount >= 10) {
            _lastVoltage = VOLTAGE_CRITICAL;
            _filtVoltage = VOLTAGE_CRITICAL;
            _emaInit = true;
            if (!_outOfRangeLogged) {
                Serial.println(F("[USV] battery ADC out of range x10 — latching CRITICAL"));
                Serial.println(F("[USV] connect the 3S pack; USB is programming only"));
                Serial.println(F("[USV] mode=4 here is NOT a radio failure"));
                _outOfRangeLogged = true;
            }
        }
    }
    return _lastVoltage;
}

float BatteryMonitor::getSoCPercent(float voltage) {
    if (voltage >= VOLTAGE_FULL) return 100.0f;
    if (voltage <= VOLTAGE_CRITICAL) return 0.0f;
    if (voltage <= VOLTAGE_RTH) {
        return (voltage - VOLTAGE_CRITICAL) /
               (VOLTAGE_RTH - VOLTAGE_CRITICAL) * 10.0f;
    }
    if (voltage > 12.0f) return 80.0f + (voltage - 12.0f) * 33.3f;
    if (voltage > 11.5f) return 50.0f + (voltage - 11.5f) * 60.0f;
    if (voltage > 11.1f) return 20.0f + (voltage - 11.1f) * 75.0f;
    return 10.0f + (voltage - VOLTAGE_RTH) / (11.1f - VOLTAGE_RTH) * 10.0f;
}

uint8_t BatteryMonitor::getBatteryPercent() {
    float soc = getSoCPercent(_lastVoltage);
    if (soc < 0.0f)   soc = 0.0f;
    if (soc > 100.0f) soc = 100.0f;
    return (uint8_t)(soc + 0.5f);
}
