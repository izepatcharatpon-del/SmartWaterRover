#include "BatteryMonitor.h"
#include "config.h"

BatteryMonitor::BatteryMonitor()
    : _totalCapacity_mAh(BATTERY_CAPACITY_MAH),
      _consumed_mAh(0.0f),
      _lastSampleTime(0),
      _currentAvg_mA(1000.0f),
      _lastVoltage(VOLTAGE_FULL) {}

void BatteryMonitor::begin() {
    analogSetPinAttenuation(BATTERY_ADC_PIN, ADC_11db);  // 0-3.1V usable range
    // Initialise timer in begin() so first dt is not inflated (fix m-10).
    _lastSampleTime = millis();
}

float BatteryMonitor::readVoltage() {
    // analogReadMilliVolts applies the factory eFuse ADC calibration.
    uint32_t mv = analogReadMilliVolts(BATTERY_ADC_PIN);
    _lastVoltage = (mv / 1000.0f) * BATTERY_DIVIDER_RATIO;
    return _lastVoltage;
}

void BatteryMonitor::updateCoulombCounter(float current_mA) {
    unsigned long now = millis();
    float dt = (now - _lastSampleTime) / 1000.0f;   // seconds
    _lastSampleTime = now;

    if (dt > 0.0f && dt < 10.0f) {                  // guard boot / stalls
        _consumed_mAh += (current_mA * dt) / 3600.0f;
    }
    // Weighted moving average of current (smooths surge draw).
    _currentAvg_mA = (_currentAvg_mA * 0.95f) + (current_mA * 0.05f);
}

float BatteryMonitor::getRemainingMinutes() {
    float remaining = _totalCapacity_mAh - _consumed_mAh;
    if (remaining < 0.0f) remaining = 0.0f;
    if (_currentAvg_mA <= 10.0f) return 999.0f;     // idle: avoid div-by-zero
    return (remaining / _currentAvg_mA) * 60.0f;
}

float BatteryMonitor::getSoCPercent(float voltage) {
    if (voltage >= 12.6f) return 100.0f;
    if (voltage <= 10.5f) return 0.0f;
    if (voltage > 12.0f) return 80.0f + (voltage - 12.0f) * 33.3f; // 12.0-12.6
    if (voltage > 11.5f) return 50.0f + (voltage - 11.5f) * 60.0f; // 11.5-12.0
    if (voltage > 11.1f) return 20.0f + (voltage - 11.1f) * 75.0f; // 11.1-11.5
    return (voltage - 10.5f) * 33.3f;                              // 10.5-11.1
}

uint8_t BatteryMonitor::getBatteryPercent() {
    float soc = getSoCPercent(_lastVoltage);
    if (soc < 0.0f)   soc = 0.0f;
    if (soc > 100.0f) soc = 100.0f;
    return (uint8_t)(soc + 0.5f);
}
