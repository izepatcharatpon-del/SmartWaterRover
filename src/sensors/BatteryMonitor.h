#ifndef USV_BATTERY_MONITOR_H
#define USV_BATTERY_MONITOR_H

#include <Arduino.h>

// Propulsion LiPo 3S monitoring: voltage divider read + Coulomb counting + SoC.
// See battery_management_prediction.md.
class BatteryMonitor {
public:
    BatteryMonitor();

    void  begin();

    // Instantaneous propulsion-pack voltage (via divider on BATTERY_ADC_PIN).
    float readVoltage();

    // Accumulate consumed charge; call regularly with measured current (mA).
    void  updateCoulombCounter(float current_mA);

    // Estimated minutes remaining (Coulomb method).
    float getRemainingMinutes();

    // Voltage -> State of Charge (%) via LiPo 3S lookup + interpolation.
    float getSoCPercent(float voltage);

    // Convenience: SoC of the last measured voltage, clamped 0-100.
    uint8_t getBatteryPercent();

private:
    float         _totalCapacity_mAh;
    float         _consumed_mAh;
    unsigned long _lastSampleTime;
    float         _currentAvg_mA;
    float         _lastVoltage;
};

#endif // USV_BATTERY_MONITOR_H
