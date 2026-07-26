#ifndef USV_BATTERY_MONITOR_H
#define USV_BATTERY_MONITOR_H

#include <Arduino.h>

// Propulsion LiPo 3S voltage monitor (AA118). No current sensor on this build.
class BatteryMonitor {
public:
    BatteryMonitor();

    void    begin();
    float   readVoltage();
    float   getSoCPercent(float voltage);
    uint8_t getBatteryPercent();

private:
    float   _lastVoltage;
    uint8_t _saneFailCount;
};

#endif // USV_BATTERY_MONITOR_H
