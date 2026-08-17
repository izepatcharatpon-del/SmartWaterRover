#ifndef USV_BATTERY_MONITOR_H
#define USV_BATTERY_MONITOR_H

#include <Arduino.h>

// Single LiPo 3S pack (motors + board) via AA118. No current sensor on this build.
class BatteryMonitor {
public:
    BatteryMonitor();

    void    begin();
    float   readVoltage();   // EMA-smoothed pack volts
    float   getSoCPercent(float voltage);
    uint8_t getBatteryPercent();

private:
    float   _lastVoltage;
    float   _filtVoltage;
    bool    _emaInit;
    uint8_t _saneFailCount;
    bool    _outOfRangeLogged;
};

#endif // USV_BATTERY_MONITOR_H
