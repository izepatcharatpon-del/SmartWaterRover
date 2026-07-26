#ifndef USV_TASKS_H
#define USV_TASKS_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#include "Packets.h"
#include "sensors/WaterSensors.h"
#include "sensors/BatteryMonitor.h"
#include "sensors/Compass.h"
#include "sensors/GpsModule.h"
#include "actuators/MotorDriver.h"
#include "control/HeadingController.h"
#include "navigation/Navigator.h"
#include "navigation/SpiralPlanner.h"
#include "detection/WaterAnomalyMonitor.h"
#include "comms/EspNowLink.h"
#include "storage/DataLogger.h"
#include "safety/Watchdog.h"
#include "safety/SafetyManager.h"
#include "core/MissionFsm.h"

// Command flags produced by the IO task and consumed by the navigation task.
// Protected by SystemContext::stateMutex.
struct CommandState {
    bool     start;
    bool     clearEstop;
    bool     emergency;
    bool     forceRth;
    bool     sealMissionUpload;
    bool     manualMode;
    int      manualSteer;
    int      manualSpeed;
    uint32_t lastManualMs;
};

struct SystemContext {
    WaterSensors*        water;
    BatteryMonitor*      battery;
    Compass*             compass;
    GpsModule*           gps;
    MotorDriver*         motor;
    HeadingController*   heading;
    Navigator*           nav;
    SpiralPlanner*       spiralPlanner;
    WaterAnomalyMonitor* anomaly;
    EspNowLink*          link;
    DataLogger*          logger;
    Watchdog*            wdt;
    SafetyManager*       safety;
    MissionFsm*          fsm;

    QueueHandle_t     logQueue;
    SemaphoreHandle_t stateMutex;

    TelemetryPacket   snapshot;
    CommandState      command;
    float             batteryVoltage;
    uint32_t          logDropCount;
    uint32_t          ackDropCount;
};

void navigationTask(void* pv);
void ioTask(void* pv);

#endif // USV_TASKS_H
