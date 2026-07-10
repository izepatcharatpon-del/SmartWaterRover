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
#include "control/SpiralController.h"
#include "control/EllipseTracker.h"
#include "navigation/Navigator.h"
#include "navigation/SpiralPlanner.h"
#include "detection/WaterAnomalyMonitor.h"
#include "comms/EspNowLink.h"
#include "storage/DataLogger.h"
#include "safety/Watchdog.h"
#include "safety/SafetyManager.h"
#include "core/MissionFsm.h"

// Command flags produced by the IO task (from inbound ESP-NOW) and consumed by
// the navigation task. Protected by SystemContext::stateMutex.
struct CommandState {
    bool  start;
    bool  emergency;
    bool  forceRth;
    bool  manualMode;
    int   manualSteer;
    int   manualSpeed;
};

// Everything the tasks need, allocated once in main and passed by pointer.
struct SystemContext {
    WaterSensors*        water;
    BatteryMonitor*      battery;
    Compass*             compass;
    GpsModule*           gps;
    MotorDriver*         motor;
    HeadingController*   heading;
    SpiralController*    spiral;
    EllipseTracker*      ellipse;
    Navigator*           nav;
    SpiralPlanner*       spiralPlanner;
    WaterAnomalyMonitor* anomaly;
    EspNowLink*          link;
    DataLogger*          logger;
    Watchdog*            wdt;
    SafetyManager*       safety;
    MissionFsm*          fsm;

    QueueHandle_t     logQueue;      // TelemetryPacket -> SD writer
    SemaphoreHandle_t stateMutex;    // guards snapshot + command

    TelemetryPacket   snapshot;      // latest telemetry (nav writes, io reads)
    CommandState      command;       // latest operator command (io writes)
};

// Core 1: navigation + control @ 20 Hz. pv = SystemContext*.
void navigationTask(void* pv);

// Core 0: ESP-NOW drain, telemetry TX, SD logging. pv = SystemContext*.
void ioTask(void* pv);

#endif // USV_TASKS_H
