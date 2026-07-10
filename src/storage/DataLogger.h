#ifndef USV_DATA_LOGGER_H
#define USV_DATA_LOGGER_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "Packets.h"

// Appends telemetry rows to a CSV on the SD card. Guards the shared SPI bus
// with a mutex and flushes at most once per second to bound data loss on power
// cut. See project_vulnerabilities.md section 6.
class DataLogger {
public:
    DataLogger();

    bool begin();                       // mounts SD, writes CSV header if new
    void logRecord(const TelemetryPacket& pkt);
    bool checkIntegrity();              // verify card still responds
    bool isReady() const { return _ready; }

private:
    SemaphoreHandle_t _spiMutex;
    bool              _ready;
    unsigned long     _lastFlush;
    char              _filename[24];

    void openAndWriteHeader();
};

#endif // USV_DATA_LOGGER_H
