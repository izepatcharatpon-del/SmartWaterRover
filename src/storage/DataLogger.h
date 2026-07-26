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

    bool begin();                       // mounts SD, opens a per-boot CSV
    bool logRecord(const TelemetryPacket& pkt);  // false on write/SD failure
    bool checkIntegrity();              // verify card still responds
    bool isReady() const { return _ready; }
    uint32_t writeFailCount() const { return _writeFailCount; }

private:
    SemaphoreHandle_t _spiMutex;
    bool              _ready;
    unsigned long     _lastFlush;
    uint32_t          _writeFailCount;
    char              _filename[32];

    void openAndWriteHeader();
};

#endif // USV_DATA_LOGGER_H
