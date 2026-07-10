#include "DataLogger.h"
#include "config.h"
#include <SPI.h>
#include <SD.h>

// Single logger instance -> keep the open file handle at file scope so the
// header need not pull in <SD.h>.
static File     g_logFile;
static SPIClass g_spi(HSPI);

static const char* CSV_HEADER =
    "seq,lat,lon,ph,turbidity,temp,mode,battery,heading\n";

DataLogger::DataLogger()
    : _spiMutex(nullptr), _ready(false), _lastFlush(0) {
    strncpy(_filename, "/mission_log.csv", sizeof(_filename));
    _filename[sizeof(_filename) - 1] = '\0';
}

void DataLogger::openAndWriteHeader() {
    bool isNew = !SD.exists(_filename);
    g_logFile = SD.open(_filename, FILE_APPEND);
    if (g_logFile && isNew) {
        g_logFile.print(CSV_HEADER);
        g_logFile.flush();
    }
}

bool DataLogger::begin() {
    _spiMutex = xSemaphoreCreateMutex();
    if (!_spiMutex) return false;

    g_spi.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
    if (!SD.begin(SD_CS_PIN, g_spi)) {
        _ready = false;
        return false;
    }
    openAndWriteHeader();
    _ready = (bool)g_logFile;
    _lastFlush = millis();
    return _ready;
}

void DataLogger::logRecord(const TelemetryPacket& pkt) {
    if (!_ready) return;
    if (xSemaphoreTake(_spiMutex, pdMS_TO_TICKS(50)) != pdTRUE) return;

    if (g_logFile) {
        g_logFile.printf("%lu,%.6f,%.6f,%.2f,%.1f,%.2f,%u,%u,%.1f\n",
                         (unsigned long)pkt.sequence_number,
                         pkt.gps_lat, pkt.gps_lon,
                         pkt.water_ph, pkt.water_turbidity, pkt.water_temp,
                         pkt.current_mode, pkt.battery_percent,
                         pkt.heading_angle);

        // Flush at most once per second to bound loss without thrashing SD.
        if (millis() - _lastFlush >= 1000) {
            g_logFile.flush();
            _lastFlush = millis();
        }
    }
    xSemaphoreGive(_spiMutex);
}

bool DataLogger::checkIntegrity() {
    if (xSemaphoreTake(_spiMutex, pdMS_TO_TICKS(50)) != pdTRUE) return false;
    bool ok = (SD.cardType() != CARD_NONE) && (bool)g_logFile;
    xSemaphoreGive(_spiMutex);
    return ok;
}
