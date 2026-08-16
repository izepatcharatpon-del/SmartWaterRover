#include "DataLogger.h"
#include "config.h"
#include <SPI.h>
#include <SdFat.h>
#include <stdio.h>

// SdFat — Arduino SD.h fails ACMD41 on this ESP32-S3 + module combo (bench).
static SdFat   g_sd;
static FsFile  g_logFile;
static SPIClass g_spi(HSPI);

static const char* CSV_HEADER =
    "seq,lat,lon,ph,turbidity,temp,mode,battery,heading\n";

DataLogger::DataLogger()
    : _spiMutex(nullptr), _ready(false), _lastFlush(0), _writeFailCount(0) {
    strncpy(_filename, "/mission_log.csv", sizeof(_filename));
    _filename[sizeof(_filename) - 1] = '\0';
}

void DataLogger::openAndWriteHeader() {
    bool isNew = !g_sd.exists(_filename);
    g_logFile = g_sd.open(_filename, O_RDWR | O_CREAT | O_AT_END);
    if (g_logFile && isNew) {
        g_logFile.print(CSV_HEADER);
        g_logFile.flush();
    }
}

bool DataLogger::begin() {
    _spiMutex = xSemaphoreCreateMutex();
    if (!_spiMutex) return false;

    pinMode(SD_CS_PIN, OUTPUT);
    digitalWrite(SD_CS_PIN, HIGH);
    pinMode(SD_MISO_PIN, INPUT_PULLUP);

    g_spi.end();
    delay(20);
    g_spi.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
    delay(200);

    // ≥80 idle clocks with CS high before first CMD (helps ACMD41).
    g_spi.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));
    for (int i = 0; i < 16; i++) g_spi.transfer(0xFF);
    g_spi.endTransaction();
    delay(10);

    bool mounted = false;
    static const uint32_t kHzList[] = { 400000UL, 1000000UL, 100000UL, 4000000UL };
    for (uint8_t i = 0; i < 4 && !mounted; i++) {
        g_sd.end();
        if (!g_sd.cardBegin(SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, kHzList[i], &g_spi))) {
            if (!g_sd.cardBegin(SdSpiConfig(SD_CS_PIN, SHARED_SPI, kHzList[i], &g_spi))) {
                continue;
            }
        }
        if (g_sd.volumeBegin()) {
            mounted = true;
        }
    }

    if (!mounted) {
        _ready = false;
        return false;
    }

    // One CSV per boot so missions do not silently append forever into one file.
    snprintf(_filename, sizeof(_filename), "/m_%lu.csv",
             (unsigned long)millis());
    openAndWriteHeader();
    _ready = (bool)g_logFile;
    _lastFlush = millis();
    _writeFailCount = 0;
    return _ready;
}

bool DataLogger::logRecord(const TelemetryPacket& pkt) {
    if (!_ready) return false;
    if (xSemaphoreTake(_spiMutex, pdMS_TO_TICKS(50)) != pdTRUE) return false;

    bool ok = false;
    if (g_logFile) {
        const int written = g_logFile.printf(
            "%lu,%.6f,%.6f,%.2f,%.1f,%.2f,%u,%u,%.1f\n",
            (unsigned long)pkt.sequence_number,
            pkt.gps_lat, pkt.gps_lon,
            pkt.water_ph, pkt.water_turbidity, pkt.water_temp,
            pkt.current_mode, pkt.battery_percent,
            pkt.heading_angle);
        ok = (written > 0);

        // Flush at most once per second to bound loss without thrashing SD.
        if (ok && millis() - _lastFlush >= 1000) {
            g_logFile.flush();
            _lastFlush = millis();
        }
    }

    if (!ok) {
        _writeFailCount++;
        _ready = false;
    }
    xSemaphoreGive(_spiMutex);
    return ok;
}

bool DataLogger::checkIntegrity() {
    if (!_spiMutex) return false;
    if (xSemaphoreTake(_spiMutex, pdMS_TO_TICKS(50)) != pdTRUE) return false;
    bool cardOk = g_sd.card() && g_sd.card()->errorCode() == 0;
    bool ok = cardOk && (bool)g_logFile && _ready;
    if (!ok) {
        _ready = false;
    }
    xSemaphoreGive(_spiMutex);
    return ok;
}
