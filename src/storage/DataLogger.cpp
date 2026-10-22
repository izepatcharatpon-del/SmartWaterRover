#include "DataLogger.h"
#include "config.h"
#include "core/BuildIdentity.h"
#include <Preferences.h>
#include <SPI.h>
#include <SdFat.h>
#include <stdio.h>
#include <string.h>

static SdFat   g_sd;
static FsFile  g_logFile;
static FsFile  g_phDiagFile;
static FsFile  g_txDiagFile;
static SPIClass g_spi(HSPI);

static const char* SD_NVS_NS   = "usv_sd";
static const char* SD_NVS_BOOT = "boot";

static const char* CSV_HEADER =
    "boot_ms,utc,local_time,seq,lat,lon,ph,ntu,temp,phV,turbV,mode,battery,heading,"
    "gps_hdop,gps_sats,gps_speed_kmh,wp_index,wp_count,target_lat,target_lon,"
    "nav_bearing,cross_track,dist_target,heading_error,sample_phase,sample_ms,"
    "base_pwm,left_pwm,right_pwm,"
    "compass_raw_x,compass_raw_y,compass_cal_nvs,"
    "gps_raw_lat,gps_raw_lon,gps_course,gps_location_age_ms,gps_valid,"
    "compass_fresh,compass_fail,direct_bearing,requested_left_pwm,"
    "requested_right_pwm,turn_in_place,drive_source,drive_stop_reason,"
    "tel_tx_seq,tel_tx_ms,tel_radio_result,tel_tx_attempts,tel_tx_lat,"
    "tel_tx_lon,tel_tx_ph,radio_tx_paused,battery_voltage\n";

static const char* PH_DIAG_HEADER =
    "boot_ms,sd_seq,ph_sample_id,ph_raw_adc,ph_sample_ms,ph_sample_age_ms,"
    "ph_new_sample,ph_adc_v,ph_sensor_ph,ph_quality_valid,ph_quiet_count,"
    "ph_sensor_temp_c,ph_effective_temp_c,ph_temp_valid,ph_cal_source,"
    "ph_read_reason,ph_suppression_reason,utc_source,utc_status,utc_age_ms\n";
static const char* TX_DIAG_HEADER =
    "sequence,send_start_ms,send_end_ms,send_duration_ms,gps_lat,gps_lon,"
    "water_ph,water_turbidity,water_temp_c,mode,battery_percent,heading_deg,"
    "mac_result,send_error,attempts,no_ack_count,send_error_count,timeout_count,"
    "wifi_channel,peer_exists\n";

static const char* telemetryRadioStatusName(uint8_t status) {
    switch (status) {
        case TELEMETRY_RADIO_NEVER:    return "NEVER";
        case TELEMETRY_RADIO_ACK:      return "ACK";
        case TELEMETRY_RADIO_NO_ACK:   return "NO_ACK";
        case TELEMETRY_RADIO_SEND_ERR: return "SEND_ERR";
        case TELEMETRY_RADIO_TIMEOUT:  return "TIMEOUT";
        default:                       return "UNKNOWN";
    }
}

struct SdDateTime {
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
};

static bool isLeapYear(uint16_t year) {
    return (year % 4 == 0 && year % 100 != 0) || year % 400 == 0;
}

static uint8_t daysInMonth(uint16_t year, uint8_t month) {
    static const uint8_t days[] = {
        31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
    };
    if (month < 1 || month > 12) return 0;
    if (month == 2 && isLeapYear(year)) return 29;
    return days[month - 1];
}

static bool utcToLocal(const SdLogRecord& rec, SdDateTime& local) {
    if (!rec.utc_valid || rec.utc_year < 1980 || rec.utc_year > 2099 ||
        rec.utc_month < 1 || rec.utc_month > 12 ||
        rec.utc_day < 1 ||
        rec.utc_day > daysInMonth(rec.utc_year, rec.utc_month) ||
        rec.utc_hour > 23 || rec.utc_minute > 59 || rec.utc_second > 59) {
        return false;
    }

    local.year = rec.utc_year;
    local.month = rec.utc_month;
    local.day = rec.utc_day;
    local.minute = rec.utc_minute;
    local.second = rec.utc_second;

    int hour = (int)rec.utc_hour + GPS_TIMEZONE_OFFSET_H;
    while (hour >= 24) {
        hour -= 24;
        local.day++;
        if (local.day > daysInMonth(local.year, local.month)) {
            local.day = 1;
            local.month++;
            if (local.month > 12) {
                local.month = 1;
                local.year++;
            }
        }
    }
    while (hour < 0) {
        hour += 24;
        if (local.day > 1) {
            local.day--;
        } else {
            if (local.month > 1) {
                local.month--;
            } else {
                local.month = 12;
                local.year--;
            }
            local.day = daysInMonth(local.year, local.month);
        }
    }
    local.hour = (uint8_t)hour;
    return local.year >= 1980 && local.year <= 2099;
}

static bool setFileTimestamp(const SdDateTime& local, bool setCreate) {
    uint8_t flags = T_ACCESS | T_WRITE;
    if (setCreate) flags |= T_CREATE;
    return g_logFile.timestamp(flags, local.year, local.month, local.day,
                               local.hour, local.minute, local.second);
}

DataLogger::DataLogger()
    : _spiMutex(nullptr), _ready(false), _lastFlush(0), _writeFailCount(0),
      _txDiagnosticWriteFailCount(0), _phDiagnosticWriteFailCount(0),
      _fatCreateTimeSet(false) {
    strncpy(_filename, "/m_00000.csv", sizeof(_filename));
    _filename[sizeof(_filename) - 1] = '\0';
}

uint32_t DataLogger::nextBootIndex() {
    Preferences prefs;
    if (!prefs.begin(SD_NVS_NS, false)) {
        return 0;
    }
    uint32_t n = prefs.getUInt(SD_NVS_BOOT, 0) + 1;
    prefs.putUInt(SD_NVS_BOOT, n);
    prefs.end();
    return n;
}

void DataLogger::openAndWriteHeader() {
    bool isNew = !g_sd.exists(_filename);
    g_logFile = g_sd.open(_filename, O_RDWR | O_CREAT | O_AT_END);
    if (g_logFile && isNew) {
        g_logFile.print(CSV_HEADER);
        g_logFile.flush();
    }
}

bool DataLogger::begin(float calV401, float calV686, float calV918,
                       bool calValid, const char* calSource) {
    _spiMutex = xSemaphoreCreateMutex();
    if (!_spiMutex) return false;

    pinMode(SD_CS_PIN, OUTPUT);
    digitalWrite(SD_CS_PIN, HIGH);
    pinMode(SD_MISO_PIN, INPUT_PULLUP);

    g_spi.end();
    delay(20);
    g_spi.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
    delay(200);

    g_spi.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));
    for (int i = 0; i < 16; i++) g_spi.transfer(0xFF);
    g_spi.endTransaction();
    delay(10);

    bool mounted = false;
    static const uint32_t kHzList[] = { 400000UL, 1000000UL, 100000UL, 4000000UL };
    for (uint8_t i = 0; i < 4 && !mounted; i++) {
        g_sd.end();
        // DEDICATED_SPI panics if begin/endTransaction are on different tasks.
        if (!g_sd.cardBegin(SdSpiConfig(SD_CS_PIN, SHARED_SPI, kHzList[i], &g_spi))) {
            continue;
        }
        if (g_sd.volumeBegin()) {
            mounted = true;
        }
    }

    if (!mounted) {
        _ready = false;
        return false;
    }

    const uint32_t boot = nextBootIndex();
    if (boot == 0) {
        snprintf(_filename, sizeof(_filename), "/m_t%lu.csv",
                 (unsigned long)millis());
    } else {
        snprintf(_filename, sizeof(_filename), "/m_%05lu.csv",
                 (unsigned long)boot);
    }
    openAndWriteHeader();
    snprintf(_txFilename, sizeof(_txFilename), "%s.tx.csv", _filename);
    snprintf(_metadataFilename, sizeof(_metadataFilename), "%s.meta.txt", _filename);
    char phName[32];
    snprintf(phName, sizeof(phName), "%s.ph.csv", _filename);
    const bool phDiagIsNew = !g_sd.exists(phName);
    g_phDiagFile = g_sd.open(phName, O_RDWR | O_CREAT | O_AT_END);
    if (!g_phDiagFile || (phDiagIsNew && g_phDiagFile.print(PH_DIAG_HEADER) == 0)) {
        _phDiagnosticWriteFailCount++;
        g_phDiagFile.close();
        Serial.println(F("[USV] pH diagnostic sidecar unavailable"));
    }
    const bool txDiagIsNew = !g_sd.exists(_txFilename);
    g_txDiagFile = g_sd.open(_txFilename, O_RDWR | O_CREAT | O_AT_END);
    if (!g_txDiagFile || (txDiagIsNew && g_txDiagFile.print(TX_DIAG_HEADER) == 0)) {
        _txDiagnosticWriteFailCount++;
        g_txDiagFile.close();
        Serial.println(F("[USV] TX diagnostic sidecar unavailable"));
    }
    writeMetadata(calV401, calV686, calV918, calValid, calSource);
    g_spi.endTransaction();
    _ready = (bool)g_logFile;
    _lastFlush = millis();
    _lastSyncCompleted = millis();
    _writeFailCount = 0;
    _fatCreateTimeSet = false;
    return _ready;
}

void DataLogger::writeMetadata(float calV401, float calV686, float calV918,
                               bool calValid, const char* calSource) {
    const bool isNew = !g_sd.exists(_metadataFilename);
    FsFile metadata = g_sd.open(_metadataFilename, O_RDWR | O_CREAT | O_AT_END);
    if (!metadata) {
        Serial.println(F("[USV] diagnostic metadata unavailable"));
        return;
    }
    if (isNew) {
        metadata.printf("schema=%s\nrun_id=%s\nfirmware_build_id=%s\n",
                        USV_DIAGNOSTIC_SCHEMA, _filename, USV_BUILD_ID);
        metadata.println("build_id_inputs=dirty bytes under include/ and src/, platformio.ini, tools/build_identity.py; SHA-256 truncated to 16 hex chars");
        metadata.printf("ph_cal_source_at_boot=%s\nph_cal_valid_at_boot=%u\n",
                        calSource ? calSource : "UNKNOWN", calValid ? 1U : 0U);
        metadata.printf("ph_cal_v401=%.6f\nph_cal_v686=%.6f\nph_cal_v918=%.6f\n",
                        (double)calV401, (double)calV686, (double)calV918);
        metadata.println("per_sample_calibration_provenance=UNKNOWN (no synchronization or conversion semantics changed)");
        metadata.println("utc_source=GPS date+time; available only under existing utcNow validity/freshness policy; boot_ms always retained");
        metadata.println("ph_sidecar_rows=SD navigation rows; repeated ph_sample_id means repeated observation, not a new ADC conversion");
        metadata.println("tx_sidecar_rows=one per sendTelemetry call; sequence is packet sequence; MAC ACK is not application ingestion");
        metadata.println("legacy_csv=unchanged column order and meaning");
        metadata.flush();
    }
}

bool DataLogger::logPhDiagnostic(
    uint32_t bootMs, uint32_t sdSequence,
    const DiagnosticFormatting::PhLogRecord& rec) {
    if (_phStopped || !g_phDiagFile || !_spiMutex ||
        xSemaphoreTake(_spiMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        _phDiagnosticWriteFailCount++;
        return false;
    }
    char suffix[384];
    char row[PH_ROW_BYTES];
    bool ok = DiagnosticFormatting::formatPhCsvSuffix(rec, suffix, sizeof(suffix));
    int length = ok ? snprintf(row, sizeof(row), "%lu,%lu%s\n",
                               (unsigned long)bootMs,
                               (unsigned long)sdSequence, suffix) : -1;
    ok = ok && length > 0 && (size_t)length < sizeof(row) &&
         _phBuffer.append(row, (size_t)length, millis());
    if (ok) ++_phAccepted;
    if (!ok) _phDiagnosticWriteFailCount++;
    xSemaphoreGive(_spiMutex);
    return ok;
}

bool DataLogger::logTxDiagnostic(const DiagnosticFormatting::TxLogRecord& rec) {
    if (!g_txDiagFile || !_spiMutex ||
        xSemaphoreTake(_spiMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        _txDiagnosticWriteFailCount++;
        return false;
    }
    char row[320];
    bool ok = DiagnosticFormatting::formatTxCsvRow(rec, row, sizeof(row));
    const size_t length = ok ? strlen(row) : 0;
    ok = ok && g_txDiagFile.print(row) == length;
    if (!ok) _txDiagnosticWriteFailCount++;
    xSemaphoreGive(_spiMutex);
    return ok;
}

bool DataLogger::logRecord(const SdLogRecord& rec) {
    if (!_ready || !g_logFile) return false;
    if (xSemaphoreTake(_spiMutex, pdMS_TO_TICKS(50)) != pdTRUE) return false;
    char row[MAIN_ROW_BYTES];
    SdDateTime local = {};
        const bool localValid = utcToLocal(rec, local);
        const char tzSign = GPS_TIMEZONE_OFFSET_H < 0 ? '-' : '+';
        const unsigned tzHours = (unsigned)(GPS_TIMEZONE_OFFSET_H < 0
            ? -GPS_TIMEZONE_OFFSET_H : GPS_TIMEZONE_OFFSET_H);
        int written;
        if (localValid) {
            written = snprintf(row, sizeof(row),
                "%lu,%04u-%02u-%02uT%02u:%02u:%02uZ,"
                "%04u-%02u-%02uT%02u:%02u:%02u%c%02u:00,"
                "%lu,%.6f,%.6f,"
                "%.2f,%.1f,%.2f,%.3f,%.3f,%u,%u,%.1f,"
                "%.2f,%u,%.2f,%u,%u,%.6f,%.6f,"
                "%.1f,%.2f,%.2f,%.1f,%u,%lu,%d,%d,%d,%d,%d,%u,"
                "%.6f,%.6f,%.1f,%lu,%u,%u,%u,%.1f,%d,%d,%u,%u,%u,"
                "%lu,%lu,%s,%u,%.6f,%.6f,%.2f,%u,%.2f\n",
                (unsigned long)rec.boot_ms,
                (unsigned)rec.utc_year, (unsigned)rec.utc_month,
                (unsigned)rec.utc_day, (unsigned)rec.utc_hour,
                (unsigned)rec.utc_minute, (unsigned)rec.utc_second,
                (unsigned)local.year, (unsigned)local.month,
                (unsigned)local.day, (unsigned)local.hour,
                (unsigned)local.minute, (unsigned)local.second,
                tzSign, tzHours,
                (unsigned long)rec.sequence_number,
                rec.gps_lat, rec.gps_lon,
                rec.water_ph, rec.water_turbidity, rec.water_temp,
                rec.ph_voltage, rec.turb_voltage,
                (unsigned)rec.current_mode,
                (unsigned)rec.battery_percent,
                rec.heading_angle,
                rec.gps_hdop, (unsigned)rec.gps_sats, rec.gps_speed_kmh,
                (unsigned)rec.waypoint_index, (unsigned)rec.waypoint_count,
                rec.target_lat, rec.target_lon,
                rec.nav_bearing_deg, rec.cross_track_m,
                rec.dist_to_target_m, rec.heading_error_deg,
                (unsigned)rec.water_sample_phase,
                (unsigned long)rec.water_sample_ms,
                (int)rec.base_pwm, (int)rec.left_pwm, (int)rec.right_pwm,
                (int)rec.compass_raw_x, (int)rec.compass_raw_y,
                (unsigned)rec.compass_cal_nvs,
                rec.gps_raw_lat, rec.gps_raw_lon, rec.gps_course_deg,
                (unsigned long)rec.gps_location_age_ms,
                (unsigned)rec.gps_fix_valid,
                (unsigned)rec.compass_fresh,
                (unsigned)rec.compass_fail_count,
                rec.direct_bearing_deg,
                (int)rec.requested_left_pwm, (int)rec.requested_right_pwm,
                (unsigned)rec.turn_in_place,
                (unsigned)rec.drive_source,
                (unsigned)rec.drive_stop_reason,
                (unsigned long)rec.telemetry_tx_seq,
                (unsigned long)rec.telemetry_tx_ms,
                telemetryRadioStatusName(rec.telemetry_radio_status),
                (unsigned)rec.telemetry_tx_attempts,
                rec.telemetry_tx_lat, rec.telemetry_tx_lon,
                rec.telemetry_tx_ph,
                (unsigned)rec.radio_tx_paused,
                rec.battery_voltage);
        } else {
            written = snprintf(row, sizeof(row),
                "%lu,,,%lu,%.6f,%.6f,%.2f,%.1f,%.2f,%.3f,%.3f,%u,%u,%.1f,"
                "%.2f,%u,%.2f,%u,%u,%.6f,%.6f,"
                "%.1f,%.2f,%.2f,%.1f,%u,%lu,%d,%d,%d,%d,%d,%u,"
                "%.6f,%.6f,%.1f,%lu,%u,%u,%u,%.1f,%d,%d,%u,%u,%u,"
                "%lu,%lu,%s,%u,%.6f,%.6f,%.2f,%u,%.2f\n",
                (unsigned long)rec.boot_ms,
                (unsigned long)rec.sequence_number,
                rec.gps_lat, rec.gps_lon,
                rec.water_ph, rec.water_turbidity, rec.water_temp,
                rec.ph_voltage, rec.turb_voltage,
                (unsigned)rec.current_mode,
                (unsigned)rec.battery_percent,
                rec.heading_angle,
                rec.gps_hdop, (unsigned)rec.gps_sats, rec.gps_speed_kmh,
                (unsigned)rec.waypoint_index, (unsigned)rec.waypoint_count,
                rec.target_lat, rec.target_lon,
                rec.nav_bearing_deg, rec.cross_track_m,
                rec.dist_to_target_m, rec.heading_error_deg,
                (unsigned)rec.water_sample_phase,
                (unsigned long)rec.water_sample_ms,
                (int)rec.base_pwm, (int)rec.left_pwm, (int)rec.right_pwm,
                (int)rec.compass_raw_x, (int)rec.compass_raw_y,
                (unsigned)rec.compass_cal_nvs,
                rec.gps_raw_lat, rec.gps_raw_lon, rec.gps_course_deg,
                (unsigned long)rec.gps_location_age_ms,
                (unsigned)rec.gps_fix_valid,
                (unsigned)rec.compass_fresh,
                (unsigned)rec.compass_fail_count,
                rec.direct_bearing_deg,
                (int)rec.requested_left_pwm, (int)rec.requested_right_pwm,
                (unsigned)rec.turn_in_place,
                (unsigned)rec.drive_source,
                (unsigned)rec.drive_stop_reason,
                (unsigned long)rec.telemetry_tx_seq,
                (unsigned long)rec.telemetry_tx_ms,
                telemetryRadioStatusName(rec.telemetry_radio_status),
                (unsigned)rec.telemetry_tx_attempts,
                rec.telemetry_tx_lat, rec.telemetry_tx_lon,
                rec.telemetry_tx_ph,
                (unsigned)rec.radio_tx_paused,
                rec.battery_voltage);
        }
    const bool ok = written > 0 && (size_t)written < sizeof(row) &&
        _mainBuffer.append(row, (size_t)written, millis());
    if (ok) {
        ++_mainAccepted;
        _haveTimestamp = localValid;
        if (localValid) {
            _haveTimestamp = true;
            _timeYear = local.year; _timeMonth = local.month; _timeDay = local.day;
            _timeHour = local.hour; _timeMinute = local.minute; _timeSecond = local.second;
        }
    } else {
        ++_writeFailCount;
        _ready = false;
    }
    xSemaphoreGive(_spiMutex);
    return ok;
}

static_assert(SD_LOG_WRITE_BYTES >= 768, "one full main row must fit a write");
static_assert(SD_LOG_MAIN_BUFFER_BYTES >= SD_LOG_WRITE_BYTES, "main buffer too small");
static_assert(SD_LOG_PH_BUFFER_BYTES >= SD_LOG_WRITE_BYTES, "pH buffer too small");

bool DataLogger::canAcceptLogPair() const {
    return _ready && _mainBuffer.freeBytes() >= MAIN_ROW_BYTES &&
          (_phStopped || !g_phDiagFile || _phBuffer.freeBytes() >= PH_ROW_BYTES);
}

template <size_t Capacity>
static bool writeLogChunk(FsFile& file, FixedLogBuffer<Capacity>& buffer,
                          size_t limit, uint32_t& rows, size_t& consumed) {
    consumed = buffer.prefix(limit < SD_LOG_WRITE_BYTES ? limit : SD_LOG_WRITE_BYTES);
    if (!consumed) return false;
    if (file.write((const uint8_t*)buffer.data(), consumed) != consumed) return false;
    for (size_t i = 0; i < consumed; ++i) if (buffer.data()[i] == '\n') ++rows;
    return buffer.consume(consumed);
}

bool DataLogger::service() {
    if (!_ready || !_spiMutex) return false;
    const uint32_t started = millis();
    if (xSemaphoreTake(_spiMutex, pdMS_TO_TICKS(50)) != pdTRUE) return false;
    bool ok = true;
    size_t consumed = 0;
    if (_syncPhase == SyncPhase::NONE &&
        (uint32_t)(started - _lastFlush) >= SD_LOG_SYNC_INTERVAL_MS) {
        // Freeze this commit's byte boundary; later rows remain for the next commit.
        _commitMain = _mainBuffer.size(); _commitPh = _phBuffer.size();
        _commitStart = started; _syncPhase = SyncPhase::MAIN_DATA;
        _commitSyncOk = !_phStopped;
        _commitHasTimestamp = _haveTimestamp;
        _commitYear = _timeYear; _commitMonth = _timeMonth; _commitDay = _timeDay;
        _commitHour = _timeHour; _commitMinute = _timeMinute; _commitSecond = _timeSecond;
    }
    switch (_syncPhase) {
        case SyncPhase::MAIN_DATA:
            if (_commitMain) {
                ok = writeLogChunk(g_logFile, _mainBuffer, _commitMain, _mainWritten, consumed);
                if (ok) _commitMain -= consumed;
            }
            if (!_commitMain) _syncPhase = SyncPhase::MAIN_SYNC;
            break;
        case SyncPhase::MAIN_SYNC:
            if (_commitHasTimestamp) {
                const SdDateTime local{_commitYear, _commitMonth, _commitDay, _commitHour, _commitMinute, _commitSecond};
                ok = setFileTimestamp(local, !_fatCreateTimeSet);
                if (ok) _fatCreateTimeSet = true;
            }
            // SdFat timestamp() already syncs the file and directory cache.
            if (!_commitHasTimestamp) ok = g_logFile.sync();
            if (ok) { _mainSynced = _mainWritten; _syncPhase = SyncPhase::PH_DATA; }
            break;
        case SyncPhase::PH_DATA:
            if (_phStopped) _commitPh = 0;
            if (_commitPh) {
                ok = writeLogChunk(g_phDiagFile, _phBuffer, _commitPh, _phWritten, consumed);
                if (ok) _commitPh -= consumed;
                else {
                    // Retain the failed bytes for diagnosis; never retry a partial row.
                    // Future pH rows are rejected/count failures without blocking main.
                    ++_phDiagnosticWriteFailCount;
                    _phStopped = true; _commitSyncOk = false; _commitPh = 0;
                    ok = true;
                }
            }
            if (!_commitPh) _syncPhase = SyncPhase::PH_SYNC;
            break;
        case SyncPhase::PH_SYNC:
            if (!_phStopped && g_phDiagFile) {
                if (g_phDiagFile.sync()) _phSynced = _phWritten;
                else { ++_phDiagnosticWriteFailCount; _phStopped = true; _commitSyncOk = false; }
            }
            _syncPhase = SyncPhase::TX_SYNC;
            break;
        case SyncPhase::TX_SYNC:
            if (g_txDiagFile && !g_txDiagFile.sync()) {
                ++_txDiagnosticWriteFailCount; _commitSyncOk = false;
            }
            {
                const uint32_t gap = (uint32_t)(millis() - _lastSyncCompleted);
                if (gap > _maxSyncGapMs) _maxSyncGapMs = gap;
                if (gap > SD_LOG_DURABILITY_MS) ++_syncMisses;
                if (_commitSyncOk) _lastSyncCompleted = millis();
                _lastFlush = _commitStart; _syncPhase = SyncPhase::NONE;
            }
            break;
        case SyncPhase::NONE: {
            const bool mainDue = _mainBuffer.size() >= SD_LOG_MIN_BATCH_BYTES ||
                (_mainBuffer.size() && _mainBuffer.age(started) >= SD_LOG_MAX_BATCH_AGE_MS);
            const bool phDue = !_phStopped && (_phBuffer.size() >= SD_LOG_MIN_BATCH_BYTES ||
                (_phBuffer.size() && _phBuffer.age(started) >= SD_LOG_MAX_BATCH_AGE_MS));
            if (phDue && (_nextPh || !mainDue)) {
                ok = writeLogChunk(g_phDiagFile, _phBuffer, _phBuffer.size(), _phWritten, consumed);
                if (!ok) { ++_phDiagnosticWriteFailCount; _phStopped = true; ok = true; }
                _nextPh = false;
            } else if (mainDue) {
                ok = writeLogChunk(g_logFile, _mainBuffer, _mainBuffer.size(), _mainWritten, consumed);
                _nextPh = true;
            }
            break;
        }
    }
    if (!ok) { ++_writeFailCount; _ready = false; }
    const uint32_t elapsed = (uint32_t)(millis() - started);
    if (elapsed > _maxServiceMs) _maxServiceMs = elapsed;
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
