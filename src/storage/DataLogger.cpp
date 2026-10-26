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

bool DataLogger::openAndWriteHeader() {
    bool isNew = !g_sd.exists(_filename);
    if (!isNew) return false; // Never append a fresh run to an existing/uncertain session.
    g_logFile = g_sd.open(_filename, O_RDWR | O_CREAT | O_EXCL);
    if (g_logFile && isNew) {
        if (g_logFile.print(CSV_HEADER) != strlen(CSV_HEADER) || !g_logFile.sync()) return false;
    }
    return (bool)g_logFile;
}

bool DataLogger::begin(float calV401, float calV686, float calV918,
                       bool calValid, const char* calSource) {
    _calV401 = calV401; _calV686 = calV686; _calV918 = calV918; _calValid = calValid;
    snprintf(_calSource, sizeof(_calSource), "%s", calSource ? calSource : "UNKNOWN");
    _spiMutex = xSemaphoreCreateMutex();
    if (!_spiMutex) return false;
    _configured = true;

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
            _spiHz = kHzList[i];
            mounted = true;
        }
    }

    if (!mounted) {
        requestRecovery("MOUNT", true);
        return false;
    }

    const uint32_t boot = nextBootIndex();
    _runNumber = boot ? boot : (uint32_t)millis(); _haveRunNumber = true;
    if (boot == 0) {
        snprintf(_filename, sizeof(_filename), "/m_t%lu.csv",
                 (unsigned long)millis());
    } else {
        snprintf(_filename, sizeof(_filename), "/m_%05lu.csv",
                 (unsigned long)boot);
    }
    snprintf(_baseFilename, sizeof(_baseFilename), "%s", _filename);
    setSidecarNames();
    if (g_sd.exists(_phFilename) || g_sd.exists(_txFilename) || g_sd.exists(_metadataFilename)) {
        requestRecovery("NAME_COLLISION", true); return false;
    }
    if (!openAndWriteHeader()) {
        requestRecovery("MAIN_HEADER", true);
        return false;
    }
    // The first successfully created main file anchors this boot's logical run.
    snprintf(_baseFilename, sizeof(_baseFilename), "%s", _filename);
    _runEstablished = true;
    setSidecarNames();
    g_phDiagFile = g_sd.open(_phFilename, O_RDWR | O_CREAT | O_EXCL);
    if (!g_phDiagFile || g_phDiagFile.print(PH_DIAG_HEADER) != strlen(PH_DIAG_HEADER) || !g_phDiagFile.sync()) {
        _phDiagnosticWriteFailCount++;
        g_phDiagFile.close();
        Serial.println(F("[USV] pH diagnostic sidecar unavailable"));
    }
    g_txDiagFile = g_sd.open(_txFilename, O_RDWR | O_CREAT | O_EXCL);
    if (!g_txDiagFile || g_txDiagFile.print(TX_DIAG_HEADER) != strlen(TX_DIAG_HEADER) || !g_txDiagFile.sync()) {
        _txDiagnosticWriteFailCount++;
        g_txDiagFile.close();
        Serial.println(F("[USV] TX diagnostic sidecar unavailable"));
    }
    _phStopped = !g_phDiagFile; _txStopped = !g_txDiagFile;
    _auxSince = (uint32_t)millis();
    writeMetadata(calV401, calV686, calV918, calValid, calSource);
    _ready = (bool)g_logFile;
    _lastFlush = millis();
    _lastSyncCompleted = millis();
    _writeFailCount = 0;
    _fatCreateTimeSet = false;
    return _ready;
}

void DataLogger::writeMetadata(float calV401, float calV686, float calV918,
                               bool calValid, const char* calSource) {
    FsFile metadata = g_sd.open(_metadataFilename, O_RDWR | O_CREAT | O_EXCL);
    if (!metadata) {
        ++_metadataWriteFailCount;
        Serial.println(F("[SD] metadata open failed; main logging continues")); return;
    }
    char text[2048];
    const int length=snprintf(text,sizeof(text),
        "schema=%s\nrun_id=%s\nfirmware_build_id=%s\n"
        "build_id_inputs=dirty bytes under include/ and src/, platformio.ini, tools/build_identity.py; SHA-256 truncated to 16 hex chars\n"
        "ph_cal_source_at_boot=%s\nph_cal_valid_at_boot=%u\n"
        "ph_cal_v401=%.6f\nph_cal_v686=%.6f\nph_cal_v918=%.6f\n"
        "parent_run=%s\nrecovery_part=%u\nph_file=%s\ntx_file=%s\n"
        "fault_stage=%s\nfault_ms=%lu\nfault_sd_seq=%lu\ncard_error=%u\ncard_error_data=%lu\n"
        "per_sample_calibration_provenance=UNKNOWN (no conversion semantics changed)\n"
        "utc_source=GPS date+time; available only under existing utcNow validity/freshness policy; boot_ms always retained\n"
        "ph_sidecar_rows=SD navigation rows; repeated ph_sample_id means repeated observation, not a new ADC conversion\n"
        "tx_sidecar_rows=one per sendTelemetry call; sequence is packet sequence; MAC ACK is not application ingestion\n"
        "legacy_csv=unchanged column order and meaning\n"
        "recovery_rule=unconfirmed records replayed with original identities; old file tail is uncertain; reconcile by seq\n",
        USV_DIAGNOSTIC_SCHEMA,_filename,USV_BUILD_ID,
        calSource?calSource:"UNKNOWN",calValid?1U:0U,(double)calV401,(double)calV686,(double)calV918,
        _baseFilename,(unsigned)_recoveryPart,_phFilename,_txFilename,_faultStage,(unsigned long)_faultAt,
        (unsigned long)_faultSequence,(unsigned)_cardError,(unsigned long)_cardErrorData);
    const bool ok=length>0 && (size_t)length<sizeof(text) &&
        metadata.print(text)==(size_t)length && metadata.sync();
    if(!ok) {
        ++_metadataWriteFailCount;
        Serial.println(F("[SD] metadata write/sync failed; fault context retained in RAM"));
    }
}

bool DataLogger::logPhDiagnostic(
    uint32_t bootMs, uint32_t sdSequence,
    const DiagnosticFormatting::PhLogRecord& rec) {
    if (!_configured || !_spiMutex ||
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
    if (!_configured || !_spiMutex || xSemaphoreTake(_spiMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        ++_txDiagnosticWriteFailCount; return false;
    }
    char row[320];
    bool ok = DiagnosticFormatting::formatTxCsvRow(rec, row, sizeof(row));
    const size_t length = ok ? strlen(row) : 0;
    const size_t before = _txBuffer.size();
    ok = ok && _txBuffer.append(row, length, millis());
    if (ok && _ready && !_txStopped && !_recoveryRequested && g_txDiagFile && _txOffset == before) {
        ok = g_txDiagFile.print(row) == length;
        if (ok) _txOffset += length;
        else { _txStopped = true; requestRecovery("TX_WRITE", false); }
    } else if (ok && _ready && (!g_txDiagFile || _txStopped)) {
        // Buffered for a new session; never join the next packet to a partial row.
        ok = false;
    }
    if (!ok) ++_txDiagnosticWriteFailCount;
    xSemaphoreGive(_spiMutex); return ok;
}

bool DataLogger::logRecord(const SdLogRecord& rec) {
    _appendInvalid = false;
    if (!_configured || !_spiMutex) return false;
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
        ++_mainAccepted; _lastAcceptedSequence = rec.sequence_number;
        _haveTimestamp = localValid;
        if (localValid) {
            _haveTimestamp = true;
            _timeYear = local.year; _timeMonth = local.month; _timeDay = local.day;
            _timeHour = local.hour; _timeMinute = local.minute; _timeSecond = local.second;
        }
    } else if (written <= 0 || (size_t)written >= sizeof(row)) {
        // A malformed record is rejected explicitly; it must not poison the disk state.
        ++_writeFailCount; _appendInvalid = true;
    }
    xSemaphoreGive(_spiMutex);
    return ok;
}

static_assert(SD_LOG_WRITE_BYTES >= 768, "one full main row must fit a write");
static_assert(SD_LOG_MAIN_BUFFER_BYTES >= SD_LOG_WRITE_BYTES, "main buffer too small");
static_assert(SD_LOG_PH_BUFFER_BYTES >= SD_LOG_WRITE_BYTES, "pH buffer too small");

bool DataLogger::canAcceptLogPair() const {
    return _configured && _mainBuffer.freeBytes() >= MAIN_ROW_BYTES &&
          (_phStopped || _phBuffer.freeBytes() >= PH_ROW_BYTES);
}

template <size_t Capacity>
static bool writeLogChunk(FsFile& file, FixedLogBuffer<Capacity>& buffer,
                          size_t limit, uint32_t& rows, size_t& offset, size_t& consumed) {
    if (offset > buffer.size()) return false;
    const size_t available = buffer.size() - offset;
    consumed = available < limit ? available : limit;
    if (consumed > SD_LOG_WRITE_BYTES) consumed = SD_LOG_WRITE_BYTES;
    const char* data = buffer.data() + offset;
    while (consumed && data[consumed - 1] != '\n') --consumed;
    if (!consumed) return false;
    if (file.write((const uint8_t*)data, consumed) != consumed) return false;
    for (size_t i=0; i<consumed; ++i) if (data[i]=='\n') ++rows;
    offset += consumed;
    // Retain the entire unconfirmed prefix until sync succeeds.
    return true;
}

bool DataLogger::service() {
    if (!_configured || !_spiMutex) return false;
    const uint32_t started = millis();
    if (xSemaphoreTake(_spiMutex, pdMS_TO_TICKS(50)) != pdTRUE) return true;
    if (_recoveryRequested || _recoveryPhase != RecoveryPhase::NONE) {
        const bool recovered = recoveryStep(started);
        const uint32_t elapsed=(uint32_t)((uint32_t)millis()-started);
        if(elapsed>_maxServiceMs) _maxServiceMs=elapsed;
        xSemaphoreGive(_spiMutex); return recovered;
    }
    if (!_ready) { xSemaphoreGive(_spiMutex); return false; }
    // Auxiliary retries share this IO owner and yield between phases. Never
    // remount or pause the main stream just because one sidecar stays unavailable.
    if (_syncPhase == SyncPhase::NONE && auxiliaryStep(started)) {
        const uint32_t elapsed=(uint32_t)((uint32_t)millis()-started);
        if(elapsed>_maxServiceMs) _maxServiceMs=elapsed;
        xSemaphoreGive(_spiMutex); return true;
    }
    bool ok = true;
    size_t consumed = 0;
    if (_syncPhase == SyncPhase::NONE &&
        (uint32_t)(started - _lastFlush) >= SD_LOG_SYNC_INTERVAL_MS) {
        // Freeze this commit's byte boundary; later rows remain for the next commit.
        _commitMain = _mainBuffer.size() - _mainOffset; _commitPh = _phBuffer.size() - _phOffset;
        _commitStart = started; _syncPhase = SyncPhase::MAIN_DATA;
        _commitSyncOk = !_phStopped;
        _commitHasTimestamp = _haveTimestamp;
        _commitYear = _timeYear; _commitMonth = _timeMonth; _commitDay = _timeDay;
        _commitHour = _timeHour; _commitMinute = _timeMinute; _commitSecond = _timeSecond;
    }
    switch (_syncPhase) {
        case SyncPhase::MAIN_DATA:
            if (_commitMain) {
                ok = writeLogChunk(g_logFile, _mainBuffer, _commitMain, _mainWritten, _mainOffset, consumed);
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
            if (ok) {
                _mainSynced = _mainWritten;
                if (_mainOffset) _mainBuffer.consume(_mainOffset);
                _mainOffset = 0; _syncPhase = SyncPhase::PH_DATA;
            }
            break;
        case SyncPhase::PH_DATA:
            if (_phStopped) _commitPh = 0;
            if (_commitPh) {
                ok = writeLogChunk(g_phDiagFile, _phBuffer, _commitPh, _phWritten, _phOffset, consumed);
                if (ok) _commitPh -= consumed;
                else {
                    // Retain the failed bytes for diagnosis; never retry a partial row.
                    // Future pH rows are rejected/count failures without blocking main.
                    ++_phDiagnosticWriteFailCount;
                    _phStopped = true; _commitSyncOk = false; _commitPh = 0;
                    requestRecovery("PH_WRITE", false); ok = true;
                }
            }
            if (!_commitPh) _syncPhase = SyncPhase::PH_SYNC;
            break;
        case SyncPhase::PH_SYNC:
            if (!_phStopped && g_phDiagFile) {
                if (g_phDiagFile.sync()) {
                    _phSynced = _phWritten;
                    if (_phOffset) _phBuffer.consume(_phOffset);
                    _phOffset = 0;
                } else { ++_phDiagnosticWriteFailCount; _phStopped = true; _commitSyncOk = false;
                    requestRecovery("PH_SYNC", false); }
            }
            _syncPhase = SyncPhase::TX_SYNC;
            break;
        case SyncPhase::TX_SYNC:
            if (g_txDiagFile && !_txStopped) {
                if (!_txStopped && _txOffset < _txBuffer.size()) {
                    uint32_t ignoredRows = 0;
                    ok = writeLogChunk(g_txDiagFile, _txBuffer, _txBuffer.size()-_txOffset,
                                       ignoredRows, _txOffset, consumed);
                    if (!ok) { ++_txDiagnosticWriteFailCount; requestRecovery("TX_REPLAY", false); }
                    break;
                }
                if (!g_txDiagFile.sync()) {
                    ++_txDiagnosticWriteFailCount; _commitSyncOk = false;
                    requestRecovery("TX_SYNC", false);
                } else { if (_txOffset) _txBuffer.consume(_txOffset); _txOffset = 0; }
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
            const size_t mainUnwritten = _mainBuffer.size() - _mainOffset;
            const size_t phUnwritten = _phBuffer.size() - _phOffset;
            const bool mainDue = mainUnwritten && (mainUnwritten >= SD_LOG_MIN_BATCH_BYTES ||
                _mainBuffer.age(started) >= SD_LOG_MAX_BATCH_AGE_MS);
            const bool phDue = !_phStopped && phUnwritten && (phUnwritten >= SD_LOG_MIN_BATCH_BYTES ||
                _phBuffer.age(started) >= SD_LOG_MAX_BATCH_AGE_MS);
            if (phDue && (_nextPh || !mainDue)) {
                ok = writeLogChunk(g_phDiagFile, _phBuffer, _phBuffer.size() - _phOffset, _phWritten, _phOffset, consumed);
                if (!ok) { ++_phDiagnosticWriteFailCount; _phStopped = true;
                    requestRecovery("PH_WRITE", false); ok = true; }
                _nextPh = false;
            } else if (mainDue) {
                ok = writeLogChunk(g_logFile, _mainBuffer, _mainBuffer.size() - _mainOffset, _mainWritten, _mainOffset, consumed);
                _nextPh = true;
            }
            break;
        }
    }
    if (!ok) { requestRecovery(_syncPhase == SyncPhase::MAIN_SYNC ? "MAIN_SYNC" : "MAIN_WRITE", true); _ready = false; }
    const uint32_t elapsed = (uint32_t)(millis() - started);
    if (elapsed > _maxServiceMs) _maxServiceMs = elapsed;
    xSemaphoreGive(_spiMutex);
    return ok;
}

bool DataLogger::checkIntegrity() {
    if (!_spiMutex) return false;
    if (xSemaphoreTake(_spiMutex, pdMS_TO_TICKS(50)) != pdTRUE) return false;
    // SdFat errorCode() is a retained LAST error, not a new card-health probe.
    // Actual checked write/sync results drive recovery and preserve the evidence.
    const bool ok = _ready && (bool)g_logFile;
    if (_ready && !g_logFile) requestRecovery("FILE_CLOSED", true);
    xSemaphoreGive(_spiMutex); return ok;
}

void DataLogger::setSidecarNames() {
    snprintf(_phFilename, sizeof(_phFilename), "%s.ph.csv", _filename);
    snprintf(_txFilename, sizeof(_txFilename), "%s.tx.csv", _filename);
    snprintf(_metadataFilename, sizeof(_metadataFilename), "%s.meta.txt", _filename);
}

void DataLogger::requestRecovery(const char* stage, bool mainFailure) {
    if (mainFailure) ++_writeFailCount;
    if (!_recoveryRequested && _recoveryPhase == RecoveryPhase::NONE) {
        _faultStage = stage; _faultAt = (uint32_t)millis(); _faultSequence = _lastAcceptedSequence;
        _cardError = g_sd.card() ? g_sd.card()->errorCode() : 0;
        _cardErrorData = g_sd.card() ? g_sd.card()->errorData() : 0;
        Serial.printf("[SD] fault stage=%s ms=%lu seq=%lu code=%u data=%lu; buffering for recovery\n",
            _faultStage, (unsigned long)_faultAt, (unsigned long)_faultSequence,
            (unsigned)_cardError, (unsigned long)_cardErrorData);
    }
    _recoveryRequested = true;
}

bool DataLogger::recoveryStep(uint32_t now) {
    if (_recoveryPhase == RecoveryPhase::NONE) {
        _ready = false; _retrySince = now; _recoveryPhase = RecoveryPhase::WAIT;
        _auxPhase = AuxPhase::NONE;
        _mainOffset = _phOffset = _txOffset = 0;
        _mainWritten = _mainSynced; _phWritten = _phSynced;
        _syncPhase = SyncPhase::NONE; _commitMain = _commitPh = 0;
    }
    bool ok = true;
    // One phase per IO iteration. Vendor operations still need real-card latency validation.
    switch (_recoveryPhase) {
        case RecoveryPhase::WAIT:
            if ((uint32_t)(now-_retrySince) >= _retryDelay) {
                ++_recoveryAttempts; _recoveryPhase = RecoveryPhase::CLOSE_MAIN;
            }
            break;
        case RecoveryPhase::CLOSE_MAIN: g_logFile.close(); _recoveryPhase=RecoveryPhase::CLOSE_PH; break;
        case RecoveryPhase::CLOSE_PH: g_phDiagFile.close(); _recoveryPhase=RecoveryPhase::CLOSE_TX; break;
        case RecoveryPhase::CLOSE_TX: g_txDiagFile.close(); _recoveryPhase=RecoveryPhase::UNMOUNT; break;
        case RecoveryPhase::UNMOUNT: g_sd.end(); _recoveryPhase=RecoveryPhase::CARD; break;
        case RecoveryPhase::CARD:
            ok=g_sd.cardBegin(SdSpiConfig(SD_CS_PIN, SHARED_SPI, _spiHz, &g_spi));
            if(ok) _recoveryPhase=RecoveryPhase::VOLUME; break;
        case RecoveryPhase::VOLUME:
            ok=g_sd.volumeBegin(); if(ok) _recoveryPhase=RecoveryPhase::NAME; break;
        case RecoveryPhase::NAME: {
            if (!_haveRunNumber) {
                const uint32_t boot=nextBootIndex(); _runNumber=boot?boot:now; _haveRunNumber=true;
                snprintf(_baseFilename,sizeof(_baseFilename),"/m_%05lu.csv",(unsigned long)_runNumber);
            }
            ++_recoveryPart;
            if (!_recoveryPart) ++_recoveryPart;
            const int length=snprintf(_filename,sizeof(_filename),"/m_%05lu_r%03lu.csv",
                (unsigned long)_runNumber,(unsigned long)_recoveryPart);
            ok=length>0 && (size_t)length<sizeof(_filename);
            if(ok) {
                setSidecarNames();
                if(!g_sd.exists(_filename) && !g_sd.exists(_phFilename) &&
                   !g_sd.exists(_txFilename) && !g_sd.exists(_metadataFilename))
                    _recoveryPhase=RecoveryPhase::OPEN_MAIN;
            }
            break;
        }
        case RecoveryPhase::OPEN_MAIN:
            g_logFile=g_sd.open(_filename,O_RDWR|O_CREAT|O_EXCL);
            ok=(bool)g_logFile; if(ok)_recoveryPhase=RecoveryPhase::HEADER_MAIN; break;
        case RecoveryPhase::HEADER_MAIN:
            ok=g_logFile.print(CSV_HEADER)==strlen(CSV_HEADER); if(ok)_recoveryPhase=RecoveryPhase::SYNC_MAIN; break;
        case RecoveryPhase::SYNC_MAIN:
            ok=g_logFile.sync();
            if(ok) {
                if (!_runEstablished) {
                    snprintf(_baseFilename,sizeof(_baseFilename),"%s",_filename);
                    _runEstablished=true;
                }
                _recoveryPhase=RecoveryPhase::OPEN_PH;
            }
            break;
        case RecoveryPhase::OPEN_PH:
            _phStopped=true;
            g_phDiagFile=g_sd.open(_phFilename,O_RDWR|O_CREAT|O_EXCL);
            if(g_phDiagFile) _recoveryPhase=RecoveryPhase::HEADER_PH;
            else { ++_phDiagnosticWriteFailCount; _recoveryPhase=RecoveryPhase::OPEN_TX; }
            break;
        case RecoveryPhase::HEADER_PH:
            if(g_phDiagFile.print(PH_DIAG_HEADER)==strlen(PH_DIAG_HEADER)) _recoveryPhase=RecoveryPhase::SYNC_PH;
            else { ++_phDiagnosticWriteFailCount; g_phDiagFile.close(); _recoveryPhase=RecoveryPhase::OPEN_TX; }
            break;
        case RecoveryPhase::SYNC_PH:
            if(g_phDiagFile.sync()) _phStopped=false;
            else { ++_phDiagnosticWriteFailCount; g_phDiagFile.close(); }
            _recoveryPhase=RecoveryPhase::OPEN_TX; break;
        case RecoveryPhase::OPEN_TX:
            _txStopped=true;
            g_txDiagFile=g_sd.open(_txFilename,O_RDWR|O_CREAT|O_EXCL);
            if(g_txDiagFile) _recoveryPhase=RecoveryPhase::HEADER_TX;
            else { ++_txDiagnosticWriteFailCount; _recoveryPhase=RecoveryPhase::METADATA; }
            break;
        case RecoveryPhase::HEADER_TX:
            if(g_txDiagFile.print(TX_DIAG_HEADER)==strlen(TX_DIAG_HEADER)) _recoveryPhase=RecoveryPhase::SYNC_TX;
            else { ++_txDiagnosticWriteFailCount; g_txDiagFile.close(); _recoveryPhase=RecoveryPhase::METADATA; }
            break;
        case RecoveryPhase::SYNC_TX:
            if(g_txDiagFile.sync()) _txStopped=false;
            else { ++_txDiagnosticWriteFailCount; g_txDiagFile.close(); }
            _recoveryPhase=RecoveryPhase::METADATA; break;
        case RecoveryPhase::METADATA:
            writeMetadata(_calV401,_calV686,_calV918,_calValid,_calSource);
            _recoveryPhase=RecoveryPhase::FINISH; break;
        case RecoveryPhase::FINISH:
            _ready=true; _recoveryRequested=false;
            _recoveryPhase=RecoveryPhase::NONE; _lastFlush=now; _fatCreateTimeSet=false;
            _retryDelay=SD_LOG_RECOVERY_RETRY_MS; ++_recoverySuccesses;
            _auxSince=now; _auxDelay=SD_LOG_RECOVERY_RETRY_MS; _auxYield=false;
            Serial.printf("[SD] recovered file=%s original=%s successes=%lu\n",
                _filename,_baseFilename,(unsigned long)_recoverySuccesses); break;
        case RecoveryPhase::NONE: break;
    }
    if (!ok) {
        _ready=false; _retrySince=(uint32_t)millis(); _recoveryPhase=RecoveryPhase::WAIT;
        const uint32_t next=_retryDelay*2;
        _retryDelay=next<SD_LOG_RECOVERY_MAX_RETRY_MS?next:SD_LOG_RECOVERY_MAX_RETRY_MS;
    }
    return ok;
}


bool DataLogger::auxiliaryStep(uint32_t now) {
    if (_auxYield) { _auxYield=false; return false; }
    if (_auxPhase == AuxPhase::NONE) {
        if ((!_phStopped && !_txStopped) || (uint32_t)(now-_auxSince)<_auxDelay) return false;
        _auxTx = _txStopped && (!_phStopped || _auxPreferTx);
        _auxPreferTx = !_auxTx;
        ++_auxPart; if (!_auxPart) ++_auxPart;
        const int length=snprintf(_auxFilename,sizeof(_auxFilename),"%s.a%lu.%s.csv",
            _filename,(unsigned long)_auxPart,_auxTx?"tx":"ph");
        const int metaLength=snprintf(_auxMetadataFilename,sizeof(_auxMetadataFilename),"%s.a%lu.meta.txt",
            _filename,(unsigned long)_auxPart);
        if(length<=0 || (size_t)length>=sizeof(_auxFilename) || metaLength<=0 || (size_t)metaLength>=sizeof(_auxMetadataFilename)) {
            _auxSince=now; _auxDelay=SD_LOG_RECOVERY_MAX_RETRY_MS; return false;
        }
        _auxPhase=AuxPhase::CLOSE;
    }
    FsFile& file=_auxTx?g_txDiagFile:g_phDiagFile;
    bool ok=true;
    switch(_auxPhase) {
        case AuxPhase::CLOSE:
            file.close(); _auxPhase=AuxPhase::OPEN; break;
        case AuxPhase::OPEN:
            file=g_sd.open(_auxFilename,O_RDWR|O_CREAT|O_EXCL);
            ok=(bool)file; if(ok)_auxPhase=AuxPhase::HEADER; break;
        case AuxPhase::HEADER: {
            const char* header=_auxTx?TX_DIAG_HEADER:PH_DIAG_HEADER;
            ok=file.print(header)==strlen(header); if(ok)_auxPhase=AuxPhase::SYNC; break;
        }
        case AuxPhase::SYNC:
            ok=file.sync();
            if(ok) {
                if(_auxTx) {
                    snprintf(_txFilename,sizeof(_txFilename),"%s",_auxFilename);
                    _txOffset=0; _txStopped=false;
                } else {
                    snprintf(_phFilename,sizeof(_phFilename),"%s",_auxFilename);
                    _phOffset=0; _phWritten=_phSynced; _phStopped=false;
                }
                _auxPhase=AuxPhase::METADATA;
            }
            break;
        case AuxPhase::METADATA: {
            // Fresh provenance file links this repaired sidecar to the same run.
            char saved[sizeof(_metadataFilename)];
            snprintf(saved,sizeof(saved),"%s",_metadataFilename);
            snprintf(_metadataFilename,sizeof(_metadataFilename),"%s",_auxMetadataFilename);
            writeMetadata(_calV401,_calV686,_calV918,_calValid,_calSource);
            snprintf(_metadataFilename,sizeof(_metadataFilename),"%s",saved);
            Serial.printf("[SD] auxiliary restored file=%s parent=%s\n",_auxFilename,_baseFilename);
            _auxPhase=AuxPhase::NONE; _auxSince=(uint32_t)millis(); _auxDelay=SD_LOG_RECOVERY_RETRY_MS;
            break;
        }
        case AuxPhase::NONE: break;
    }
    if(!ok) {
        if(_auxTx) ++_txDiagnosticWriteFailCount; else ++_phDiagnosticWriteFailCount;
        // Leave the stream stopped, retaining its unconfirmed RAM journal.
        // CLOSE occurs in its own phase on the next bounded attempt.
        _auxPhase=AuxPhase::NONE; _auxSince=(uint32_t)millis();
        const uint32_t next=_auxDelay*2;
        _auxDelay=next<SD_LOG_RECOVERY_MAX_RETRY_MS?next:SD_LOG_RECOVERY_MAX_RETRY_MS;
    }
    _auxYield=true;
    return true;
}
