#include "DataLogger.h"
#include "config.h"
#include <Preferences.h>
#include <SPI.h>
#include <SdFat.h>
#include <stdio.h>
#include <string.h>

static SdFat   g_sd;
static FsFile  g_logFile;
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
    g_spi.endTransaction();
    _ready = (bool)g_logFile;
    _lastFlush = millis();
    _writeFailCount = 0;
    _fatCreateTimeSet = false;
    return _ready;
}

bool DataLogger::logRecord(const SdLogRecord& rec) {
    if (!_ready) return false;
    if (xSemaphoreTake(_spiMutex, pdMS_TO_TICKS(50)) != pdTRUE) return false;

    bool ok = false;
    if (g_logFile) {
        SdDateTime local = {};
        const bool localValid = utcToLocal(rec, local);
        const char tzSign = GPS_TIMEZONE_OFFSET_H < 0 ? '-' : '+';
        const unsigned tzHours = (unsigned)(GPS_TIMEZONE_OFFSET_H < 0
            ? -GPS_TIMEZONE_OFFSET_H : GPS_TIMEZONE_OFFSET_H);
        int written;
        if (localValid) {
            written = g_logFile.printf(
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
            written = g_logFile.printf(
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
        ok = (written > 0);

        if (ok && millis() - _lastFlush >= 1000) {
            g_logFile.flush();
            if (localValid &&
                setFileTimestamp(local, !_fatCreateTimeSet)) {
                _fatCreateTimeSet = true;
            }
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
