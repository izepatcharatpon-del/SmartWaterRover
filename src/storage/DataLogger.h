#ifndef USV_DATA_LOGGER_H
#define USV_DATA_LOGGER_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <stdint.h>
#include "core/DiagnosticFormatting.h"
#include "config.h"
#include "FixedLogBuffer.h"

// Radio callback result for the latest telemetry attempt. ACK confirms only
// ESP-NOW link-layer delivery to the peer, not gateway storage or dashboard use.
enum TelemetryRadioStatus : uint8_t {
    TELEMETRY_RADIO_NEVER = 0,
    TELEMETRY_RADIO_ACK,
    TELEMETRY_RADIO_NO_ACK,
    TELEMETRY_RADIO_SEND_ERR,
    TELEMETRY_RADIO_TIMEOUT
};

// SD-only row. Not a radio packet — do not put this in Packets.h.
struct SdLogRecord {
    uint32_t boot_ms;
    uint32_t sequence_number;
    double   gps_lat;
    double   gps_lon;
    float    water_ph;
    float    water_turbidity;
    float    water_temp;
    float    ph_voltage;
    float    turb_voltage;
    float    heading_angle;
    float    gps_hdop;
    float    gps_speed_kmh;
    double   target_lat;
    double   target_lon;
    float    nav_bearing_deg;
    float    cross_track_m;
    float    dist_to_target_m;
    float    heading_error_deg;
    uint32_t water_sample_ms;
    int16_t  base_pwm;
    int16_t  left_pwm;
    int16_t  right_pwm;
    int16_t  compass_raw_x;
    int16_t  compass_raw_y;
    uint8_t  current_mode;
    uint8_t  battery_percent;
    uint8_t  gps_sats;
    uint8_t  waypoint_index;
    uint8_t  waypoint_count;
    uint8_t  water_sample_phase;
    uint8_t  compass_cal_nvs;
    uint16_t utc_year;
    uint8_t  utc_month;
    uint8_t  utc_day;
    uint8_t  utc_hour;
    uint8_t  utc_minute;
    uint8_t  utc_second;
    uint8_t  utc_valid;

    // Appended CSV diagnostics; keep existing column order stable.
    double   gps_raw_lat;
    double   gps_raw_lon;
    float    gps_course_deg;
    uint32_t gps_location_age_ms;
    uint8_t  gps_fix_valid;
    uint8_t  compass_fresh;
    uint8_t  compass_fail_count;
    float    direct_bearing_deg;
    int16_t  requested_left_pwm;
    int16_t  requested_right_pwm;
    uint8_t  turn_in_place;
    uint8_t  drive_source;
    uint8_t  drive_stop_reason;
    uint32_t telemetry_tx_seq;
    uint32_t telemetry_tx_ms;
    double   telemetry_tx_lat;
    double   telemetry_tx_lon;
    float    telemetry_tx_ph;
    uint8_t  telemetry_radio_status;
    uint8_t  telemetry_tx_attempts;
    uint8_t  radio_tx_paused;
    float    battery_voltage;
    DiagnosticFormatting::PhLogRecord ph_diagnostic;
    uint32_t utc_age_ms;
    bool utc_age_known;
};

class DataLogger {
public:
    DataLogger();

    bool begin(float calV401, float calV686, float calV918,
               bool calValid, const char* calSource);
    bool logRecord(const SdLogRecord& rec);
    bool logPhDiagnostic(uint32_t bootMs, uint32_t sdSequence,
                         const DiagnosticFormatting::PhLogRecord& rec);
    bool logTxDiagnostic(const DiagnosticFormatting::TxLogRecord& rec);
    // IO owner only: append calls do not touch SD for main/pH rows.
    bool canAcceptLogPair() const;
    bool service(); // At most one write/sync phase per IO iteration.
    size_t mainPendingBytes() const { return _mainBuffer.size(); }
    size_t phPendingBytes() const { return _phBuffer.size(); }
    uint32_t mainAcceptedRows() const { return _mainAccepted; }
    uint32_t mainWrittenRows() const { return _mainWritten; }
    uint32_t mainSyncedRows() const { return _mainSynced; }
    uint32_t phAcceptedRows() const { return _phAccepted; }
    uint32_t phWrittenRows() const { return _phWritten; }
    uint32_t phSyncedRows() const { return _phSynced; }
    uint32_t maxServiceMs() const { return _maxServiceMs; }
    uint32_t maxSyncGapMs() const { return _maxSyncGapMs; }
    uint32_t syncDeadlineMisses() const { return _syncMisses; }
    bool phDiagnosticStopped() const { return _phStopped; }
    bool checkIntegrity();
    bool isReady() const { return _ready; }
    uint32_t writeFailCount() const { return _writeFailCount; }
    uint32_t txDiagnosticWriteFailCount() const { return _txDiagnosticWriteFailCount; }
    uint32_t phDiagnosticWriteFailCount() const { return _phDiagnosticWriteFailCount; }
    const char* filename() const { return _filename; }

private:
    SemaphoreHandle_t _spiMutex;
    bool              _ready;
    unsigned long     _lastFlush;
    uint32_t          _writeFailCount;
    uint32_t          _txDiagnosticWriteFailCount;
    uint32_t          _phDiagnosticWriteFailCount;
    bool              _fatCreateTimeSet;
    char              _filename[24];
    char              _txFilename[32];
    char              _metadataFilename[32];

    static constexpr size_t MAIN_ROW_BYTES = 768;
    static constexpr size_t PH_ROW_BYTES = 512;
    FixedLogBuffer<SD_LOG_MAIN_BUFFER_BYTES> _mainBuffer;
    FixedLogBuffer<SD_LOG_PH_BUFFER_BYTES> _phBuffer;
    enum class SyncPhase : uint8_t { NONE, MAIN_DATA, MAIN_SYNC, PH_DATA, PH_SYNC, TX_SYNC };
    SyncPhase _syncPhase = SyncPhase::NONE;
    size_t _commitMain = 0, _commitPh = 0;
    uint32_t _commitStart = 0, _maxServiceMs = 0;
    uint32_t _mainAccepted = 0, _mainWritten = 0, _mainSynced = 0;
    uint32_t _phAccepted = 0, _phWritten = 0, _phSynced = 0;
    uint32_t _lastSyncCompleted = 0, _maxSyncGapMs = 0, _syncMisses = 0;
    bool _nextPh = false, _haveTimestamp = false, _commitHasTimestamp = false;
    bool _phStopped = false, _commitSyncOk = true;
    uint16_t _commitYear = 0;
    uint8_t _commitMonth = 0, _commitDay = 0, _commitHour = 0, _commitMinute = 0, _commitSecond = 0;
    uint16_t _timeYear = 0;
    uint8_t _timeMonth = 0, _timeDay = 0, _timeHour = 0, _timeMinute = 0, _timeSecond = 0;
    void openAndWriteHeader();
    uint32_t nextBootIndex();
    void writeMetadata(float calV401, float calV686, float calV918,
                       bool calValid, const char* calSource);
};

#endif // USV_DATA_LOGGER_H
