#ifndef USV_DATA_LOGGER_H
#define USV_DATA_LOGGER_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <stdint.h>

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
};

class DataLogger {
public:
    DataLogger();

    bool begin();
    bool logRecord(const SdLogRecord& rec);
    bool checkIntegrity();
    bool isReady() const { return _ready; }
    uint32_t writeFailCount() const { return _writeFailCount; }
    const char* filename() const { return _filename; }

private:
    SemaphoreHandle_t _spiMutex;
    bool              _ready;
    unsigned long     _lastFlush;
    uint32_t          _writeFailCount;
    bool              _fatCreateTimeSet;
    char              _filename[24];

    void openAndWriteHeader();
    uint32_t nextBootIndex();
};

#endif // USV_DATA_LOGGER_H
