#ifndef USV_PACKETS_H
#define USV_PACKETS_H

#include <stdint.h>

// Packed little-endian. Shore copies this file as-is.
// Demux is by length: 5 / 194 / 38 / 2 / 20 / 12.

typedef struct __attribute__((packed)) {
    uint8_t sys_command;
    int16_t manual_steer;
    int16_t manual_speed;
} CommandPacket;

typedef struct __attribute__((packed)) {
    double lat;
    double lon;
} Waypoint;

typedef struct __attribute__((packed)) {
    uint8_t  packet_index;
    uint8_t  waypoint_count;
    Waypoint waypoints[12];
} WaypointArrayPacket;

typedef struct __attribute__((packed)) {
    uint8_t packet_index;
    uint8_t ack_status;
} WaypointACKPacket;

typedef struct __attribute__((packed)) {
    uint32_t sequence_number;
    double   gps_lat;
    double   gps_lon;
    float    water_ph;
    float    water_turbidity;
    float    water_temp;
    uint8_t  current_mode;
    uint8_t  battery_percent;
    float    heading_angle;
} TelemetryPacket;

typedef struct __attribute__((packed)) {
    uint32_t report_timestamp;
    float    battery_voltage;
    float    current_draw_ma;
    float    state_of_charge;
    float    estimated_tte_min;
} BatteryReportPacket;

typedef struct __attribute__((packed)) {
    uint32_t sequence_number;
    uint8_t  current_stage;
    uint8_t  intent;
    uint8_t  next_stage;
    uint8_t  reason;
    uint8_t  flags;
    uint8_t  waypoint_count;
    uint8_t  waypoint_index;
    uint8_t  expected_batch_index;
} StageStatusPacket;

enum StageIntentCode {
    STAGE_INTENT_WAIT_WAYPOINT_BATCH = 0,
    STAGE_INTENT_WAIT_MISSION_SEAL   = 1,
    STAGE_INTENT_WAIT_START          = 2,
    STAGE_INTENT_WAIT_NAV_READY      = 3,
    STAGE_INTENT_PATROL_WAYPOINT     = 4,
    STAGE_INTENT_MANUAL_CONTROL      = 5,
    STAGE_INTENT_MAP_ANOMALY         = 6,
    STAGE_INTENT_RETURN_HOME         = 7,
    STAGE_INTENT_WAIT_CLEAR_ESTOP    = 8,
    STAGE_INTENT_WAIT_SAFETY_CLEAR   = 9
};

enum StageReasonCode {
    STAGE_REASON_BOOT               = 0,
    STAGE_REASON_STATUS_UPDATE      = 1,
    STAGE_REASON_START_ACCEPTED     = 2,
    STAGE_REASON_ANOMALY_DETECTED   = 3,
    STAGE_REASON_SPIRAL_COMPLETE    = 4,
    STAGE_REASON_MISSION_COMPLETE   = 5,
    STAGE_REASON_FORCE_RTH          = 6,
    STAGE_REASON_LOW_BATTERY        = 7,
    STAGE_REASON_EMERGENCY_COMMAND  = 8,
    STAGE_REASON_GPS_FAULT          = 9,
    STAGE_REASON_CRITICAL_SAFETY    = 10,
    STAGE_REASON_ARRIVED_HOME       = 11,
    STAGE_REASON_CLEAR_ESTOP        = 12,
    STAGE_REASON_FORCE_SPIRAL       = 13
};

enum StageStatusFlags {
    STAGE_FLAG_UPLOAD_COMPLETE = (1U << 0),
    STAGE_FLAG_MISSION_READY   = (1U << 1),
    STAGE_FLAG_START_PENDING   = (1U << 2),
    STAGE_FLAG_GPS_READY       = (1U << 3),
    STAGE_FLAG_COMPASS_READY   = (1U << 4),
    STAGE_FLAG_HOME_SET        = (1U << 5),
    STAGE_FLAG_MANUAL_ACTIVE   = (1U << 6),
    STAGE_FLAG_SAFETY_NORMAL   = (1U << 7)
};

enum {
    STAGE_NEXT_DYNAMIC = 0xFF
};

enum CommandCode {
    CMD_NORMAL              = 0,
    CMD_EMERGENCY_STOP      = 1,
    CMD_FORCE_RTH           = 2,
    CMD_START_ARM           = 3,
    CMD_CLEAR_ESTOP         = 4,
    CMD_MISSION_UPLOAD_DONE = 5,
    CMD_SET_HOME            = 6,
    CMD_FORCE_SPIRAL        = 7
};

enum AckStatus {
    ACK_OK   = 0,
    ACK_NACK = 1
};

#if defined(__cplusplus) && __cplusplus >= 201103L
static_assert(sizeof(CommandPacket)      == 5,   "CommandPacket must be 5 bytes");
static_assert(sizeof(Waypoint)           == 16,  "Waypoint must be 16 bytes");
static_assert(sizeof(WaypointArrayPacket)== 194, "WaypointArrayPacket must be 194 bytes");
static_assert(sizeof(WaypointACKPacket)  == 2,   "WaypointACKPacket must be 2 bytes");
static_assert(sizeof(TelemetryPacket)    == 38,  "TelemetryPacket must be 38 bytes");
static_assert(sizeof(BatteryReportPacket)== 20,  "BatteryReportPacket must be 20 bytes");
static_assert(sizeof(StageStatusPacket)  == 12,  "StageStatusPacket must be 12 bytes");
#endif

#endif // USV_PACKETS_H
