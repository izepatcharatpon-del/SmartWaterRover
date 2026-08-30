#ifndef USV_PACKETS_H
#define USV_PACKETS_H

#include <stdint.h>

// =============================================================================
// ESP-NOW wire packet contract (single source of truth).
// Layout matches 2_Documentation/water_rover_system_plan.md exactly.
// Share THIS FILE verbatim with the shore/gateway side (friend's project).
// little-endian, packed structs. Sizes are enforced with static_assert below.
// =============================================================================

// ---------- Shore -> Boat -----------------------------------------------------

// General command / emergency button (5 bytes)
typedef struct __attribute__((packed)) {
    uint8_t sys_command;    // see CommandCode
    int16_t manual_steer;   // -255..255
    int16_t manual_speed;   // -255..255
} CommandPacket;

typedef struct __attribute__((packed)) {
    double lat;
    double lon;
} Waypoint;

// Patrol waypoint batch (194 bytes, max 12 points per packet)
typedef struct __attribute__((packed)) {
    uint8_t  packet_index;      // batch index (for multi-batch sends)
    uint8_t  waypoint_count;    // valid points in this packet (<=12)
    Waypoint waypoints[12];     // 12 * 16 = 192 bytes
} WaypointArrayPacket;

// ---------- Boat -> Shore -----------------------------------------------------

// Waypoint acknowledgement (2 bytes)
typedef struct __attribute__((packed)) {
    uint8_t packet_index;   // batch index being acknowledged
    uint8_t ack_status;     // 0 = ACK, 1 = NACK (request resend)
} WaypointACKPacket;

// Live telemetry (38 bytes) sent every 1-2s
typedef struct __attribute__((packed)) {
    uint32_t sequence_number;   // monotonic, replay protection
    double   gps_lat;
    double   gps_lon;
    // Water fields are NaN in STANDBY/ESTOP. Filled only while GRID/SPIRAL/RTH.
    float    water_ph;
    float    water_turbidity;   // NTU
    float    water_temp;
    uint8_t  current_mode;      // BoatState enum value
    uint8_t  battery_percent;   // 0-100
    float    heading_angle;     // compass degrees
} TelemetryPacket;

// Battery / power report (20 bytes), optional, ~every 5 min
typedef struct __attribute__((packed)) {
    uint32_t report_timestamp;
    float    battery_voltage;
    float    current_draw_ma;
    float    state_of_charge;
    float    estimated_tte_min;
} BatteryReportPacket;

// Mission stage / intent event (12 bytes). Sent on status edges; telemetry
// current_mode remains the 1 Hz heartbeat if this event is lost.
typedef struct __attribute__((packed)) {
    uint32_t sequence_number;     // monotonic within StageStatusPacket stream
    uint8_t  current_stage;       // BoatState value: 0..4
    uint8_t  intent;              // StageIntentCode
    uint8_t  next_stage;          // BoatState value or STAGE_NEXT_DYNAMIC
    uint8_t  reason;              // StageReasonCode
    uint8_t  flags;               // StageStatusFlags bitmask
    uint8_t  waypoint_count;      // total accepted points: 0..64
    uint8_t  waypoint_index;      // active point (0-based), ==count when done
    uint8_t  expected_batch_index;// next WaypointArrayPacket index
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
    STAGE_REASON_FORCE_SPIRAL       = 13  // shore CMD_FORCE_SPIRAL (not water anomaly)
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

// ---------- Command codes -----------------------------------------------------
// Wire size of CommandPacket stays 5 bytes; only the meaning of sys_command grows.
enum CommandCode {
    CMD_NORMAL              = 0,  // idle / clear force-RTH latch (does NOT clear E-Stop)
    CMD_EMERGENCY_STOP      = 1,  // latch E-Stop immediately
    CMD_FORCE_RTH           = 2,  // force return-to-home
    CMD_START_ARM           = 3,  // seal pending upload if any, then arm (needs GPS+compass)
    CMD_CLEAR_ESTOP         = 4,  // explicit clear of latched E-Stop -> STANDBY
    // Seal multi-batch waypoint upload when the last batch is full (count==12).
    // Batches with waypoint_count < 12 auto-seal. Wire size unchanged.
    CMD_MISSION_UPLOAD_DONE = 5,
    CMD_SET_HOME            = 6,  // latch current GPS as RTH home (STANDBY + fix)
    CMD_FORCE_SPIRAL        = 7   // GRID -> spiral now (no water anomaly required)
};

enum AckStatus {
    ACK_OK   = 0,
    ACK_NACK = 1
};

// ---------- Wire-size guarantees (compile-time) -------------------------------
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
