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
    uint8_t sys_command;    // 0=Normal, 1=Emergency Stop, 2=Force RTH
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

// ---------- Command codes -----------------------------------------------------
enum CommandCode {
    CMD_NORMAL          = 0,
    CMD_EMERGENCY_STOP  = 1,
    CMD_FORCE_RTH       = 2
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
#endif

#endif // USV_PACKETS_H
