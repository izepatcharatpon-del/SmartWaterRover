#include "core/StageReporter.h"

static bool isEmergencySafety(SafetyState safety) {
    return safety == SafetyState::CRITICAL_STOP ||
           safety == SafetyState::OVERCURRENT_STOP ||
           safety == SafetyState::LEAK_STOP;
}

uint8_t StageReporter::deriveIntent(const StageReportContext& ctx) {
    if (ctx.currentState == STATE_EMERGENCY_STOP) {
        return STAGE_INTENT_WAIT_CLEAR_ESTOP;
    }

    if (ctx.currentState == STATE_STANDBY) {
        const bool routeFinished =
            ctx.waypointCount > 0 && ctx.waypointIndex >= ctx.waypointCount;
        if (ctx.waypointCount == 0 || routeFinished) {
            return STAGE_INTENT_WAIT_WAYPOINT_BATCH;
        }
        if (!ctx.uploadComplete) {
            return STAGE_INTENT_WAIT_MISSION_SEAL;
        }
        if (!ctx.fsm.hasWaypoints) {
            return STAGE_INTENT_WAIT_WAYPOINT_BATCH;
        }
        if (ctx.fsm.safety != SafetyState::NORMAL) {
            return STAGE_INTENT_WAIT_SAFETY_CLEAR;
        }
        if (ctx.fsm.startCommand && (!ctx.fsm.gpsOk || !ctx.fsm.compassOk)) {
            return STAGE_INTENT_WAIT_NAV_READY;
        }
        return STAGE_INTENT_WAIT_START;
    }

    if (!ctx.fsm.gpsOk || !ctx.fsm.compassOk) {
        return STAGE_INTENT_WAIT_NAV_READY;
    }

    switch (ctx.currentState) {
        case STATE_GRID_PATROL:
            return ctx.manualMode
                ? STAGE_INTENT_MANUAL_CONTROL
                : STAGE_INTENT_PATROL_WAYPOINT;
        case STATE_SPIRAL_MAPPING:
            return STAGE_INTENT_MAP_ANOMALY;
        case STATE_RTH:
            return STAGE_INTENT_RETURN_HOME;
        case STATE_STANDBY:
        case STATE_EMERGENCY_STOP:
            break;
    }
    return STAGE_INTENT_WAIT_CLEAR_ESTOP;
}

uint8_t StageReporter::deriveNextStage(const StageReportContext& ctx) {
    switch (ctx.currentState) {
        case STATE_STANDBY:
            return STATE_GRID_PATROL;
        case STATE_GRID_PATROL:
            if (ctx.fsm.anomalyDetected || ctx.fsm.forceSpiralCommand)
                return STATE_SPIRAL_MAPPING;
            if (ctx.fsm.gridComplete) {
                return ctx.fsm.homeSet ? STATE_RTH : STATE_STANDBY;
            }
            return STAGE_NEXT_DYNAMIC;
        case STATE_SPIRAL_MAPPING:
            return STATE_GRID_PATROL;
        case STATE_RTH:
            return STATE_STANDBY;
        case STATE_EMERGENCY_STOP:
            return STATE_STANDBY;
    }
    return STAGE_NEXT_DYNAMIC;
}

uint8_t StageReporter::deriveReason(const StageReportContext& ctx) {
    if (ctx.initialReport && ctx.previousState == ctx.currentState) {
        return STAGE_REASON_BOOT;
    }
    if (ctx.previousState == ctx.currentState) {
        return STAGE_REASON_STATUS_UPDATE;
    }

    if (ctx.currentState == STATE_EMERGENCY_STOP) {
        if (ctx.fsm.emergencyCommand) return STAGE_REASON_EMERGENCY_COMMAND;
        if (ctx.fsm.gpsFault) return STAGE_REASON_GPS_FAULT;
        if (isEmergencySafety(ctx.fsm.safety)) {
            return STAGE_REASON_CRITICAL_SAFETY;
        }
        if (ctx.fsm.forceRthCommand) return STAGE_REASON_FORCE_RTH;
        if (ctx.fsm.safety == SafetyState::LOW_BATTERY_RTH) {
            return STAGE_REASON_LOW_BATTERY;
        }
    }

    if (ctx.currentState == STATE_RTH) {
        if (ctx.fsm.forceRthCommand) return STAGE_REASON_FORCE_RTH;
        if (ctx.fsm.safety == SafetyState::LOW_BATTERY_RTH) {
            return STAGE_REASON_LOW_BATTERY;
        }
        return STAGE_REASON_MISSION_COMPLETE;
    }

    if (ctx.currentState == STATE_SPIRAL_MAPPING) {
        if (ctx.fsm.anomalyDetected) return STAGE_REASON_ANOMALY_DETECTED;
        if (ctx.fsm.forceSpiralCommand) return STAGE_REASON_FORCE_SPIRAL;
        return STAGE_REASON_ANOMALY_DETECTED;
    }

    if (ctx.currentState == STATE_GRID_PATROL) {
        return ctx.previousState == STATE_SPIRAL_MAPPING
            ? STAGE_REASON_SPIRAL_COMPLETE
            : STAGE_REASON_START_ACCEPTED;
    }

    if (ctx.currentState == STATE_STANDBY) {
        if (ctx.previousState == STATE_RTH && ctx.fsm.arrivedHome) {
            return STAGE_REASON_ARRIVED_HOME;
        }
        if (ctx.previousState == STATE_EMERGENCY_STOP && ctx.fsm.clearEstop) {
            return STAGE_REASON_CLEAR_ESTOP;
        }
        if (ctx.previousState == STATE_GRID_PATROL && ctx.fsm.gridComplete) {
            return STAGE_REASON_MISSION_COMPLETE;
        }
    }

    return STAGE_REASON_STATUS_UPDATE;
}

uint8_t StageReporter::deriveFlags(const StageReportContext& ctx) {
    uint8_t flags = 0;
    if (ctx.uploadComplete) flags |= STAGE_FLAG_UPLOAD_COMPLETE;
    if (ctx.fsm.hasWaypoints) flags |= STAGE_FLAG_MISSION_READY;
    if (ctx.fsm.startCommand) flags |= STAGE_FLAG_START_PENDING;
    if (ctx.fsm.gpsOk) flags |= STAGE_FLAG_GPS_READY;
    if (ctx.fsm.compassOk) flags |= STAGE_FLAG_COMPASS_READY;
    if (ctx.homeSet) flags |= STAGE_FLAG_HOME_SET;
    if (ctx.manualMode) flags |= STAGE_FLAG_MANUAL_ACTIVE;
    if (ctx.fsm.safety == SafetyState::NORMAL) {
        flags |= STAGE_FLAG_SAFETY_NORMAL;
    }
    return flags;
}

StageStatusPacket StageReporter::build(const StageReportContext& ctx) {
    StageStatusPacket packet = {};
    packet.current_stage = (uint8_t)ctx.currentState;
    packet.intent = deriveIntent(ctx);
    packet.next_stage = deriveNextStage(ctx);
    packet.reason = deriveReason(ctx);
    packet.flags = deriveFlags(ctx);
    packet.waypoint_count = ctx.waypointCount;
    packet.waypoint_index = ctx.waypointIndex;
    packet.expected_batch_index = ctx.expectedBatchIndex;
    return packet;
}

bool StageReporter::sameStatus(const StageStatusPacket& a,
                               const StageStatusPacket& b) {
    const uint8_t edgeFlags =
        STAGE_FLAG_UPLOAD_COMPLETE |
        STAGE_FLAG_MISSION_READY |
        STAGE_FLAG_START_PENDING |
        STAGE_FLAG_MANUAL_ACTIVE |
        STAGE_FLAG_HOME_SET;

    return a.current_stage == b.current_stage &&
           a.intent == b.intent &&
           a.next_stage == b.next_stage &&
           (a.flags & edgeFlags) == (b.flags & edgeFlags) &&
           a.waypoint_count == b.waypoint_count &&
           a.waypoint_index == b.waypoint_index &&
           a.expected_batch_index == b.expected_batch_index;
}
