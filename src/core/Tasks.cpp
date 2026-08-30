#include "core/Tasks.h"
#include "core/StageReporter.h"
#include "config.h"
#include <math.h>
#include <string.h>

// Home position — captured on every STANDBY -> GRID_PATROL with valid GPS.
static double g_homeLat = 0.0, g_homeLon = 0.0;
static bool   g_homeSet = false;
static bool   g_homeLeft = false;  // GPS dist >= RTH_DEPART (log only)
static float    g_rthEntryDistM = NAN;
static uint32_t g_rthEnterMs = 0;

// Spiral chase target.
static Waypoint g_spiralTgt = {0.0, 0.0};
static bool     g_haveSpiralTgt = false;
static bool     g_spiralInitialized = false;

// Shore gets GPS/heading/mode/batt in every 1 Hz heartbeat. Water quality
// rides the same 38 B packet but only while a mission is actually running.
static bool radioSendsWaterQuality(uint8_t mode) {
    return mode == STATE_GRID_PATROL ||
           mode == STATE_SPIRAL_MAPPING ||
           mode == STATE_RTH;
}

static int chooseBasePwm(float headingErrorDeg, float distToEndM, int cruisePwm) {
    float absErr = fabsf(headingErrorDeg);
    if (absErr > LOS_TURN_FIRST_DEG) return 0;
    if (distToEndM < LOS_NEAR_SLOW_M) return MOTOR_APPROACH_PWM;
    return cruisePwm;
}

static void toggleRadioTxPause(SystemContext* ctx) {
    static uint32_t lastMs = 0;
    const uint32_t now = millis();
    // One key or a pasted burst → one toggle.
    if ((now - lastMs) < 250UL) {
        return;
    }
    lastMs = now;
    ctx->radioTxPaused = !ctx->radioTxPaused;
    Serial.printf("[USV] radio TX %s (RX still on)\n",
                  ctx->radioTxPaused ? "PAUSED" : "ON");
}

static void pollCalButton(SystemContext* ctx) {
    static bool armed = false;
    static bool lastRaw = true;
    static bool stableReleased = true;
    static uint32_t lastEdgeMs = 0;
    const uint32_t now = millis();
    if (!armed) {
        if (now < COMPASS_CAL_BUTTON_ARM_MS) {
            return;
        }
        armed = true;
        lastRaw = (digitalRead(COMPASS_CAL_BUTTON_PIN) != LOW);
        stableReleased = lastRaw;
        lastEdgeMs = now;
        return;
    }

    const bool released = (digitalRead(COMPASS_CAL_BUTTON_PIN) != LOW);
    if (released != lastRaw) {
        lastRaw = released;
        lastEdgeMs = now;
        return;
    }
    if ((now - lastEdgeMs) < COMPASS_CAL_BUTTON_DEBOUNCE_MS) {
        return;
    }
    if (stableReleased == released) {
        return;
    }
    stableReleased = released;
    if (released) {
        return;
    }
    if (xSemaphoreTake(ctx->stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        ctx->command.compassCalStart = true;
        xSemaphoreGive(ctx->stateMutex);
        Serial.println(F("[USV] CALMAG button (BOOT)"));
    }
}

static void pollSerialCommands(SystemContext* ctx) {
    static char line[24];
    static uint8_t len = 0;
    while (Serial.available() > 0) {
        const int ch = Serial.read();
        if (ch < 0) break;
        if (ch == '\r' || ch == '\n') {
            if (len == 0) continue;
            line[len] = '\0';
            len = 0;
            char* s = line;
            while (*s == ' ' || *s == '\t') s++;
            char* e = s + strlen(s);
            while (e > s && (e[-1] == ' ' || e[-1] == '\t')) {
                *--e = '\0';
            }
            bool startCal = (strcasecmp(s, "CALMAG") == 0);
            bool clearCal = (strcasecmp(s, "CALMAGCLR") == 0);
            if (startCal) {
                Serial.println(F("[USV] CALMAG received on Serial"));
            } else if (clearCal) {
                Serial.println(F("[USV] CALMAGCLR received on Serial"));
            } else if (s[0] != '\0') {
                toggleRadioTxPause(ctx);
            }
            if ((startCal || clearCal) &&
                xSemaphoreTake(ctx->stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                if (startCal) ctx->command.compassCalStart = true;
                if (clearCal) ctx->command.compassCalClear = true;
                xSemaphoreGive(ctx->stateMutex);
            }
            continue;
        }
        if (ch < 32) {
            continue;
        }
        // Any key except C/c (start of CALMAG) toggles TX immediately.
        if (len == 0 && ch != 'C' && ch != 'c') {
            toggleRadioTxPause(ctx);
            continue;
        }
        if (len < (uint8_t)(sizeof(line) - 1)) {
            line[len++] = (char)ch;
        } else {
            len = 0;
        }
    }
}

#if RADIO_TRACE
static const char* radioCmdName(uint8_t code) {
    switch (code) {
        case CMD_NORMAL:              return "NORMAL";
        case CMD_EMERGENCY_STOP:      return "ESTOP";
        case CMD_FORCE_RTH:           return "RTH";
        case CMD_START_ARM:           return "START_ARM";
        case CMD_CLEAR_ESTOP:         return "CLEAR_ESTOP";
        case CMD_MISSION_UPLOAD_DONE: return "UPLOAD_DONE";
        case CMD_SET_HOME:            return "SET_HOME";
        case CMD_FORCE_SPIRAL:        return "FORCE_SPIRAL";
        default:                      return "?";
    }
}
#endif

// ---------------------------------------------------------------------------
// Core 1 — navigation & control loop
// ---------------------------------------------------------------------------
void navigationTask(void* pv) {
    SystemContext* ctx = static_cast<SystemContext*>(pv);
    ctx->wdt->subscribeCurrentTask();

    BoatState prevState = STATE_STANDBY;
    bool spiralDone = false;
    CommandState lastCmd;
    memset(&lastCmd, 0, sizeof(lastCmd));
    uint32_t logSequence = 0;
    Waypoint lastLosTarget = {0.0, 0.0};
    bool haveLastLosTarget = false;
    StageStatusPacket lastStageStatus = {};
    bool haveLastStageStatus = false;
    unsigned long lastHardThrustMs = 0;
    int lastAbsPwm = 0;
    bool hadHardThrust = false;

    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(NAV_LOOP_PERIOD_MS);
    uint32_t lastLoopUs = micros();

    // Held ADS samples: refreshed every WATER_ADS_EVERY_N_LOOPS ticks so the
    // 20 Hz heading/motor path is not blocked by ~16 ms of ADC conversion.
    uint8_t waterAdsDiv = 0;
    WaterReading phR   = {NAN, false};
    WaterReading turbR = {NAN, false};
    bool anomaly = false;

    for (;;) {
        // ---- Measured dt (clamped later inside PID) ----
        uint32_t nowUs = micros();
        float dt = (nowUs - lastLoopUs) / 1000000.0f;
        lastLoopUs = nowUs;
        if (dt < 0.001f) dt = 0.001f;
        if (dt > 0.25f)  dt = 0.25f;

        // ---- 1. Sensor acquisition ----
        ctx->gps->update();
        ctx->compass->update();
        ctx->water->updateTemperatureNonBlocking();

        GpsFix fix     = ctx->gps->getFilteredFix();
        float  heading = ctx->compass->getHeadingDeg();
        bool   compassOk = ctx->compass->isOk() && isfinite(heading);
        float  temp    = ctx->water->getWaterTemp();
        float  battV   = ctx->battery->readVoltage();
        int hour = ctx->gps->localHour();

        // ---- 2. Water ADS + anomaly (rate-limited) ----
        const bool motionDisturbance =
            hadHardThrust &&
            (millis() - lastHardThrustMs) < ANOMALY_DISTURBANCE_MS;
        if (waterAdsDiv == 0) {
            phR   = ctx->water->readPh(temp);
            turbR = ctx->water->readTurbidityNtu();
            const bool sensorsValid =
                phR.valid && turbR.valid && ctx->water->tempValid();
            anomaly = ctx->anomaly->updateReadings(
                phR.value, turbR.value, temp, sensorsValid, hour,
                motionDisturbance);
        } else if (!phR.valid || !turbR.valid || !ctx->water->tempValid()) {
            // Brief sensor glitch: suppress trigger only — do not call
            // updateReadings(false) which would wipe Schmitt hysteresis.
            anomaly = false;
        } else {
            anomaly = ctx->anomaly->shouldTriggerSpiral();
        }
        if (++waterAdsDiv >= WATER_ADS_EVERY_N_LOOPS) {
            waterAdsDiv = 0;
        }

        // ---- 3. Drain inbound commands (nav owns command state updates too) ----
        // Commands still arrive via EspNowLink::commandQueue; IO also drains them
        // into ctx->command. We re-read under mutex below.

        // ---- 3b. Drain waypoint queue HERE (single owner of Navigator) ----
        WaypointArrayPacket wps;
        while (EspNowLink::waypointQueue &&
               xQueueReceive(EspNowLink::waypointQueue, &wps, 0) == pdTRUE) {
            const bool allowNewMission = (ctx->fsm->state() == STATE_STANDBY);
            bool ok = ctx->nav->acceptBatch(wps, allowNewMission);
            WaypointACKPacket ack;
            ack.packet_index = wps.packet_index;
            ack.ack_status   = ok ? ACK_OK : ACK_NACK;
#if RADIO_TRACE
            Serial.printf("[RX] WP idx=%u count=%u ack=%s",
                          (unsigned)wps.packet_index,
                          (unsigned)wps.waypoint_count,
                          ok ? "OK" : "NACK");
            if (wps.waypoint_count > 0 && wps.waypoint_count <= 12) {
                const uint8_t last = (uint8_t)(wps.waypoint_count - 1);
                Serial.printf(" first=%.6f,%.6f last=%.6f,%.6f",
                              wps.waypoints[0].lat, wps.waypoints[0].lon,
                              wps.waypoints[last].lat, wps.waypoints[last].lon);
            }
            Serial.println();
#endif
            if (EspNowLink::ackQueue) {
                if (xQueueSend(EspNowLink::ackQueue, &ack,
                               pdMS_TO_TICKS(5)) != pdTRUE) {
                    ctx->ackDropCount++;
                }
            }
            // Waypoints alone no longer auto-START; need CMD_START_ARM.
        }

        // ---- 4. Snapshot operator command ----
        CommandState cmd;
        if (xSemaphoreTake(ctx->stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            cmd = ctx->command;
            lastCmd = cmd;
            xSemaphoreGive(ctx->stateMutex);
        } else {
            // Fail closed: retain the previous command snapshot instead of
            // treating contention as "no emergency".
            cmd = lastCmd;
        }

        // Seal multi-batch upload when shore says the last full (12/12) batch
        // is done, or when START_ARM arrives (Start implies the list is complete).
        if (cmd.sealMissionUpload ||
            (cmd.start && ctx->nav->hasWaypoints() &&
             !ctx->nav->uploadComplete())) {
            const bool sealedByStart = cmd.start && !cmd.sealMissionUpload;
            ctx->nav->sealMissionUpload();
            if (sealedByStart) {
                Serial.printf("[USV] START_ARM sealed upload n=%d\n",
                              ctx->nav->count());
            }
            if (cmd.sealMissionUpload &&
                xSemaphoreTake(ctx->stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                ctx->command.sealMissionUpload = false;
                xSemaphoreGive(ctx->stateMutex);
            }
        }

        // Discard stale CLEAR / FORCE_RTH when they cannot apply — otherwise a
        // CLEAR sent in STANDBY latches and auto-exits the *next* E-Stop.
        if (cmd.clearEstop && ctx->fsm->state() != STATE_EMERGENCY_STOP) {
            cmd.clearEstop = false;
            if (xSemaphoreTake(ctx->stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                ctx->command.clearEstop = false;
                xSemaphoreGive(ctx->stateMutex);
            }
        }
        if (cmd.forceRth && ctx->fsm->state() == STATE_STANDBY) {
            cmd.forceRth = false;
            if (xSemaphoreTake(ctx->stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                ctx->command.forceRth = false;
                xSemaphoreGive(ctx->stateMutex);
            }
        }

        if (cmd.compassCalClear) {
            if (xSemaphoreTake(ctx->stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                ctx->command.compassCalClear = false;
                xSemaphoreGive(ctx->stateMutex);
            }
            const bool cleared = ctx->compass->clearStoredCalibration();
            Serial.printf("[USV] CALMAGCLR %s — factory hard-iron\n",
                          cleared ? "ok" : "FAIL");
        }
        if (cmd.compassCalStart) {
            if (xSemaphoreTake(ctx->stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                ctx->command.compassCalStart = false;
                xSemaphoreGive(ctx->stateMutex);
            }
            if (ctx->compass->isCalibrating()) {
                float xMin = 0, xMax = 0, yMin = 0, yMax = 0;
                uint16_t samples = 0;
                uint32_t elapsedMs = 0;
                ctx->compass->getCalProgress(xMin, xMax, yMin, yMax, samples, elapsedMs);
                if (ctx->compass->finishCalibration()) {
                    ctx->compass->getHardIron(xMin, xMax, yMin, yMax);
                    Serial.printf(
                        "[USV] CALMAG saved NVS x=[%.0f,%.0f] y=[%.0f,%.0f] n=%u\n",
                        xMin, xMax, yMin, yMax, (unsigned)samples);
                } else {
                    Serial.printf(
                        "[USV] CALMAG not ready x=[%.0f,%.0f] y=[%.0f,%.0f] n=%u "
                        "(keep spinning; press BOOT again to save)\n",
                        xMin, xMax, yMin, yMax, (unsigned)samples);
                }
            } else if (ctx->fsm->state() == STATE_STANDBY &&
                       ctx->compass->i2cAddress() != 0 &&
                       ctx->compass->startCalibration()) {
                Serial.println(F("[USV] CALMAG started — keep level, two slow yaw turns. "
                                 "Pause to untangle USB is OK. Press BOOT to save, "
                                 "CALMAGCLR to abort"));
            } else {
                Serial.println(F("[USV] CALMAG refused — need STANDBY and compass OK"));
            }
        }
        if (ctx->compass->isCalibrating()) {
            if (ctx->fsm->state() != STATE_STANDBY) {
                ctx->compass->abortCalibration();
                Serial.println(F("[USV] CALMAG abort — left STANDBY"));
            } else {
                static uint8_t calProgDiv = 0;
                if (++calProgDiv >= (uint8_t)(1000 / NAV_LOOP_PERIOD_MS)) {
                    calProgDiv = 0;
                    float xMin = 0, xMax = 0, yMin = 0, yMax = 0;
                    uint16_t samples = 0;
                    uint32_t elapsedMs = 0;
                    ctx->compass->getCalProgress(
                        xMin, xMax, yMin, yMax, samples, elapsedMs);
                    const float dx = xMax - xMin;
                    const float dy = yMax - yMin;
                    Serial.printf(
                        "[USV] CALMAG %s x=[%.0f,%.0f] dx=%.0f y=[%.0f,%.0f] dy=%.0f "
                        "need=%.0f n=%u t=%lus\n",
                        ctx->compass->calibrationReady() ? "READY press BOOT"
                                                         : "spin",
                        xMin, xMax, dx, yMin, yMax, dy,
                        (double)COMPASS_CAL_MIN_SPAN,
                        (unsigned)samples,
                        (unsigned long)(elapsedMs / 1000UL));
                }
            }
        }

        // Dedicated ESP-NOW safety event cannot be lost when commandQueue is full.
        // When CLEAR is pending, do NOT re-assert emergency from that event bit —
        // otherwise FSM always sees emergencyCommand and CLEAR can never succeed
        // (deadlock: pending cleared only after clearAccepted).
        if (EspNowLink::emergencyPending() && !cmd.clearEstop) {
            cmd.emergency = true;
        }

        // GPS fault is latched. Only attempt CLEAR-driven unlatch while in E-Stop.
        if (cmd.clearEstop &&
            ctx->fsm->state() == STATE_EMERGENCY_STOP &&
            ctx->gps->clearFaultIfFresh()) {
            fix = ctx->gps->getFilteredFix();
        }

        // Manual dead-man timeout.
        if (cmd.manualMode) {
            uint32_t age = millis() - cmd.lastManualMs;
            if (age > MANUAL_CMD_TIMEOUT_MS) {
                cmd.manualMode = false;
                cmd.manualSpeed = 0;
                cmd.manualSteer = 0;
                if (xSemaphoreTake(ctx->stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    ctx->command.manualMode = false;
                    ctx->command.manualSpeed = 0;
                    ctx->command.manualSteer = 0;
                    xSemaphoreGive(ctx->stateMutex);
                }
            }
        }

        // ---- 5. Safety (AA118 voltage only; load-aware RTH threshold) ----
        const bool underLoad = (lastAbsPwm >= BATTERY_LOAD_PWM_THRESH) ||
                               motionDisturbance;
        SafetyState safety = ctx->safety->evaluate(battV, underLoad);

        // ---- 6. Mission FSM ----
        GpsLossTier gpsTier = ctx->gps->getLossTier();
        bool gpsOk = (gpsTier == GpsLossTier::OK) && fix.valid;

        if (cmd.setHome) {
            if (xSemaphoreTake(ctx->stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                ctx->command.setHome = false;
                xSemaphoreGive(ctx->stateMutex);
            }
            if (ctx->fsm->state() == STATE_STANDBY && gpsOk && fix.valid) {
                g_homeLat = fix.lat;
                g_homeLon = fix.lon;
                g_homeSet = true;
                g_homeLeft = false;
                Serial.printf("[USV] SET_HOME lat=%.6f lon=%.6f\n",
                              g_homeLat, g_homeLon);
            } else {
                Serial.println(F("[USV] SET_HOME refused — need STANDBY and GPS OK"));
            }
        }

        // Shore spiral is GRID-only. Drop it outside patrol so a later START
        // cannot inherit a stale latch and loop GRID->SPIRAL.
        if (cmd.forceSpiral && ctx->fsm->state() != STATE_GRID_PATROL) {
            if (xSemaphoreTake(ctx->stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                ctx->command.forceSpiral = false;
                xSemaphoreGive(ctx->stateMutex);
            }
            Serial.println(F("[USV] FORCE_SPIRAL refused — need GRID_PATROL"));
            cmd.forceSpiral = false;
        }

        FsmInputs in;
        memset(&in, 0, sizeof(in));
        in.startCommand        = cmd.start;
        in.clearEstop          = cmd.clearEstop;
        in.emergencyCommand    = cmd.emergency;
        in.forceRthCommand     = cmd.forceRth;
        in.forceSpiralCommand  = cmd.forceSpiral;
        in.anomalyDetected     = anomaly;
        in.spiralComplete   = spiralDone;
        in.gridComplete     = ctx->nav->missionComplete();
        float distHomeM = NAN;
        if (g_homeSet && fix.valid) {
            distHomeM = (float)Navigator::calculateDistance(
                fix.lat, fix.lon, g_homeLat, g_homeLon);
            if (distHomeM >= RTH_DEPART_RADIUS_M) {
                g_homeLeft = true;
            }
        }
        // Arrive only if this RTH started outside the 2 m bubble. GPS wander
        // can set g_homeLeft while the boat is still on the START fix.
        in.arrivedHome      = g_homeSet && fix.valid &&
            isfinite(distHomeM) && isfinite(g_rthEntryDistM) &&
            (g_rthEntryDistM > RTH_ARRIVE_RADIUS_M) &&
            (distHomeM <= RTH_ARRIVE_RADIUS_M) &&
            ((millis() - g_rthEnterMs) >= RTH_MIN_HOLD_MS);
        in.gpsOk            = gpsOk;
        in.gpsFault         = (gpsTier == GpsLossTier::GPS_FAULT);
        in.homeSet          = g_homeSet;
        in.compassOk        = compassOk;
        in.hasWaypoints     = ctx->nav->isMissionReady();
        in.safety           = safety;

        BoatState stateBefore = ctx->fsm->state();
        BoatState state = ctx->fsm->tick(in);

        // Clear one-shot flags only when accepted. Keep START pending while
        // waiting for GPS/compass/waypoints, but reject it on low battery.
        const bool startAccepted =
            stateBefore == STATE_STANDBY && state == STATE_GRID_PATROL;
        const bool startUnsafe =
            stateBefore == STATE_STANDBY && safety != SafetyState::NORMAL;
        const bool clearAccepted =
            stateBefore == STATE_EMERGENCY_STOP && state == STATE_STANDBY;
        const bool forceSpiralAccepted =
            stateBefore == STATE_GRID_PATROL &&
            state == STATE_SPIRAL_MAPPING &&
            cmd.forceSpiral;
        if ((cmd.start && (startAccepted || startUnsafe)) ||
            (cmd.clearEstop && clearAccepted) ||
            forceSpiralAccepted) {
            if (xSemaphoreTake(ctx->stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                if (cmd.start && (startAccepted || startUnsafe))
                    ctx->command.start = false;
                if (cmd.clearEstop && clearAccepted) {
                    ctx->command.clearEstop = false;
                    // Clear dedicated E-Stop event only after FSM left ESTOP.
                    EspNowLink::clearEmergencyPending();
                }
                if (forceSpiralAccepted)
                    ctx->command.forceSpiral = false;
                xSemaphoreGive(ctx->stateMutex);
            }
        }
        if (forceSpiralAccepted) {
            Serial.println(F("[USV] FORCE_SPIRAL accepted"));
        }

        // On entering E-Stop: clear start/manual/clear so we cannot auto-resume
        // or auto-clear from a previously latched CLEAR_ESTOP.
        if (state == STATE_EMERGENCY_STOP && prevState != STATE_EMERGENCY_STOP) {
            if (xSemaphoreTake(ctx->stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                ctx->command.start = false;
                ctx->command.clearEstop = false;
                ctx->command.forceSpiral = false;
                ctx->command.manualMode = false;
                ctx->command.manualSpeed = 0;
                ctx->command.manualSteer = 0;
                xSemaphoreGive(ctx->stateMutex);
            }
            ctx->heading->reset();
        }

        // Mission origin follows START. RTH home is CMD_SET_HOME; START
        // only fills it if shore never latched a dock fix.
        if (state == STATE_GRID_PATROL && prevState == STATE_STANDBY) {
            if (fix.valid) {
                g_homeLeft = false;
                g_rthEntryDistM = NAN;
                g_rthEnterMs = 0;
                ctx->nav->setMissionOrigin(fix.lat, fix.lon);
                if (!g_homeSet) {
                    g_homeLat = fix.lat;
                    g_homeLon = fix.lon;
                    g_homeSet = true;
                    Serial.printf("[USV] home lat=%.6f lon=%.6f (START fallback)\n",
                                  g_homeLat, g_homeLon);
                } else {
                    const float startDist = (float)Navigator::calculateDistance(
                        fix.lat, fix.lon, g_homeLat, g_homeLon);
                    Serial.printf("[USV] start dist from home=%.1fm\n", startDist);
                }
            } else {
                // Should not happen (FSM requires gpsOk); stop as failsafe.
                ctx->motor->stop();
            }
            ctx->heading->reset();
            haveLastLosTarget = false;
        }

        if (state == STATE_RTH && prevState != STATE_RTH) {
            g_rthEntryDistM = distHomeM;
            g_rthEnterMs = millis();
            Serial.printf(
                "[USV] RTH entry dist=%.1fm left=%u home=%.6f,%.6f now=%.6f,%.6f\n",
                distHomeM, (unsigned)g_homeLeft,
                g_homeLat, g_homeLon, fix.lat, fix.lon);
        }

        // Mission finished at home.
        if (state == STATE_STANDBY && prevState == STATE_RTH) {
            if (xSemaphoreTake(ctx->stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                ctx->command.start = false;
                ctx->command.forceRth = false;
                xSemaphoreGive(ctx->stateMutex);
            }
            ctx->heading->reset();
        }

        // Spiral entry.
        if (state == STATE_SPIRAL_MAPPING && prevState != STATE_SPIRAL_MAPPING) {
            spiralDone = false;
            g_haveSpiralTgt = false;
            g_spiralInitialized = false;
            ctx->heading->reset();
            haveLastLosTarget = false;
        }
        if (state != STATE_SPIRAL_MAPPING) {
            // Completion, RTH, E-Stop and GPS-fault exits all need cooldown;
            // otherwise recovery can immediately re-enter SPIRAL.
            if (prevState == STATE_SPIRAL_MAPPING) {
                ctx->anomaly->notifySpiralDone();
                anomaly = false;  // local flag must match cooldown latch
            }
            spiralDone = false;
            g_spiralInitialized = false;
        }

        // Announce stage/intent edges. current_mode in telemetry remains the
        // 1 Hz heartbeat; this event adds why/next/upload progress.
        StageReportContext stageCtx = {};
        stageCtx.previousState = prevState;
        stageCtx.currentState = state;
        stageCtx.fsm = in;
        stageCtx.initialReport = !haveLastStageStatus;
        stageCtx.manualMode = cmd.manualMode;
        stageCtx.uploadComplete = ctx->nav->uploadComplete();
        stageCtx.homeSet = g_homeSet;
        stageCtx.waypointCount = (uint8_t)ctx->nav->count();
        stageCtx.waypointIndex = (uint8_t)ctx->nav->index();
        stageCtx.expectedBatchIndex = ctx->nav->expectedPacketIndex();

        StageStatusPacket stageStatus = StageReporter::build(stageCtx);
        if (!haveLastStageStatus ||
            !StageReporter::sameStatus(stageStatus, lastStageStatus)) {
            if (ctx->stageQueue &&
                xQueueSend(ctx->stageQueue, &stageStatus, 0) == pdTRUE) {
                lastStageStatus = stageStatus;
                haveLastStageStatus = true;
            } else {
                ctx->stageDropCount++;
            }
        }

        // ---- 7. Act on state ----
        int leftPwm = 0, rightPwm = 0;
        bool navSensorsOk = gpsOk && compassOk;

        switch (state) {
            case STATE_STANDBY:
            case STATE_EMERGENCY_STOP:
                ctx->motor->stop();
                ctx->heading->reset();
                break;

            case STATE_GRID_PATROL: {
                if (cmd.manualMode) {
                    leftPwm  = constrain(cmd.manualSpeed + cmd.manualSteer,
                                         -MOTOR_PWM_LIMIT, MOTOR_PWM_LIMIT);
                    rightPwm = constrain(cmd.manualSpeed - cmd.manualSteer,
                                         -MOTOR_PWM_LIMIT, MOTOR_PWM_LIMIT);
                } else if (navSensorsOk && ctx->nav->hasWaypoints() &&
                           !ctx->nav->missionComplete()) {
                    LosGuidance los = ctx->nav->computeLos(
                        fix.lat, fix.lon,
                        LOS_LOOKAHEAD_M, WAYPOINT_RADIUS_M, LOS_PASS_MARGIN_M);

                    // Do not instantly skip WP1 merely because it is within the
                    // 2.5 m acceptance disk at launch; require some along-track
                    // progress on the first segment.
                    const bool reachedWithProgress =
                        los.reached &&
                        (ctx->nav->index() > 0 || los.alongTrackM >= 0.5f);
                    if (los.valid && (reachedWithProgress || los.passed)) {
                        ctx->nav->advanceWaypoint();
                        ctx->heading->reset();
                        haveLastLosTarget = false;
                        if (!ctx->nav->missionComplete()) {
                            los = ctx->nav->computeLos(
                                fix.lat, fix.lon,
                                LOS_LOOKAHEAD_M, WAYPOINT_RADIUS_M, LOS_PASS_MARGIN_M);
                        } else {
                            los.valid = false;
                        }
                    }

                    if (los.valid && !ctx->nav->missionComplete()) {
                        Waypoint tgt = ctx->nav->currentTarget();
                        if (haveLastLosTarget &&
                            (tgt.lat != lastLosTarget.lat || tgt.lon != lastLosTarget.lon)) {
                            ctx->heading->reset();
                        }
                        lastLosTarget = tgt;
                        haveLastLosTarget = true;

                        float herr = HeadingController::computeHeadingError(
                            los.bearingDeg, heading);
                        int base = chooseBasePwm(herr, los.distToEndM, MOTOR_CRUISE_PWM);
                        ctx->heading->calculateThrusterOutputs(
                            los.bearingDeg, heading, base, dt, leftPwm, rightPwm);
                    }
                }
                ctx->motor->writeThrust(leftPwm, rightPwm);
                break;
            }

            case STATE_SPIRAL_MAPPING: {
                if (!g_spiralInitialized) {
                    if (fix.valid) {
                        ctx->spiralPlanner->begin(fix.lat, fix.lon);
                        g_spiralInitialized = true;
                        g_haveSpiralTgt = false;
                    } else {
                        ctx->motor->stop();
                        break;
                    }
                }

                if (g_spiralInitialized &&
                    (!g_haveSpiralTgt || (fix.valid &&
                        Navigator::calculateDistance(fix.lat, fix.lon,
                            g_spiralTgt.lat, g_spiralTgt.lon) <= SPIRAL_WAYPOINT_RADIUS_M))) {
                    g_haveSpiralTgt = ctx->spiralPlanner->nextSpiralPoint(g_spiralTgt);
                    if (!g_haveSpiralTgt) {
                        spiralDone = true;
                    } else {
                        ctx->heading->reset();
                    }
                }

                if (g_haveSpiralTgt && navSensorsOk) {
                    float bearing = (float)Navigator::calculateBearing(
                        fix.lat, fix.lon, g_spiralTgt.lat, g_spiralTgt.lon);
                    float herr = HeadingController::computeHeadingError(bearing, heading);
                    float dist = (float)Navigator::calculateDistance(
                        fix.lat, fix.lon, g_spiralTgt.lat, g_spiralTgt.lon);
                    int base = chooseBasePwm(herr, dist, MOTOR_MIN_PWM + 30);
                    ctx->heading->calculateThrusterOutputs(
                        bearing, heading, base, dt, leftPwm, rightPwm);
                }
                ctx->motor->writeThrust(leftPwm, rightPwm);
                break;
            }

            case STATE_RTH: {
                if (g_homeSet && navSensorsOk) {
                    const float dist = isfinite(distHomeM) ? distHomeM :
                        (float)Navigator::calculateDistance(
                            fix.lat, fix.lon, g_homeLat, g_homeLon);
                    // Started RTH already inside 2 m: hold mode 3, do not spin.
                    if (isfinite(g_rthEntryDistM) &&
                        g_rthEntryDistM <= RTH_ARRIVE_RADIUS_M &&
                        dist <= RTH_ARRIVE_RADIUS_M) {
                        leftPwm = 0;
                        rightPwm = 0;
                    } else {
                        float bearing = (float)Navigator::calculateBearing(
                            fix.lat, fix.lon, g_homeLat, g_homeLon);
                        float herr = HeadingController::computeHeadingError(
                            bearing, heading);
                        int base = chooseBasePwm(herr, dist, MOTOR_CRUISE_PWM);
                        ctx->heading->calculateThrusterOutputs(
                            bearing, heading, base, dt, leftPwm, rightPwm);
                    }
                }
                ctx->motor->writeThrust(leftPwm, rightPwm);
                break;
            }
        }

        // GPS safe-stop / fault: stop motors regardless of state.
        if (gpsTier == GpsLossTier::SAFE_STOP ||
            gpsTier == GpsLossTier::GPS_FAULT ||
            !fix.valid ||
            !compassOk) {
            ctx->motor->stop();
            leftPwm = 0;
            rightPwm = 0;
        }

        // Track propulsion for battery sag + anomaly disturbance gating.
        const int absL = leftPwm < 0 ? -leftPwm : leftPwm;
        const int absR = rightPwm < 0 ? -rightPwm : rightPwm;
        lastAbsPwm = absL > absR ? absL : absR;
        if (lastAbsPwm >= BATTERY_LOAD_PWM_THRESH) {
            lastHardThrustMs = millis();
            hadHardThrust = true;
        }

        // ---- 8. Publish telemetry snapshot (under mutex) ----
        TelemetryPacket logRec;
        memset(&logRec, 0, sizeof(logRec));
        bool snapshotPublished = false;
        if (xSemaphoreTake(ctx->stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            ctx->snapshot.gps_lat         = fix.valid ? fix.lat : NAN;
            ctx->snapshot.gps_lon         = fix.valid ? fix.lon : NAN;
            ctx->snapshot.water_ph        = phR.valid ? phR.value : NAN;
            ctx->snapshot.water_turbidity = turbR.valid ? turbR.value : NAN;
            ctx->snapshot.water_temp      = ctx->water->tempValid() ? temp : NAN;
            ctx->snapshot.current_mode    = (uint8_t)state;
            ctx->snapshot.battery_percent = ctx->battery->getBatteryPercent();
            ctx->snapshot.heading_angle   = compassOk ? heading : NAN;
            ctx->batteryVoltage            = battV;
            ctx->turbVoltage               = ctx->water->lastTurbVoltage();
            ctx->compassAddr               = ctx->compass->i2cAddress();
            ctx->compassFailCount          = ctx->compass->failCount();
            ctx->compassFresh              = ctx->compass->isFresh() ? 1 : 0;
            ctx->compassRawX               = ctx->compass->rawX();
            ctx->compassRawY               = ctx->compass->rawY();
            ctx->compassNvs                = ctx->compass->nvsLoaded() ? 1 : 0;
            ctx->compassCalRun             = ctx->compass->isCalibrating() ? 1 : 0;
            ctx->homeDistM                 = distHomeM;
            ctx->homeSetFlag               = g_homeSet ? 1 : 0;
            logRec = ctx->snapshot;
            // SD records have their own monotonic 20 Hz sequence. ESP-NOW
            // sequence remains owned exclusively by ioTask.
            logRec.sequence_number = ++logSequence;
            snapshotPublished = true;
            xSemaphoreGive(ctx->stateMutex);
        }

        if (snapshotPublished && ctx->logger &&
            xQueueSend(ctx->logQueue, &logRec, 0) != pdTRUE) {
            ctx->logDropCount++;
        }

        prevState = state;
        ctx->wdt->feed();
        vTaskDelayUntil(&lastWake, period);
    }
}

// ---------------------------------------------------------------------------
// Core 0 — comms & storage
// ---------------------------------------------------------------------------
void ioTask(void* pv) {
    SystemContext* ctx = static_cast<SystemContext*>(pv);
    ctx->wdt->subscribeCurrentTask();

    unsigned long lastTelemetry = 0;
    unsigned long lastBattReport = 0;
    unsigned long lastSdCheck = 0;
    unsigned long lastRadioDiag = 0;

    for (;;) {
        pollCalButton(ctx);
        pollSerialCommands(ctx);

        // ---- Drain inbound command queue ----
        CommandPacket cmdPkt;
        while (EspNowLink::commandQueue) {
            // Acquire state storage before dequeueing so a mutex timeout can
            // never discard a command.
            if (xSemaphoreTake(ctx->stateMutex, pdMS_TO_TICKS(10)) != pdTRUE) {
                break;
            }
            if (xQueueReceive(EspNowLink::commandQueue, &cmdPkt, 0) != pdTRUE) {
                xSemaphoreGive(ctx->stateMutex);
                break;
            }

            switch (cmdPkt.sys_command) {
                case CMD_EMERGENCY_STOP:
                    ctx->command.emergency = true;
                    ctx->command.start = false;
                    ctx->command.clearEstop = false;
                    ctx->command.manualMode = false;
                    ctx->command.manualSpeed = 0;
                    ctx->command.manualSteer = 0;
                    break;
                case CMD_FORCE_RTH:
                    ctx->command.forceRth = true;
                    ctx->command.emergency = false;
                    break;
                case CMD_START_ARM:
                    ctx->command.start = true;
                    break;
                case CMD_CLEAR_ESTOP:
                    ctx->command.clearEstop = true;
                    ctx->command.emergency = false;
                    ctx->command.forceRth = false;
                    ctx->command.start = false;
                    ctx->command.forceSpiral = false;
                    ctx->command.manualMode = false;
                    ctx->command.manualSpeed = 0;
                    ctx->command.manualSteer = 0;
                    break;
                case CMD_MISSION_UPLOAD_DONE:
                    ctx->command.sealMissionUpload = true;
                    break;
                case CMD_SET_HOME:
                    ctx->command.setHome = true;
                    break;
                case CMD_FORCE_SPIRAL:
                    ctx->command.forceSpiral = true;
                    break;
                case CMD_NORMAL:
                    ctx->command.forceRth = false;
                    // Manual sticks apply only on CMD_NORMAL (not START/CLEAR/…).
                    {
                        const int manualSpeed =
                            constrain((int)cmdPkt.manual_speed, -255, 255);
                        const int manualSteer =
                            constrain((int)cmdPkt.manual_steer, -255, 255);
                        if (manualSpeed != 0 || manualSteer != 0) {
                            ctx->command.manualMode  = true;
                            ctx->command.manualSpeed = manualSpeed;
                            ctx->command.manualSteer = manualSteer;
                            ctx->command.lastManualMs = millis();
                        } else {
                            ctx->command.manualMode = false;
                            ctx->command.manualSpeed = 0;
                            ctx->command.manualSteer = 0;
                        }
                    }
                    break;
                default:
                    xSemaphoreGive(ctx->stateMutex);
#if RADIO_TRACE
                    Serial.printf("[RX] CMD ?(%u) steer=%d speed=%d (ignored)\n",
                                  (unsigned)cmdPkt.sys_command,
                                  (int)cmdPkt.manual_steer,
                                  (int)cmdPkt.manual_speed);
#endif
                    continue;
            }
            xSemaphoreGive(ctx->stateMutex);
#if RADIO_TRACE
            Serial.printf("[RX] CMD %s(%u) steer=%d speed=%d\n",
                          radioCmdName(cmdPkt.sys_command),
                          (unsigned)cmdPkt.sys_command,
                          (int)cmdPkt.manual_steer,
                          (int)cmdPkt.manual_speed);
#endif
        }

        // ---- Drain ACK queue from nav -> shore ----
        WaypointACKPacket ack;
        while (EspNowLink::ackQueue &&
               xQueueReceive(EspNowLink::ackQueue, &ack, 0) == pdTRUE) {
            const bool ok = ctx->link->sendAck(ack);
#if RADIO_TRACE
            Serial.printf("[TX] ACK %s idx=%u status=%u %s\n",
                          ok ? "ok" : "FAIL",
                          (unsigned)ack.packet_index,
                          (unsigned)ack.ack_status,
                          EspNowLink::describeLastTx());
#else
            (void)ok;
#endif
        }

        // ---- Drain stage/intent events from nav -> shore ----
        StageStatusPacket stageStatus;
        while (ctx->stageQueue &&
               xQueueReceive(ctx->stageQueue, &stageStatus, 0) == pdTRUE) {
            if (!ctx->link->sendStageStatus(stageStatus)) {
                ctx->stageDropCount++;
#if RADIO_TRACE
                Serial.printf(
                    "[TX] STAGE FAIL seq=%lu stage=%u intent=%u next=%u reason=%u %s\n",
                    (unsigned long)stageStatus.sequence_number,
                    (unsigned)stageStatus.current_stage,
                    (unsigned)stageStatus.intent,
                    (unsigned)stageStatus.next_stage,
                    (unsigned)stageStatus.reason,
                    EspNowLink::describeLastTx());
#endif
            }
#if RADIO_TRACE
            else {
                Serial.printf(
                    "[TX] STAGE ok seq=%lu stage=%u intent=%u next=%u reason=%u "
                    "flags=0x%02X wp=%u/%u batch=%u %s\n",
                    (unsigned long)stageStatus.sequence_number,
                    (unsigned)stageStatus.current_stage,
                    (unsigned)stageStatus.intent,
                    (unsigned)stageStatus.next_stage,
                    (unsigned)stageStatus.reason,
                    (unsigned)stageStatus.flags,
                    (unsigned)stageStatus.waypoint_index,
                    (unsigned)stageStatus.waypoint_count,
                    (unsigned)stageStatus.expected_batch_index,
                    EspNowLink::describeLastTx());
            }
#endif
        }

        // ---- Periodic telemetry TX (1 Hz even if a send fails — do not spin) ----
        if (!ctx->radioTxPaused &&
            millis() - lastTelemetry >= TELEMETRY_INTERVAL_MS) {
            lastTelemetry = millis();
            TelemetryPacket tp;
            float turbV = NAN;
            if (xSemaphoreTake(ctx->stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                tp = ctx->snapshot;
                turbV = ctx->turbVoltage;
                xSemaphoreGive(ctx->stateMutex);
                if (!radioSendsWaterQuality(tp.current_mode)) {
                    tp.water_ph = NAN;
                    tp.water_turbidity = NAN;
                    tp.water_temp = NAN;
                }
                const bool ok = ctx->link->sendTelemetry(tp);
                if (ok) {
                    // Mirror stamped seq back into telemetry snapshot only.
                    if (xSemaphoreTake(ctx->stateMutex,
                                       pdMS_TO_TICKS(5)) == pdTRUE) {
                        ctx->snapshot.sequence_number = tp.sequence_number;
                        xSemaphoreGive(ctx->stateMutex);
                    }
                }
#if RADIO_TRACE
                Serial.printf(
                    "[TX] TEL %s seq=%lu lat=%.6f lon=%.6f ph=%.2f ntu=%.1f "
                    "turbV=%.3f t=%.1f mode=%u batt=%u hdg=%.1f %s\n",
                    ok ? "ok" : "FAIL",
                    (unsigned long)tp.sequence_number,
                    tp.gps_lat, tp.gps_lon,
                    tp.water_ph, tp.water_turbidity,
                    turbV, tp.water_temp,
                    (unsigned)tp.current_mode,
                    (unsigned)tp.battery_percent,
                    tp.heading_angle,
                    EspNowLink::describeLastTx());
#endif
            }
        }

        // ---- Optional battery report (same cadence as TELEMETRY: do not spin) ----
        // Advancing only on TX success flooded ESP-NOW + Serial when the peer
        // was down (~20 ms retries after the first 5 min window).
        if (!ctx->radioTxPaused &&
            millis() - lastBattReport >= BATTERY_REPORT_MS) {
            BatteryReportPacket br;
            br.report_timestamp = millis();
            br.current_draw_ma  = 0.0f;  // no ACS712
            br.estimated_tte_min = NAN;  // no current sensor -> cannot estimate
            bool haveBattery = false;
            if (xSemaphoreTake(ctx->stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                br.battery_voltage = ctx->batteryVoltage;
                br.state_of_charge = (float)ctx->snapshot.battery_percent;
                haveBattery = true;
                xSemaphoreGive(ctx->stateMutex);
            }
            if (haveBattery) {
                lastBattReport = millis();
                const bool ok = ctx->link->sendBatteryReport(br);
#if RADIO_TRACE
                Serial.printf("[TX] BATT %s V=%.2f soc=%.0f %s\n",
                              ok ? "ok" : "FAIL",
                              br.battery_voltage, br.state_of_charge,
                              EspNowLink::describeLastTx());
#else
                (void)ok;
#endif
            }
        }

        // ---- Drain log queue to SD (optional; SD may be absent) ----
        TelemetryPacket logRec;
        while (xQueueReceive(ctx->logQueue, &logRec, 0) == pdTRUE) {
            if (ctx->logger && ctx->logger->isReady()) {
                if (!ctx->logger->logRecord(logRec)) {
                    ctx->logDropCount++;
                }
            }
        }

        // Periodic SD health check — mark logger not-ready if card vanished.
        if (ctx->logger &&
            millis() - lastSdCheck >= SD_INTEGRITY_MS) {
            lastSdCheck = millis();
            if (!ctx->logger->checkIntegrity()) {
                Serial.println(F("[USV] SD integrity FAIL — logging paused"));
            }
        }

        if (millis() - lastRadioDiag >= RADIO_DIAG_MS) {
            lastRadioDiag = millis();
#if RADIO_TRACE
            const uint8_t* peer = EspNowLink::peerMac();
            Serial.printf(
                "[USV] radio tx=%s rxDrop=%lu (mac=%lu len=%lu cmd=%lu) rxOk=%lu "
                "isrDrop=%lu txFail=%lu ackDrop=%lu stageDrop=%lu logDrop=%lu %s "
                "peer=%02X:%02X:%02X:%02X:%02X:%02X\n",
                ctx->radioTxPaused ? "PAUSE" : "ON",
                (unsigned long)EspNowLink::droppedPackets,
                (unsigned long)EspNowLink::dropMac,
                (unsigned long)EspNowLink::dropLen,
                (unsigned long)EspNowLink::dropCmd,
                (unsigned long)EspNowLink::rxAcceptCount,
                (unsigned long)EspNowLink::droppedIsrPackets,
                (unsigned long)EspNowLink::txFailCount,
                (unsigned long)ctx->ackDropCount,
                (unsigned long)ctx->stageDropCount,
                (unsigned long)ctx->logDropCount,
                EspNowLink::describeLastTx(),
                peer[0], peer[1], peer[2], peer[3], peer[4], peer[5]);
            if (EspNowLink::lastTx.result != EspNowLink::TX_RESULT_OK) {
                Serial.printf("[USV] radio hint: %s\n", EspNowLink::pairingHint());
            }
            uint8_t cAddr = 0, cFail = 0, cFresh = 0, cNvs = 0, cCal = 0;
            int16_t cX = 0, cY = 0;
            float hdg = NAN;
            float homeD = NAN;
            uint8_t homeSet = 0;
            if (xSemaphoreTake(ctx->stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                cAddr  = ctx->compassAddr;
                cFail  = ctx->compassFailCount;
                cFresh = ctx->compassFresh;
                cX     = ctx->compassRawX;
                cY     = ctx->compassRawY;
                cNvs   = ctx->compassNvs;
                cCal   = ctx->compassCalRun;
                hdg    = ctx->snapshot.heading_angle;
                homeD  = ctx->homeDistM;
                homeSet = ctx->homeSetFlag;
                xSemaphoreGive(ctx->stateMutex);
            }
            Serial.printf(
                "[USV] compass addr=0x%02X fresh=%u fail=%u raw=%d,%d hdg=%.1f cal=%s "
                "homeDist=%.1fm set=%u\n",
                (unsigned)cAddr, (unsigned)cFresh, (unsigned)cFail,
                (int)cX, (int)cY, hdg,
                cCal ? "RUN" : (cNvs ? "NVS" : "FACT"),
                homeD, (unsigned)homeSet);
#endif
        }

        ctx->wdt->feed();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
