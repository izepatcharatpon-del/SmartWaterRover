#include "core/Tasks.h"
#include "config.h"
#include <math.h>
#include <string.h>

// Home position — captured on every STANDBY -> GRID_PATROL with valid GPS.
static double g_homeLat = 0.0, g_homeLon = 0.0;
static bool   g_homeSet = false;

// Spiral chase target.
static Waypoint g_spiralTgt = {0.0, 0.0};
static bool     g_haveSpiralTgt = false;
static bool     g_spiralInitialized = false;

static int chooseBasePwm(float headingErrorDeg, float distToEndM, int cruisePwm) {
    float absErr = fabsf(headingErrorDeg);
    if (absErr > LOS_TURN_FIRST_DEG) return 0;
    if (distToEndM < LOS_NEAR_SLOW_M) return MOTOR_APPROACH_PWM;
    return cruisePwm;
}

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
        if (waterAdsDiv == 0) {
            phR   = ctx->water->readPh(temp);
            turbR = ctx->water->readTurbidityNtu();
            const bool sensorsValid =
                phR.valid && turbR.valid && ctx->water->tempValid();
            anomaly = ctx->anomaly->updateReadings(
                phR.value, turbR.value, temp, sensorsValid, hour);
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

        // Seal multi-batch upload when shore says the last full (12/12) batch is done.
        if (cmd.sealMissionUpload) {
            ctx->nav->sealMissionUpload();
            if (xSemaphoreTake(ctx->stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
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

        // ---- 5. Safety (AA118 voltage only) ----
        SafetyState safety = ctx->safety->evaluate(battV);

        // ---- 6. Mission FSM ----
        GpsLossTier gpsTier = ctx->gps->getLossTier();
        bool gpsOk = (gpsTier == GpsLossTier::OK) && fix.valid;

        FsmInputs in;
        memset(&in, 0, sizeof(in));
        in.startCommand     = cmd.start;
        in.clearEstop       = cmd.clearEstop;
        in.emergencyCommand = cmd.emergency;
        in.forceRthCommand  = cmd.forceRth;
        in.anomalyDetected  = anomaly;
        in.spiralComplete   = spiralDone;
        in.gridComplete     = ctx->nav->missionComplete();
        in.arrivedHome      = g_homeSet && fix.valid &&
            (Navigator::calculateDistance(fix.lat, fix.lon, g_homeLat, g_homeLon)
                 <= WAYPOINT_RADIUS_M);
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
        if ((cmd.start && (startAccepted || startUnsafe)) ||
            (cmd.clearEstop && clearAccepted)) {
            if (xSemaphoreTake(ctx->stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                if (cmd.start && (startAccepted || startUnsafe))
                    ctx->command.start = false;
                if (cmd.clearEstop && clearAccepted) {
                    ctx->command.clearEstop = false;
                    // Clear dedicated E-Stop event only after FSM left ESTOP.
                    EspNowLink::clearEmergencyPending();
                }
                xSemaphoreGive(ctx->stateMutex);
            }
        }

        // On entering E-Stop: clear start/manual/clear so we cannot auto-resume
        // or auto-clear from a previously latched CLEAR_ESTOP.
        if (state == STATE_EMERGENCY_STOP && prevState != STATE_EMERGENCY_STOP) {
            if (xSemaphoreTake(ctx->stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                ctx->command.start = false;
                ctx->command.clearEstop = false;
                ctx->command.manualMode = false;
                ctx->command.manualSpeed = 0;
                ctx->command.manualSteer = 0;
                xSemaphoreGive(ctx->stateMutex);
            }
            ctx->heading->reset();
        }

        // Capture home on STANDBY -> GRID_PATROL (every mission).
        if (state == STATE_GRID_PATROL && prevState == STATE_STANDBY) {
            if (fix.valid) {
                g_homeLat = fix.lat;
                g_homeLon = fix.lon;
                g_homeSet = true;
                ctx->nav->setMissionOrigin(fix.lat, fix.lon);
            } else {
                // Should not happen (FSM requires gpsOk); stop as failsafe.
                ctx->motor->stop();
            }
            ctx->heading->reset();
            haveLastLosTarget = false;
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
                    float bearing = (float)Navigator::calculateBearing(
                        fix.lat, fix.lon, g_homeLat, g_homeLon);
                    float herr = HeadingController::computeHeadingError(bearing, heading);
                    float dist = (float)Navigator::calculateDistance(
                        fix.lat, fix.lon, g_homeLat, g_homeLon);
                    int base = chooseBasePwm(herr, dist, MOTOR_CRUISE_PWM);
                    ctx->heading->calculateThrusterOutputs(
                        bearing, heading, base, dt, leftPwm, rightPwm);
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
        }

        // ---- 8. Publish telemetry snapshot (under mutex) ----
        TelemetryPacket logRec;
        memset(&logRec, 0, sizeof(logRec));
        bool snapshotPublished = false;
        if (xSemaphoreTake(ctx->stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            ctx->snapshot.gps_lat         = fix.lat;
            ctx->snapshot.gps_lon         = fix.lon;
            ctx->snapshot.water_ph        = phR.valid ? phR.value : NAN;
            ctx->snapshot.water_turbidity = turbR.valid ? turbR.value : NAN;
            ctx->snapshot.water_temp      = ctx->water->tempValid() ? temp : NAN;
            ctx->snapshot.current_mode    = (uint8_t)state;
            ctx->snapshot.battery_percent = ctx->battery->getBatteryPercent();
            ctx->snapshot.heading_angle   = compassOk ? heading : NAN;
            ctx->batteryVoltage            = battV;
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

    for (;;) {
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
                    ctx->command.manualMode = false;
                    ctx->command.manualSpeed = 0;
                    ctx->command.manualSteer = 0;
                    break;
                case CMD_MISSION_UPLOAD_DONE:
                    ctx->command.sealMissionUpload = true;
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
                    continue;
            }
            xSemaphoreGive(ctx->stateMutex);
        }

        // ---- Drain ACK queue from nav -> shore ----
        WaypointACKPacket ack;
        while (EspNowLink::ackQueue &&
               xQueueReceive(EspNowLink::ackQueue, &ack, 0) == pdTRUE) {
            ctx->link->sendAck(ack);
        }

        // ---- Periodic telemetry TX ----
        if (millis() - lastTelemetry >= TELEMETRY_INTERVAL_MS) {
            TelemetryPacket tp;
            if (xSemaphoreTake(ctx->stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                tp = ctx->snapshot;
                xSemaphoreGive(ctx->stateMutex);
                if (ctx->link->sendTelemetry(tp)) {
                    // Mirror stamped seq back into telemetry snapshot only.
                    if (xSemaphoreTake(ctx->stateMutex,
                                       pdMS_TO_TICKS(5)) == pdTRUE) {
                        ctx->snapshot.sequence_number = tp.sequence_number;
                        xSemaphoreGive(ctx->stateMutex);
                    }
                    lastTelemetry = millis();
                }
            }
        }

        // ---- Optional battery report ----
        if (millis() - lastBattReport >= BATTERY_REPORT_MS) {
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
            if (haveBattery && ctx->link->sendBatteryReport(br)) {
                lastBattReport = millis();
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

        ctx->wdt->feed();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
