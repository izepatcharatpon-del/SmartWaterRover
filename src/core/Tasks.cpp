#include "core/Tasks.h"
#include "config.h"
#include <math.h>

// Rough current estimate from commanded thrust (no INA219 on this build; optional ACS712).
// Peak ~30 A so SafetyManager's OVERCURRENT_LIMIT_A (25 A) can actually trip.
static float estimateCurrentA(int leftPwm, int rightPwm) {
    float frac = (fabsf((float)leftPwm) + fabsf((float)rightPwm)) / (2.0f * 255.0f);
    return frac * 30.0f;
}

// Home position captured when the mission first leaves STANDBY.
static double g_homeLat = 0.0, g_homeLon = 0.0;
static bool   g_homeSet = false;

// Spiral chase target (reset on each entry to STATE_SPIRAL_MAPPING).
static Waypoint g_spiralTgt = {0.0, 0.0};
static bool     g_haveSpiralTgt = false;

// ---------------------------------------------------------------------------
// Core 1 — navigation & control loop
// ---------------------------------------------------------------------------
void navigationTask(void* pv) {
    SystemContext* ctx = static_cast<SystemContext*>(pv);
    ctx->wdt->subscribeCurrentTask();

    BoatState prevState = STATE_STANDBY;
    bool spiralDone = false;

    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(NAV_LOOP_PERIOD_MS);
    const float dt = NAV_LOOP_PERIOD_MS / 1000.0f;

    for (;;) {
        // ---- 1. Sensor acquisition ----
        ctx->gps->update();
        ctx->compass->update();
        ctx->water->updateTemperatureNonBlocking();

        GpsFix fix     = ctx->gps->getFilteredFix();
        float  heading = ctx->compass->getHeadingDeg();
        float  temp    = ctx->water->getWaterTemp();
        float  ph      = ctx->water->readPh(temp);
        float  turb    = ctx->water->readTurbidityNtu();
        float  battV   = ctx->battery->readVoltage();

        // ---- 2. Anomaly detection ----
        bool anomaly = ctx->anomaly->updateReadings(ph, turb, temp, 12);

        // ---- 3. Snapshot operator command ----
        CommandState cmd;
        if (xSemaphoreTake(ctx->stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            cmd = ctx->command;
            xSemaphoreGive(ctx->stateMutex);
        } else {
            memset(&cmd, 0, sizeof(cmd));
        }

        // ---- 4. Safety evaluation ----
        // Use the PREVIOUS loop's thrust to estimate current, breaking the
        // safety -> FSM -> motor -> current circular dependency.
        static int prevLeft = 0, prevRight = 0;
        float currentA = estimateCurrentA(prevLeft, prevRight);
        ctx->battery->updateCoulombCounter(currentA * 1000.0f);
        SafetyState safety = ctx->safety->evaluate(battV, currentA, /*leak=*/false);

        // ---- 5. Mission FSM ----
        FsmInputs in;
        in.startCommand     = cmd.start;
        in.emergencyCommand = cmd.emergency;
        in.forceRthCommand  = cmd.forceRth;
        in.anomalyDetected  = anomaly;
        in.spiralComplete   = spiralDone;
        in.gridComplete     = ctx->nav->missionComplete();
        in.arrivedHome      = g_homeSet && fix.valid &&
            (Navigator::calculateDistance(fix.lat, fix.lon, g_homeLat, g_homeLon)
                 <= WAYPOINT_RADIUS_M);
        in.safety           = safety;

        BoatState state = ctx->fsm->tick(in);

        // Mission finished at home: drop START so we do not re-arm GRID_PATROL.
        if (state == STATE_STANDBY && prevState == STATE_RTH) {
            if (xSemaphoreTake(ctx->stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                ctx->command.start = false;
                xSemaphoreGive(ctx->stateMutex);
            }
        }

        // Capture home the first time we start moving.
        if (state != STATE_STANDBY && !g_homeSet && fix.valid) {
            g_homeLat = fix.lat; g_homeLon = fix.lon; g_homeSet = true;
        }
        // Initialise the spiral generator on entry.
        if (state == STATE_SPIRAL_MAPPING && prevState != STATE_SPIRAL_MAPPING) {
            if (fix.valid) ctx->spiralPlanner->begin(fix.lat, fix.lon);
            spiralDone = false;
            g_haveSpiralTgt = false;
        }
        if (state != STATE_SPIRAL_MAPPING) spiralDone = false;

        // ---- 6. Act on state ----
        int leftPwm = 0, rightPwm = 0;
        switch (state) {
            case STATE_STANDBY:
            case STATE_EMERGENCY_STOP:
                ctx->motor->stop();
                ctx->heading->reset();
                break;

            case STATE_GRID_PATROL: {
                if (cmd.manualMode) {
                    leftPwm  = cmd.manualSpeed + cmd.manualSteer;
                    rightPwm = cmd.manualSpeed - cmd.manualSteer;
                } else if (ctx->nav->hasWaypoints() && fix.valid &&
                           !ctx->nav->missionComplete()) {
                    // Advance first, then steer to the *new* target (avoid
                    // one loop chasing the waypoint that was just reached).
                    if (ctx->nav->isWaypointReached(fix.lat, fix.lon, WAYPOINT_RADIUS_M))
                        ctx->nav->advanceWaypoint();
                    if (!ctx->nav->missionComplete()) {
                        Waypoint tgt = ctx->nav->currentTarget();
                        float bearing = Navigator::calculateBearing(fix.lat, fix.lon,
                                                                    tgt.lat, tgt.lon);
                        ctx->heading->calculateThrusterOutputs(bearing, heading,
                                                               MOTOR_CRUISE_PWM, dt,
                                                               leftPwm, rightPwm);
                    }
                }
                ctx->motor->writeThrust(leftPwm, rightPwm);
                break;
            }

            case STATE_SPIRAL_MAPPING: {
                if (!g_haveSpiralTgt || (fix.valid &&
                        Navigator::calculateDistance(fix.lat, fix.lon,
                            g_spiralTgt.lat, g_spiralTgt.lon) <= WAYPOINT_RADIUS_M)) {
                    g_haveSpiralTgt = ctx->spiralPlanner->nextSpiralPoint(g_spiralTgt);
                    if (!g_haveSpiralTgt) spiralDone = true;
                }
                if (g_haveSpiralTgt && fix.valid) {
                    float bearing = Navigator::calculateBearing(fix.lat, fix.lon,
                                                                g_spiralTgt.lat, g_spiralTgt.lon);
                    ctx->heading->calculateThrusterOutputs(bearing, heading,
                                                           MOTOR_MIN_PWM + 30, dt,
                                                           leftPwm, rightPwm);
                }
                ctx->motor->writeThrust(leftPwm, rightPwm);
                break;
            }

            case STATE_RTH: {
                if (g_homeSet && fix.valid) {
                    float bearing = Navigator::calculateBearing(fix.lat, fix.lon,
                                                                g_homeLat, g_homeLon);
                    ctx->heading->calculateThrusterOutputs(bearing, heading,
                                                           MOTOR_CRUISE_PWM, dt,
                                                           leftPwm, rightPwm);
                }
                ctx->motor->writeThrust(leftPwm, rightPwm);
                break;
            }
        }

        // GPS safe-float: if fix is stale, stop motors regardless of state.
        GpsLossTier tier = ctx->gps->getLossTier();
        if (tier == GpsLossTier::SAFE_FLOAT || tier == GpsLossTier::EMERGENCY_RTH) {
            ctx->motor->stop();
        }

        // ---- 7. Publish telemetry snapshot ----
        if (xSemaphoreTake(ctx->stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            ctx->snapshot.gps_lat         = fix.lat;
            ctx->snapshot.gps_lon         = fix.lon;
            ctx->snapshot.water_ph        = ph;
            ctx->snapshot.water_turbidity = turb;
            ctx->snapshot.water_temp      = temp;
            ctx->snapshot.current_mode    = (uint8_t)state;
            ctx->snapshot.battery_percent = ctx->battery->getBatteryPercent();
            ctx->snapshot.heading_angle   = heading;
            xSemaphoreGive(ctx->stateMutex);
        }

        // ---- 8. Log (non-blocking hand-off to IO task) ----
        TelemetryPacket logRec = ctx->snapshot;
        xQueueSend(ctx->logQueue, &logRec, 0);

        prevLeft = leftPwm; prevRight = rightPwm;
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

    for (;;) {
        // ---- Drain inbound command queue ----
        CommandPacket cmdPkt;
        while (xQueueReceive(EspNowLink::commandQueue, &cmdPkt, 0) == pdTRUE) {
            if (xSemaphoreTake(ctx->stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                ctx->command.emergency = (cmdPkt.sys_command == CMD_EMERGENCY_STOP);
                ctx->command.forceRth  = (cmdPkt.sys_command == CMD_FORCE_RTH);
                if (cmdPkt.manual_speed != 0 || cmdPkt.manual_steer != 0) {
                    ctx->command.manualMode  = true;
                    ctx->command.manualSpeed = cmdPkt.manual_speed;
                    ctx->command.manualSteer = cmdPkt.manual_steer;
                } else {
                    ctx->command.manualMode = false;
                }
                xSemaphoreGive(ctx->stateMutex);
            }
        }

        // ---- Drain inbound waypoint queue (+ ACK) ----
        WaypointArrayPacket wps;
        while (xQueueReceive(EspNowLink::waypointQueue, &wps, 0) == pdTRUE) {
            bool ok = (wps.packet_index == 0) ? (ctx->nav->loadWaypoints(wps), true)
                                              : ctx->nav->appendBatch(wps);
            WaypointACKPacket ack;
            ack.packet_index = wps.packet_index;
            ack.ack_status   = ok ? ACK_OK : ACK_NACK;
            ctx->link->sendAck(ack);

            // Receiving a mission implicitly arms START.
            if (ok && xSemaphoreTake(ctx->stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                ctx->command.start = true;
                xSemaphoreGive(ctx->stateMutex);
            }
        }

        // ---- Periodic telemetry TX ----
        if (millis() - lastTelemetry >= TELEMETRY_INTERVAL_MS) {
            TelemetryPacket tp;
            if (xSemaphoreTake(ctx->stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                tp = ctx->snapshot;
                xSemaphoreGive(ctx->stateMutex);
                ctx->link->sendTelemetry(tp);   // stamps sequence_number
            }
            lastTelemetry = millis();
        }

        // ---- Drain log queue to SD ----
        TelemetryPacket logRec;
        while (xQueueReceive(ctx->logQueue, &logRec, 0) == pdTRUE) {
            ctx->logger->logRecord(logRec);
        }

        ctx->wdt->feed();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
