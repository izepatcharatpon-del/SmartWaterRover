#include "core/Tasks.h"
#include "core/StageReporter.h"
#include "core/WaterSamplePolicy.h"
#include "core/DiagnosticFormatting.h"
#include "control/DriveController.h"
#include "control/GridHeadingController.h"
#include "config.h"
#include <Preferences.h>
#include <math.h>
#include <string.h>

static double g_homeLat = 0.0, g_homeLon = 0.0;
static bool   g_homeSet = false;
static bool   g_homeCapturing = false;
static uint32_t g_homeCapStartMs = 0;
static uint32_t g_homeCapLastSampleMs = 0;
static MissionFsm::HomeCapture g_homeCap = {};
static bool   g_homeLeft = false;
static uint32_t g_homeOutsideAccumMs = 0;
static float    g_rthEntryDistM = NAN;
static uint32_t g_rthEnterMs = 0;
static uint32_t g_rthInsideAccumMs = 0;

static const char* HOME_NVS_NS  = "usv_home";
static const char* HOME_NVS_LAT = "lat";
static const char* HOME_NVS_LON = "lon";
static const char* HOME_NVS_OK  = "ok";

static bool homeCoordsSane(double lat, double lon) {
    if (!isfinite(lat) || !isfinite(lon)) return false;
    if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) return false;
    if (lat == 0.0 && lon == 0.0) return false;
    return true;
}

static void saveHomeToNvs(double lat, double lon) {
    Preferences prefs;
    if (!prefs.begin(HOME_NVS_NS, false)) {
        Serial.println(F("[USV] SET_HOME NVS write failed"));
        return;
    }
    prefs.putDouble(HOME_NVS_LAT, lat);
    prefs.putDouble(HOME_NVS_LON, lon);
    prefs.putUChar(HOME_NVS_OK, 1);
    prefs.end();
}

static bool loadHomeFromNvs(double& lat, double& lon) {
    Preferences prefs;
    if (!prefs.begin(HOME_NVS_NS, true)) return false;
    const bool ok = prefs.getUChar(HOME_NVS_OK, 0) != 0;
    lat = prefs.getDouble(HOME_NVS_LAT, 0.0);
    lon = prefs.getDouble(HOME_NVS_LON, 0.0);
    prefs.end();
    return ok && homeCoordsSane(lat, lon);
}

static Waypoint g_spiralTgt = {0.0, 0.0};
static Waypoint g_spiralStart = {0.0, 0.0};
static bool     g_haveSpiralTgt = false;
static bool     g_spiralInitialized = false;
static uint32_t g_spiralEnterMs = 0;
static uint32_t g_spiralPointMs = 0;
static uint32_t g_spiralCleanSinceMs = 0;
static bool     g_spiralFromForce = false;
static const char* g_spiralAbortWhy = nullptr;
static bool     g_waterSampleActive = false;
static uint32_t g_waterSampleStartMs = 0;
static float    g_phQuietSum = 0.0f;
static uint16_t g_phQuietCount = 0;

static int chooseBasePwm(float headingErrorDeg, float distToEndM, int cruisePwm) {
    float absErr = fabsf(headingErrorDeg);
    if (absErr > LOS_TURN_FIRST_DEG) return 0;
    if (distToEndM < LOS_NEAR_SLOW_M) return MOTOR_APPROACH_PWM;
    return cruisePwm;
}

static uint8_t radioStatusForLog(EspNowLink::TxResult result) {
    switch (result) {
        case EspNowLink::TX_RESULT_OK:       return TELEMETRY_RADIO_ACK;
        case EspNowLink::TX_RESULT_NO_ACK:   return TELEMETRY_RADIO_NO_ACK;
        case EspNowLink::TX_RESULT_SEND_ERR: return TELEMETRY_RADIO_SEND_ERR;
        case EspNowLink::TX_RESULT_TIMEOUT:  return TELEMETRY_RADIO_TIMEOUT;
    }
    return TELEMETRY_RADIO_TIMEOUT;
}

static DiagnosticFormatting::PhReadReason phDiagnosticReason(PhSampleReason reason) {
    using DiagnosticFormatting::PhReadReason;
    switch (reason) {
        case PhSampleReason::OK: return PhReadReason::OK;
        case PhSampleReason::ADS_UNAVAILABLE: return PhReadReason::ADS_UNAVAILABLE;
        case PhSampleReason::ADC_INVALID: return PhReadReason::ADC_INVALID;
        case PhSampleReason::COMPUTATION_INVALID: return PhReadReason::COMPUTATION_INVALID;
        case PhSampleReason::ELECTRODE_RANGE: return PhReadReason::ELECTRODE_RANGE;
        case PhSampleReason::TEMP_INVALID: return PhReadReason::TEMP_INVALID;
        case PhSampleReason::PH_IMPLAUSIBLE: return PhReadReason::PH_IMPLAUSIBLE;
        default: return PhReadReason::NO_SAMPLE;
    }
}

static DiagnosticFormatting::TxResult txDiagnosticResult(EspNowLink::TxResult result) {
    switch (result) {
        case EspNowLink::TX_RESULT_OK: return DiagnosticFormatting::TxResult::ACK;
        case EspNowLink::TX_RESULT_SEND_ERR: return DiagnosticFormatting::TxResult::SEND_ERR;
        case EspNowLink::TX_RESULT_NO_ACK: return DiagnosticFormatting::TxResult::NO_ACK;
        case EspNowLink::TX_RESULT_TIMEOUT: return DiagnosticFormatting::TxResult::TIMEOUT;
        default: return DiagnosticFormatting::TxResult::UNKNOWN;
    }
}

static void toggleRadioTxPause(SystemContext* ctx) {
    static uint32_t lastMs = 0;
    const uint32_t now = millis();
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
    bool lastManualMode = false;
    unsigned long lastHardThrustMs = 0;
    int lastAbsPwm = 0;
    bool hadHardThrust = false;
    DriveController driveController(
        MOTOR_PWM_LIMIT, MOTOR_MIN_PWM, MOTOR_PWM_SLEW_PER_SEC);

    GridHeadingController gridHeading;

    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(NAV_LOOP_PERIOD_MS);
    uint32_t lastLoopUs = micros();

    if (loadHomeFromNvs(g_homeLat, g_homeLon)) {
        g_homeSet = true;
        g_homeLeft = false;
        Serial.printf("[USV] home from NVS lat=%.6f lon=%.6f\n",
                      g_homeLat, g_homeLon);
    }

    // ADS ~16 ms — do not run every 50 ms tick.
    uint8_t waterAdsDiv = 0;
    WaterReading phR   = {NAN, false};
    WaterReading turbR = {NAN, false};
    float lastPh = NAN;
    float lastNtu = NAN;
    float lastTemp = NAN;
    bool anomaly = false;

    for (;;) {
        uint32_t nowUs = micros();
        float dt = (nowUs - lastLoopUs) / 1000000.0f;
        lastLoopUs = nowUs;
        if (dt < 0.001f) dt = 0.001f;
        if (dt > 0.25f)  dt = 0.25f;

        ctx->gps->update();
        ctx->compass->update();
        const uint32_t compassSampleUs = micros();
        ctx->water->updateTemperatureNonBlocking();
        const bool adsWasReady = ctx->water->adsReady();
        if (ctx->water->pollAds(millis()) && !adsWasReady) {
            Serial.println(F("[USV] ADS1115 recovered"));
        }

        GpsFix fix     = ctx->gps->getFilteredFix();
        double rawGpsLat = NAN, rawGpsLon = NAN;
        ctx->gps->getRawPosition(rawGpsLat, rawGpsLon);
        const float gpsCourseDeg = ctx->gps->courseDeg();
        const uint32_t gpsLocationAgeMs = ctx->gps->locationAgeMs();
        float  heading = ctx->compass->getHeadingDeg();
        bool   compassOk = ctx->compass->isOk() && isfinite(heading);
        float  temp    = ctx->water->getWaterTemp();
        float  battV   = ctx->battery->readVoltage();
        const bool motionDisturbance =
            hadHardThrust &&
            (millis() - lastHardThrustMs) < ANOMALY_DISTURBANCE_MS;
        const BoatState navState = ctx->fsm->state();
        const bool watchWater =
            (navState == STATE_GRID_PATROL ||
             navState == STATE_SPIRAL_MAPPING);
        bool waterAdsSampled = false;
        if (waterAdsDiv == 0) {
            waterAdsSampled = true;
            phR   = ctx->water->readPh(temp);
            turbR = ctx->water->readTurbidityNtu();
            if (watchWater) {
                if (turbR.valid) {
                    anomaly = ctx->anomaly->updateTurbidity(
                        turbR.value, motionDisturbance, millis());
                } else {
                    ctx->anomaly->notifyTurbidityInvalid(millis());
                    anomaly = ctx->anomaly->shouldTriggerSpiral();
                }
            } else {
                anomaly = false;
            }
        } else if (watchWater) {
            anomaly = ctx->anomaly->shouldTriggerSpiral();
        } else {
            anomaly = false;
        }
        if (++waterAdsDiv >= WATER_ADS_EVERY_N_LOOPS) {
            waterAdsDiv = 0;
        }
        const uint32_t waterSampleElapsed =
            g_waterSampleActive ? millis() - g_waterSampleStartMs : 0;
        const bool phQuietMeasure =
            navState == STATE_GRID_PATROL && g_waterSampleActive &&
            waterSampleElapsed >= WATER_SAMPLE_SETTLE_MS &&
            waterSampleElapsed < WATER_SAMPLE_HOLD_MS;
        if (!phQuietMeasure) {
            // Never smear a quiet waypoint pH value along the moving track.
            lastPh = NAN;
        } else if (waterAdsSampled) {
            // Report every finite calibrated conversion during the quiet
            // window. The quality gate remains in WaterReading::valid for
            // diagnostics, but does not suppress telemetry at this waypoint.
            if (isfinite(phR.value)) {
                g_phQuietSum += phR.value;
                g_phQuietCount++;
                lastPh = g_phQuietSum / (float)g_phQuietCount;
            } else if (g_phQuietCount == 0) {
                lastPh = NAN;
            }
        }
        if (turbR.valid && isfinite(turbR.value)) {
            lastNtu = turbR.value;
        }
        if (ctx->water->tempValid()) {
            lastTemp = temp;
        }

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
        }

        CommandState cmd;
        if (xSemaphoreTake(ctx->stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            cmd = ctx->command;
            lastCmd = cmd;
            xSemaphoreGive(ctx->stateMutex);
        } else {
            cmd = lastCmd;
        }

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

        // Stale CLEAR in STANDBY would auto-exit the next E-Stop.
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

        // Don't re-apply the EventGroup bit while CLEAR is in flight.
        if (EspNowLink::emergencyPending() && !cmd.clearEstop) {
            cmd.emergency = true;
        }

        if (cmd.clearEstop &&
            ctx->fsm->state() == STATE_EMERGENCY_STOP &&
            ctx->gps->clearFaultIfFresh()) {
            fix = ctx->gps->getFilteredFix();
        }

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
        if (cmd.manualMode != lastManualMode) {
            ctx->heading->reset();
            haveLastLosTarget = false;
            lastManualMode = cmd.manualMode;
        }

        const bool underLoad = (lastAbsPwm >= BATTERY_LOAD_PWM_THRESH) ||
                               motionDisturbance;
        SafetyState safety = ctx->safety->evaluate(battV, underLoad);

        GpsLossTier gpsTier = ctx->gps->getLossTier();
        bool gpsOk = (gpsTier == GpsLossTier::OK) && fix.valid;

        if (cmd.setHome) {
            if (xSemaphoreTake(ctx->stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                ctx->command.setHome = false;
                xSemaphoreGive(ctx->stateMutex);
            }
            if (!g_homeCapturing) {
                if (ctx->fsm->state() == STATE_STANDBY && gpsOk && fix.valid) {
                    MissionFsm::homeCaptureReset(g_homeCap);
                    g_homeCapturing = true;
                    g_homeCapStartMs = millis();
                    g_homeCapLastSampleMs = 0;
                    Serial.printf("[USV] SET_HOME capturing — keep still %lus\n",
                                  (unsigned long)(SET_HOME_DWELL_MS / 1000UL));
                } else {
                    Serial.println(F("[USV] SET_HOME refused — need STANDBY and GPS OK"));
                }
            }
        }

        if (g_homeCapturing) {
            if (ctx->fsm->state() != STATE_STANDBY || !gpsOk || !fix.valid) {
                g_homeCapturing = false;
                Serial.println(F("[USV] SET_HOME aborted — need STANDBY and GPS OK"));
            } else if (fix.speedKmh > SET_HOME_MAX_SPEED_KMH) {
                g_homeCapturing = false;
                Serial.printf("[USV] SET_HOME aborted — moving %.1f km/h\n",
                              (double)fix.speedKmh);
            } else {
                const uint32_t nowMs = millis();
                if (g_homeCapLastSampleMs == 0 ||
                    (nowMs - g_homeCapLastSampleMs) >= SET_HOME_SAMPLE_MS) {
                    MissionFsm::homeCaptureAdd(g_homeCap, fix.lat, fix.lon);
                    g_homeCapLastSampleMs = nowMs;
                }
                if ((nowMs - g_homeCapStartMs) >= SET_HOME_DWELL_MS) {
                    double lat = 0.0, lon = 0.0;
                    const float spreadM = MissionFsm::homeCaptureSpreadM(g_homeCap);
                    const bool haveMean =
                        MissionFsm::homeCaptureMean(g_homeCap, lat, lon);
                    g_homeCapturing = false;
                    if (haveMean &&
                        g_homeCap.count >= SET_HOME_MIN_SAMPLES &&
                        spreadM <= SET_HOME_MAX_SPREAD_M) {
                        g_homeLat = lat;
                        g_homeLon = lon;
                        g_homeSet = true;
                        g_homeLeft = false;
                        g_homeOutsideAccumMs = 0;
                        saveHomeToNvs(g_homeLat, g_homeLon);
                        Serial.printf(
                            "[USV] SET_HOME lat=%.6f lon=%.6f n=%u spread=%.1fm\n",
                            g_homeLat, g_homeLon,
                            (unsigned)g_homeCap.count, (double)spreadM);
                    } else {
                        Serial.printf(
                            "[USV] SET_HOME refused — n=%u spread=%.1fm (retry still)\n",
                            (unsigned)g_homeCap.count, (double)spreadM);
                    }
                }
            }
        }

        const uint32_t decisionNowMs = millis();
        if (cmd.forceSpiral && ctx->fsm->state() != STATE_GRID_PATROL) {
            if (xSemaphoreTake(ctx->stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                ctx->command.forceSpiral = false;
                xSemaphoreGive(ctx->stateMutex);
            }
            Serial.println(F("[USV] FORCE_SPIRAL refused — need GRID_PATROL"));
            cmd.forceSpiral = false;
        } else if (cmd.forceSpiral &&
                   ctx->nav->index() < SPIRAL_MIN_WAYPOINT_INDEX) {
            if (xSemaphoreTake(ctx->stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                ctx->command.forceSpiral = false;
                xSemaphoreGive(ctx->stateMutex);
            }
            Serial.println(F("[USV] FORCE_SPIRAL refused — pass first waypoint"));
            cmd.forceSpiral = false;
        } else if (cmd.forceSpiral &&
                   ctx->fsm->forceSpiralCooldownActive(decisionNowMs)) {
            if (xSemaphoreTake(ctx->stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                ctx->command.forceSpiral = false;
                xSemaphoreGive(ctx->stateMutex);
            }
            Serial.println(F("[USV] FORCE_SPIRAL refused — cooldown"));
            cmd.forceSpiral = false;
        }

        FsmInputs in;
        memset(&in, 0, sizeof(in));
        in.startCommand        = cmd.start;
        in.clearEstop          = cmd.clearEstop;
        in.emergencyCommand    = cmd.emergency;
        in.forceRthCommand     = cmd.forceRth;
        in.forceSpiralCommand  = cmd.forceSpiral;
        in.nowMs               = decisionNowMs;
        in.anomalyDetected     = anomaly;
        in.spiralComplete   = spiralDone;
        in.spiralAborted    = false;
        if (ctx->fsm->state() == STATE_SPIRAL_MAPPING && g_spiralInitialized) {
            const uint32_t nowMs = millis();
            if ((nowMs - g_spiralEnterMs) >= SPIRAL_MAX_MS) {
                in.spiralAborted = true;
                g_spiralAbortWhy = "wall";
            } else if (!g_spiralFromForce && !anomaly) {
                if (g_spiralCleanSinceMs == 0) {
                    g_spiralCleanSinceMs = nowMs;
                } else if ((nowMs - g_spiralCleanSinceMs) >= SPIRAL_CLEAR_ABORT_MS) {
                    in.spiralAborted = true;
                    g_spiralAbortWhy = "clean";
                }
            } else {
                g_spiralCleanSinceMs = 0;
            }
        }
        in.gridComplete = WaterSamplePolicy::missionCompleteForFsm(
            ctx->nav->missionComplete(), g_waterSampleActive);
        float distHomeM = NAN;
        if (g_homeSet && fix.valid) {
            distHomeM = (float)Navigator::calculateDistance(
                fix.lat, fix.lon, g_homeLat, g_homeLon);
            g_homeLeft = MissionFsm::latchDeparted(
                g_homeLeft, distHomeM, g_homeOutsideAccumMs, NAV_LOOP_PERIOD_MS);
        } else {
            g_homeOutsideAccumMs = 0;
        }
        in.arrivedHome = false;
        if (ctx->fsm->state() == STATE_RTH) {
            in.arrivedHome = MissionFsm::isArrivedHome(
                g_homeSet, fix.valid, g_homeLeft,
                distHomeM, g_rthEntryDistM, g_rthInsideAccumMs, NAV_LOOP_PERIOD_MS);
        } else {
            g_rthInsideAccumMs = 0;
        }
        in.gpsOk            = gpsOk;
        in.gpsFault         = (gpsTier == GpsLossTier::GPS_FAULT);
        in.homeSet          = g_homeSet;
        in.compassOk        = compassOk;
        in.hasWaypoints     = ctx->nav->isMissionReady();
        in.passedFirstWaypoint =
            ctx->nav->index() >= SPIRAL_MIN_WAYPOINT_INDEX;
        in.safety           = safety;
        static bool loggedSpiralHold = false;
        if (in.passedFirstWaypoint) {
            loggedSpiralHold = false;
        } else if (anomaly &&
                   ctx->fsm->state() == STATE_GRID_PATROL &&
                   !loggedSpiralHold) {
            loggedSpiralHold = true;
            Serial.println(F("[USV] SPIRAL hold — pass first waypoint"));
        }

        BoatState stateBefore = ctx->fsm->state();
        BoatState state = ctx->fsm->tick(in);
        if (state != prevState) {
            // Bearing ownership changed. Do not carry PID derivative/integral
            // or turn hysteresis from GRID/manual/SPIRAL/RTH into the next mode.
            ctx->heading->reset();
            haveLastLosTarget = false;
        }

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

        if (state == STATE_GRID_PATROL && prevState == STATE_STANDBY) {
            ctx->anomaly->clearPendingTrigger();
            anomaly = false;
            if (fix.valid) {
                g_homeLeft = false;
                g_homeOutsideAccumMs = 0;
                g_rthEntryDistM = NAN;
                g_rthEnterMs = 0;
                g_rthInsideAccumMs = 0;
                ctx->nav->setMissionOrigin(fix.lat, fix.lon);
                if (g_homeSet) {
                    const float startDist = (float)Navigator::calculateDistance(
                        fix.lat, fix.lon, g_homeLat, g_homeLon);
                    Serial.printf("[USV] start dist from home=%.1fm\n", startDist);
                } else {
                    Serial.println(F("[USV] no home — SET_HOME before RTH"));
                }
            }
            ctx->heading->reset();
            haveLastLosTarget = false;
        }

        if (state == STATE_RTH && prevState != STATE_RTH) {
            g_rthEntryDistM = distHomeM;
            g_rthEnterMs = millis();
            g_rthInsideAccumMs = 0;
            Serial.printf(
                "[USV] RTH to SET_HOME dist=%.1fm left=%u home=%.6f,%.6f now=%.6f,%.6f\n",
                distHomeM, (unsigned)g_homeLeft,
                g_homeLat, g_homeLon, fix.lat, fix.lon);
        }

        if (state == STATE_STANDBY && prevState == STATE_RTH) {
            if (xSemaphoreTake(ctx->stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                ctx->command.start = false;
                ctx->command.forceRth = false;
                xSemaphoreGive(ctx->stateMutex);
            }
            ctx->heading->reset();
        }

        if (state == STATE_SPIRAL_MAPPING && prevState != STATE_SPIRAL_MAPPING) {
            if (fix.valid) {
                const int skipped = ctx->nav->skipReached(
                    fix.lat, fix.lon,
                    GRID_LOS_LOOKAHEAD_M, WAYPOINT_RADIUS_M, LOS_PASS_MARGIN_M);
                if (skipped > 0) {
                    Serial.printf("[USV] spiral enter skipped %d wp idx=%d\n",
                                  skipped, ctx->nav->index());
                }
            }
            spiralDone = false;
            g_haveSpiralTgt = false;
            g_spiralInitialized = false;
            g_spiralEnterMs = millis();
            g_spiralPointMs = g_spiralEnterMs;
            g_spiralCleanSinceMs = 0;
            g_spiralFromForce = forceSpiralAccepted;
            g_spiralAbortWhy = nullptr;
            ctx->heading->reset();
            haveLastLosTarget = false;
            if (!forceSpiralAccepted) {
                Serial.printf(
                    "[USV] SPIRAL turbidity ntu=%.1f ph_context=%.2f\n",
                    isfinite(turbR.value) ? (double)turbR.value : NAN,
                    isfinite(phR.value) ? (double)phR.value : NAN);
            }
        }
        if (state != STATE_SPIRAL_MAPPING) {
            if (prevState == STATE_SPIRAL_MAPPING) {
                if (g_spiralAbortWhy) {
                    Serial.printf("[USV] spiral abort %s\n", g_spiralAbortWhy);
                    g_spiralAbortWhy = nullptr;
                }
                const bool completedAutomaticSpiral =
                    !g_spiralFromForce &&
                    state == STATE_GRID_PATROL &&
                    (in.spiralComplete || in.spiralAborted);
                if (completedAutomaticSpiral) {
                    ctx->anomaly->notifySpiralDone(millis());
                } else {
                    // A forced demo or safety/RTH interruption must not hide a
                    // real plume for the full anomaly cooldown.
                    ctx->anomaly->clearPendingTrigger();
                }
                anomaly = false;
            }
            spiralDone = false;
            g_spiralInitialized = false;
        }

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

        int leftPwm = 0, rightPwm = 0;
        DriveSource driveSource = DriveSource::NONE;
        bool driveRequestValid = false;
        bool gridGuidanceUsed = false;
        int logBasePwm = 0;
        double logTargetLat = NAN, logTargetLon = NAN;
        float logNavBearing = NAN;
        float logCrossTrack = NAN;
        float logDistToTarget = NAN;
        float logHeadingError = NAN;
        float logDirectBearing = NAN;
        uint8_t logWaterSamplePhase = 0;
        uint32_t logWaterSampleMs = 0;
        bool navSensorsOk = gpsOk && compassOk;

        if (state != STATE_GRID_PATROL) {
            g_waterSampleActive = false;
        }

        switch (state) {
            case STATE_STANDBY:
            case STATE_EMERGENCY_STOP:
                ctx->heading->reset();
                break;

            case STATE_GRID_PATROL: {
                if (cmd.manualMode) {
                    g_waterSampleActive = false;
                    leftPwm  = constrain(cmd.manualSpeed + cmd.manualSteer,
                                         -MOTOR_PWM_LIMIT, MOTOR_PWM_LIMIT);
                    rightPwm = constrain(cmd.manualSpeed - cmd.manualSteer,
                                         -MOTOR_PWM_LIMIT, MOTOR_PWM_LIMIT);
                    driveSource = DriveSource::MANUAL;
                    driveRequestValid = true;
                } else {
                    if (g_waterSampleActive) {
                        const uint32_t elapsed = millis() - g_waterSampleStartMs;
                        if (elapsed >= WATER_SAMPLE_HOLD_MS) {
                            g_waterSampleActive = false;
                            ctx->heading->reset();
                            gridHeading.reset();
                            haveLastLosTarget = false;
                            Serial.println(F("[USV] water sample done"));
                        } else {
                            logWaterSampleMs = elapsed;
                            logWaterSamplePhase =
                                elapsed < WATER_SAMPLE_SETTLE_MS ? 1 : 2;
                        }
                    }

                    if (!g_waterSampleActive && navSensorsOk &&
                        ctx->nav->hasWaypoints() &&
                        !ctx->nav->missionComplete()) {
                        const int indexBefore = ctx->nav->index();
                        const int skipped = ctx->nav->skipReached(
                            fix.lat, fix.lon,
                             GRID_LOS_LOOKAHEAD_M, WAYPOINT_RADIUS_M,
                            LOS_PASS_MARGIN_M);
                        if (skipped > 0) {
                            ctx->heading->reset();
                            gridHeading.reset();
                            haveLastLosTarget = false;
                        }
                        if (WaterSamplePolicy::crossedBoundary(
                                indexBefore, ctx->nav->index(),
                                ctx->nav->count(),
                                WATER_SAMPLE_EVERY_N_WAYPOINTS)) {
                            g_waterSampleActive = true;
                            g_waterSampleStartMs = millis();
                            g_phQuietSum = 0.0f;
                            g_phQuietCount = 0;
                            lastPh = NAN;
                            logWaterSamplePhase = 1;
                            logWaterSampleMs = 0;
                            Serial.printf(
                                "[USV] water sample hold wp=%d settle=%lums measure=%lums\n",
                                ctx->nav->index(),
                                (unsigned long)WATER_SAMPLE_SETTLE_MS,
                                (unsigned long)WATER_SAMPLE_MEASURE_MS);
                        }

                        if (!g_waterSampleActive) {
                            LosGuidance los = ctx->nav->computeLos(
                                fix.lat, fix.lon,
                                 GRID_LOS_LOOKAHEAD_M, WAYPOINT_RADIUS_M,
                                LOS_PASS_MARGIN_M);

                            if (los.valid && !ctx->nav->missionComplete()) {
                                Waypoint tgt = ctx->nav->currentTarget();
                                logTargetLat = tgt.lat;
                                logTargetLon = tgt.lon;
                                logNavBearing = los.bearingDeg;
                                logCrossTrack = los.crossTrackM;
                                logDistToTarget = los.distToEndM;
                                if (haveLastLosTarget &&
                                    (tgt.lat != lastLosTarget.lat ||
                                     tgt.lon != lastLosTarget.lon)) {
                                    ctx->heading->reset();
                                    gridHeading.reset();
                                }
                                lastLosTarget = tgt;
                                haveLastLosTarget = true;

                                float herr = HeadingController::computeHeadingError(
                                    los.bearingDeg, heading);
                                int base = chooseBasePwm(
                                    herr, los.distToEndM, GRID_CRUISE_PWM);
                                logHeadingError = herr;
                                logBasePwm = base;
                                const DriveRequest gridRequest = gridHeading.update(
                                    los.bearingDeg, heading, base, compassSampleUs,
                                    compassOk);
                                leftPwm = gridRequest.leftPwm;
                                rightPwm = gridRequest.rightPwm;
                                driveSource = gridRequest.source;
                                driveRequestValid = gridRequest.valid;
                                gridGuidanceUsed = true;
                            }
                        }
                    }
                }
                break;
            }

            case STATE_SPIRAL_MAPPING: {
                if (!g_spiralInitialized) {
                    if (fix.valid) {
                        ctx->spiralPlanner->begin(fix.lat, fix.lon);
                        g_spiralStart.lat = fix.lat;
                        g_spiralStart.lon = fix.lon;
                        g_spiralInitialized = true;
                        g_haveSpiralTgt = false;
                        g_spiralEnterMs = millis();
                        g_spiralPointMs = g_spiralEnterMs;
                    } else {
                        break;
                    }
                }

                if (g_spiralInitialized && !g_haveSpiralTgt) {
                    g_haveSpiralTgt = ctx->spiralPlanner->nextSpiralPoint(g_spiralTgt);
                    if (!g_haveSpiralTgt) {
                        spiralDone = true;
                    } else {
                        ctx->heading->reset();
                        g_spiralPointMs = millis();
                    }
                }

                if (g_haveSpiralTgt && navSensorsOk) {
                    LosGuidance los = Navigator::computeSegmentLos(
                        fix.lat, fix.lon, g_spiralStart, g_spiralTgt,
                        LOS_LOOKAHEAD_M, SPIRAL_WAYPOINT_RADIUS_M,
                        LOS_PASS_MARGIN_M);

                    const bool reachedWithProgress =
                        los.valid && los.reached && (los.alongTrackM >= 0.5f);
                    const bool passedEnd = los.valid && los.passed;
                    const bool pointTimeout =
                        (millis() - g_spiralPointMs) >= SPIRAL_POINT_TIMEOUT_MS;
                    if (reachedWithProgress || passedEnd || pointTimeout) {
                        if (pointTimeout && !(reachedWithProgress || passedEnd)) {
                            Serial.printf("[USV] spiral skip timeout d=%.1fm\n",
                                          los.valid ? (double)los.distToEndM : NAN);
                        } else {
                            Serial.printf("[USV] spiral hit d=%.1fm along=%.1f\n",
                                          los.distToEndM, los.alongTrackM);
                        }
                        g_spiralStart = g_spiralTgt;
                        g_haveSpiralTgt = ctx->spiralPlanner->nextSpiralPoint(g_spiralTgt);
                        ctx->heading->reset();
                        haveLastLosTarget = false;
                        g_spiralPointMs = millis();
                        if (!g_haveSpiralTgt) {
                            spiralDone = true;
                        } else {
                            los = Navigator::computeSegmentLos(
                                fix.lat, fix.lon, g_spiralStart, g_spiralTgt,
                                LOS_LOOKAHEAD_M, SPIRAL_WAYPOINT_RADIUS_M,
                                LOS_PASS_MARGIN_M);
                        }
                    }

                    if (g_haveSpiralTgt && los.valid) {
                        logTargetLat = g_spiralTgt.lat;
                        logTargetLon = g_spiralTgt.lon;
                        logNavBearing = los.bearingDeg;
                        logCrossTrack = los.crossTrackM;
                        logDistToTarget = los.distToEndM;
                        float herr = HeadingController::computeHeadingError(
                            los.bearingDeg, heading);
                        int base = chooseBasePwm(
                            herr, los.distToEndM, MOTOR_APPROACH_PWM);
                        logHeadingError = herr;
                        logBasePwm = base;
                        ctx->heading->calculateThrusterOutputs(
                            los.bearingDeg, heading, base, dt, leftPwm, rightPwm);
                        driveSource = DriveSource::SPIRAL;
                        driveRequestValid = true;
                    }
                }
                break;
            }

            case STATE_RTH: {
                if (g_homeSet && navSensorsOk) {
                    const float dist = isfinite(distHomeM) ? distHomeM :
                        (float)Navigator::calculateDistance(
                            fix.lat, fix.lon, g_homeLat, g_homeLon);
                    const float bearing = (float)Navigator::calculateBearing(
                        fix.lat, fix.lon, g_homeLat, g_homeLon);
                    logTargetLat = g_homeLat;
                    logTargetLon = g_homeLon;
                    logNavBearing = bearing;
                    logDistToTarget = dist;
                    logHeadingError = HeadingController::computeHeadingError(
                        bearing, heading);
                    // Park on the stored SET_HOME fix. Keep thrusting through
                    // the bubble and the boat overshoots the dock GPS.
                    if (dist <= RTH_ARRIVE_RADIUS_M) {
                        leftPwm = 0;
                        rightPwm = 0;
                        driveSource = DriveSource::RTH;
                        driveRequestValid = true;
                    } else {
                        int base = chooseBasePwm(
                            logHeadingError, dist, MOTOR_CRUISE_PWM);
                        logBasePwm = base;
                        ctx->heading->calculateThrusterOutputs(
                            bearing, heading, base, dt, leftPwm, rightPwm);
                        driveSource = DriveSource::RTH;
                        driveRequestValid = true;
                    }
                }
                break;
            }
        }

        if (fix.valid && isfinite(logTargetLat) && isfinite(logTargetLon)) {
            logDirectBearing = (float)Navigator::calculateBearing(
                fix.lat, fix.lon, logTargetLat, logTargetLon);
        }
        if (!gridGuidanceUsed) gridHeading.reset();

        DriveRequest driveRequest = {
            driveSource, leftPwm, rightPwm, driveRequestValid
        };
        DriveInterlocks driveInterlocks = {
            state,
            safety,
            cmd.emergency || EspNowLink::emergencyPending(),
            navSensorsOk,
            state == STATE_GRID_PATROL && !cmd.manualMode &&
                g_waterSampleActive
        };
        const int requestedLeftPwm = leftPwm;
        const int requestedRightPwm = rightPwm;
        const bool navTurnInPlace = driveRequestValid &&
            (requestedLeftPwm != 0 || requestedRightPwm != 0) &&
            (driveSource == DriveSource::GRID ||
             driveSource == DriveSource::SPIRAL || driveSource == DriveSource::RTH) &&
            (driveSource == DriveSource::GRID ? gridHeading.isTurningInPlace()
                                               : ctx->heading->isTurningInPlace());
        const DriveDecision driveDecision =
            driveController.resolve(driveRequest, driveInterlocks, dt);
        leftPwm = driveDecision.leftPwm;
        rightPwm = driveDecision.rightPwm;
        ctx->motor->writeThrust(leftPwm, rightPwm);
        if (driveDecision.stopReason != DriveStopReason::NONE) {
            ctx->heading->reset();
        }
        if (driveDecision.stopReason != DriveStopReason::NONE &&
            driveDecision.stopReason != DriveStopReason::REQUESTED_STOP) {
            gridHeading.reset();
        }

        const int absL = leftPwm < 0 ? -leftPwm : leftPwm;
        const int absR = rightPwm < 0 ? -rightPwm : rightPwm;
        lastAbsPwm = absL > absR ? absL : absR;
        if (lastAbsPwm >= BATTERY_LOAD_PWM_THRESH) {
            lastHardThrustMs = millis();
            hadHardThrust = true;
        }

        // The hold can end later in this tick than the sensor read above.
        // Never publish the previous quiet sample after thrust resumes, nor
        // while a manual command is moving the boat through a sample window.
        if (logWaterSamplePhase != 2 || leftPwm != 0 || rightPwm != 0) {
            lastPh = NAN;
        }

        SdLogRecord logRec;
        memset(&logRec, 0, sizeof(logRec));
        logRec.target_lat = NAN;
        logRec.target_lon = NAN;
        logRec.nav_bearing_deg = NAN;
        logRec.cross_track_m = NAN;
        logRec.dist_to_target_m = NAN;
        logRec.heading_error_deg = NAN;
        logRec.gps_raw_lat = rawGpsLat;
        logRec.gps_raw_lon = rawGpsLon;
        logRec.gps_course_deg = gpsCourseDeg;
        logRec.gps_location_age_ms = gpsLocationAgeMs;
        logRec.gps_fix_valid = fix.valid ? 1 : 0;
        logRec.compass_fresh = ctx->compass->isFresh() ? 1 : 0;
        logRec.compass_fail_count = ctx->compass->failCount();
        logRec.direct_bearing_deg = logDirectBearing;
        logRec.requested_left_pwm = (int16_t)requestedLeftPwm;
        logRec.requested_right_pwm = (int16_t)requestedRightPwm;
        logRec.turn_in_place = navTurnInPlace ? 1 : 0;
        logRec.drive_source = (uint8_t)driveSource;
        logRec.drive_stop_reason = (uint8_t)driveDecision.stopReason;
        logRec.radio_tx_paused = ctx->radioTxPaused ? 1 : 0;
        logRec.battery_voltage = battV;
        const PhSampleDiagnostics phDiag = ctx->water->lastPhDiagnostics();
        DiagnosticFormatting::PhLogRecord& phLog = logRec.ph_diagnostic;
        phLog.sampleId = phDiag.sampleId;
        phLog.rawAdc = phDiag.rawAdc;
        phLog.sampleMs = phDiag.sampleMs;
        phLog.samplePresent = phDiag.samplePresent;
        phLog.newSample = waterAdsSampled && phDiag.sampledThisCall;
        phLog.adcVoltage = phDiag.voltage;
        phLog.sensorPh = phDiag.sensorPh;
        phLog.qualityValid = phDiag.qualityValid && phR.valid;
        phLog.quietCount = g_phQuietCount;
        phLog.sensorTempC = phDiag.sensorTempC;
        phLog.effectiveTempC = phDiag.effectiveTempC;
        phLog.temperatureValid = phDiag.temperatureValid;
        phLog.calibrationSource = "UNKNOWN";
        phLog.readReason = phDiagnosticReason(phDiag.reason);
        if (state != STATE_GRID_PATROL) {
            phLog.suppressionReason = DiagnosticFormatting::PhSuppressionReason::MODE_SUPPRESSED;
        } else if (logWaterSamplePhase != 2) {
            phLog.suppressionReason = DiagnosticFormatting::PhSuppressionReason::NOT_QUIET_WINDOW;
        } else if (leftPwm != 0 || rightPwm != 0) {
            phLog.suppressionReason = DiagnosticFormatting::PhSuppressionReason::MOTORS_NOT_STOPPED;
        } else if (!isfinite(lastPh)) {
            phLog.suppressionReason = DiagnosticFormatting::PhSuppressionReason::NO_FINITE_SAMPLE;
        } else {
            phLog.suppressionReason = DiagnosticFormatting::PhSuppressionReason::NONE;
        }
        bool snapshotPublished = false;
        if (xSemaphoreTake(ctx->stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            ctx->snapshot.gps_lat         = fix.valid ? fix.lat : NAN;
            ctx->snapshot.gps_lon         = fix.valid ? fix.lon : NAN;
            // Mode 0: nan so dock samples stay off the heatmap. pH is only
            // published as the running quiet-window mean; NTU/temp retain the
            // latest valid value across a brief sensor glitch.
            if (state == STATE_STANDBY) {
                ctx->snapshot.water_ph        = NAN;
                ctx->snapshot.water_turbidity = NAN;
                ctx->snapshot.water_temp      = NAN;
            } else {
                ctx->snapshot.water_ph        = lastPh;
                ctx->snapshot.water_turbidity = lastNtu;
                ctx->snapshot.water_temp      = lastTemp;
            }
            ctx->snapshot.current_mode    = (uint8_t)state;
            ctx->snapshot.battery_percent = ctx->battery->getBatteryPercent();
            ctx->snapshot.heading_angle   = compassOk ? heading : NAN;
            ctx->batteryVoltage            = battV;
            ctx->phVoltage                 = ctx->water->lastPhVoltage();
            ctx->turbVoltage               = ctx->water->lastTurbVoltage();
            ctx->compassAddr               = ctx->compass->i2cAddress();
            ctx->compassFailCount          = ctx->compass->failCount();
            ctx->compassFresh              = ctx->compass->isFresh() ? 1 : 0;
            ctx->compassRawX               = ctx->compass->rawX();
            ctx->compassRawY               = ctx->compass->rawY();
            ctx->compassNvs                = ctx->compass->nvsLoaded() ? 1 : 0;
            ctx->compassCalRun             = ctx->compass->isCalibrating() ? 1 : 0;
            SensorMonitorSnapshot& sensors = ctx->sensorMonitor;
            sensors.available = true;
            sensors.capturedMs = millis();
            sensors.temperature = ctx->water->temperatureDiagnostics();
            sensors.adsInitialized = ctx->water->adsReady();
            if (waterAdsSampled) {
                sensors.adsSampleAttempted = true;
                sensors.adsSampleMs = sensors.capturedMs;
            }
            sensors.ph = phDiag;
            sensors.phRead = phR.value;
            sensors.phQuality = phR.valid;
            sensors.turbRead = turbR.value;
            sensors.turbQuality = turbR.valid;
            sensors.phPublishReason = (uint8_t)phLog.suppressionReason;
            sensors.gpsReady = gpsOk;
            sensors.gpsAgeMs = gpsLocationAgeMs;
            sensors.gpsSats = fix.sats;
            sensors.gpsHdop = fix.hdop;
            sensors.gpsSpeedKmh = fix.speedKmh;
            sensors.gpsRawLat = rawGpsLat;
            sensors.gpsRawLon = rawGpsLon;
            sensors.motorLeft = (int16_t)leftPwm;
            sensors.motorRight = (int16_t)rightPwm;
            sensors.compassInitStage = ctx->compass->initStage();
            sensors.compassIoError = ctx->compass->lastIoError();
            sensors.compassRegister = ctx->compass->lastRegister();
            sensors.compassReadCount = ctx->compass->readCount();
            sensors.compassReadExpected = ctx->compass->readExpected();
            sensors.compassGoodPresent = ctx->compass->hasGoodRead();
            sensors.compassGoodMs = ctx->compass->lastGoodMs();
            ctx->homeDistM                 = distHomeM;
            ctx->homeSetFlag               = g_homeSet ? 1 : 0;
            logRec.boot_ms            = millis();
            if (phLog.samplePresent) {
                phLog.sampleAgeMs = logRec.boot_ms - phLog.sampleMs;
            }
            logRec.sequence_number    = ++logSequence;
            logRec.gps_lat            = ctx->snapshot.gps_lat;
            logRec.gps_lon            = ctx->snapshot.gps_lon;
            logRec.water_ph           = ctx->snapshot.water_ph;
            logRec.water_turbidity    = ctx->snapshot.water_turbidity;
            logRec.water_temp         = ctx->snapshot.water_temp;
            logRec.ph_voltage         = ctx->phVoltage;
            logRec.turb_voltage       = ctx->turbVoltage;
            logRec.heading_angle      = ctx->snapshot.heading_angle;
            logRec.current_mode       = ctx->snapshot.current_mode;
            logRec.battery_percent    = ctx->snapshot.battery_percent;
            logRec.gps_hdop           = fix.hdop;
            logRec.gps_sats           = fix.sats;
            logRec.gps_speed_kmh      = fix.speedKmh;
            logRec.waypoint_index     = (uint8_t)ctx->nav->index();
            logRec.waypoint_count     = (uint8_t)ctx->nav->count();
            logRec.target_lat         = logTargetLat;
            logRec.target_lon         = logTargetLon;
            logRec.nav_bearing_deg    = logNavBearing;
            logRec.cross_track_m      = logCrossTrack;
            logRec.dist_to_target_m   = logDistToTarget;
            logRec.heading_error_deg  = logHeadingError;
            logRec.water_sample_phase = logWaterSamplePhase;
            logRec.water_sample_ms    = logWaterSampleMs;
            logRec.base_pwm           = (int16_t)logBasePwm;
            logRec.left_pwm           = (int16_t)leftPwm;
            logRec.right_pwm          = (int16_t)rightPwm;
            logRec.compass_raw_x      = ctx->compassRawX;
            logRec.compass_raw_y      = ctx->compassRawY;
            logRec.compass_cal_nvs    = ctx->compassNvs;
            logRec.telemetry_tx_seq   = ctx->telemetryTxSeq;
            logRec.telemetry_tx_ms    = ctx->telemetryTxMs;
            logRec.telemetry_tx_lat   = ctx->telemetryTxLat;
            logRec.telemetry_tx_lon   = ctx->telemetryTxLon;
            logRec.telemetry_tx_ph    = ctx->telemetryTxPh;
            logRec.telemetry_radio_status = ctx->telemetryRadioStatus;
            logRec.telemetry_tx_attempts  = ctx->telemetryTxAttempts;
            snapshotPublished = true;
            xSemaphoreGive(ctx->stateMutex);
        }

        if (snapshotPublished) {
            const GpsDateTime utc = ctx->gps->utcNow();
            logRec.utc_age_ms = ctx->gps->utcAgeMs();
            logRec.utc_age_known = logRec.utc_age_ms != UINT32_MAX;
            logRec.ph_diagnostic.utcAvailable = utc.valid;
            logRec.ph_diagnostic.utcSource = logRec.utc_age_known ? "GPS" : "NONE";
            logRec.ph_diagnostic.utcAgeKnown = logRec.utc_age_known;
            logRec.ph_diagnostic.utcAgeMs = logRec.utc_age_ms;
            if (utc.valid) {
                logRec.utc_year   = utc.year;
                logRec.utc_month  = utc.month;
                logRec.utc_day    = utc.day;
                logRec.utc_hour   = utc.hour;
                logRec.utc_minute = utc.minute;
                logRec.utc_second = utc.second;
                logRec.utc_valid  = 1;
            }
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

        CommandPacket cmdPkt;
        while (EspNowLink::commandQueue) {
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

        if (!ctx->radioTxPaused &&
            millis() - lastTelemetry >= TELEMETRY_INTERVAL_MS) {
            lastTelemetry = millis();
            TelemetryPacket tp;
            float phV = NAN;
            float turbV = NAN;
            float tC = NAN;
            if (xSemaphoreTake(ctx->stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                tp = ctx->snapshot;
                phV = ctx->phVoltage;
                turbV = ctx->turbVoltage;
                tC = tp.water_temp;
                xSemaphoreGive(ctx->stateMutex);
                const uint32_t txStartMs = millis();
                const bool ok = ctx->link->sendTelemetry(tp);
                const uint32_t txEndMs = millis();
                const EspNowLink::TxDiag txDiag = EspNowLink::lastTx;
                if (xSemaphoreTake(ctx->stateMutex, portMAX_DELAY) == pdTRUE) {
                    ctx->telemetryTxSeq = tp.sequence_number;
                    ctx->telemetryTxMs = millis();
                    ctx->telemetryTxLat = tp.gps_lat;
                    ctx->telemetryTxLon = tp.gps_lon;
                    ctx->telemetryTxPh = tp.water_ph;
                    ctx->telemetryRadioStatus = radioStatusForLog(txDiag.result);
                    ctx->telemetryTxAttempts = txDiag.attemptsUsed;
                    if (ok) {
                        ctx->snapshot.sequence_number = tp.sequence_number;
                    }
                    xSemaphoreGive(ctx->stateMutex);
                }
                if (ctx->logger && ctx->logger->isReady()) {
                    DiagnosticFormatting::TxLogRecord txLog{};
                    txLog.sequence = tp.sequence_number;
                    txLog.sendStartMs = txStartMs;
                    txLog.sendEndMs = txEndMs;
                    txLog.latitude = tp.gps_lat;
                    txLog.longitude = tp.gps_lon;
                    txLog.ph = tp.water_ph;
                    txLog.turbidity = tp.water_turbidity;
                    txLog.temperatureC = tp.water_temp;
                    txLog.mode = tp.current_mode;
                    txLog.batteryPercent = tp.battery_percent;
                    txLog.headingDeg = tp.heading_angle;
                    txLog.result = txDiagnosticResult(txDiag.result);
                    txLog.sendError = (int32_t)txDiag.sendErr;
                    txLog.attempts = txDiag.attemptsUsed;
                    txLog.noAckCount = txDiag.noAckCount;
                    txLog.sendErrorCount = txDiag.sendErrCount;
                    txLog.timeoutCount = txDiag.timeoutCount;
                    txLog.wifiChannel = txDiag.wifiChannel;
                    txLog.peerExists = txDiag.peerExist != 0;
                    if (!ctx->logger->logTxDiagnostic(txLog) &&
                        ctx->logger->txDiagnosticWriteFailCount() == 1) {
                        Serial.println(F("[USV] TX diagnostic writer failed; radio result unchanged"));
                    }
                }
#if RADIO_TRACE
                Serial.printf(
                    "[TX] TEL %s seq=%lu lat=%.6f lon=%.6f ph=%.2f ntu=%.1f "
                    "phV=%.3f turbV=%.3f t=%.1f mode=%u batt=%u hdg=%.1f %s\n",
                    ok ? "ok" : "FAIL",
                    (unsigned long)tp.sequence_number,
                    tp.gps_lat, tp.gps_lon,
                    tp.water_ph, tp.water_turbidity,
                    phV, turbV, tC,
                    (unsigned)tp.current_mode,
                    (unsigned)tp.battery_percent,
                    tp.heading_angle,
                    EspNowLink::describeLastTx());
#endif
            }
        }

        if (!ctx->radioTxPaused &&
            millis() - lastBattReport >= BATTERY_REPORT_MS) {
            BatteryReportPacket br;
            br.report_timestamp = millis();
            br.current_draw_ma  = 0.0f;
            br.estimated_tte_min = NAN;
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

        SdLogRecord logRec;
        const uint32_t collectStarted = millis();
        for (unsigned rows = 0; rows < SD_LOG_ROWS_PER_IO_LOOP &&
             (uint32_t)(millis() - collectStarted) < SD_LOG_COLLECT_BUDGET_MS; ++rows) {
            if (ctx->logger && ctx->logger->isReady() &&
                !ctx->logger->canAcceptLogPair()) break; // Retain the record in the queue.
            if (xQueueReceive(ctx->logQueue, &logRec, 0) != pdTRUE) break;
            if (ctx->logger && ctx->logger->isReady()) {
                if (!ctx->logger->logRecord(logRec)) {
                    ctx->logDropCount++;
                }
                if (!ctx->logger->logPhDiagnostic(
                        logRec.boot_ms, logRec.sequence_number,
                        logRec.ph_diagnostic) &&
                    ctx->logger->phDiagnosticWriteFailCount() == 1) {
                    Serial.println(F("[USV] pH diagnostic writer failed; continuing"));
                }
            } else {
                ++ctx->logDropCount; // Explicit loss accounting when logging is unavailable.
            }
        }
        if (ctx->logger && ctx->logger->isReady() && !ctx->logger->service()) {
            Serial.println(F("[USV] SD service failed; inspect SD counters and queued data"));
        }

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
            SensorMonitorSnapshot sensors{};
            TelemetryPacket published{};
            float sensorPhV = NAN, sensorTurbV = NAN, sensorBattV = NAN;
            bool haveSensorSnapshot = false;
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
                sensors = ctx->sensorMonitor;
                published = ctx->snapshot;
                sensorPhV = ctx->phVoltage;
                sensorTurbV = ctx->turbVoltage;
                sensorBattV = ctx->batteryVoltage;
                haveSensorSnapshot = sensors.available;
                xSemaphoreGive(ctx->stateMutex);
            }
            if (haveSensorSnapshot) {
                const uint32_t diagnosticMs = millis();
                Serial.printf(
                    "[USV] compass addr=0x%02X fresh=%u fail=%u raw=%d,%d hdg=%.1f cal=%s "
                    "homeDist=%.1fm set=%u\n",
                    (unsigned)cAddr, (unsigned)cFresh, (unsigned)cFail,
                    (int)cX, (int)cY, hdg,
                    cCal ? "RUN" : (cNvs ? "NVS" : "FACT"),
                    homeD, (unsigned)homeSet);
                Serial.printf(
                    "[SENSOR] COMPASS init=%s io_error=%u reg=0x%02X read_bytes=%u/%u "
                    "good_present=%u good_age_ms=%lu snapshot_age_ms=%lu\n",
                    sensors.compassInitStage, (unsigned)sensors.compassIoError,
                    (unsigned)sensors.compassRegister, (unsigned)sensors.compassReadCount,
                    (unsigned)sensors.compassReadExpected, (unsigned)sensors.compassGoodPresent,
                    (unsigned long)(sensors.compassGoodPresent
                        ? diagnosticMs - sensors.compassGoodMs : 0),
                    (unsigned long)(diagnosticMs - sensors.capturedMs));
                const TemperatureDiagnostics& td = sensors.temperature;
                const char* tempPublish = published.current_mode == STATE_STANDBY
                    ? "MODE_SUPPRESSED" : (!td.lastGoodPresent ? "NO_VALID_SAMPLE"
                        : (td.valid ? "LAST_VALID" : "CACHED_LAST_GOOD"));
                Serial.printf(
                    "[SENSOR] TEMP pin=%u devices_boot=%u present=%u read=%.3f valid=%u "
                    "reason=%s read_age_ms=%lu good_present=%u last_good=%.3f good_age_ms=%lu "
                    "published=%.3f publish=%s\n",
                    (unsigned)TEMP_SENSOR_PIN, (unsigned)td.devicesAtBoot,
                    (unsigned)td.samplePresent, td.readC, (unsigned)td.valid,
                    WaterSensors::temperatureReasonName(td.reason),
                    (unsigned long)(td.samplePresent ? diagnosticMs - td.sampleMs : 0),
                    (unsigned)td.lastGoodPresent, td.lastGoodC,
                    (unsigned long)(td.lastGoodPresent ? diagnosticMs - td.lastGoodMs : 0),
                    published.water_temp, tempPublish);
                // ADS init success is not proof that subsequent ADC transactions succeed.
                Serial.printf(
                    "[SENSOR] PH ads_init_ok=%u attempted=%u attempt_age_ms=%lu "
                    "adc_present=%u adc_raw=%d V=%.3f computed=%.3f quality=%u "
                    "read_reason=%s calc_temp=%.3f temp_valid=%u published=%.3f publish=%s\n",
                    (unsigned)sensors.adsInitialized, (unsigned)sensors.adsSampleAttempted,
                    (unsigned long)(sensors.adsSampleAttempted
                        ? diagnosticMs - sensors.adsSampleMs : 0),
                    (unsigned)sensors.ph.samplePresent, (int)sensors.ph.rawAdc,
                    sensorPhV, sensors.phRead, (unsigned)sensors.phQuality,
                    DiagnosticFormatting::phReadReasonName(phDiagnosticReason(sensors.ph.reason)),
                    sensors.ph.effectiveTempC, (unsigned)sensors.ph.temperatureValid,
                    published.water_ph,
                    DiagnosticFormatting::phSuppressionReasonName(
                        (DiagnosticFormatting::PhSuppressionReason)sensors.phPublishReason));
                const char* turbReason = !sensors.adsInitialized ? "ADS_UNAVAILABLE"
                    : (!sensors.adsSampleAttempted ? "NO_SAMPLE"
                        : (!isfinite(sensors.turbRead) ? "ADC_OR_COMPUTATION_INVALID"
                            : (!TURBIDITY_OPTICS_OK ? "OPTICS_DISABLED"
                                : (sensors.turbQuality ? "OK" : "LOW_VOLTAGE"))));
                Serial.printf(
                    "[SENSOR] TURB ads_init_ok=%u V=%.3f computed=%.3f quality=%u "
                    "read_reason=%s published=%.3f publish=%s attempt_age_ms=%lu\n",
                    (unsigned)sensors.adsInitialized, sensorTurbV, sensors.turbRead,
                    (unsigned)sensors.turbQuality, turbReason, published.water_turbidity,
                    published.current_mode == STATE_STANDBY ? "MODE_SUPPRESSED"
                        : (sensors.turbQuality ? "LAST_VALID" : "CACHED_OR_NO_VALID_SAMPLE"),
                    (unsigned long)(sensors.adsSampleAttempted
                        ? diagnosticMs - sensors.adsSampleMs : 0));
                Serial.printf(
                    "[SENSOR] NAV gps_ready=%u raw_lat=%.6f raw_lon=%.6f sats=%u "
                    "hdop=%.2f location_age_ms=%lu speed_kmh=%.2f "
                    "mode=%u applied_pwm=%d,%d batt_V=%.3f\n",
                    (unsigned)sensors.gpsReady, sensors.gpsRawLat, sensors.gpsRawLon,
                    (unsigned)sensors.gpsSats, sensors.gpsHdop,
                    (unsigned long)sensors.gpsAgeMs, sensors.gpsSpeedKmh,
                    (unsigned)published.current_mode, (int)sensors.motorLeft,
                    (int)sensors.motorRight, sensorBattV);
            } else {
                Serial.println(F("[SENSOR] SNAPSHOT_UNAVAILABLE (not a sensor failure)"));
            }
            if (ctx->logger) {
                Serial.printf("[SD] ready=%u main=%lu/%lu/%lu ph=%lu/%lu/%lu pending_bytes=%u,%u max_service_ms=%lu max_sync_gap_ms=%lu sync_misses=%lu write_fail=%lu ph_fail=%lu tx_fail=%lu ph_stopped=%u queue=%u/%u\n",
                    (unsigned)ctx->logger->isReady(),
                    (unsigned long)ctx->logger->mainAcceptedRows(), (unsigned long)ctx->logger->mainWrittenRows(), (unsigned long)ctx->logger->mainSyncedRows(),
                    (unsigned long)ctx->logger->phAcceptedRows(), (unsigned long)ctx->logger->phWrittenRows(), (unsigned long)ctx->logger->phSyncedRows(),
                    (unsigned)ctx->logger->mainPendingBytes(), (unsigned)ctx->logger->phPendingBytes(),
                    (unsigned long)ctx->logger->maxServiceMs(), (unsigned long)ctx->logger->maxSyncGapMs(),
                    (unsigned long)ctx->logger->syncDeadlineMisses(), (unsigned long)ctx->logger->writeFailCount(),
                    (unsigned long)ctx->logger->phDiagnosticWriteFailCount(), (unsigned long)ctx->logger->txDiagnosticWriteFailCount(),
                    (unsigned)ctx->logger->phDiagnosticStopped(),
                    (unsigned)uxQueueMessagesWaiting(ctx->logQueue), (unsigned)SD_LOG_QUEUE_LENGTH);
            }
#endif
        }

        ctx->wdt->feed();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
