#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <string.h>

#include "config.h"
#include "core/Tasks.h"

static WaterSensors        water;
static BatteryMonitor      battery;
static Compass             compass;
static GpsModule           gps;
static MotorDriver         motor;
static HeadingController   heading(HEADING_KP, HEADING_KI, HEADING_KD);
static Navigator           nav;
static SpiralPlanner       spiralPlanner;
static WaterAnomalyMonitor anomaly;
static EspNowLink          commsLink;
static DataLogger          logger;
static Watchdog            watchdog;
static SafetyManager       safety;
static MissionFsm          fsm;

static SystemContext ctx;

static uint8_t GATEWAY_MAC[6] = {0x88, 0x57, 0x21, 0xAD, 0xAE, 0x8C};
static const uint8_t DUMMY_GATEWAY_MAC[6] = {0x24, 0x6F, 0x28, 0x00, 0x00, 0x01};

static void printI2cScan() {
    Serial.print(F("[USV] I2C scan:"));
    uint8_t found = 0;
    for (uint8_t a = 1; a < 127; a++) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) {
            Serial.printf(" 0x%02X", a);
            found++;
        }
    }
    if (found == 0) {
        Serial.print(F(" (none)"));
    }
    Serial.println();
}

static void printMac(const char* label, const uint8_t mac[6]) {
    Serial.printf("[USV] %s %02X:%02X:%02X:%02X:%02X:%02X\n",
                  label, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static void printPairingBanner(bool commsOk) {
    uint8_t selfMac[6] = {0};
    WiFi.macAddress(selfMac);
    printMac("boat MAC  (give to gateway)", selfMac);
    printMac("peer MAC  (GATEWAY_MAC)", GATEWAY_MAC);
    if (memcmp(GATEWAY_MAC, DUMMY_GATEWAY_MAC, 6) == 0) {
        Serial.println(F("[USV] GATEWAY_MAC is still DUMMY — RX will drop the real gateway"));
    }
    Serial.printf("[USV] ESP-NOW ch=%u LR+encrypt %s\n",
                  (unsigned)ESPNOW_WIFI_CHANNEL,
                  commsOk ? "init OK" : "INIT FAIL");
    Serial.println(F("[USV] keys PMK=USV_PMK_16bytes! LMK=USV_LMK_16bytes! encrypt=1 LR=1 PS=NONE"));
    Serial.println(F("[USV] gateway must match: Packets.h sizes 5/194/38/2/20/12"));
    Serial.println(F("[USV] CMD 0=NORMAL 1=ESTOP 2=RTH 3=START_ARM 4=CLEAR_ESTOP 5=UPLOAD_DONE 6=SET_HOME 7=FORCE_SPIRAL"));
    Serial.println(F("[USV] indoor: no GPS => STANDBY + motors stopped. Pass = telemetry ~1 Hz"));
    Serial.println(F("[USV] BOOT (not RST) in STANDBY = CALMAG; press again to save"));
    Serial.println(F("[USV] click this monitor, press any key to pause/resume TEL (not C = CALMAG)"));
    Serial.println(F("[USV] turb: TEL turbV= ADS A1 volts; NTU valid only if turbV>=2.50"));
    Serial.println(F("[USV] turb trim: blue pot on analog board, ~4.1V in air/clear water"));
#if RADIO_TRACE
    Serial.println(F("[USV] radio trace [TX]/[RX] on this Serial — compare with gateway"));
#endif
}

static void haltFatal(const char* msg) {
    Serial.println(msg);
    motor.stop();
    while (true) {
        delay(1000);
    }
}

void setup() {
    Serial.begin(115200);
    Serial.setDebugOutput(false);
    delay(200);
    Serial.println(F("[USV] boot"));
    pinMode(COMPASS_CAL_BUTTON_PIN, INPUT_PULLUP);

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setTimeOut(I2C_TIMEOUT_MS);

    const bool waterOk = water.begin();
    if (!waterOk) {
        Serial.println(F("[USV] ADS1115 init FAILED — anomaly detection disabled"));
    } else {
        PhCalibration pc = water.getCalibration();
        if (pc.valid) {
            Serial.printf("[USV] pH cal OK  V4.01=%.3f V6.86=%.3f V9.18=%.3f\n",
                          pc.v401, pc.v686, pc.v918);
        } else {
            Serial.println(F("[USV] pH cal not set — using default Nernst (run 3-point cal)"));
        }
    }
    battery.begin();
    if (compass.begin()) {
        Serial.printf("[USV] compass OK (QMC5883P @ 0x%02X)\n", compass.i2cAddress());
        float xMin = 0, xMax = 0, yMin = 0, yMax = 0;
        compass.getHardIron(xMin, xMax, yMin, yMax);
        Serial.printf("[USV] compass hard-iron %s x=[%.0f,%.0f] y=[%.0f,%.0f]\n",
                      compass.nvsLoaded() ? "NVS" : "factory",
                      xMin, xMax, yMin, yMax);
        Serial.printf("[USV] compass yaw offset=%.1f deg mirror=%d (bow north ~0)\n",
                      (double)COMPASS_YAW_OFFSET_DEG, (int)COMPASS_YAW_MIRROR);
    } else {
        Serial.println(F("[USV] compass FAIL — need QMC5883P@0x2C chipID 0x80"));
        printI2cScan();
        Serial.println(F("[USV] if scan shows 0x0D it is QMC5883L not P; 0x48 only = ADS, compass not on bus"));
    }
    compass.setDeclination(MAGNETIC_DECLINATION_DEG);
    gps.begin();
    gps.configureSeaMode();
    motor.begin();
    const bool loggerOk = logger.begin();
    if (loggerOk) {
        Serial.println(F("[USV] SD logger OK (SdFat)"));
    } else {
        Serial.println(F("[USV] SD logger init FAILED — continuing without SD"));
    }

    if (!watchdog.begin(WATCHDOG_TIMEOUT_S)) {
        Serial.println(F("[USV] watchdog init FAILED — continuing without TWDT"));
    }

    const bool commsOk = commsLink.begin(ESPNOW_WIFI_CHANNEL, GATEWAY_MAC);
    if (!commsOk) {
        Serial.println(F("[USV] ESP-NOW init FAILED"));
    }
    printPairingBanner(commsOk);

    fsm.begin();

    memset(&ctx, 0, sizeof(ctx));
    ctx.water = &water;   ctx.battery = &battery; ctx.compass = &compass;
    ctx.gps = &gps;       ctx.motor = &motor;     ctx.heading = &heading;
    ctx.nav = &nav;
    ctx.spiralPlanner = &spiralPlanner; ctx.anomaly = &anomaly;
    ctx.link = &commsLink;
    ctx.logger = loggerOk ? &logger : nullptr;
    ctx.wdt = &watchdog;
    ctx.safety = &safety; ctx.fsm = &fsm;
    ctx.batteryVoltage = VOLTAGE_FULL;
    ctx.turbVoltage = NAN;
    ctx.homeDistM = NAN;
    ctx.homeSetFlag = 0;
    ctx.logDropCount = 0;
    ctx.ackDropCount = 0;
    ctx.stageDropCount = 0;

    ctx.logQueue   = xQueueCreate(16, sizeof(TelemetryPacket));
    ctx.stageQueue = xQueueCreate(STAGE_STATUS_QUEUE_LENGTH,
                                  sizeof(StageStatusPacket));
    ctx.stateMutex = xSemaphoreCreateMutex();

    if (!commsOk || !ctx.logQueue || !ctx.stageQueue || !ctx.stateMutex) {
        haltFatal("[FATAL] init failed (comms/queue/mutex)");
    }

    BaseType_t navOk = xTaskCreatePinnedToCore(
        navigationTask, "nav", 8192, &ctx, 3, nullptr, 1);
    BaseType_t ioOk = xTaskCreatePinnedToCore(
        ioTask, "io", 8192, &ctx, 2, nullptr, 0);

    if (navOk != pdPASS || ioOk != pdPASS) {
        haltFatal("[FATAL] task create failed");
    }

    Serial.println(F("[USV] tasks started"));
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
