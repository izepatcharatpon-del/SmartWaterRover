#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <string.h>
#include <esp_system.h>

#include "config.h"
#include "core/BuildIdentity.h"
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

static const char* i2cStatusName(uint8_t status) {
    switch (status) {
        case 0: return "ACK";
        case 2: return "NACK_ADDRESS";
        case 3: return "NACK_DATA";
        case 5: return "TIMEOUT";
        default: return "BUS_ERROR";
    }
}

static void printI2cProbe(uint8_t address) {
    Wire.beginTransmission(address);
    const uint8_t status = Wire.endTransmission();
    Serial.printf("[USV] I2C probe addr=0x%02X status=%u reason=%s SDA=%d SCL=%d\n",
                  (unsigned)address, (unsigned)status, i2cStatusName(status),
                  digitalRead(I2C_SDA_PIN), digitalRead(I2C_SCL_PIN));
}

static void printI2cScan() {
    const uint32_t started = millis();
    Serial.println(F("[USV] I2C scan begin (1..126; 2=NACK 4=ERROR 5=TIMEOUT)"));
    uint8_t found = 0, nack = 0, timeout = 0, other = 0;
    for (uint8_t a = 1; a < 127; a++) {
        Wire.beginTransmission(a);
        const uint8_t status = Wire.endTransmission();
        if (status == 0) {
            Serial.printf("[USV] I2C found addr=0x%02X\n", (unsigned)a);
            found++;
        } else if (status == 2 || status == 3) {
            nack++;
        } else if (status == 5) {
            timeout++;
        } else {
            other++;
        }
        if ((a % 16) == 0) {
            Serial.printf("[USV] I2C scan progress addr=0x%02X elapsed_ms=%lu\n",
                          (unsigned)a, (unsigned long)(millis() - started));
        }
    }
    Serial.printf("[USV] I2C scan done found=%u nack=%u timeout=%u error=%u "
                  "elapsed_ms=%lu SDA=%d SCL=%d\n",
                  (unsigned)found, (unsigned)nack, (unsigned)timeout, (unsigned)other,
                  (unsigned long)(millis() - started),
                  digitalRead(I2C_SDA_PIN), digitalRead(I2C_SCL_PIN));
}

static const char* resetReasonName(esp_reset_reason_t reason) {
    switch (reason) {
        case ESP_RST_POWERON: return "POWERON";
        case ESP_RST_EXT: return "EXTERNAL";
        case ESP_RST_SW: return "SOFTWARE";
        case ESP_RST_PANIC: return "PANIC";
        case ESP_RST_INT_WDT: return "INT_WATCHDOG";
        case ESP_RST_TASK_WDT: return "TASK_WATCHDOG";
        case ESP_RST_WDT: return "WATCHDOG";
        case ESP_RST_DEEPSLEEP: return "DEEP_SLEEP";
        case ESP_RST_BROWNOUT: return "BROWNOUT";
        case ESP_RST_SDIO: return "SDIO";
        default: return "UNKNOWN";
    }
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
    Serial.printf(
        "[USV] turb: A1 pond-relative if turbV>=%.2f (clear~%.2f dirty~%.2f)\n",
        (double)TURBIDITY_FAULT_V,
        (double)TURBIDITY_CLEAR_V,
        (double)TURBIDITY_DIRTY_V);
    Serial.println(F("[USV] TEL phV=ADS A0 turbV=ADS A1 t=DS18B20 C (raw even if ph/ntu nan)"));
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
    const esp_reset_reason_t resetReason = esp_reset_reason();
    Serial.printf("[USV] reset_reason=%s code=%u (ESP-IDF enum, not ROM rst code)\n",
                  resetReasonName(resetReason), (unsigned)resetReason);
    Serial.printf("[USV] build_id=%s schema=%s (dirty source bytes; not git HEAD)\n",
                  USV_BUILD_ID, USV_DIAGNOSTIC_SCHEMA);
    pinMode(COMPASS_CAL_BUTTON_PIN, INPUT_PULLUP);

    const bool wireOk = Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setTimeOut(I2C_TIMEOUT_MS);
    Serial.printf("[USV] I2C begin=%s SDA_pin=%u SCL_pin=%u timeout_ms=%u "
                  "SDA=%d SCL=%d (line levels only; not a voltage measurement)\n",
                  wireOk ? "OK" : "FAIL", (unsigned)I2C_SDA_PIN,
                  (unsigned)I2C_SCL_PIN, (unsigned)I2C_TIMEOUT_MS,
                  digitalRead(I2C_SDA_PIN), digitalRead(I2C_SCL_PIN));
    printI2cProbe(QMC5883P_I2C_ADDR);
    printI2cProbe(ADS1115_I2C_ADDR);

    const bool waterOk = water.begin();
    const PhCalibration phCalAtBoot = water.getCalibration();
    const char* phCalSourceAtBoot =
        WaterSensors::calibrationSourceName(water.calibrationSource());
    if (!waterOk) {
        Serial.println(F("[USV] ADS1115 init FAILED — anomaly detection disabled"));
    } else {
        if (phCalAtBoot.valid) {
            Serial.printf("[USV] pH cal OK  V4.01=%.3f V6.86=%.3f V9.18=%.3f\n",
                          phCalAtBoot.v401, phCalAtBoot.v686, phCalAtBoot.v918);
        } else {
            Serial.println(F("[USV] pH cal not set — using default Nernst (run 3-point cal)"));
        }
    }
    Serial.printf("[USV] pH calibration source=%s valid=%u per-sample provenance=UNKNOWN\n",
                  phCalSourceAtBoot, phCalAtBoot.valid ? 1U : 0U);
    Serial.printf("[USV] TEMP OneWire pin=%u devices_boot=%u async_wait_ms=750\n",
                  (unsigned)TEMP_SENSOR_PIN,
                  (unsigned)water.temperatureDiagnostics().devicesAtBoot);
    Serial.println(F("[USV] compass begin start"));
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
        Serial.printf("[USV] compass init stage=%s io_error=%u reg=0x%02X "
                      "read_bytes=%u/%u observed=0x%02X\n",
                      compass.initStage(), (unsigned)compass.lastIoError(),
                      (unsigned)compass.lastRegister(), (unsigned)compass.readCount(),
                      (unsigned)compass.readExpected(), (unsigned)compass.observedValue());
        printI2cScan();
        Serial.println(F("[USV] if scan shows 0x0D it is QMC5883L not P; 0x48 only = ADS, compass not on bus"));
    }
    compass.setDeclination(MAGNETIC_DECLINATION_DEG);
    gps.begin();
    gps.configureSeaMode();
    motor.begin();
    const bool loggerOk = logger.begin(
        phCalAtBoot.v401, phCalAtBoot.v686, phCalAtBoot.v918,
        phCalAtBoot.valid, phCalSourceAtBoot);
    if (loggerOk) {
        Serial.printf("[USV] SD logger OK %s\n", logger.filename());
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
    ctx.logger = &logger; // IO owns bounded retry even when startup SD is unavailable.
    ctx.wdt = &watchdog;
    ctx.safety = &safety; ctx.fsm = &fsm;
    ctx.batteryVoltage = VOLTAGE_FULL;
    ctx.phVoltage = NAN;
    ctx.turbVoltage = NAN;
    ctx.homeDistM = NAN;
    ctx.homeSetFlag = 0;
    ctx.snapshot.gps_lat = NAN;
    ctx.snapshot.gps_lon = NAN;
    ctx.snapshot.water_ph = NAN;
    ctx.snapshot.water_turbidity = NAN;
    ctx.snapshot.water_temp = NAN;
    ctx.snapshot.heading_angle = NAN;
    ctx.telemetryTxLat = NAN;
    ctx.telemetryTxLon = NAN;
    ctx.telemetryTxPh = NAN;
    ctx.telemetryRadioStatus = TELEMETRY_RADIO_NEVER;
    ctx.logDropCount = 0;
    ctx.ackDropCount = 0;
    ctx.stageDropCount = 0;

    ctx.logQueue   = xQueueCreate(SD_LOG_QUEUE_LENGTH, sizeof(SdLogRecord));
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
