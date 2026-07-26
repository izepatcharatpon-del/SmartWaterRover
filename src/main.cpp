#include <Arduino.h>
#include <Wire.h>

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

// TODO: replace with the shore/gateway board's real MAC.
static uint8_t GATEWAY_MAC[6] = {0x24, 0x6F, 0x28, 0x00, 0x00, 0x01};

static void haltFatal(const char* msg) {
    Serial.println(msg);
    motor.stop();
    while (true) {
        delay(1000);
    }
}

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println(F("[USV] boot"));

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setTimeOut(I2C_TIMEOUT_MS);

    const bool waterOk = water.begin();
    if (!waterOk) {
        Serial.println(F("[USV] ADS1115 init FAILED — anomaly detection disabled"));
    }
    battery.begin();
    if (compass.begin()) {
        Serial.printf("[USV] compass OK (QMC5883P @ 0x%02X)\n", compass.i2cAddress());
    } else {
        Serial.println(F("[USV] compass FAIL — need QMC5883P@0x2C chipID 0x80"));
    }
    compass.setDeclination(MAGNETIC_DECLINATION_DEG);
    gps.begin();
    gps.configureSeaMode();
    motor.begin();
    const bool loggerOk = logger.begin();
    if (!loggerOk) {
        // SD is optional: boat must still run without a card. ioTask skips
        // logRecord when logger is null / not ready.
        Serial.println(F("[USV] SD logger init FAILED — continuing without SD"));
    }

    if (!watchdog.begin(WATCHDOG_TIMEOUT_S)) {
        Serial.println(F("[USV] watchdog init FAILED — continuing without TWDT"));
    }

    const bool commsOk = commsLink.begin(ESPNOW_WIFI_CHANNEL, GATEWAY_MAC);
    if (!commsOk) {
        Serial.println(F("[USV] ESP-NOW init FAILED"));
    }

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
    ctx.logDropCount = 0;
    ctx.ackDropCount = 0;

    ctx.logQueue   = xQueueCreate(16, sizeof(TelemetryPacket));
    ctx.stateMutex = xSemaphoreCreateMutex();

    // Logging failure must not prevent navigation / motor control tasks.
    if (!commsOk || !ctx.logQueue || !ctx.stateMutex) {
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
