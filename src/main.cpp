#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "core/Tasks.h"

// ---------------------------------------------------------------------------
// Global module instances (constructed once, wired via SystemContext).
// ---------------------------------------------------------------------------
static WaterSensors        water;
static BatteryMonitor      battery;
static Compass             compass;
static GpsModule           gps;
static MotorDriver         motor;
static HeadingController   heading(HEADING_KP, HEADING_KI, HEADING_KD);
static SpiralController    spiral;
static EllipseTracker      ellipse;
static Navigator           nav;
static SpiralPlanner       spiralPlanner;
static WaterAnomalyMonitor anomaly;
static EspNowLink          commsLink;
static DataLogger          logger;
static Watchdog            watchdog;
static SafetyManager       safety;
static MissionFsm          fsm;

static SystemContext ctx;

// TODO: replace with the shore/gateway board's real MAC (see interface contract).
static uint8_t GATEWAY_MAC[6] = {0x24, 0x6F, 0x28, 0x00, 0x00, 0x01};

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println(F("[USV] boot"));

    // --- Buses ---
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

    // --- Peripherals ---
    water.begin();
    battery.begin();
    compass.begin();
    compass.setDeclination(0.0f);        // TODO: set local magnetic declination
    gps.begin();
    gps.configureSeaMode();
    motor.begin();
    logger.begin();

    // --- Safety first ---
    watchdog.begin(WATCHDOG_TIMEOUT_S);

    // --- Comms ---
    if (!commsLink.begin(ESPNOW_WIFI_CHANNEL, GATEWAY_MAC)) {
        Serial.println(F("[USV] ESP-NOW init FAILED"));
    }

    fsm.begin();

    // --- Shared IPC ---
    ctx.water = &water;   ctx.battery = &battery; ctx.compass = &compass;
    ctx.gps = &gps;       ctx.motor = &motor;     ctx.heading = &heading;
    ctx.spiral = &spiral; ctx.ellipse = &ellipse; ctx.nav = &nav;
    ctx.spiralPlanner = &spiralPlanner; ctx.anomaly = &anomaly;
    ctx.link = &commsLink;ctx.logger = &logger;   ctx.wdt = &watchdog;
    ctx.safety = &safety; ctx.fsm = &fsm;

    ctx.logQueue   = xQueueCreate(16, sizeof(TelemetryPacket));
    ctx.stateMutex = xSemaphoreCreateMutex();
    memset(&ctx.snapshot, 0, sizeof(ctx.snapshot));
    memset(&ctx.command,  0, sizeof(ctx.command));

    // --- Tasks: Core 1 = navigation/control, Core 0 = IO ---
    xTaskCreatePinnedToCore(navigationTask, "nav", 8192, &ctx, 3, nullptr, 1);
    xTaskCreatePinnedToCore(ioTask,         "io",  8192, &ctx, 2, nullptr, 0);

    Serial.println(F("[USV] tasks started"));
}

void loop() {
    // All work runs in the pinned FreeRTOS tasks.
    vTaskDelay(pdMS_TO_TICKS(1000));
}
