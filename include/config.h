#ifndef USV_CONFIG_H
#define USV_CONFIG_H

// =============================================================================
// Smart Water Surface Rover (USV) - Central configuration
// Pins/addresses: 2_Documentation/USV_Hardware_GPIO_Mapping.md
// Tuning/thresholds: 2_Documentation/hardware_pinout_tuning_specs.md
// Locked defaults: 1_Firmware/DECISIONS.md
//
// ESP32-S3-DevKitC-1 pin layout (USB at bottom, headers toward you):
//   LEFT header:  4,5,6,8,9,10,16,17  (+ 3V3/GND/RST)
//   RIGHT header: 40,41,21,35,36,37,38  (+ GND; avoid 19/20 = USB, 43/44 = UART0)
// =============================================================================

// ----------------------- I2C bus (QMC5883L + ADS1115 via NA019) ----------------
// ADS1115 runs at 5V; SDA/SCL pass through NA019 (5V -> 3.3V) to ESP32.
// pH / turbidity analog inputs go directly to ADS1115 A0/A1 (0-5V). No INA219.
#define I2C_SDA_PIN            8    // LEFT — Wire.begin(SDA, SCL)
#define I2C_SCL_PIN            9    // LEFT
#define QMC5883L_I2C_ADDR      0x0D
#define ADS1115_I2C_ADDR       0x48

// ----------------------- UART2 (GPS NEO-8M) ----------------------------------
#define GPS_RX_PIN             16   // LEFT — <- GPS TX
#define GPS_TX_PIN             17   // LEFT — -> GPS RX
#define GPS_BAUD               9600

// ----------------------- 1-Wire (DS18B20 water temperature) ------------------
#define TEMP_SENSOR_PIN        38   // RIGHT

// ----------------------- SPI (Micro SD card) ---------------------------------
#define SD_CS_PIN              21   // RIGHT
#define SD_MOSI_PIN            36   // RIGHT
#define SD_SCK_PIN             35   // RIGHT
#define SD_MISO_PIN            37   // RIGHT

// ----------------------- Battery voltage ADC ---------------------------------
#define BATTERY_ADC_PIN        4    // LEFT — ADC1
// Decision #4: AA118 voltage sensor module (built-in 5:1 divider, 0-25V in)
#define BATTERY_DIVIDER_RATIO  5.0f

// ----------------------- Motor driver (BTS7960 x2) -----------------------------
// Each BTS7960: RPWM = forward PWM, LPWM = reverse PWM (R_EN/L_EN tied HIGH).
// MotorDriver: positive thrust -> RPWM, negative -> LPWM (never both active).
//
//   LEFT thruster  (header LEFT):  GPIO5 = RPWM, GPIO6 = LPWM
//   RIGHT thruster (header RIGHT): GPIO40 = RPWM, GPIO41 = LPWM
//
// LEDC channels are arbitrary software slots (not GPIO numbers).
#define LEFT_MOTOR_RPWM        5    // LEFT header — forward
#define LEFT_MOTOR_LPWM        6    // LEFT header — reverse
#define RIGHT_MOTOR_RPWM       40   // RIGHT header — forward
#define RIGHT_MOTOR_LPWM       41   // RIGHT header — reverse

// Legacy aliases (used by MotorDriver.cpp)
#define LEFT_MOTOR_IN1         LEFT_MOTOR_RPWM
#define LEFT_MOTOR_IN2         LEFT_MOTOR_LPWM
#define RIGHT_MOTOR_IN3        RIGHT_MOTOR_RPWM
#define RIGHT_MOTOR_IN4        RIGHT_MOTOR_LPWM

#define LEFT_LEDC_CH_RPWM      0 // R_EN
#define LEFT_LEDC_CH_LPWM      1 // L_EN
#define RIGHT_LEDC_CH_RPWM     2 // R_EN
#define RIGHT_LEDC_CH_LPWM     3 // L_EN

// Legacy LEDC aliases
#define LEFT_LEDC_CH_IN1       LEFT_LEDC_CH_RPWM
#define LEFT_LEDC_CH_IN2       LEFT_LEDC_CH_LPWM
#define RIGHT_LEDC_CH_IN3      RIGHT_LEDC_CH_RPWM
#define RIGHT_LEDC_CH_IN4      RIGHT_LEDC_CH_LPWM
#define MOTOR_PWM_FREQ         20000  // 20 kHz (avoid coil whine)
#define MOTOR_PWM_RES          8      // 8-bit (0-255)
#define MOTOR_PWM_LIMIT        178    // 70% throttle cap for battery runtime

// ----------------------- Current sensor (ACS712) -----------------------------
#define CURRENT_SENSE_PIN      10     // LEFT — ADC1 (optional)
#define OVERCURRENT_LIMIT_A    25.0f  // motor stall cutoff

// ----------------------- Navigation / control tuning -------------------------
#define NAV_LOOP_HZ            20
#define NAV_LOOP_PERIOD_MS     50     // 1000 / NAV_LOOP_HZ
#define HEADING_KP             2.50f
#define HEADING_KI             0.05f
#define HEADING_KD             0.80f
#define PID_DERIV_ALPHA        0.30f  // low-pass filter on derivative term

#define MOTOR_MIN_PWM          80
#define MOTOR_CRUISE_PWM       150
#define MOTOR_MAX_PWM          255

#define WAYPOINT_RADIUS_M      2.5f

// ----------------------- Archimedean spiral parameters -----------------------
#define SPIRAL_A_M             2.0f   // start radius
#define SPIRAL_B_M             1.5f   // growth per radian
#define SPIRAL_MAX_RADIUS_M    15.0f
#define SPIRAL_MAX_THETA       (6.0f * 3.14159265f)  // 3 turns
#define SPIRAL_DTHETA          0.5236f // ~30 deg per step

// ----------------------- Wastewater thresholds -------------------------------
#define TURBIDITY_WARNING_NTU  25.0f
#define TURBIDITY_WASTEWATER_NTU 50.0f
#define TURBIDITY_ROC_LIMIT    15.0f   // NTU/min spike
#define PH_LOW_LIMIT           6.0f
#define PH_HIGH_LIMIT          9.0f    // static reference
#define PH_HIGH_LIMIT_DAY      9.2f    // 12:00-17:00 relaxed
#define PH_HIGH_LIMIT_NIGHT    8.8f
// Decision #3: real NH3 danger fraction
#define AMMONIA_TOXICITY_LIMIT 0.05f

// ----------------------- Battery safety thresholds (LiPo 3S) -----------------
#define BATTERY_CAPACITY_MAH   2200.0f
#define VOLTAGE_FULL           12.6f
#define VOLTAGE_RTH            10.5f   // return-to-home trigger
#define VOLTAGE_CRITICAL       10.0f   // software motor cutoff
#define VOLTAGE_PHYSICAL_MIN   9.9f    // absolute empty (hardware limit)

// ----------------------- GPS loss tiers --------------------------------------
#define GPS_MIN_SATS           6
#define GPS_MAX_HDOP           2.0f
#define GPS_FILTER_ALPHA       0.2f
#define GPS_LOSS_T1_MS         10000UL  // < 10s dead reckoning
#define GPS_LOSS_T2_MS         60000UL  // 10-60s safe float

// ----------------------- Communication (ESP-NOW) -----------------------------
// Decision (ESP-NOW): must match the shore/gateway side (see section 10 of plan)
#define ESPNOW_WIFI_CHANNEL    1
#define TELEMETRY_INTERVAL_MS  1000     // 1-2s cadence
#define WATCHDOG_TIMEOUT_S     3

#endif // USV_CONFIG_H
