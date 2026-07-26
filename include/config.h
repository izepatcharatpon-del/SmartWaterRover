#ifndef USV_CONFIG_H
#define USV_CONFIG_H

// =============================================================================
// Smart Water Surface Rover (USV) - Central configuration
// Pins/addresses: 2_Documentation/USV_Hardware_GPIO_Mapping.md
// =============================================================================

// ----------------------- I2C (QMC5883P + ADS1115 via NA019) ------------------
#define I2C_SDA_PIN            8
#define I2C_SCL_PIN            9
#define I2C_TIMEOUT_MS         25
#define QMC5883P_I2C_ADDR      0x2C
#define QMC5883P_CHIP_ID       0x80
#define ADS1115_I2C_ADDR       0x48
// Magnetic declination (degrees, east positive). Approx. central Thailand.
#define MAGNETIC_DECLINATION_DEG (-0.6f)

// ----------------------- UART2 (GPS NEO-8M) ----------------------------------
#define GPS_RX_PIN             16
#define GPS_TX_PIN             17
#define GPS_BAUD               9600
#define GPS_MAX_BYTES_PER_UPDATE 256

// ----------------------- 1-Wire (DS18B20) ------------------------------------
#define TEMP_SENSOR_PIN        38

// ----------------------- SPI (Micro SD) --------------------------------------
#define SD_CS_PIN              21
#define SD_MOSI_PIN            36
#define SD_SCK_PIN             35
#define SD_MISO_PIN            37

// ----------------------- Battery voltage ADC (AA118 5:1) ---------------------
#define BATTERY_ADC_PIN        4
#define BATTERY_DIVIDER_RATIO  5.0f

// ----------------------- Motors (BTS7960 x2) ---------------------------------
#define LEFT_MOTOR_RPWM        5
#define LEFT_MOTOR_LPWM        6
#define RIGHT_MOTOR_RPWM       40
#define RIGHT_MOTOR_LPWM       41
#define LEFT_MOTOR_IN1         LEFT_MOTOR_RPWM
#define LEFT_MOTOR_IN2         LEFT_MOTOR_LPWM
#define RIGHT_MOTOR_IN3        RIGHT_MOTOR_RPWM
#define RIGHT_MOTOR_IN4        RIGHT_MOTOR_LPWM
#define LEFT_LEDC_CH_RPWM      0
#define LEFT_LEDC_CH_LPWM      1
#define RIGHT_LEDC_CH_RPWM     2
#define RIGHT_LEDC_CH_LPWM     3
#define LEFT_LEDC_CH_IN1       LEFT_LEDC_CH_RPWM
#define LEFT_LEDC_CH_IN2       LEFT_LEDC_CH_LPWM
#define RIGHT_LEDC_CH_IN3      RIGHT_LEDC_CH_RPWM
#define RIGHT_LEDC_CH_IN4      RIGHT_LEDC_CH_LPWM
#define MOTOR_PWM_FREQ         20000
#define MOTOR_PWM_RES          8
#define MOTOR_PWM_LIMIT        178
#define MOTOR_MIN_PWM          80
#define MOTOR_CRUISE_PWM       150
#define MOTOR_APPROACH_PWM     110

// ----------------------- Navigation / control --------------------------------
#define NAV_LOOP_PERIOD_MS     50
#define WATER_ADS_EVERY_N_LOOPS 5
#define HEADING_KP             2.50f
#define HEADING_KI             0.05f
#define HEADING_KD             0.80f
#define PID_DERIV_ALPHA        0.30f
#define WAYPOINT_RADIUS_M      2.5f
#define LOS_LOOKAHEAD_M        4.0f
#define LOS_PASS_MARGIN_M      0.5f
#define LOS_NEAR_SLOW_M        5.0f
#define LOS_TURN_FIRST_DEG     60.0f
#define HEADING_STOP_TOL_DEG   3.0f
#define MANUAL_CMD_TIMEOUT_MS  500UL

// ----------------------- Archimedean spiral ----------------------------------
#define SPIRAL_A_M             2.0f
#define SPIRAL_B_M             0.6897f
#define SPIRAL_MAX_RADIUS_M    15.0f
#define SPIRAL_MAX_THETA       (6.0f * 3.14159265f)
#define SPIRAL_DTHETA          0.5236f
// Must be << chord at r=SPIRAL_A_M (~1.03 m).
#define SPIRAL_WAYPOINT_RADIUS_M 0.45f

// ----------------------- Anomaly / wastewater --------------------------------
#define ANOMALY_CLEAR_HYST_NTU 10.0f
#define ANOMALY_COOLDOWN_MS    45000UL
#define ANOMALY_ROC_WINDOW_MS  2000UL   // ROC uses ≥2 s window (not per-ADS dt)
#define TURBIDITY_WARNING_NTU  25.0f
#define TURBIDITY_WASTEWATER_NTU 50.0f
#define TURBIDITY_ROC_LIMIT    15.0f    // NTU/min over ANOMALY_ROC_WINDOW_MS
#define PH_LOW_LIMIT           6.0f
#define PH_HIGH_LIMIT_DAY      9.2f
#define PH_HIGH_LIMIT_NIGHT    8.8f

// ----------------------- Battery (LiPo 3S) -----------------------------------
#define VOLTAGE_FULL           12.6f
#define VOLTAGE_RTH            10.5f
#define VOLTAGE_CRITICAL       10.0f
#define VOLTAGE_RECOVER_HYST   0.2f     // CRITICAL stays until ≥ RTH+hyst
#define VOLTAGE_SANE_MIN       5.0f
#define VOLTAGE_SANE_MAX       15.0f

// ----------------------- GPS loss tiers --------------------------------------
#define GPS_MIN_SATS           6
#define GPS_MAX_HDOP           2.0f
#define GPS_FILTER_ALPHA       0.2f
#define GPS_FRESH_FIX_MS       2000UL
#define GPS_TIMEZONE_OFFSET_H  7
#define GPS_LOSS_HOLD_MS       10000UL
#define GPS_LOSS_SAFE_STOP_MS  60000UL
#define GPS_LOSS_FAULT_MS      120000UL

// ----------------------- ESP-NOW / logging / WDT -----------------------------
#define ESPNOW_WIFI_CHANNEL    1
#define TELEMETRY_INTERVAL_MS  1000
#define BATTERY_REPORT_MS      300000UL
#define ESPNOW_TX_RETRIES      3
#define ESPNOW_TX_TIMEOUT_MS   40
#define SD_INTEGRITY_MS        30000UL
#define WATCHDOG_TIMEOUT_S     3

#endif // USV_CONFIG_H
