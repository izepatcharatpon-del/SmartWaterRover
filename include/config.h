#ifndef USV_CONFIG_H
#define USV_CONFIG_H

// I2C — QMC5883P + ADS1115
#define I2C_SDA_PIN            8
#define I2C_SCL_PIN            9
#define I2C_TIMEOUT_MS         25
#define QMC5883P_I2C_ADDR      0x2C
#define QMC5883P_CHIP_ID       0x80
#define ADS1115_I2C_ADDR       0x48
#define MAGNETIC_DECLINATION_DEG (-0.6f)  // east positive, central Thailand
// Applied after atan2. Positive = clockwise from the chip axes.
#define COMPASS_YAW_OFFSET_DEG   (-188.6f)
#define COMPASS_YAW_MIRROR       1         // 1 = 360 - heading
#define COMPASS_X_MIN          (-2048.0f)
#define COMPASS_X_MAX          ( 2048.0f)
#define COMPASS_Y_MIN          (-2048.0f)
#define COMPASS_Y_MAX          ( 2048.0f)
#define COMPASS_CAL_MIN_SPAN   600.0f
#define COMPASS_CAL_MIN_SAMPLES 40
#define COMPASS_CAL_BUTTON_PIN         0   // DevKit BOOT, active low
#define COMPASS_CAL_BUTTON_DEBOUNCE_MS 50
#define COMPASS_CAL_BUTTON_ARM_MS      3000UL

// GPS NEO-8M on UART2 (GPIO17 = ESP RX)
#define GPS_RX_PIN             17
#define GPS_TX_PIN             16
#define GPS_BAUD               9600
#define GPS_MAX_BYTES_PER_UPDATE 256

#define TEMP_SENSOR_PIN        18

#define SD_MOSI_PIN            11
#define SD_SCK_PIN             12
#define SD_MISO_PIN            13
#define SD_CS_PIN              14

// AA118 5:1. One 3S pack feeds motors + board; USB is programming only.
#define BATTERY_ADC_PIN        4
#define BATTERY_DIVIDER_RATIO  5.0f
#define BATTERY_VOLTAGE_EMA_ALPHA 0.15f
#define VOLTAGE_RTH_LOADED     10.30f
#define BATTERY_LOAD_PWM_THRESH 80

// BTS7960 — R_EN / L_EN tied HIGH, not driven from GPIO
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

#define NAV_LOOP_PERIOD_MS     50
#define WATER_ADS_EVERY_N_LOOPS 5
#define HEADING_KP             2.50f
#define HEADING_KI             0.05f
#define HEADING_KD             0.80f
#define PID_DERIV_ALPHA        0.30f
#define WAYPOINT_RADIUS_M      2.5f
#define LOS_LOOKAHEAD_M        2.0f   // shorter than 2.5 m → less corner cut
#define LOS_PASS_MARGIN_M      0.5f
#define LOS_NEAR_SLOW_M        5.0f
#define LOS_TURN_FIRST_DEG     25.0f  // in-place sooner; 35° still carved a wide arc
#define RTH_DEPART_RADIUS_M    12.0f
#define RTH_ARRIVE_RADIUS_M    1.0f   // park on stored SET_HOME; GPS cannot hit 0 m
#define RTH_MIN_HOLD_MS        3000UL // must stay inside arrive radius this long
#define RTH_DEPART_DWELL_MS    2000UL
#define HEADING_STOP_TOL_DEG   3.0f
#define MANUAL_CMD_TIMEOUT_MS  500UL

// Pond GPS cannot hit a 0.45 m ball: the boat orbited each vertex (ellipse).
// First ring and chord must stay outside SPIRAL_WAYPOINT_RADIUS_M.
#define SPIRAL_A_M             5.0f
#define SPIRAL_MAX_RADIUS_M    15.0f
#define SPIRAL_MAX_THETA       (6.0f * 3.14159265f)   // 3 turns
#define SPIRAL_B_M             ((SPIRAL_MAX_RADIUS_M - SPIRAL_A_M) / SPIRAL_MAX_THETA)
#define SPIRAL_DTHETA          (3.14159265f / 4.0f)   // 45°; inner chord ~3.8 m
#define SPIRAL_WAYPOINT_RADIUS_M 2.0f                 // same family as RTH arrive

#define PH_BUF_401             4.01f
#define PH_BUF_686             6.86f
#define PH_BUF_918             9.18f
#define PH_CAL_V401            3.0095f
#define PH_CAL_V686            2.5725f
#define PH_CAL_V918            2.1836f
#define PH_NEUTRAL_VOLTAGE_V   PH_CAL_V686
#define PH_SLOPE_25C_V         0.05916f
#define PH_SLOPE_SCALE         1.00f
// Bench 2026-09-03 dip series (LM393 A → ADS A1). Not SEN0189 4.1 V.
// dry 1.569 | clear 1.855 | mild 1.764 | step-2 discarded (1.782 clearer) | very turbid 1.20
#define TURBIDITY_CLEAR_V      1.86f
#define TURBIDITY_DIRTY_V      1.20f
#define TURBIDITY_DIRTY_NTU    80.0f
#define TURBIDITY_FAULT_V      0.50f
#define TURBIDITY_NTU_SCALE    1.00f
#define TURBIDITY_NTU_OFFSET   0.00f
#define TURBIDITY_OPTICS_OK    1

#define ANOMALY_CLEAR_HYST_NTU 10.0f
#define ANOMALY_COOLDOWN_MS    45000UL
#define ANOMALY_ROC_WINDOW_MS  2000UL
#define ANOMALY_BASELINE_DELTA_NTU 12.0f
#define ANOMALY_BASELINE_DWELL_MS  4000UL
#define ANOMALY_DISTURBANCE_MS     3000UL
#define TURBIDITY_WARNING_NTU  25.0f
#define TURBIDITY_WASTEWATER_NTU 50.0f
#define TURBIDITY_ROC_LIMIT    15.0f
#define PH_LOW_LIMIT           6.0f
#define PH_HIGH_LIMIT_DAY      9.2f
#define PH_HIGH_LIMIT_NIGHT    8.8f

#define VOLTAGE_FULL           12.6f
#define VOLTAGE_RTH            10.5f
#define VOLTAGE_CRITICAL       10.0f
#define VOLTAGE_RECOVER_HYST   0.2f
#define VOLTAGE_SANE_MIN       5.0f
#define VOLTAGE_SANE_MAX       15.0f

#define GPS_MIN_SATS           6
#define GPS_MAX_HDOP           2.0f
#define GPS_FILTER_ALPHA       0.2f
#define GPS_FILTER_ALPHA_FAST  0.55f
#define GPS_FILTER_SPEED_KMH   0.8f
#define GPS_FILTER_HDOP_GOOD   1.5f
#define GPS_FRESH_FIX_MS       2000UL
#define GPS_TIMEZONE_OFFSET_H  7
#define GPS_LOSS_HOLD_MS       10000UL
#define GPS_LOSS_SAFE_STOP_MS  60000UL
#define GPS_LOSS_FAULT_MS      120000UL

#define ESPNOW_WIFI_CHANNEL    6
#define TELEMETRY_INTERVAL_MS  1000
#define RADIO_TRACE            1
#define RADIO_DIAG_MS          5000UL
#define BATTERY_REPORT_MS      300000UL
#define ESPNOW_TX_RETRIES      3
#define ESPNOW_TX_TIMEOUT_MS   40
#define STAGE_STATUS_QUEUE_LENGTH 8
#define SD_INTEGRITY_MS        30000UL
#define WATCHDOG_TIMEOUT_S     3

#endif // USV_CONFIG_H
