# Bench / HIL Test Checklist

รายการทดสอบทีละโมดูลก่อนประกอบรวม แล้วจึงทดสอบภารกิจแบบครบวงจร
(อ้างอิง Phase 9 ของแผน) ทำเครื่องหมาย `[x]` เมื่อผ่าน

## A. ทดสอบแยกโมดูล (bench)

- [ ] **WaterSensors** — จุ่มบัฟเฟอร์ pH 4/7/10 อ่านค่าใกล้เคียง; DS18B20 อ่านอุณหภูมิห้องได้; ยืนยัน I2C จาก ADS1115 ผ่าน NA019 เข้า GPIO 8/9
- [ ] **BatteryMonitor** — จ่ายแรงดันรู้ค่าเข้าโมดูล AA118 เทียบ `readVoltage()` ให้คลาดเคลื่อน < 0.1V (`BATTERY_DIVIDER_RATIO` = 5.0)
- [ ] **Compass** — รัน `runCalibration()` แบบ figure-8; หันหัวไปทิศเหนือจริงแล้ว `getHeadingDeg()` ~ 0/360
- [ ] **GpsModule** — กลางแจ้งได้ fix, `isFixValid()` เมื่อ sats>=6 & HDOP<2.0; ยืนยัน `configureSeaMode()` (UBX ACK)
- [ ] **MotorDriver** — จับสัญญาณด้วยออสซิลโลสโคป: ความถี่ ~20 kHz, duty ตาม PWM, ทิศ fwd/rev ถูก, cap ที่ ±178
- [ ] **PidController** — ป้อน error คงที่/สลับเครื่องหมาย ตรวจ anti-windup ไม่พุ่งเกิน out limits
- [ ] **SpiralController (STSMC)** — ป้อน state จำลอง ตรวจ `pwm_port/pwm_stbd` อิ่มตัวที่ ±178 และ chattering ต่ำ
- [ ] **Navigator/SpiralPlanner** — รัน `pio test -e native` ให้ผ่านทุกเคส
- [ ] **WaterAnomalyMonitor** — ป้อน turbidity spike > 15 NTU/min และ pH หลุดช่วง ตรวจ `shouldTriggerSpiral()`
- [ ] **EspNowLink** — loopback กับบอร์ด gateway (หรือ sniffer): รับ Command/Waypoint เข้า queue, ส่ง Telemetry/ACK, `sequence_number` เพิ่มขึ้น
- [ ] **DataLogger** — เขียน CSV ลง SD, header ถูก, `flush` ทุก 1 วินาที, ถอดไฟกลางคันแล้วไฟล์ไม่พัง
- [ ] **Watchdog** — จงใจ block ลูปเกิน timeout ตรวจว่ารีเซ็ตจริง
- [ ] **SafetyManager** — ไล่แรงดัน 10.5V → RTH, 10.0V → CRITICAL_STOP; จำลอง overcurrent > 25A (debounce 500ms)

## B. ทดสอบภารกิจรวม (HIL / บ่อทดลอง)

- [ ] STANDBY → (อัป waypoints + START) → **GRID_PATROL**
- [ ] จำลอง anomaly → **SPIRAL_MAPPING** → ครบรอบ → กลับ **GRID_PATROL**
- [ ] waypoints หมด → **RTH** → ถึง home (< 2.5m) → **STANDBY**
- [ ] ส่ง `CMD_EMERGENCY_STOP` ระหว่างวิ่ง → **EMERGENCY_STOP** ทันที + telemetry ยืนยัน mode=4
- [ ] แรงดันตก ≤ 10.5V ระหว่างภารกิจ → เข้า **RTH** อัตโนมัติ
- [ ] ดึง GPS หลุด > 60s → มอเตอร์หยุด (SAFE_FLOAT) และ escalate

## C. ก่อนออกภาคสนามจริง

- [ ] เปลี่ยน `GATEWAY_MAC`, `PMK`, `LMK`, `ESPNOW_WIFI_CHANNEL` เป็นค่าจริงที่ตกลงกับฝั่ง gateway
- [ ] ตั้งค่า magnetic declination ท้องถิ่นใน `compass.setDeclination()`
- [ ] ยืนยัน `BATTERY_DIVIDER_RATIO` = 5.0 (โมดูล AA118)
