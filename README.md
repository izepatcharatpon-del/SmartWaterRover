# Smart Water Rover — Boat Firmware

เฟิร์มแวร์เรือสำรวจคุณภาพน้ำ รันบน ESP32-S3  
โปรเจกต์นี้เป็นฝั่งเรืออย่างเดียว คุยกับบอร์ดฝั่งชายฝั่งผ่าน ESP-NOW ไม่ใช้ HTTP

สัญญาแพ็กเก็ตอยู่ที่ `include/Packets.h` — ฝั่งเกตเวย์ต้องใช้ไฟล์นี้ชุดเดียวกัน

## สิ่งที่เรือทำได้

- ลาดตระเวนตาม waypoint (LOS + PID หัวเรือ)
- กลับบ้าน (RTH) และหยุดฉุกเฉิน (E-Stop)
- อ่าน GPS, เข็มทิศ, pH, อุณหภูมิ, แรงดันแบต
- วนเกลียวตอนน้ำผิดปกติ (ตอนนี้ปิด NTU ไว้ เพราะหัวขุ่นชุดเก่ายังอ่านไม่ขึ้น)

โหมดในเทเลเมทรี: `0` STANDBY, `1` GRID, `2` SPIRAL, `3` RTH, `4` ESTOP

## ฮาร์ดแวร์หลัก

- ESP32-S3 DevKit
- GPS NEO-8M (UART2, GPIO17 RX / GPIO16 TX)
- เข็มทิศ QMC5883P (I2C 0x2C)
- ADS1115 + หัว pH / ความขุ่น, DS18B20
- มอเตอร์ BTS7960 สองตัว
- แบต LiPo 3S ก้อนเดียว เลี้ยงทั้งบอร์ดและมอเตอร์ (USB ใช้แฟลชอย่างเดียว)

พินและค่าจูนทั้งหมดอยู่ใน `include/config.h`

## Build / แฟลช

ต้องมี [PlatformIO](https://platformio.org/)

```bash
pio run -e esp32s3
pio run -e esp32s3 -t upload
pio device monitor
```

เทสต์โลจิกบนคอม (ไม่ต้องเสียบบอร์ด):

```bash
pio test -e native
```

อย่าใช้ `-e native` ตอนแฟลช

## จับคู่วิทยุ

ค่าที่ต้องตรงกับเกตเวย์:

- ช่อง Wi-Fi: `ESPNOW_WIFI_CHANNEL` ใน `config.h`
- MAC ของเกตเวย์: `GATEWAY_MAC` ใน `src/main.cpp`
- PMK / LMK / Long Range เปิดทั้งสองฝั่ง
- ขนาดแพ็กเก็ต: 5 / 194 / 38 / 2 / 20 / 12 ไบต์

คำสั่งจากฝั่งชายฝั่ง: `0` NORMAL, `1` ESTOP, `2` RTH, `3` START, `4` CLEAR_ESTOP, `5` UPLOAD_DONE, `6` SET_HOME, `7` FORCE_SPIRAL

ลำดับปกติ: อัป waypoint ตอน STANDBY → START → แล่น  
หลัง E-Stop ต้องกด CLEAR ก่อน อัปจุดใหม่หรือ START อีกรอบ START อย่างเดียวปลด E-Stop ไม่ได้

## โฟลเดอร์

```
include/     config.h, Packets.h
src/core/    task, FSM
src/navigation/
src/control/
src/comms/
src/sensors/
src/actuators/
src/safety/
src/detection/
src/storage/
test/        Unity native tests
```

มี 2 task: `navigationTask` (core 1, 20 Hz) คุมมอเตอร์/FSM, `ioTask` (core 0) ส่งวิทยุและเขียน SD
