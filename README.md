# Smart Water Rover — Boat Firmware

Firmware for an unmanned surface vehicle that surveys water quality on a pond or canal. It runs on an **ESP32-S3** and talks to a shore gateway over **ESP-NOW**. There is no HTTP stack and no phone app on the boat.

This repository is the boat side only. The shore board is a separate project. Both sides must use the same `include/Packets.h`.

Tunable numbers (pins, PWM, GPS gates, battery limits, radio channel) live in `include/config.h`. Do not scatter magic numbers through the `.cpp` files.

---

## What the boat does

A typical outing:

1. On the bank, with a GPS fix and the boat in STANDBY, the operator presses **SET_HOME**. That coordinate is the only return point.
2. The gateway uploads a waypoint list.
3. **START** arms grid patrol. START does not write home — otherwise a launch from mid-pond would store the wrong dock.
4. The boat follows the list with line-of-sight (LOS) guidance and a heading PID on two thrusters.
5. If the water looks like wastewater, it can break off into an Archimedean spiral, then resume the grid.
6. After the last waypoint it returns home (RTH), or it stops if home was never set.
7. Shore can force RTH, force a spiral, or latch E-Stop at any time.

Telemetry `current_mode` is the same enum the FSM uses:

| Value | Name | Meaning |
|------:|------|---------|
| 0 | STANDBY | Motors off, waiting for upload / START / SET_HOME |
| 1 | GRID_PATROL | Following the waypoint list |
| 2 | SPIRAL_MAPPING | Mapping around an anomaly |
| 3 | RTH | Driving back to the SET_HOME fix |
| 4 | EMERGENCY_STOP | Latched stop; only CLEAR_ESTOP leaves this state |

---

## Hardware

One **3S LiPo** feeds the board (through a buck/BEC) and both motor drivers. USB is for flashing and serial only. If you power the ESP32 from USB alone, the battery ADC looks out of range and the boat will sit in E-Stop.

| Piece | Role | Boat connection |
|-------|------|-----------------|
| ESP32-S3 DevKit | Controller, Wi-Fi radio | — |
| u-blox NEO-8M | GPS | UART2, GPIO17 = ESP RX, GPIO16 = ESP TX, 9600 baud |
| QMC5883P | Magnetic heading | I2C 0x2C, SDA 8 / SCL 9 |
| ADS1115 | pH and turbidity voltages | I2C 0x48, A0 = pH, A1 = turbidity |
| pH probe + analog front-end | pH | Calibrated at 4.01 / 6.86 / 9.18 |
| Optical turbidity probe | NTU | LM393 analog `A` → ADS A1; linear map clear 1.86 V / very turbid 1.20 V (`TURBIDITY_OPTICS_OK = 1`) |
| DS18B20 | Water temperature | 1-Wire on GPIO18 |
| AA118 5:1 divider | Pack voltage | ADC GPIO4 |
| 2× BTS7960 | Left / right thrusters | Left RPWM/LPWM = 5/6, right = 40/41. R_EN and L_EN tied HIGH, not driven from GPIO |
| MicroSD (optional) | CSV log | HSPI 11/12/13, CS 14. The boat still runs if the card is missing |
| Shore ESP32 | Gateway | ESP-NOW unicast to `GATEWAY_MAC` in `src/main.cpp` |

Compass hard-iron cal is stored in NVS (`usv_mag`). Hold the DevKit BOOT button (GPIO0) or send `CALMAG` on serial, spin the hull, press BOOT again to save. pH 3-point voltages also live in NVS (`usv_ph`).

---

## Code layout

```
include/
  config.h          pins and every tuned constant
  Packets.h         ESP-NOW wire structs (share with the gateway)
src/
  main.cpp          object wiring, peer MAC, create the two tasks
  core/             FreeRTOS tasks, mission FSM, stage events
  navigation/       waypoints, geodesy, LOS, spiral planner
  control/          heading PID, left/right PWM mix
  comms/            ESP-NOW queues, TX confirm, E-Stop event bit
  sensors/          GPS, compass, ADS/pH/NTU, battery
  actuators/        BTS7960 PWM
  safety/           voltage policy, task watchdog
  detection/        water-anomaly trigger (Schmitt / ROC / baseline)
  storage/          SdFat CSV logger
test/test_navigation/
                    Unity tests on the host (no Arduino drivers)
```

Logic that must stay unit-testable (`Navigator`, `MissionFsm`, `PidController`, `SafetyManager`, `SpiralPlanner`) does not include `Arduino.h`.

---

## Two tasks

`loop()` is empty. Everything runs in two FreeRTOS tasks.

**`navigationTask`** — core 1, priority 3, 50 ms (20 Hz)

Owns GPS/compass/water reads, LOS + heading, motors, the FSM, and waypoint-queue drain. ADS conversions take ~16 ms, so pH/NTU are sampled every `WATER_ADS_EVERY_N_LOOPS` ticks instead of every cycle.

**`ioTask`** — core 0, priority 2

Owns ESP-NOW transmit, command-queue drain, and SD writes. A mutex (`stateMutex`) copies the telemetry snapshot and operator flags between the two cores. A timeout on that mutex must not drop an inbound command; the IO task keeps the packet and retries.

E-Stop is not only a queue message. The radio callback also sets an EventGroup bit so a full command queue cannot swallow the stop.

---

## Mission FSM

`MissionFsm` takes a plain `FsmInputs` struct. It never talks to hardware.

```
STANDBY --START (GPS + compass + sealed waypoints, battery NORMAL)--> GRID
GRID    --anomaly or FORCE_SPIRAL--> SPIRAL --done--> GRID
GRID    --list finished, home set--> RTH --arrived--> STANDBY
GRID    --list finished, no home--> STANDBY
any active --FORCE_RTH or low battery--> RTH (or ESTOP if home is missing)
any      --E-Stop / GPS fault / CRITICAL--> ESTOP --CLEAR--> STANDBY
```

Rules that are easy to get wrong:

- START never clears E-Stop. Only `CMD_CLEAR_ESTOP`.
- A CLEAR received while not in E-Stop is discarded, so it cannot auto-exit the next stop.
- RTH with no home is E-Stop, not a drive toward (0, 0).
- Arrival at home is allowed even if the pack is still in the low-battery band, otherwise the boat loops in RTH forever.
- RTH only finishes after the boat has been **≥ 12 m** from home for two seconds (so one GPS jump does not count as “departed”), then parked inside **1 m** of the stored SET_HOME fix for three seconds.
- Waypoints are accepted only in STANDBY. Batch `packet_index == 0` replaces the list. Later batches append. Uploads during GRID / RTH / E-Stop are NACK’d and the old list stays.

---

## Navigation and control

Waypoints are WGS84 degrees. Distance and bearing are haversine / forward azimuth. LOS projects the boat onto the current segment (previous point → current target; the first segment starts at the GPS fix when START is accepted — that origin is **not** home).

If heading error is larger than `LOS_TURN_FIRST_DEG` (25°), forward PWM is cut and the hull yaws in place. Near a waypoint the base PWM drops. A side that would sit inside the BTS7960 deadband is lifted to `MOTOR_MIN_PWM`.

The spiral is Archimedean, first ring 5 m, 45° steps, three turns out to 15 m. Guidance is LOS between consecutive points (not a 0.45 m go-to). The centre is the GPS fix at spiral entry. Turbidity is ignored until a new optical probe is fitted; **out-of-range pH or `CMD_FORCE_SPIRAL` can still start a spiral**.

GPS is quality-gated (sats, HDOP, age) then EMA-filtered. There is no dead reckoning. After a long outage the boat safe-stops, then latches GPS fault → E-Stop.

---

## Shore radio

ESP-NOW unicast, same Wi-Fi channel and PMK/LMK on both boards, modem sleep off (`WIFI_PS_NONE`) so ACKs are not dropped. Long-range 802.11 LR is enabled with b/g/n.

The air link is demuxed by **length**, not by a type byte:

| Bytes | Packet | Direction |
|------:|--------|-----------|
| 5 | `CommandPacket` | shore → boat |
| 194 | `WaypointArrayPacket` (up to 12 points) | shore → boat |
| 2 | `WaypointACKPacket` | boat → shore |
| 38 | `TelemetryPacket` (~1 Hz) | boat → shore |
| 20 | `BatteryReportPacket` | boat → shore |
| 12 | `StageStatusPacket` (on mode edges) | boat → shore |

`sys_command` values: `0` NORMAL (also manual stick), `1` ESTOP, `2` RTH, `3` START, `4` CLEAR_ESTOP, `5` UPLOAD_DONE, `6` SET_HOME, `7` FORCE_SPIRAL.

Manual stick dies after 500 ms without a NORMAL packet. Water fields in telemetry are NaN in STANDBY and E-Stop; they are filled only while GRID / SPIRAL / RTH.

`Packets.h` has `static_assert` on every size. If you add a field, the gateway breaks until both sides rebuild from the same header.

---

## Safety and power

`SafetyManager` looks at pack voltage only (over-current and leak inputs are reserved).

- Idle RTH threshold 10.5 V, loaded RTH 10.30 V (thrust sag is normal).
- CRITICAL at 10.0 V; it stays latched until the pack recovers to RTH + 0.2 V.
- Missing home + low battery or Force RTH → E-Stop.
- A 3 s task watchdog is subscribed from both tasks.

SD logging is optional. A full log queue or a missing card must not stall navigation.

---

## Tests

Mission math and the FSM are covered by Unity tests under `test/`. Those run on the PC and never pull in the Arduino drivers. Bench and pond checks stay on the boat; a passing host test does not mean the radio or motors are proven.
