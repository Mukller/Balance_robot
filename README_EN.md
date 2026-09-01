<div align="center">

**English** • [Русский](README.md)

</div>

# ESP32 Self-Balancing Robot

<p align="center">
  <a href="https://github.com/Mukller">
    <img src="https://img.shields.io/badge/Anton%20Petnitsky-Developer-0d1117?style=for-the-badge&logo=github&logoColor=white&labelColor=0d1117&color=58a6ff" alt="Anton Petnitsky" />
  </a>
</p>


An autonomous two-wheeled self-balancing robot with line following, obstacle detection, and control via a web interface.
## Project files

### Main sketch:
- **main/esp32_robot_monolith/esp32_robot_monolith.ino** — all robot logic (WiFi, web UI, sensors, motors, gripper) in a single file

### Config template:
- **secrets.h.example** — WiFi SSID/password template. Copy to `secrets.h` (gitignored, never committed)

### Test sketches (in `main/test_*/`):
- **test_i2c.ino** — I²C scanner (SDA=21, SCL=22)
- **test_gyro.ino** — gyroscope/accelerometer test (MPU6050)
- **test_line_sensor.ino** — line sensor test (5 sensors, calibrated values + graph)
- **test_line_pins.ino** — raw ADC diagnostic (find stuck pins)
- **test_line_calibrate.ino** — min/max calibration for the line sensors
- **test_distance_sensor.ino** — distance sensor test (VL53L0X)
- **test_motors.ino** — motor test: DIR reversal every 3 s
- **test_motor_simple.ino** — simplest possible motor test
- **test_servo_gripper.ino** — gripper servo test (two servos)
- **test_emergency_stop.ino** — procedural check of VL53L0X-triggered emergency stop
- **test_wifi.ino** — minimal WiFi-radio test (reboot diagnostics)

### Documentation:
- **main/LINE_FOLLOWING_GUIDE.md** — line-following guide
- **main/LINE_FOLLOWER_README.md** — line-sensor API

---

## Quick start

### 1. Install libraries (into `~/Documents/Arduino/libraries/`)
- `AsyncTCP` — `git clone https://github.com/me-no-dev/AsyncTCP.git`
- `ESPAsyncWebServer` — `git clone https://github.com/me-no-dev/ESPAsyncWebServer.git`
- `VL53L0X` — `git clone https://github.com/pololu/vl53l0x-arduino.git`

### 2. Configure WiFi (pick one)

**Option A — `secrets.h` file (recommended for competitions):**
```bash
cp secrets.h.example secrets.h
# edit secrets.h and put your SSID + password
```

**Option B — without a file (via AP + web UI):**
1. Flash the sketch with no `secrets.h`
2. ESP32 brings up the AP **`BalanceRobot-Setup`** (password `balance123`)
3. Connect to the AP, open `http://192.168.4.1/`
4. Call `GET /api/save-wifi?ssid=YOUR_SSID&pass=YOUR_PASSWORD` — creds are stored in NVS and survive reboots

### 3. Upload the sketch
Open `main/esp32_robot_monolith/esp32_robot_monolith.ino` in the Arduino IDE (ESP32 Arduino Core ≥ 3.x), pick the ESP32 board, upload.

### 4. Open the web interface
The IP is printed to the Serial Monitor (115200). At a competition: `http://<ip>/`

---

## Robot API (HTTP)

| Endpoint                              | Action                                   |
|---------------------------------------|------------------------------------------|
| `GET /`                               | Web UI (joystick + status)               |
| `GET /api/status`                     | JSON with all sensors and state          |
| `GET /api/command?cmd=straight`       | Go to STRAIGHT (2 s forward)             |
| `GET /api/command?cmd=line`           | Go to LINE_FOLLOW                        |
| `GET /api/command?cmd=stop`           | Go to IDLE (stop)                        |
| `GET /api/command?cmd=manual`         | Go to MANUAL (joystick-driven)           |
| `GET /api/joystick?throttle=&steering=`| Throttle/steering control (±200/±100)     |
| `GET /api/command?cmd=gripper_open`   | Open the gripper                          |
| `GET /api/command?cmd=gripper_close`  | Close the gripper                         |
| `GET /api/command?cmd=gripper_reset`  | Reset gripper to 0° (neutral)              |
| `GET /api/save-wifi?ssid=&pass=`      | Save WiFi in NVS and reboot               |
| `GET /api/reset-wifi`                 | Clear WiFi creds and reboot                |

---

## Control (web interface)

### Joystick
- **Up-down** = Throttle (speed)
- **Left-right** = Steering (turning)

### Buttons
- **▶ Straight** - drive 2 sec, then follow the line
- **〰 Line** - follow the line
- **⏹ Stop** - stop

### Real-time sensors
- Distance to obstacle (mm)
- 5 line sensors (graph)
- Tilt angle
- Slope status

---

## How the robot works

```
1. IDLE (waiting for a command)
   ├─ "Straight" button → STRAIGHT
   ├─ "Line" button → LINE_FOLLOW
   └─ Gripper button → Open/Close (independent)

2. STRAIGHT (2 sec forward motion)
   └─ Automatic transition → LINE_FOLLOW

3. LINE_FOLLOW (follows the line)
   ├─ If it sees an obstacle → STOPPED
   ├─ If on a slope → slow down (×0.5)
   └─ Gripper is controlled separately

4. STOPPED (obstacle detected)
   ├─ Motors stopped
   └─ "Stop" button → IDLE

Gripper (independent):
- Open: 180° (any time)
- Close: 90° (any time)
```

---

## Testing

### 1. Line sensor test

```cpp
// Upload: test_line_sensor.ino
// Serial Monitor: 115200 baud
// Outputs: Raw values, Calibrated, ASCII graph
```

### 2. Distance sensor test

```cpp
// Upload: test_distance_sensor.ino
// Serial Monitor: 115200 baud
// Outputs: distance in mm, status, ASCII graph
// Checks: 0-200mm (very close) to >1000mm (far)
```

### 3. Motor test

```cpp
// Upload: test_motors.ino
// Serial Monitor: 115200 baud
// Tests:
//   1. Both slow forward
//   2. Both fast forward
//   3. Both backward
//   4. Different speeds (synchronization)
// Outputs: step count, speed ratio
```

### 4. Gripper servo test

```cpp
// Upload: test_servo_gripper.ino
// Serial Monitor: 115200 baud
// Tests:
//   1. Left servo: 0° → 180° → 0°
//   2. Right servo: 0° → 180° → 0°
//   3. Both together (Open 180° → Close 90° → Reset 0°)
// Pins: GPIO 13 (left), GPIO 33 (right)
// Outputs: ASCII graph of rotation angles
```

### 5. Gyroscope test (MPU6050)

```cpp
// Upload: test_gyro.ino
// Serial Monitor: 115200 baud
// Checks: WHO_AM_I (0x68), then reads data
// Outputs:
//   - Gyroscope X/Y/Z (°/sec)
//   - Accelerometer X/Y/Z (g)
//   - Pitch / Roll angles
//   - Temperature (°C)
// I2C pins: SDA=GPIO 21, SCL=GPIO 22
```

---

## Parameters in code

```cpp
#define DISTANCE_THRESHOLD 300      // Stop < 300mm
#define STRAIGHT_TIME 2000          // 2 sec straight
#define SLOPE_ACCEL_THRESHOLD 2000  // Slope threshold
#define MAX_THROTTLE 550            // Max speed
#define MAX_STEERING 140            // Max steering
```

---

## GPIO wiring (ESP32)

```
I2C (for sensors):
  GPIO 21 (SDA) ← MPU6050, VL53L0X
  GPIO 22 (SCL) ← MPU6050, VL53L0X

Motors (movement):
  GPIO 27 ← Motor1 DIR (direction)
  GPIO 14 ← Motor1 STEP (steps)
  GPIO 25 ← Motor2 DIR (direction)
  GPIO 26 ← Motor2 STEP (steps)
  GPIO 12 ← Enable

Gripper (two servos):
  GPIO 13 ← Servo Left (left arm)
  GPIO 33 ← Servo Right (right arm)

Line sensor:
  GPIO 34 ← S1 (left edge)
  GPIO 35 ← S2 (left quarter)
  GPIO 36 ← S3 (center)
  GPIO 39 ← S4 (right quarter)
  GPIO 32 ← S5 (right edge)
```

---

## Help

**The robot won't connect:**
- Check the WiFi SSID/password
- Watch the Serial Monitor (115200)

**It doesn't balance:**
- Check I2C (MPU6050)
- Check the motors (test_motors.ino)

**It doesn't follow the line:**
- Calibrate the sensor (test_line_sensor.ino)
- Check the line contrast

**It doesn't see obstacles:**
- Test the sensor (test_distance_sensor.ino)
- Adjust DISTANCE_THRESHOLD

### Known issue: motor drivers sometimes "drop out" (DIR pin stops working)

Hardware bug observed at competitions: **periodically one of the stepper drivers stops responding to its DIR pin** — the motor spins in only one direction or freezes completely. The robot gets pulled sideways, balance breaks, run is lost.

**Symptoms:**
- Motor drives only forward or only backward, direction doesn't change
- May work again after a restart (then drops out again)
- One motor spins, the other is dead — even though `test_motors.ino` passed before

**Causes (by frequency):**
1. **Bad contact** on Dupont wires for DIR/STEP (GPIO 27/25 → DIR, GPIO 14/26 → STEP) — motor vibration loosens the connectors
2. **Driver overheating** — thermal shutdown kicks in until it cools down
3. **12V supply sag** when both motors start simultaneously
4. **Noise from gripper servos** on signal lines (if servos don't have a separate supply)

**What to do (in order):**
1. Re-seat and ideally **solder** the DIR/STEP wires — contact bounce is cause #1
2. Check the **common ground**: ESP32 ↔ driver ↔ motor PSU
3. Touch the driver: hot = thermal protection. Add a heatsink, lower current via Vref
4. Power the gripper servos from a **strictly separate 5V source**, otherwise noise will take drivers down
5. Quick driver reset without rebooting: pulse ENABLE (GPIO 12) HIGH then back LOW
6. **Before every run**, run test_motors.ino (test #4 — different speeds/directions): it catches a dead DIR immediately

> At a competition: if the robot starts pulling to one side — suspect the driver's DIR pin first, not the PID tuning.

---

## Pre-use checklist

- [ ] ESP32 connected
- [ ] All sensors connected
- [ ] WiFi SSID/password entered
- [ ] test_motors.ino passes (both rotation directions!)
- [ ] **DIR pins of both drivers alive** — motors change direction (common failure!)
- [ ] Drivers not overheating (heatsinks in place)
- [ ] test_line_sensor.ino works
- [ ] test_distance_sensor.ino works
- [ ] The web interface opens

---

**Version:** 3.4.0 (with web interface)
**Status:** Production Ready

---

## FULL WIRING DIAGRAM

### ESP32 Pinout (30 pin version)

```
         ┌─────────────────────┐
         │      ESP32-30       │
    GND  │  ⬤ ⬤ ⬤ ⬤ ⬤ ⬤ ⬤ ⬤  │  3.3V
    GND  │  ⬤                 ⬤ │  VUSB
    3.3V │  ⬤                 ⬤ │  GND
    RST  │  ⬤                 ⬤ │  EN
    12   │  ⬤ ENABLE MOTORS   ⬤ │  27 (MOTOR1_DIR)
    13   │  ⬤ SERVO           ⬤ │  25 (MOTOR2_DIR)
    14   │  ⬤ MOTOR1_STEP     ⬤ │  32 (LINE_S5)
    26   │  ⬀                 ⬀ │  33 (SERVO RIGHT)
    35   │  ⬀ LINE_S2         ⬀ │  34 (LINE_S1)
    39   │  ⬀ LINE_S4         ⬀ │  36 (LINE_S3)
    GND  │  ⬀                 ⬀ │  GND
    GND  │  ⬀                 ⬀ │  15
    23   │  ⬀ VSPI_MOSI       ⬀ │  2 (WIFI_LED)
    22   │  ⬀ SCL (I2C)       ⬀ │  4
    21   │  ⬀ SDA (I2C)       ⬀ │  5
    19   │  ⬀ VSPI_MISO       ⬀ │  18 (VSPI_CLK)
    GND  │  ⬀                 ⬀ │  GND
    GND  │  ⬀                 ⬀ │  17
    16   │  ⬀ RX2             ⬀ │  GND
         └─────────────────────┘
```

---

## DETAILED COMPONENT WIRING

### 1. MOTORS (NEMA17 stepper with driver)

```
Motor 1:
┌──────────────────┐
│  NEMA17 Motor   │
├──────────────────┤
│ DIR ─────→ GPIO 27
│ STEP ────→ GPIO 14
│ GND ─────→ GND
│ +12V ────→ 12V (via driver)
└──────────────────┘

Motor 2:
┌──────────────────┐
│  NEMA17 Motor   │
├──────────────────┤
│ DIR ─────→ GPIO 25
│ STEP ────→ GPIO 26
│ GND ─────→ GND
│ +12V ────→ 12V (via driver)
└──────────────────┘

Motor Enable:
GPIO 12 → Motor Driver Enable
GND ────→ GND
```

### 2. MPU6050 (Gyroscope + Accelerometer)

```
┌──────────────────┐
│     MPU6050      │
├──────────────────┤
│ SDA ─────→ GPIO 21 (I2C)
│ SCL ─────→ GPIO 22 (I2C)
│ GND ─────→ GND
│ VCC ─────→ 3.3V
│ INT ─────→ leave unconnected (optional)
└──────────────────┘

I2C address: 0x68
```

### 3. VL53L0X (Distance sensor)

```
┌──────────────────┐
│    VL53L0X       │
├──────────────────┤
│ SDA ─────→ GPIO 21 (I2C)
│ SCL ─────→ GPIO 22 (I2C)
│ GND ─────→ GND
│ VCC ─────→ 3.3V
│ XSHUT ───→ leave unconnected (optional)
└──────────────────┘

I2C address: 0x29
```

### 4. LINE SENSOR (Ldabrye 5-sensor)

```
┌─────────────────────────────┐
│  Ldabrye Line Sensor Array  │
├─────────────────────────────┤
│ S1 (Left)   ─→ GPIO 34 (ADC)
│ S2          ─→ GPIO 35 (ADC)
│ S3 (Center) ─→ GPIO 36 (ADC)
│ S4          ─→ GPIO 39 (ADC)
│ S5 (Right)  ─→ GPIO 32 (ADC)
│ GND         ─→ GND
│ VCC         ─→ 5V
└─────────────────────────────┘
```

### 5. GRIPPER (two servos)

```
┌──────────────────────────┐
│  Servo Left (MG996R)     │
├──────────────────────────┤
│ Signal ────→ GPIO 13 (PWM)
│ GND ───────→ GND
│ VCC ───────→ 5V
└──────────────────────────┘

┌──────────────────────────┐
│  Servo Right (MG996R)    │
├──────────────────────────┤
│ Signal ────→ GPIO 33 (PWM)
│ GND ───────→ GND
│ VCC ───────→ 5V
└──────────────────────────┘

Angles:
- OPEN (180°) - gripper open
- CLOSE (90°) - gripper closed
```

### 6. LEDS (optional)

```
WiFi LED:
GPIO 2 → LED+ → ⎕ → 220Ω → GND
```

### 7. POWER (Very important!)

```
┌──────────────────────────────┐
│   Power Distribution         │
├──────────────────────────────┤
│ Battery/PSU                  │
│ ├─→ 5V ────→ 5V rail         │
│ │  ├─→ Line Sensor VCC       │
│ │  ├─→ VL53L0X VCC           │
│ │  └─→ Servo Left + Right    │
│ │
│ ├─→ 12V ───→ Motor Driver    │
│ │  └─→ Stepper Motors        │
│ │
│ ├─→ 3.3V ──→ ESP32 (Zener)   │
│ │  └─→ MPU6050 VCC           │
│ │
│ └─→ GND ───→ GND rail (COMMON)│
│    ├─→ ESP32 GND            │
│    ├─→ Driver GND           │
│    ├─→ Sensors GND          │
│    └─→ Servo GND            │
└──────────────────────────────┘

CRITICAL:
- All GNDs must be tied together!
- Servos need a separate power source (a beefy 5V!)
- Protect servo signals from motor noise
```

---

## WIRING BOARD (breadboard layout)

```
I2C (GPIO 21, 22):
┌─ 3.3V ─ Pull-up 4.7K ─ GPIO 22 (SCL) ─ MPU6050 SCL
├─ 3.3V ─ Pull-up 4.7K ─ GPIO 21 (SDA) ─ MPU6050 SDA
│                                      └─ VL53L0X SDA/SCL
└─ GND ─────────────────────────────── Both sensors GND

ADC (Line Sensors):
GPIO 34 ← Sensor1 (ADC1_CH6)
GPIO 35 ← Sensor2 (ADC1_CH7)
GPIO 36 ← Sensor3 (ADC1_CH0)
GPIO 39 ← Sensor4 (ADC1_CH3)
GPIO 32 ← Sensor5 (ADC1_CH4)

Motor Control:
GPIO 27 → Driver DIR1 → Motor1 DIR
GPIO 14 → Driver STEP1 → Motor1 STEP
GPIO 25 → Driver DIR2 → Motor2 DIR
GPIO 26 → Driver STEP2 → Motor2 STEP
GPIO 12 → Driver ENABLE

Audio/Visual:
GPIO 2 → WiFi LED (WiFi status indicator)
```

---

## SCHEMATIC

```
                     ┌──────────────────┐
                     │       ESP32      │
                     └──────────────────┘
                              │
         ┌────────────────────┼─────────────────────┐
         │                    │                     │
    ┌────────┐         ┌───────────┐        ┌──────────┐
    │ Motors │         │ Sensors   │        │ WiFi/Web │
    │ ────── │         │ ──────── │        │ ──────── │
    │ M1/M2  │         │ MPU6050   │        │ Interface│
    │ Driver │         │ VL53L0X   │        │ Browser  │
    │ 12V    │         │ LineSensor│        │ Joystick │
    └────────┘         └───────────┘        └──────────┘
         │                    │                     │
         └────────────────────┼─────────────────────┘
                              │
                         [Battery]
```

---

## WIRING CHECK

```
On the bench:
1. GPIO 21/22 (I2C) → sensors
   - SDA → GPIO 21 (MPU6050, VL53L0X)
   - SCL → GPIO 22 (MPU6050, VL53L0X)
   - 3.3V, GND

2. GPIO 34/35/36/39/32 (ADC) → line sensor
   - S1-S5 → GPIO 34/35/36/39/32
   - 5V, GND

3. GPIO 27/14/25/26/12 (Motor) → driver
   - DIR/STEP → GPIO 27/14/25/26
   - Enable → GPIO 12
   - GND, 12V

4. GPIO 13/33 (PWM) → gripper servos
   - Servo Left  → GPIO 13
   - Servo Right → GPIO 33
   - GND, 5V (separate source!)

5. Verify:
   [ ] All 3.3V tied together
   [ ] All GND tied together (COMMON ground!)
   [ ] MPU6050 + VL53L0X on I2C (GPIO 21/22)
   [ ] I2C pull-up resistors (4.7K)
   [ ] Line sensor on GPIO 34/35/36/39/32
   [ ] Motors on GPIO 27/14/25/26/12
   [ ] Servos on GPIO 13 and GPIO 33
   [ ] Servo power separate and beefy!
   [ ] Motor power 12V separate
```

---

## I2C PULL-UP RESISTORS (IMPORTANT!)

```
If the sensors don't respond, add:

GPIO 22 (SCL):
3.3V ──[4.7K]──┬── GPIO 22
               │
            MPU6050 SCL
            VL53L0X SCL

GPIO 21 (SDA):
3.3V ──[4.7K]──┬── GPIO 21
               │
            MPU6050 SDA
            VL53L0X SDA
```

---

**Schematic version:** 1.0
**Last updated:** 2026-06-07
