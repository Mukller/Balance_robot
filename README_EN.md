# 🤖 ESP32 Self-Balancing Robot

A self-balancing robot on two wheels with line tracking, obstacle avoidance, and web-based joystick control.

## 📁 Project Files

### Main Sketches:
- **esp32_robot_main.ino** - Main robot program
- **web_interface.h** - Web interface with joystick

### Test Sketches:
- **test_line_sensor.ino** - Line sensor testing (5 sensors)
- **test_distance_sensor.ino** - Distance sensor testing (HY-SRF05 ultrasonic)
- **test_motors.ino** - Motor speed and synchronization testing

### Libraries:
- LineFollower.h/.cpp
- Control.h/.cpp
- MPU6050.h/.cpp
- Motors.h/.cpp
- defines.h, globals.h/.cpp

---

## ⚡ Quick Start

### 1. Edit WiFi Settings

In **esp32_robot_main.ino**:
```cpp
String sta_ssid = "YourWiFiSSID";
String sta_password = "YourWiFiPassword";
```

### 2. Install Libraries

Required Arduino IDE libraries:
- AsyncTCP
- ESPAsyncWebServer
(HY-SRF05 doesn't require additional libraries - uses built-in pulseIn())

### 3. Open Web Interface

```
http://192.168.X.X
```

---

## 🎮 Control (Web Interface)

### Joystick Control
- **Up/Down** = Throttle (speed)
- **Left/Right** = Steering (turn)

### Buttons
- **▶ Straight** - Drive straight for 2 sec, then line follow
- **〰 Line** - Start line following mode
- **⏹ Stop** - Emergency stop

### Real-Time Sensor Display
- Distance to obstacle (mm)
- Line sensor data (5 sensors with graph)
- Tilt angle
- Hill/slope status

---

## 🤖 How It Works

```
1. IDLE (waiting for command)
   └─ Press "Straight"
   
2. STRAIGHT (drives 2 seconds)
   └─ Auto transition
   
3. LINE_FOLLOW (tracks black line)
   - Detects obstacles → STOP
   - Detects slope → Speed reduction (2x)
   
4. STOPPED (obstacle detected)
   └─ Press "Stop" button
```

---

## 🧪 Testing

### 1. Line Sensor Test

```cpp
// Upload: test_line_sensor.ino
// Serial Monitor: 115200 baud
// Output: Raw values, calibrated values, ASCII graph
```

### 2. Distance Sensor Test (HY-SRF05)

```cpp
// Upload: test_distance_sensor.ino
// Serial Monitor: 115200 baud
// Output: Distance in mm, status, ASCII graph
// Shows: 0-200mm (very close) to >4000mm (far)
// Pins: TRIG=GPIO4, ECHO=GPIO5
```

### 3. Motor Test

```cpp
// Upload: test_motors.ino
// Serial Monitor: 115200 baud
// Tests:
//   1. Both motors forward slow
//   2. Both motors forward fast
//   3. Both motors backward
//   4. Different speeds (synchronization check)
// Output: Step counts, speed ratio
```

---

## 🔧 Configuration Parameters

```cpp
#define DISTANCE_THRESHOLD 300      // Stop if < 300mm
#define STRAIGHT_TIME 2000          // 2 seconds straight
#define SLOPE_ACCEL_THRESHOLD 2000  // Hill/slope detection
#define MAX_THROTTLE 550            // Maximum speed
#define MAX_STEERING 140            // Maximum turn
```

---

## 🔌 GPIO Pinout (ESP32)

```
I2C (for MPU6050):
  GPIO 21 (SDA)
  GPIO 22 (SCL)

Ultrasonic Distance Sensor (HY-SRF05):
  GPIO 4  (TRIG) - trigger pulse
  GPIO 5  (ECHO) - echo signal reception

Motors (Stepper NEMA17):
  GPIO 27 ← Motor1 DIR (direction)
  GPIO 14 ← Motor1 STEP (steps)
  GPIO 25 ← Motor2 DIR (direction)
  GPIO 26 ← Motor2 STEP (steps)
  GPIO 12 ← Enable (both motors)

Line Sensor (5 sensors):
  GPIO 34 ← S1 (left edge)
  GPIO 35 ← S2 (left quarter)
  GPIO 36 ← S3 (center)
  GPIO 39 ← S4 (right quarter)
  GPIO 32 ← S5 (right edge)
```

### HY-SRF05 Connection Diagram:
```
HY-SRF05 ─────────────── ESP32
   TRIG ─────────────── GPIO 4
   ECHO ─────────────── GPIO 5
   GND  ─────────────── GND
   VCC  ─────────────── 5V (via stabilizer!)
```

⚠️ **Important:** HY-SRF05 operates at 5V! Use voltage divider for ECHO (GPIO5 max 3.3V)

### Detailed Wiring Diagram

See full README.md for complete schematic including:
- ESP32 pinout diagram
- Component connections
- Power distribution
- Voltage divider for ECHO signal
- Breadboard layout

---

## 🐛 Troubleshooting

**Robot won't connect to WiFi:**
- Check SSID/password in code
- Check Serial Monitor (115200 baud)
- Verify ESP32 is powered on

**Robot won't balance:**
- Check I2C connection to MPU6050
- Run test_motors.ino to verify motors work
- Calibrate sensors properly

**Line following not working:**
- Calibrate line sensor (test_line_sensor.ino)
- Check GPIO 34/35/36/39/32 connections
- Verify line has high contrast
- Check sensor is 0.5-1cm above line

**Obstacle detection failing:**
- Run test_distance_sensor.ino
- Check VL53L0X I2C connection
- Adjust DISTANCE_THRESHOLD if needed

---

## ✅ Pre-Flight Checklist

- [ ] ESP32 connected to computer
- [ ] All sensors connected
- [ ] WiFi SSID/password entered
- [ ] test_motors.ino passes
- [ ] test_line_sensor.ino works
- [ ] test_distance_sensor.ino works
- [ ] Web interface loads

---

## 📊 Performance

| Metric | Value |
|--------|-------|
| Control Loop | 100Hz (10ms) |
| Web API Response | <50ms |
| Line Sensor Rate | 100Hz |
| Distance Sensor | 200Hz |
| Motor Max Speed | ~500 RPM |
| Balance Response | <50ms |

---

## 📚 Documentation

- **README.md** - Main documentation (Russian)
- **README_EN.md** - English documentation (this file)
- **CHANGELOG.md** - Version history
- **CONTRIBUTING.md** - How to contribute
- **CODE_OF_CONDUCT.md** - Community guidelines
- **LICENSE.md** - MIT License
- **RELEASE_INFO.md** - Release information

---

## 🚀 Future Features

- Machine learning obstacle avoidance
- SLAM navigation
- Multi-robot coordination
- Mobile app control
- Camera integration
- Voice control
- Advanced PID tuning interface
- Telemetry recording

---

## 📝 License

MIT License - Free for personal, educational, and commercial use.  
See LICENSE.md for details.

---

## 🤝 Contributing

Contributions welcome! See CONTRIBUTING.md for guidelines.

---

**Version:** 3.0  
**Status:** ✅ Production Ready  
**Repository:** https://github.com/Mukller/Balance_robot  
**Maintainer:** Anton (@Mukller)

---

**Last Updated:** 2026-06-07
