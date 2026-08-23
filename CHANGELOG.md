# Changelog

All notable changes to this project will be documented in this file.

## [3.4.0] - 2026-08-23

### Fixed
- Slope detection: Z-axis baseline captured at startup (before: `on_slope` was always true, throttle permanently halved) (#1)
- PID state corruption from duplicate diagnostic controller calls every 10 loops (`prev_error` reset, double integral accumulation) (#2)
- VL53L0X moved to continuous mode with register-level data-ready polling (0x13) — control loop no longer blocked ~33 ms per cycle (#3)
- Speed feedback restored: `actual_robot_speed` estimated from wheel commands with low-pass filter (was stuck at 0 → open-loop speed PI) (#4)

### Added
- `AUTO_START_ON_BOOT` config flag: set 0 to keep the robot balancing in IDLE instead of auto-starting after 5 s (competition safety) (#5)

## [3.0] - 2026-06-07

### Added
- Web interface with joystick control (HTML/CSS/JS)
- Real-time sensor data display (line sensors, distance)
- Three test sketches for debugging:
  - test_line_sensor.ino - Line sensor array testing
  - test_distance_sensor.ino - VL53L0X distance sensor
  - test_motors.ino - Stepper motor control testing
- Detailed wiring diagram with pinout
- Complete GPIO documentation
- I2C pull-up resistor instructions
- Breadboard layout guide
- API endpoints for robot control
  - /api/joystick - Joystick control
  - /api/command - State commands
  - /api/status - Robot status

### Features
- 4 robot states: IDLE, STRAIGHT, LINE_FOLLOW, STOPPED
- Automatic 2-second straight, then line following
- Slope detection using Z-axis accelerometer
- Speed reduction (2x) when climbing hills
- Emergency stop on obstacle detection (< 300mm)
- WiFi web control panel
- Real-time debugging output to Serial Monitor

### Fixed
- String concatenation optimization (snprintf instead of String +)
- Type safety for abs() function (int32_t)
- control_output constraints
- millis() overflow safe comparison
- Non-blocking delays (vTaskDelay instead of delay)
- Error handling in sensor initialization

## [2.0] - 2026-06-07

### Added
- Line following integration with 5-sensor array (Ldabrye)
- Slope detection by vertical acceleration (Z-axis)
- Distance sensor support (VL53L0X)
- WiFi API for robot control
- Comprehensive code review document

### Features
- Complementary filter for angle calculation
- Weighted average position detection for line
- Automatic obstacle avoidance
- State machine for robot control

### Fixed
- Race conditions on volatile variables
- Memory optimization for JSON responses
- Proper interrupt handler synchronization

## [1.0] - 2026-06-01

### Initial Release
- Self-balancing robot core functionality
- PD controller for balance (MPU6050)
- Stepper motor control with acceleration limiting
- Basic WiFi connectivity
- GPIO pin configuration
- Interrupt-driven motor control at 100Hz

### Components
- ESP32 microcontroller
- MPU6050 gyroscope + accelerometer
- 2x NEMA17 stepper motors
- Stepper motor drivers

---

## Version Compatibility

| Version | Date       | Status      | Notes                              |
|---------|------------|-------------|-----------------------------------|
| 3.0     | 2026-06-07 | Stable      | Web interface, tests, full docs   |
| 2.0     | 2026-06-07 | Archived    | Line following basics             |
| 1.0     | 2026-06-01 | Archived    | Initial release                   |

---

## Roadmap

### Planned for v4.0
- [ ] Machine learning obstacle avoidance
- [ ] SLAM navigation
- [ ] Multi-robot coordination
- [ ] Advanced PID tuning interface
- [ ] Telemetry recording
- [ ] Mobile app control

### Under Consideration
- [ ] Camera integration
- [ ] Voice control
- [ ] Auto-tuning algorithm
- [ ] Battery monitoring
- [ ] Over-the-air updates

---

## Migration Guide

### From v2.0 to v3.0
1. Update web_interface.h (new file)
2. Replace esp32_robot_main.ino with new version
3. Add test sketches to verify hardware
4. Update WiFi credentials
5. Test sensors individually before running main sketch

### From v1.0 to v2.0
1. Add LineFollower library files
2. Update GPIO configuration in defines.h
3. Add new sensor initialization code
4. Update main control loop for slope detection

---

**Current Version:** 3.0  
**Last Updated:** 2026-06-07  
**Maintainer:** Anton (github.com/Mukller)
