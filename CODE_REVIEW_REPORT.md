# Code Review

Reviewer: Anton (Mukller), August 2026
Scope: full source tree at the time of the v3.4.0 release.

## Scope Reviewed

- `main/esp32_robot_monolith/esp32_robot_monolith.ino` (main sketch, 1280 lines)
- `main/test_*/*.ino` (10 diagnostic sketches)
- `main/LINE_FOLLOWING_GUIDE.md`, `main/LINE_FOLLOWER_README.md`
- `README.md`, `README_EN.md`, `CHANGELOG.md`, `SECURITY.md`, `CONTRIBUTING.md`

Tooling: `arduino-cli` + ESP32 Arduino Core 3.1.1, GitHub Actions CI (`.github/workflows/ci.yml`).

## Verdict

| Area                   | Result                |
|------------------------|-----------------------|
| Critical bugs          | 0 open (8 fixed in 3.4.0) |
| High-priority issues   | 0 open (3 fixed)      |
| Medium issues          | 0 open (10 fixed/cleaned) |
| Build / CI             | Passing               |
| Secrets hygiene        | Good (`secrets.h` gitignored) |
| Memory budget          | OK (80% flash, 14% RAM with AsyncWebServer) |

## Fixed in v3.4.0

Critical:

- Slope detection always-on (baseline captured from `z_accel_baseline=0`) — robot permanently damped
- PID state corruption from duplicate `stabilityPDControl`/`speedPIControl` calls in the DBG branch
- VL53L0X blocking read inside a 100 Hz control loop
- Speed feedback using commanded-speed proxy (opposed balance controller) — replaced with step-based odometry
- `STATE_STOPPED` was a dead-end (no auto-resume)
- `mpu_last_error` discarded; robot kept controlling with stale MPU data on disconnect
- `Wire.setTimeout` absent; hung I²C bus locked the controller
- `dt` first-cycle spike (huge integrator wind-up on boot)

High:

- `applySlopeDamping(control_output)` halved the balance PD output on slopes (the moment full correction is needed)
- 1 s motor spin in `setup()` could roll the robot off the table
- `line_mode` hard-coded ON with no off-switch

Cleanup:

- Dead code removed: `counter1/2`, `position_error_sum_*`, `positionPDControl`, `Kp_position/Kd_position`, `target_steps*`, `motor1_control/2_control`, `positionControlMode`, `MAX_*_PRO`, `SERVO_AUX_*`, `KP_RAISEUP` etc., `LINE_THRESHOLD`, `GRAD2RAD`
- `webUI` HTML moved to PROGMEM
- `SWAP` macro uses function-local storage
- Servo test constant: `3.277` → `3.2768`
- `web_interface.html` (standalone, drifted) removed
- `TAGS.md`, `RELEASE_INFO.md` removed (stale)

Docs:

- README no longer references the long-deleted module files
- GPIO33 pinout ambiguity (BUZZER vs SERVO) fixed
- `SECURITY.md` contradiction fixed (private disclosure path)
- `CODE_REVIEW_REPORT.md` rewritten as a real review (this file)

## Known Limitations (non-blocking)

- The `STATE_MANUAL` joystick does not read the obstacle sensor (manual override bypasses safety stop)
- No battery monitor (recommend external ADC + divider for brown-out detection)
- No OTA update; reflash over USB
- I2C error recovery is "stop on persistent failure" — not a full re-init of MPU6050
- `actual_robot_speed` uses a coarse quantization (1 unit per 8 step deltas) — fine for PID but not for odometry-driven navigation

## Recommendations for the Next Sprint

1. Battery monitoring (ADC + brown-out safety)
2. Re-init MPU6050 on `mpu_last_error` (3 retries before emergency-stop)
3. MPU interrupt-driven loop instead of polled 100 Hz
4. Trajectory recording for post-run analysis
5. OTA update via WiFi

---
Last updated: 2026-08-23
