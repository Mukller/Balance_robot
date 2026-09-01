# Changelog

All notable changes to this project will be documented in this file.

## [3.4.0] - 2026-08-23

### Fixed
- Slope detection: Z-axis baseline captured at startup before motor test (was always "on slope") (#1)
- PID state corruption from duplicate diagnostic controller calls every 10 loops (`prev_error` reset, double integral accumulation) (#2)
- VL53L0X moved to continuous mode with register-level data-ready polling (0x13) — control loop no longer blocked ~33 ms per cycle (#3)
- Speed feedback now uses true step-based odometry instead of commanded-speed proxy (which fought balance controller) (#4)
- `AUTO_START_ON_BOOT` flag to disable 5s auto-start for competition safety (#5)
- Setup-time motor test gated behind `SKIP_SETUP_MOTOR_TEST=1` (was rolling the robot off the table)
- Slope damping no longer halves the balance PD output (only throttle/steering)
- MPU6050 I2C read returns its error code; controlLoop emergency-stops on persistent failures + `Wire.setTimeout(50)` for fail-fast on hung bus
- `dt` clamped to a safe range on first cycle (no more huge integrator spike on boot)
- Line-following mode gated behind `LINE_MODE_ENABLED` flag

### Added
- WiFi STA + AsyncWebServer with AP-mode fallback (creds via `secrets.h` or runtime `/api/save-wifi` → NVS)
- `STATE_MANUAL` for joystick override (set via `/api/joystick`)
- `STATE_STOPPED` auto-resume after obstacle clears for `STOPPED_RESUME_HOLD_MS` (1 s)
- `STATE_MANUAL` exposed via `/api/command?cmd=manual`
- Servo/gripper API: `gripper_open` / `gripper_close` / `gripper_reset`
- WiFi LED status (solid = connected, fast blink = connecting, slow blink = AP, off = offline)
- `secrets.h.example` template (gitignored `secrets.h` is the live one)
- `LINE_FOLLOWING_GUIDE.md` / `LINE_FOLLOWER_README.md` cross-link to `test_line_calibrate.ino`
- `test_emergency_stop.ino` procedural test for obstacle-stop behavior
- Compile check CI (GitHub Actions, arduino-cli + esp32 3.1.1)

### Changed
- `webUI` HTML moved to PROGMEM (frees ~5 KB of RAM)
- I2C bus clocked at 400 kHz (was 100 kHz)
- `SWAP` macro uses function-local storage instead of file-scope `swap_var`
- Servo test constant corrected: 3.277 → 3.2768
- README: removed references to non-existent files (`esp32_robot_main.ino`, `web_interface.h`, `*.h/.cpp` modules)
- README: GPIO33 marked as SERVO RIGHT (was inconsistently labelled BUZZER in the pinout diagram)
- Removed dead code: `counter1/2`, `position_error_sum_*`, `positionPDControl`, `Kp_position/Kd_position`, `target_steps*`, `motor1_control/2_control`, `positionControlMode`, `MAX_*_PRO`, `SERVO_AUX_*` / `SERVO*_NEUTRO` / `SERVO*_PULSEWIDTH`, `KP_RAISEUP` etc., `LINE_THRESHOLD`, `GRAD2RAD`
- `web_interface.html` (standalone, drifted from in-tree copy) removed
- `TAGS.md`, `RELEASE_INFO.md` removed (stale/duplicative)
- `CODE_REVIEW_REPORT.md` rewritten as a real review
- `SECURITY.md` contradiction fixed (private disclosure path)

## [3.3.1] - 2026-07-10

### Changed
- Refactor: monolithic single-file `esp32_robot_monolith.ino`
- Author/source header added to all source files
- README bilingual switcher collapsed into separate README.md / README_EN.md

## [3.0] - 2026-06-07