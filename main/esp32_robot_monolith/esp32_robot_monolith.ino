// ============================================================================
// ESP32 SELF-BALANCING ROBOT - MONOLITHIC VERSION v3.4.0
// Балансирующий робот на ESP32 с веб-интерфейсом
// All code combined into single .ino file for ease of deployment
// ============================================================================

// Build-time feature flags MUST be defined before the includes section
// 1 = auto-start STRAIGHT 5s after boot (legacy behavior)
// 0 = robot balances in IDLE forever, waits for a command (safer at competitions)
#define AUTO_START_ON_BOOT 1
// 1 = run a 1s spin test in setup() (diagnostic, robot will roll)
// 0 = skip the test, safer if robot is already on its wheels
#define SKIP_SETUP_MOTOR_TEST 1
// 1 = enable line-following mode at boot
// 0 = robot only balances, no line following (line sensor still readable via /api/status)
#define LINE_MODE_ENABLED 1
// 1 = start WiFi + AsyncWebServer (requires AsyncTCP + ESPAsyncWebServer libs in libraries/)
#define ENABLE_WIFI 1
// AP mode fallback when no WiFi creds are stored / sketch can not connect
#define WIFI_AP_FALLBACK 1
#define WIFI_AP_SSID "BalanceRobot-Setup"
#define WIFI_AP_PASS "balance123"      // at least 8 chars for WPA2
// How long to wait for STA connection before falling back to AP (ms)
#define WIFI_CONNECT_TIMEOUT_MS 10000

#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#if ENABLE_WIFI
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncWebSocket.h>
#include <Preferences.h>
#if __has_include("secrets.h")
  #include "secrets.h"
#endif
#endif

// ============================================================================
// DEFINES & CONFIGURATION
// ============================================================================

// GPIO Pinout (соответствует README)
#define PIN_ENABLE_MOTORS 12   // ENABLE (общий на оба мотора)
#define PIN_MOTOR1_DIR 27      // Motor1 DIR
#define PIN_MOTOR1_STEP 14     // Motor1 STEP
#define PIN_MOTOR2_DIR 25      // Motor2 DIR
#define PIN_MOTOR2_STEP 26     // Motor2 STEP
#define PIN_SERVO_LEFT 13      // Servo Left
#define PIN_SERVO_RIGHT 33     // Servo Right
#define PIN_WIFI_LED 2         // WiFi LED

// Line Follower Sensors
#define LINE_SENSOR_1 34   // GPIO 34 (left)
#define LINE_SENSOR_2 35   // GPIO 35 (left-center)
#define LINE_SENSOR_3 36   // GPIO 36 (center)
#define LINE_SENSOR_4 39   // GPIO 39 (right-center)
#define LINE_SENSOR_5 32   // GPIO 32 (right)

#define LINE_SENSOR_SAMPLES 5
#define LINE_MAX_ERROR 200
#define LINE_MODE_OFF 0
#define LINE_MODE_ON 1
#define LINE_MODE_AUTO 2

// Control Parameters
#define MAX_THROTTLE 550
#define MAX_STEERING 140
#define MAX_TARGET_ANGLE 14

#define KP 0.32
#define KD 0.050
#define KP_THROTTLE 0.080
#define KI_THROTTLE 0.1

// PID Runtime Tuning (NVS-backed, can be changed via WebSocket/UI)
#define KP_DEFAULT 0.32f
#define KD_DEFAULT 0.050f
#define KP_THROTTLE_DEFAULT 0.080f
#define KI_THROTTLE_DEFAULT 0.1f

#define MAX_CONTROL_OUTPUT 500
#define ITERM_MAX_ERROR 30
#define ITERM_MAX 10000
#define ANGLE_OFFSET 0.0

// WebSocket Configuration
#define WS_STATUS_INTERVAL_MS 20     // 50Hz status push
#define WS_MAX_FRAME_SIZE 1024

#define ZERO_SPEED 0xffffff
#define MAX_ACCEL 14
#define MICROSTEPPING 8

#define RAD2GRAD 57.2957795

// Robot States
#define STATE_IDLE 0
#define STATE_STRAIGHT 1
#define STATE_LINE_FOLLOW 2
#define STATE_STOPPED 3
#define STATE_MANUAL 4

// MPU6050 Register Definitions
#define MPU6050_I2C_ADDRESS 0x68
#define MPU6050_SMPLRT_DIV 0x19
#define MPU6050_CONFIG 0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_INT_ENABLE 0x38
#define MPU6050_INT_STATUS 0x3A
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_WHO_AM_I 0x75

#define MPU6050_DEVICE_RESET 7
#define MPU6050_FS_SEL_500 bit(4)
#define MPU6050_AFS_SEL_2G 0
#define MPU6050_DLPF_10HZ 4
#define MPU6050_DATA_RDY_EN bit(0)

#define SLOPE_ACCEL_THRESHOLD 2000
#define DISTANCE_THRESHOLD 300
#define STRAIGHT_TIME 2000
#define STOPPED_RESUME_DISTANCE 350   // resume when obstacle clears this far (mm)
#define STOPPED_RESUME_HOLD_MS 1000   // hold-clear time before auto-resume

// (feature flags AUTO_START_ON_BOOT / SKIP_SETUP_MOTOR_TEST / LINE_MODE_ENABLED /
//  ENABLE_WIFI / WIFI_AP_FALLBACK / WIFI_AP_SSID / WIFI_AP_PASS / WIFI_CONNECT_TIMEOUT_MS
//  live at the very top of the file, before the includes section)
// Pin/timeout config
#define WIRE_TIMEOUT_MS 50             // I2C bus timeout (ms) — fail-fast on hung Wire
#define I2C_CLOCK_HZ 400000
// MPU sanity — if angle beyond this, emergency-stop
#define EMERGENCY_TILT_DEG 70.0f
// True step-based odometry speed PI scaler: per-10ms step delta * SCALE ~= speed_M1 units
// Derived from setMotorSpeed: step rate = speed_M1 * 25 / 2 Hz -> d per 10ms = speed_M1 * 0.125
// SCALE = 1 / 0.125 = 8
#define SPEED_ODOM_SCALE 8
#define SPEED_ODOM_FILTER 0.30f        // low-pass on odometry feedback

// ============================================================================
// STRUCTURES
// ============================================================================

typedef union accel_t_gyro_union {
  struct {
    uint8_t x_accel_h, x_accel_l;
    uint8_t y_accel_h, y_accel_l;
    uint8_t z_accel_h, z_accel_l;
    uint8_t t_h, t_l;
    uint8_t x_gyro_h, x_gyro_l;
    uint8_t y_gyro_h, y_gyro_l;
    uint8_t z_gyro_h, z_gyro_l;
  } reg;
  struct {
    int16_t x_accel, y_accel, z_accel, temperature;
    int16_t x_gyro, y_gyro, z_gyro;
  } value;
};

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// System
hw_timer_t * timer1 = NULL;
hw_timer_t * timer2 = NULL;
uint8_t cascade_control_loop_counter = 0, loop_counter = 0;

long timer_old = 0, timer_value = 0;
float dt = 0;

// Angle & Control
float angle_adjusted = 0;
float Kp = KP_DEFAULT, Kd = KD_DEFAULT, Kp_thr = KP_THROTTLE_DEFAULT, Ki_thr = KI_THROTTLE_DEFAULT;
float PID_errorSum = 0;
float target_angle = 0, steering = 0;
int16_t throttle = 0;
float max_throttle = MAX_THROTTLE, max_steering = MAX_STEERING;
float control_output = 0, angle_offset = ANGLE_OFFSET;

// Motors
volatile int32_t steps1 = 0, steps2 = 0;
int16_t speed_M1 = 0, speed_M2 = 0;
int8_t dir_M1 = 0, dir_M2 = 0;
int16_t actual_robot_speed = 0, actual_robot_speed_Old = 0;
float estimated_speed_filtered = 0;

// True step-based odometry (per-10ms control cycle, scaled)
int32_t last_steps1 = 0, last_steps2 = 0;

// MPU6050
accel_t_gyro_union accel_t_gyro;
float x_gyro_value = 0, x_gyro_offset = 0, accel_angle = 0, angle = 0;
volatile int8_t mpu_last_error = 0;

// SWAP macro kept for MPU byte-swap; uses function-local storage
#define SWAP(x,y) do { uint8_t _s = (x); (x) = (y); (y) = _s; } while (0)

// Line Follower
int16_t line_sensor_raw[5] = {0}, line_sensor_calibrated[5] = {0};
int16_t line_position_error = 0, line_steering_output = 0;
uint8_t line_mode = LINE_MODE_OFF;
uint16_t line_min_values[5] = {500, 500, 500, 500, 500};
uint16_t line_max_values[5] = {3500, 3500, 3500, 3500, 3500};
static const uint8_t sensor_pins[5] = {LINE_SENSOR_1, LINE_SENSOR_2, LINE_SENSOR_3, LINE_SENSOR_4, LINE_SENSOR_5};
static const int8_t sensor_weights[5] = {-2, -1, 0, 1, 2};

// Robot State Machine
volatile uint8_t robot_state = STATE_IDLE;
unsigned long state_start_time = 0;
volatile bool on_slope = false;
int16_t z_accel_baseline = 0;
unsigned long obstacle_clear_since_ms = 0;  // for STATE_STOPPED auto-resume

// Distance Sensor
VL53L0X distanceSensor;
bool vl53_ready = false;
uint16_t distance_mm = 0;

// Manual control values (used when ENABLE_WIFI is 0: defined unconditionally so
// the STATE_MANUAL branch in controlLoop compiles even without WiFi).
int16_t api_manual_throttle_val = 0;
int16_t api_manual_steering_val  = 0;

#if ENABLE_WIFI
// WiFi state
enum WiFiMode { WIFI_OFFLINE, WIFI_CONNECTING, WIFI_AP_MODE, WIFI_STA_OK };
WiFiMode wifi_mode = WIFI_OFFLINE;
AsyncWebServer webServer(80);
AsyncWebSocket ws("/ws");
bool api_manual_throttle = false;  // joystick override flag
unsigned long wifi_led_toggle_ms = 0;
bool wifi_led_state = false;

// WebSocket state
unsigned long ws_last_status_ms = 0;
bool ws_connected = false;
uint32_t ws_client_id = 0;
#endif

// ============================================================================
// WEB INTERFACE HTML (EMBEDDED, stored in flash via PROGMEM)
// ============================================================================

const char webUI[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="ru">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ESP32 Balance Robot</title>
<style>
* { box-sizing: border-box; margin: 0; padding: 0; }
body {
  font: 14px/1.5 system-ui, -apple-system, sans-serif;
  background: #0d1117;
  color: #c9d1d9;
  min-height: 100vh;
}
header {
  padding: 14px 24px;
  border-bottom: 1px solid #21262d;
  display: flex;
  justify-content: space-between;
  align-items: center;
  background: #0d1117;
  position: sticky;
  top: 0;
  z-index: 10;
  flex-wrap: wrap;
  gap: 8px;
}
header h1 { font-size: 16px; font-weight: 600; color: #e6edf3; }
header .badges { display: flex; gap: 6px; align-items: center; }
main { padding: 24px; max-width: 1100px; margin: 0 auto; }
h2 { font-size: 11px; color: #8b949e; text-transform: uppercase; letter-spacing: 1px; font-weight: 600; margin: 0 0 10px; }
.grid { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 16px; }
@media (max-width: 960px) { .grid { grid-template-columns: 1fr 1fr; } }
@media (max-width: 640px) { .grid { grid-template-columns: 1fr; } }
.card { background: #161b22; border: 1px solid #21262d; border-radius: 8px; padding: 18px; }
.tag { display: inline-block; padding: 2px 8px; border-radius: 10px; font-size: 11px; background: #21262d; color: #8b949e; }
.tag.green  { background: rgba(63,185,80,.15);  color: #3fb950; }
.tag.red    { background: rgba(248,81,73,.15);  color: #f85149; }
.tag.blue   { background: rgba(88,166,255,.15); color: #58a6ff; }
.tag.yellow { background: rgba(210,153,34,.15); color: #d29922; }
.tag.purple { background: rgba(168,85,247,.15); color: #a371f7; }
.joystick-wrap { display: flex; justify-content: center; margin-bottom: 14px; }
.joystick-container { width: 180px; height: 180px; background: #0d1117; border: 1px solid #21262d; border-radius: 50%; position: relative; touch-action: none; }
.joystick-ring { position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%); width: 40px; height: 40px; border: 1px solid #21262d; border-radius: 50%; pointer-events: none; }
.joystick-dot { width: 44px; height: 44px; background: #58a6ff; border-radius: 50%; position: absolute; top: 68px; left: 68px; cursor: grab; box-shadow: 0 0 12px rgba(88,166,255,.4); transition: box-shadow .1s; }
.joystick-dot:active { cursor: grabbing; box-shadow: 0 0 20px rgba(88,166,255,.6); }
.btn-row { display: grid; grid-template-columns: 1fr 1fr; gap: 6px; margin-bottom: 10px; }
.btn-row.three { grid-template-columns: 1fr 1fr 1fr; }
button { background: #238636; color: #fff; border: none; padding: 7px 10px; border-radius: 6px; cursor: pointer; font-size: 12px; font-family: inherit; font-weight: 500; transition: background .15s; }
button:hover { background: #2ea043; }
button:disabled { background: #21262d; color: #6e7681; cursor: not-allowed; }
button.secondary { background: #21262d; color: #c9d1d9; }
button.secondary:hover:not(:disabled) { background: #30363d; }
button.danger { background: #da3633; }
button.danger:hover { background: #f85149; }
button.gripper { background: #6e40c9; }
button.gripper:hover { background: #8957e5; }
.status-row { display: flex; align-items: center; justify-content: space-between; padding: 8px 0; border-top: 1px solid #21262d; margin-top: 4px; }
.status-label { font-size: 12px; color: #8b949e; }
.kv { display: flex; justify-content: space-between; padding: 5px 0; border-bottom: 1px solid #21262d; font-size: 13px; gap: 8px; }
.kv:last-child { border: none; }
.kv-k { color: #8b949e; }
.kv-v { font-family: monospace; color: #c9d1d9; text-align: right; }
.bar { height: 4px; background: #21262d; border-radius: 2px; overflow: hidden; margin: 4px 0 10px; }
.bar-fill { height: 100%; background: #58a6ff; transition: width .2s; border-radius: 2px; }
.line-bars { display: flex; gap: 5px; justify-content: center; margin: 8px 0 10px; }
.s-bar { width: 26px; height: 72px; background: #0d1117; border: 1px solid #21262d; border-radius: 3px; position: relative; overflow: hidden; }
.s-fill { width: 100%; position: absolute; bottom: 0; background: #58a6ff; transition: height .15s, background .2s; border-radius: 2px; }
.s-bar.calibrating .s-fill { background: #d29922; }
.section-gap { margin-top: 16px; }
input[type="number"], input[type="text"], input[type="password"] {
  background: #0d1117; color: #e6edf3; border: 1px solid #21262d; border-radius: 4px;
  padding: 4px 6px; font-size: 12px; font-family: monospace; width: 100%;
}
input[type="number"] { width: 70px; }
label { font-size: 11px; color: #8b949e; display: block; margin-bottom: 2px; }
.calib-grid { display: grid; grid-template-columns: repeat(5, 1fr); gap: 6px; margin-bottom: 10px; }
.calib-cell { display: flex; flex-direction: column; align-items: center; }
.calib-cell .v { color: #8b949e; font-size: 10px; margin-bottom: 2px; }
.toast {
  position: fixed; bottom: 20px; left: 50%; transform: translateX(-50%);
  background: #161b22; border: 1px solid #30363d; padding: 10px 16px; border-radius: 6px;
  font-size: 13px; box-shadow: 0 4px 12px rgba(0,0,0,.4); display: none; z-index: 100;
  max-width: 80%;
}
.toast.show { display: block; animation: fade 3s forwards; }
.toast.error { border-color: #f85149; color: #ffa198; }
.toast.ok { border-color: #3fb950; color: #56d364; }
@keyframes fade { 0% { opacity: 0; } 10% { opacity: 1; } 90% { opacity: 1; } 100% { opacity: 0; } }
</style>
</head>
<body>
<header>
  <h1>ESP32 Balance Robot</h1>
  <div class="badges">
    <span class="tag blue" id="wifiTag">WiFi: ...</span>
    <span class="tag" id="statusTag">IDLE</span>
  </div>
</header>
<main>
<div class="grid">
  <!-- Card 1: Control -->
  <div class="card">
    <h2>Управление</h2>
    <div class="joystick-wrap">
      <div class="joystick-container" id="joystickBox">
        <div class="joystick-ring"></div>
        <div class="joystick-dot" id="joystick"></div>
      </div>
    </div>
    <div class="btn-row three">
      <button onclick="cmd('straight')">Прямо</button>
      <button class="secondary" onclick="cmd('line')">Линия</button>
      <button class="danger" onclick="cmd('stop')">Стоп</button>
    </div>
    <div class="btn-row">
      <button class="secondary" onclick="cmd('manual')">Ручной</button>
      <button class="secondary" onclick="refresh()">Обновить</button>
    </div>
    <h2 class="section-gap">Захват</h2>
    <div class="btn-row three">
      <button class="gripper" onclick="cmd('gripper_open')">Открыть</button>
      <button class="gripper" onclick="cmd('gripper_close')">Закрыть</button>
      <button class="gripper" onclick="cmd('gripper_reset')">Сброс</button>
    </div>
    <div class="status-row">
      <span class="status-label">Состояние</span>
      <span class="tag" id="statusDisplay">IDLE</span>
    </div>
  </div>

  <!-- Card 2: Sensors -->
  <div class="card">
    <h2>Датчики</h2>
    <div class="kv"><span class="kv-k">Дистанция (мм)</span><span class="kv-v" id="distance">--</span></div>
    <div class="kv"><span class="kv-k">Ошибка линии</span><span class="kv-v" id="lineError">0</span></div>
    <div class="kv"><span class="kv-k">Угол</span><span class="kv-v" id="angle">0.0°</span></div>
    <div class="kv"><span class="kv-k">На уклоне</span><span class="kv-v" id="onSlope">НЕТ</span></div>
    <div class="kv"><span class="kv-k">Throttle</span><span class="kv-v" id="throttle">0</span></div>
    <div class="kv"><span class="kv-k">M1 / M2</span><span class="kv-v" id="motors">0 / 0</span></div>
    <h2 class="section-gap">Датчик линии (5x ADC)</h2>
    <div class="line-bars" id="lineSensors">
      <div class="s-bar"><div class="s-fill" id="sensor0" style="height:0%"></div></div>
      <div class="s-bar"><div class="s-fill" id="sensor1" style="height:0%"></div></div>
      <div class="s-bar"><div class="s-fill" id="sensor2" style="height:0%"></div></div>
      <div class="s-bar"><div class="s-fill" id="sensor3" style="height:0%"></div></div>
      <div class="s-bar"><div class="s-fill" id="sensor4" style="height:0%"></div></div>
    </div>
    <div class="kv"><span class="kv-k">Heap free</span><span class="kv-v" id="heap">--</span></div>
  </div>

  <!-- Card 3: Calibration -->
  <div class="card">
    <h2>Калибровка линии (NVS)</h2>
    <p style="font-size:11px;color:#8b949e;margin-bottom:8px">
      Min/max АЦП для каждого из 5 датчиков. Сохраняются в NVS, переживают рефлеш.
    </p>
    <div style="display:flex;gap:10px;margin-bottom:6px">
      <div style="flex:1">
        <label>min</label>
        <div class="calib-grid" id="minRow"></div>
      </div>
      <div style="flex:1">
        <label>max</label>
        <div class="calib-grid" id="maxRow"></div>
      </div>
    </div>
    <div class="btn-row">
      <button class="secondary" onclick="calibRun(3000)">Калибровать 3с</button>
      <button class="secondary" onclick="calibRun(5000)">5с</button>
    </div>
    <div class="btn-row">
      <button onclick="calibSave()">Сохранить</button>
      <button class="danger" onclick="calibReset()">Сброс</button>
    </div>
    <h2 class="section-gap">MPU offset</h2>
    <div class="kv"><span class="kv-k">gyro offset</span><span class="kv-v" id="gyroOffset">--</span></div>
    <h2 class="section-gap">Wi-Fi настройка</h2>
    <label>SSID</label>
    <input type="text" id="wifiSsid" placeholder="BalanceRobot-Setup">
    <label style="margin-top:6px">Пароль</label>
    <input type="password" id="wifiPass" placeholder="минимум 8 символов">
    <div class="btn-row" style="margin-top:8px">
      <button onclick="wifiSave()">Сохранить и перезагрузить</button>
      <button class="danger" onclick="wifiReset()">Очистить Wi-Fi</button>
    </div>
  </div>
</div>
</main>
<div class="toast" id="toast"></div>
<script>
  const STATE_NAMES  = ['IDLE', 'STRAIGHT', 'LINE', 'STOPPED', 'MANUAL'];
  const STATE_CLASSES = ['', 'green', 'blue', 'red', 'purple'];
  const SENSOR_DEFAULT_MIN = [500, 500, 500, 500, 500];
  const SENSOR_DEFAULT_MAX = [3500, 3500, 3500, 3500, 3500];

  function toast(msg, kind) {
    const el = document.getElementById('toast');
    el.textContent = msg;
    el.className = 'toast show ' + (kind || '');
    setTimeout(() => el.className = 'toast', 2900);
  }
  async function api(path, opts) {
    try {
      const r = await fetch(path, opts);
      if (!r.ok) { toast('HTTP ' + r.status + ' ' + r.statusText, 'error'); return null; }
      return await r.json();
    } catch (e) { toast('Сеть: ' + e.message, 'error'); return null; }
  }
  async function cmd(c) { const d = await api('/api/command?cmd=' + c); if (d) render(d); }
  async function refresh() { const d = await api('/api/status'); if (d) render(d); }

  // Calibration inputs
  function buildCalibInputs() {
    const mk = (rowId) => {
      const r = document.getElementById(rowId);
      r.innerHTML = '';
      for (let i = 0; i < 5; i++) {
        const c = document.createElement('div');
        c.className = 'calib-cell';
        c.innerHTML = '<div class="v">S' + i + '</div><input type="number" min="0" max="4095" id="' + rowId + i + '">';
        r.appendChild(c);
      }
    };
    mk('minRow'); mk('maxRow');
  }
  buildCalibInputs();

  async function calibLoad() {
    const d = await api('/api/calib');
    if (!d) return;
    const fill = (id, vals) => vals.forEach((v, i) => document.getElementById(id + i).value = v);
    fill('minRow', d.line_min);
    fill('maxRow', d.line_max);
    document.getElementById('gyroOffset').textContent = d.gyro_offset;
  }
  async function calibSave() {
    const line_min = [], line_max = [];
    for (let i = 0; i < 5; i++) {
      line_min.push(parseInt(document.getElementById('minRow' + i).value) || 0);
      line_max.push(parseInt(document.getElementById('maxRow' + i).value) || 0);
    }
    const r = await fetch('/api/calib', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({ line_min, line_max })
    });
    if (r.ok) toast('Калибровка сохранена', 'ok');
    else toast('Ошибка сохранения: ' + r.status, 'error');
  }
  async function calibReset() {
    const r = await api('/api/calib/reset');
    if (r) { toast('Сброшено к дефолтам', 'ok'); calibLoad(); }
  }
  async function calibRun(ms) {
    if (!confirm('Авто-калибровка ' + (ms/1000) + 'с: водите датчиком по белому/чёрному. Продолжить?')) return;
    toast('Калибрую ' + (ms/1000) + 'с...', '');
    document.querySelectorAll('.s-bar').forEach(b => b.classList.add('calibrating'));
    const d = await api('/api/calib/line/run?ms=' + ms);
    document.querySelectorAll('.s-bar').forEach(b => b.classList.remove('calibrating'));
    if (d) {
      toast('Готово: ' + JSON.stringify(d.line_min) + ' / ' + JSON.stringify(d.line_max), 'ok');
      calibLoad();
    }
  }

  async function wifiSave() {
    const ssid = document.getElementById('wifiSsid').value.trim();
    const pass = document.getElementById('wifiPass').value;
    if (!ssid) { toast('SSID пустой', 'error'); return; }
    if (pass.length < 8) { toast('Пароль минимум 8 символов', 'error'); return; }
    const r = await fetch('/api/save-wifi?ssid=' + encodeURIComponent(ssid) + '&pass=' + encodeURIComponent(pass));
    toast(r.ok ? 'Сохранено, перезагрузка...' : 'Ошибка: ' + r.status, r.ok ? 'ok' : 'error');
  }
  async function wifiReset() {
    if (!confirm('Очистить Wi-Fi креды? Робот перезагрузится в режим AP.')) return;
    await api('/api/reset-wifi');
    toast('Очищено, перезагрузка...', 'ok');
  }

  // Joystick
  const joystick = document.getElementById('joystick');
  const container = document.getElementById('joystickBox');
  let joystickActive = false;
  function setJoystick(x, y) {
    const rect = container.getBoundingClientRect();
    const cx = rect.width / 2, cy = rect.height / 2;
    const radius = cy - 22;
    joystick.style.left = (cx + x - 22) + 'px';
    joystick.style.top  = (cy + y - 22) + 'px';
  }
  function joystickReset() {
    joystickActive = false;
    setJoystick(0, 0);
    sendJoystick(0, 0);
  }
  function joystickMove(clientX, clientY) {
    if (!joystickActive) return;
    const rect = container.getBoundingClientRect();
    const cx = rect.width / 2, cy = rect.height / 2;
    const radius = cy - 22;
    let x = clientX - rect.left - cx;
    let y = clientY - rect.top  - cy;
    const d = Math.sqrt(x*x + y*y);
    if (d > radius) { const a = Math.atan2(y, x); x = Math.cos(a)*radius; y = Math.sin(a)*radius; }
    setJoystick(x, y);
    sendJoystick(Math.round(-y/radius*200), Math.round(x/radius*100));
  }
  async function sendJoystick(t, s) {
    await api('/api/joystick?throttle=' + t + '&steering=' + s);
  }
  joystick.addEventListener('mousedown',  e => { joystickActive = true; joystickMove(e.clientX, e.clientY); });
  joystick.addEventListener('touchstart', e => { joystickActive = true; joystickMove(e.touches[0].clientX, e.touches[0].clientY); e.preventDefault(); }, {passive:false});
  document.addEventListener('mousemove',  e => { if (joystickActive) joystickMove(e.clientX, e.clientY); });
  document.addEventListener('touchmove',  e => { if (joystickActive && e.touches[0]) joystickMove(e.touches[0].clientX, e.touches[0].clientY); }, {passive:true});
  document.addEventListener('mouseup',   joystickReset);
  document.addEventListener('touchend',  joystickReset);

  function render(d) {
    const label = STATE_NAMES[d.state] || 'UNKNOWN';
    const cls   = STATE_CLASSES[d.state] || '';
    ['statusDisplay', 'statusTag'].forEach(id => {
      const el = document.getElementById(id);
      el.textContent = label;
      el.className = 'tag' + (cls ? ' ' + cls : '');
    });
    document.getElementById('wifiTag').textContent = 'WiFi: ' + (d.wifi || 'off');
    document.getElementById('distance').textContent  = d.distance ?? '--';
    document.getElementById('lineError').textContent = d.lineError ?? '0';
    document.getElementById('angle').textContent     = (d.angle || 0).toFixed(1) + '°';
    document.getElementById('onSlope').textContent   = d.slope ? 'ДА' : 'НЕТ';
    document.getElementById('throttle').textContent  = d.throttle ?? '0';
    document.getElementById('motors').textContent    = (d.m1 ?? 0) + ' / ' + (d.m2 ?? 0);
    document.getElementById('heap').textContent      = d.heap ? Math.round(d.heap/1024) + ' KB' : '--';
    if (d.sensors) {
      for (let i = 0; i < 5; i++) {
        const f = document.getElementById('sensor' + i);
        if (f) f.style.height = ((d.sensors[i] || 0) / 200 * 100) + '%';
      }
    }
  }

  calibLoad();
  setInterval(refresh, 1000);
  refresh();
</script>
</body>
</html>
)rawliteral";

// ============================================================================
// CONTROL FUNCTIONS (PID)
// ============================================================================

float stabilityPDControl(float DT, float input, float setPoint, float Kp, float Kd) {
  static float prev_error = 0;
  float error = setPoint - input;
  float derivative = (error - prev_error) / DT;
  float output = 0;
  if (fabs(error) > 0.3) {
    output = Kp * error + Kd * derivative;
  }
  prev_error = error;
  output = constrain(output, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);
  return output;
}

float speedPIControl(float DT, int16_t input, int16_t setPoint, float Kp, float Ki) {
  int16_t error = setPoint - input;
  PID_errorSum += constrain(error, -ITERM_MAX_ERROR, ITERM_MAX_ERROR);
  PID_errorSum = constrain(PID_errorSum, -ITERM_MAX, ITERM_MAX);
  float output = Kp * error + Ki * PID_errorSum * DT;
  return output;
}

// ============================================================================
// MOTOR FUNCTIONS (with Timer API v3.x)
// ============================================================================

void setMotorSpeedM1(int16_t tspeed) {
  long timer_period;
  int16_t speed;
  if ((speed_M1 - tspeed) > MAX_ACCEL) speed_M1 -= MAX_ACCEL;
  else if ((speed_M1 - tspeed) < -MAX_ACCEL) speed_M1 += MAX_ACCEL;
  else speed_M1 = tspeed;
#if MICROSTEPPING==16
  speed = speed_M1 * 50;
#else
  speed = speed_M1 * 25;
#endif
  if (speed == 0) {
    timer_period = ZERO_SPEED;
    dir_M1 = 0;
  } else if (speed > 0) {
    timer_period = 2000000 / speed;
    dir_M1 = 1;
    digitalWrite(PIN_MOTOR1_DIR, HIGH);
  } else {
    timer_period = 2000000 / -speed;
    dir_M1 = -1;
    digitalWrite(PIN_MOTOR1_DIR, LOW);
  }
  if (timer_period > ZERO_SPEED) timer_period = ZERO_SPEED;
  timerAlarm(timer1, timer_period, true, 0);
}

void setMotorSpeedM2(int16_t tspeed) {
  long timer_period;
  int16_t speed;
  if ((speed_M2 - tspeed) > MAX_ACCEL) speed_M2 -= MAX_ACCEL;
  else if ((speed_M2 - tspeed) < -MAX_ACCEL) speed_M2 += MAX_ACCEL;
  else speed_M2 = tspeed;
#if MICROSTEPPING==16
  speed = speed_M2 * 50;
#else
  speed = speed_M2 * 25;
#endif
  if (speed == 0) {
    timer_period = ZERO_SPEED;
    dir_M2 = 0;
    digitalWrite(PIN_MOTOR2_STEP, LOW);
  } else if (speed > 0) {
    timer_period = 2000000 / speed;
    dir_M2 = 1;
    digitalWrite(PIN_MOTOR2_DIR, HIGH);
  } else {
    timer_period = 2000000 / -speed;
    dir_M2 = -1;
    digitalWrite(PIN_MOTOR2_DIR, LOW);
  }
  if (timer_period > ZERO_SPEED) timer_period = ZERO_SPEED;
  timerAlarm(timer2, timer_period, true, 0);
}

// ============================================================================
// TIMER ISR FUNCTIONS
// ============================================================================

portMUX_TYPE muxer1 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE muxer2 = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR timer1ISR() {
  portENTER_CRITICAL_ISR(&muxer1);
  if (dir_M1 != 0) {
    digitalWrite(PIN_MOTOR1_STEP, HIGH);
    if (dir_M1 > 0) steps1--; else steps1++;
    digitalWrite(PIN_MOTOR1_STEP, LOW);
  }
  portEXIT_CRITICAL_ISR(&muxer1);
}

void IRAM_ATTR timer2ISR() {
  portENTER_CRITICAL_ISR(&muxer2);
  if (dir_M2 != 0) {
    digitalWrite(PIN_MOTOR2_STEP, HIGH);
    if (dir_M2 > 0) steps2--; else steps2++;
    digitalWrite(PIN_MOTOR2_STEP, LOW);
  }
  portEXIT_CRITICAL_ISR(&muxer2);
}

void initTimers() {
  timer1 = timerBegin(2000000);
  timerAttachInterrupt(timer1, &timer1ISR);
  timerAlarm(timer1, ZERO_SPEED, true, 0);
  timer2 = timerBegin(2000000);
  timerAttachInterrupt(timer2, &timer2ISR);
  timerAlarm(timer2, ZERO_SPEED, true, 0);
}

// ============================================================================
// MPU6050 FUNCTIONS
// ============================================================================

float MPU6050_getAngle(float dt) {
  if (dt > 0.05) dt = 0.02;
  if (dt < 0.001) dt = 0.01;
  if (accel_t_gyro.value.z_accel != 0) {
    accel_angle = atan2f((float)accel_t_gyro.value.y_accel, (float)accel_t_gyro.value.z_accel) * RAD2GRAD;
  }
  x_gyro_value = (accel_t_gyro.value.x_gyro - x_gyro_offset) / 65.5;
  angle = 0.98 * (angle + x_gyro_value * dt) + 0.02 * accel_angle;
  int16_t correction = constrain(accel_t_gyro.value.x_gyro, x_gyro_offset - 10, x_gyro_offset + 10);
  x_gyro_offset = x_gyro_offset * 0.9995 + correction * 0.0005;
  return angle;
}

void MPU6050_calibrate() {
  int i;
  long value = 0;
  float dev;
  int16_t values[100];
  bool gyro_cal_ok = false;
  uint8_t attempts = 0;
  delay(500);
  while (!gyro_cal_ok && attempts < 10) {
    attempts++;
    Serial.print("[CALIB] Attempt ");
    Serial.print(attempts);
    Serial.println("/10 - DONT MOVE!");
    value = 0;
    for (i = 0; i < 100; i++) {
      MPU6050_read_3axis();
      values[i] = accel_t_gyro.value.x_gyro;
      value += accel_t_gyro.value.x_gyro;
      delay(25);
    }
    value = value / 100;
    dev = 0;
    for (i = 0; i < 100; i++) {
      dev += (float)(values[i] - value) * (values[i] - value);
    }
    dev = sqrt((1 / 100.0) * dev);
    Serial.print("offset: ");
    Serial.print(value);
    Serial.print("  stddev: ");
    Serial.println(dev);
    if (dev < 50.0) gyro_cal_ok = true;
  }
  x_gyro_offset = value;
  angle = atan2f((float)accel_t_gyro.value.y_accel, (float)accel_t_gyro.value.z_accel) * RAD2GRAD;
  Serial.print("[CALIB] Final offset: ");
  Serial.print(x_gyro_offset);
  Serial.print(" initial angle: ");
  Serial.println(angle);
}

void MPU6050_setup() {
  int error;
  uint8_t c;
  error = MPU6050_read(MPU6050_WHO_AM_I, &c, 1);
  Serial.print("WHO_AM_I: 0x");
  Serial.print(c, HEX);
  Serial.print(", error=");
  Serial.println(error);
  MPU6050_write_reg(MPU6050_PWR_MGMT_1, bit(MPU6050_DEVICE_RESET));
  delay(125);
  MPU6050_write_reg(MPU6050_PWR_MGMT_1, 0x01);
  MPU6050_write_reg(MPU6050_GYRO_CONFIG, MPU6050_FS_SEL_500);
  MPU6050_write_reg(MPU6050_ACCEL_CONFIG, MPU6050_AFS_SEL_2G);
  MPU6050_write_reg(MPU6050_CONFIG, MPU6050_DLPF_10HZ);
  MPU6050_write_reg(MPU6050_SMPLRT_DIV, 9);
  MPU6050_write_reg(MPU6050_INT_ENABLE, MPU6050_DATA_RDY_EN);
  MPU6050_write_reg(MPU6050_PWR_MGMT_1, 0x01);
}

void MPU6050_read_3axis() {
  mpu_last_error = MPU6050_read(MPU6050_ACCEL_XOUT_H, (uint8_t *)&accel_t_gyro, sizeof(accel_t_gyro));
  if (mpu_last_error != 0) return;  // leave previous data; controlLoop will notice
  SWAP(accel_t_gyro.reg.x_accel_h, accel_t_gyro.reg.x_accel_l);
  SWAP(accel_t_gyro.reg.y_accel_h, accel_t_gyro.reg.y_accel_l);
  SWAP(accel_t_gyro.reg.z_accel_h, accel_t_gyro.reg.z_accel_l);
  SWAP(accel_t_gyro.reg.t_h, accel_t_gyro.reg.t_l);
  SWAP(accel_t_gyro.reg.x_gyro_h, accel_t_gyro.reg.x_gyro_l);
  SWAP(accel_t_gyro.reg.y_gyro_h, accel_t_gyro.reg.y_gyro_l);
  SWAP(accel_t_gyro.reg.z_gyro_h, accel_t_gyro.reg.z_gyro_l);
}

int MPU6050_read(int start, uint8_t *buffer, int size) {
  int i, n, error;
  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);
  if (n != 1) return (-10);
  n = Wire.endTransmission(false);
  if (n != 0) return (n);
  Wire.requestFrom(MPU6050_I2C_ADDRESS, size, true);
  i = 0;
  while (Wire.available() && i < size) {
    buffer[i++] = Wire.read();
  }
  if (i != size) return (-11);
  return (0);
}

int MPU6050_write_reg(int reg, uint8_t data) {
  int error = MPU6050_write(reg, &data, 1);
  return (error);
}

int MPU6050_write(int start, const uint8_t *pData, int size) {
  int n, error;
  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);
  if (n != 1) return (-20);
  n = Wire.write(pData, size);
  if (n != size) return (-21);
  error = Wire.endTransmission(true);
  if (error != 0) return (error);
  return (0);
}

// ============================================================================
// LINE FOLLOWER FUNCTIONS
// ============================================================================

void LineFollower_init() {
  analogSetWidth(12);
  analogSetAttenuation(ADC_11db);
  Serial.println("[LineFollower] Initialized - 5 sensor array");
  for (uint8_t i = 0; i < 5; i++) {
    line_min_values[i] = 500;
    line_max_values[i] = 3500;
  }
}

void LineFollower_readSensors() {
  static uint16_t sample_buffer[5][LINE_SENSOR_SAMPLES];
  static uint8_t sample_index = 0;
  for (uint8_t i = 0; i < 5; i++) {
    sample_buffer[i][sample_index] = analogRead(sensor_pins[i]);
  }
  sample_index++;
  if (sample_index >= LINE_SENSOR_SAMPLES) {
    sample_index = 0;
  }
  for (uint8_t i = 0; i < 5; i++) {
    uint32_t sum = 0;
    for (uint8_t j = 0; j < LINE_SENSOR_SAMPLES; j++) {
      sum += sample_buffer[i][j];
    }
    line_sensor_raw[i] = sum / LINE_SENSOR_SAMPLES;
    uint16_t min_val = line_min_values[i];
    uint16_t max_val = line_max_values[i];
    if (line_sensor_raw[i] < min_val) {
      line_sensor_calibrated[i] = 0;
    } else if (line_sensor_raw[i] > max_val) {
      line_sensor_calibrated[i] = 200;
    } else {
      line_sensor_calibrated[i] = (uint32_t)(line_sensor_raw[i] - min_val) * 200 / (max_val - min_val);
    }
  }
}

int16_t LineFollower_calculateError() {
  uint32_t total_weight = 0;
  int32_t weighted_sum = 0;
  uint8_t active_sensors = 5;
  bool center_stuck = (line_sensor_calibrated[1] == 0 &&
                       line_sensor_calibrated[2] == 0 &&
                       line_sensor_calibrated[3] == 0);
  if (center_stuck && (line_sensor_calibrated[0] > 0 || line_sensor_calibrated[4] > 0)) {
    active_sensors = 2;
    weighted_sum = (line_sensor_calibrated[0] * (-2)) + (line_sensor_calibrated[4] * 2);
    total_weight = line_sensor_calibrated[0] + line_sensor_calibrated[4];
  } else {
    for (uint8_t i = 0; i < 5; i++) {
      uint16_t sensor_value = line_sensor_calibrated[i];
      weighted_sum += sensor_value * sensor_weights[i];
      total_weight += sensor_value;
    }
  }
  if (total_weight == 0) {
    line_position_error = 0;
  } else {
    line_position_error = constrain((int16_t)(weighted_sum * 200 / total_weight), -200, 200);
  }
  return line_position_error;
}

int16_t LineFollower_getSteering() {
  int16_t error = LineFollower_calculateError();
  float Kp_line = 0.5;
  line_steering_output = constrain((int16_t)(error * Kp_line), -LINE_MAX_ERROR, LINE_MAX_ERROR);
  return line_steering_output;
}

void LineFollower_setMode(uint8_t mode) {
  if (mode <= LINE_MODE_AUTO) {
    line_mode = mode;
    if (mode == LINE_MODE_OFF) {
      line_steering_output = 0;
      Serial.println("[LineFollower] Mode: OFF");
    } else if (mode == LINE_MODE_ON) {
      Serial.println("[LineFollower] Mode: ON");
    } else {
      Serial.println("[LineFollower] Mode: AUTO");
    }
  }
}

uint8_t LineFollower_getMode() {
  return line_mode;
}

// ============================================================================
// INITIALIZATION
// ============================================================================

bool initMPU6050() {
  Serial.println("[MPU6050] Initializing...");
  Wire.begin(21, 22);
  Wire.setClock(400000);
  vTaskDelay(pdMS_TO_TICKS(100));
  MPU6050_setup();
  vTaskDelay(pdMS_TO_TICKS(500));
  uint8_t cal_attempts = 0;
  while (cal_attempts < 3) {
    cal_attempts++;
    Serial.print("[CALIB] Attempt ");
    Serial.print(cal_attempts);
    Serial.println("/3");
    MPU6050_calibrate();
  }
  vTaskDelay(pdMS_TO_TICKS(500));
  Serial.println("[OK] MPU6050 ready");
  return true;
}

void initDistanceSensor() {
  Serial.println("[VL53L0X] Initializing...");
  Wire.beginTransmission(0x29);
  if (Wire.endTransmission() != 0) {
    Serial.println("[ERROR] VL53L0X not found!");
    return;
  }
  distanceSensor.setTimeout(500);
  if (!distanceSensor.init()) {
    Serial.println("[ERROR] VL53L0X init failed!");
    return;
  }
  distanceSensor.setMeasurementTimingBudget(33000);
  distanceSensor.startContinuous(0);
  vl53_ready = true;
  Serial.println("[OK] VL53L0X ready (continuous mode)");
}

// ============================================================================
// CONTROL LOOP
// ============================================================================

bool detectSlope() {
  int32_t current_z = accel_t_gyro.value.z_accel;
  int32_t z_delta = (current_z > z_accel_baseline) ?
                    (current_z - z_accel_baseline) :
                    (z_accel_baseline - current_z);
  return (z_delta > SLOPE_ACCEL_THRESHOLD);
}

int16_t applySlopeDamping(int16_t speed) {
  if (on_slope) return (speed >> 1);
  return speed;
}

void controlLoop() {
  // Emergency: angle past limit OR MPU I2C error -> stop motors, keep monitoring.
  if (fabsf(angle_adjusted) > EMERGENCY_TILT_DEG || mpu_last_error != 0) {
    setMotorSpeedM1(0);
    setMotorSpeedM2(0);
    if (mpu_last_error != 0 && (loop_counter % 50) == 0) {
      Serial.print("[MPU ERR] code=");
      Serial.println((int)mpu_last_error);
    }
    return;
  }

  // Non-blocking VL53L0X read (Pololu lib has no public dataReady()):
  // poll RESULT_INTERRUPT_STATUS (0x13), bits [2:0] == 0x04 => new range ready.
  // readRangeContinuousMillimeters() then returns immediately and clears the IRQ itself.
  if (vl53_ready && ((distanceSensor.readReg(0x13) & 0x07) == 0x04)) {
    uint16_t new_mm = distanceSensor.readRangeContinuousMillimeters();
    if (new_mm > 0 && new_mm != 65535) {
      distance_mm = new_mm;
    }
  }

  // Obstacle check applies to all autonomous states; STATE_MANUAL still reacts for safety
  if (robot_state != STATE_IDLE && distance_mm > 0 && distance_mm < DISTANCE_THRESHOLD) {
    robot_state = STATE_STOPPED;
    throttle = 0;
    steering = 0;
    obstacle_clear_since_ms = 0;
    setMotorSpeedM1(0);
    setMotorSpeedM2(0);
    if (loop_counter % 100 == 0) {
      Serial.print("[OBSTACLE] ");
      Serial.print(distance_mm);
      Serial.println(" mm");
    }
    return;
  }

  on_slope = detectSlope();

  // True step-based odometry: read step counters atomically, compute delta since last cycle
  // (forward = previous - current, per ISR sign convention). Scale to speed_M units.
  int32_t d1 = last_steps1 - steps1;
  int32_t d2 = last_steps2 - steps2;
  last_steps1 = steps1;
  last_steps2 = steps2;
  actual_robot_speed_Old = actual_robot_speed;
  actual_robot_speed = (int16_t)(((int32_t)(d1 + d2) * SPEED_ODOM_SCALE) / 2);
  estimated_speed_filtered = estimated_speed_filtered * (1.0f - SPEED_ODOM_FILTER)
                            + (float)actual_robot_speed * SPEED_ODOM_FILTER;

  float balance_output = stabilityPDControl(dt, angle_adjusted, target_angle, Kp, Kd);
  float speed_output = speedPIControl(dt, (int16_t)estimated_speed_filtered, throttle, Kp_thr, Ki_thr);
  control_output = balance_output + speed_output;

  switch (robot_state) {
    case STATE_IDLE:
#if AUTO_START_ON_BOOT
      if ((unsigned long)(millis() - state_start_time) > 5000) {
        robot_state = STATE_STRAIGHT;
        state_start_time = millis();
        Serial.println("[AUTO] IDLE -> STRAIGHT (5s balance complete)");
      }
#endif
      break;
    case STATE_STRAIGHT:
      throttle = 150;
      steering = 0;
      if ((unsigned long)(millis() - state_start_time) > STRAIGHT_TIME) {
        robot_state = STATE_LINE_FOLLOW;
        Serial.println("[AUTO] STRAIGHT -> LINE_FOLLOW");
      }
      break;
    case STATE_LINE_FOLLOW:
      throttle = 150;
      if (line_mode != LINE_MODE_OFF) {
        int16_t line_steering = LineFollower_getSteering();
        steering = constrain(line_steering, -max_steering, max_steering);
      } else {
        steering = 0;
      }
      break;
    case STATE_STOPPED:
      // Auto-resume once obstacle has been clear for STOPPED_RESUME_HOLD_MS
      throttle = 0;
      steering = 0;
      if (distance_mm > STOPPED_RESUME_DISTANCE) {
        if (obstacle_clear_since_ms == 0) {
          obstacle_clear_since_ms = millis();
        } else if ((unsigned long)(millis() - obstacle_clear_since_ms) > STOPPED_RESUME_HOLD_MS) {
          robot_state = STATE_LINE_FOLLOW;
          obstacle_clear_since_ms = 0;
          Serial.println("[AUTO] STOPPED -> LINE_FOLLOW (obstacle cleared)");
        }
      } else {
        obstacle_clear_since_ms = 0;
      }
      break;
    case STATE_MANUAL:
      // Joystick values written by /api/joystick handler
      throttle = api_manual_throttle_val;
      steering = api_manual_steering_val;
      break;
  }

  // Slope damping slows forward motion only — NEVER touch balance_output, that is
  // exactly when full correction is needed.
  if (on_slope) {
    throttle = applySlopeDamping(throttle);
    steering = applySlopeDamping(steering);
  }
  control_output = constrain(control_output, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);
  steering = constrain(steering, -max_steering, max_steering);
  throttle = constrain(throttle, -max_throttle, max_throttle);
  setMotorSpeedM1(control_output - steering);
  setMotorSpeedM2(control_output + steering);
  if (loop_counter % 10 == 0) {
    Serial.print("[DBG] A:");
    Serial.print(angle_adjusted, 2);
    Serial.print("\xb0 Bal:");  // ° in extended ASCII for ASCII-clean terminals
    Serial.print((int)balance_output);
    Serial.print(" Spd:");
    Serial.print((int)speed_output);
    Serial.print(" Out:");
    Serial.print((int)control_output);
    Serial.print(" M1:");
    Serial.print(speed_M1);
    Serial.print(" Str:");
    Serial.print(steering);
    Serial.print(" St:");
    Serial.print(robot_state);
    if (line_mode != LINE_MODE_OFF && robot_state == STATE_LINE_FOLLOW) {
      Serial.print(" [Line] ");
      for (int i = 0; i < 5; i++) {
        Serial.print(line_sensor_calibrated[i]);
        Serial.print(" ");
      }
      Serial.print("| Error:");
      Serial.print(line_position_error);
    }
    Serial.println();
  }
}

void readSensors() {
  MPU6050_read_3axis();
  angle_adjusted = MPU6050_getAngle(dt);
  if (line_mode != LINE_MODE_OFF) {
    LineFollower_readSensors();
  }
}

// ============================================================================
// SERVO (gripper)
// ============================================================================
#if ENABLE_WIFI
// 50Hz PWM, 16-bit: 1ms pulse = ~3277 ticks
#define SERVO_PWM_FREQ  50
#define SERVO_PWM_RES   16
#define SERVO_TICKS_PER_US 3.2768f
#define SERVO_MIN_US    1000
#define SERVO_MAX_US    2000
#define SERVO_OPEN_DEG  180
#define SERVO_CLOSE_DEG 90

static bool servo_attached = false;
static int  servo_left_deg  = 0;
static int  servo_right_deg = 0;

void servoAttachOnce() {
  if (servo_attached) return;
  ledcAttach(PIN_SERVO_LEFT,  SERVO_PWM_FREQ, SERVO_PWM_RES);
  ledcAttach(PIN_SERVO_RIGHT, SERVO_PWM_FREQ, SERVO_PWM_RES);
  servo_attached = true;
}

void setServoAngle(int pin, int angle) {
  servoAttachOnce();
  angle = constrain(angle, 0, 180);
  int pulse_us = SERVO_MIN_US + (angle * (SERVO_MAX_US - SERVO_MIN_US)) / 180;
  int pwm = (int)(pulse_us * SERVO_TICKS_PER_US);
  ledcWrite(pin, pwm);
}

void gripperOpen()  { setServoAngle(PIN_SERVO_LEFT, SERVO_OPEN_DEG);  setServoAngle(PIN_SERVO_RIGHT, SERVO_OPEN_DEG);  servo_left_deg = servo_right_deg = SERVO_OPEN_DEG; }
void gripperClose() { setServoAngle(PIN_SERVO_LEFT, SERVO_CLOSE_DEG); setServoAngle(PIN_SERVO_RIGHT, SERVO_CLOSE_DEG); servo_left_deg = servo_right_deg = SERVO_CLOSE_DEG; }
void gripperReset() { setServoAngle(PIN_SERVO_LEFT, 0);               setServoAngle(PIN_SERVO_RIGHT, 0);               servo_left_deg = servo_right_deg = 0; }
#endif

// ============================================================================
// WIFI + WEB API
// ============================================================================
#if ENABLE_WIFI
static const char WIFI_NAMESPACE[] = "balance_robot";
static const char PREF_SSID[]      = "ssid";
static const char PREF_PASS[]      = "pass";
static const char PREF_BLANK[]     = "";

// Non-blocking WiFi LED blinker (called from loop())
void wifiLedTick() {
  unsigned long now = millis();
  bool target = false;
  switch (wifi_mode) {
    case WIFI_OFFLINE:    target = false; break;
    case WIFI_CONNECTING: target = (now % 400) < 200; break;     // fast blink
    case WIFI_AP_MODE:    target = (now % 1500) < 300; break;    // slow blink
    case WIFI_STA_OK:     target = true; break;                  // solid
  }
  if (target != wifi_led_state) {
    wifi_led_state = target;
    digitalWrite(PIN_WIFI_LED, target ? HIGH : LOW);
  }
}

bool wifiLoadCreds(String &ssid, String &pass) {
  Preferences prefs;
  prefs.begin(WIFI_NAMESPACE, true);
  ssid = prefs.getString(PREF_SSID, PREF_BLANK);
  pass = prefs.getString(PREF_PASS, PREF_BLANK);
  prefs.end();
  return ssid.length() > 0;
}

void wifiSaveCreds(const String &ssid, const String &pass) {
  Preferences prefs;
  prefs.begin(WIFI_NAMESPACE, false);
  prefs.putString(PREF_SSID, ssid);
  prefs.putString(PREF_PASS, pass);
  prefs.end();
}

// ============================================================================
// CALIBRATION (NVS-backed, survives reflash)
// ============================================================================
//
// Stored in Preferences namespace "calib":
//   line_min_0..4 / line_max_0..4  : uint16_t, ADC raw thresholds
//   gyro_offset                     : float, MPU6050 X-gyro zero-rate
//   z_baseline                      : int16_t, Z-axis accel at boot
//
// Defaults: 500 / 3500 (hardcoded in this file).
// To reset, call /api/calib/reset or erase the namespace via Preferences.clear().

static const char CALIB_NAMESPACE[] = "calib";
static const uint16_t CALIB_DEFAULT_MIN[5] = {500, 500, 500, 500, 500};
static const uint16_t CALIB_DEFAULT_MAX[5] = {3500, 3500, 3500, 3500, 3500};

void loadCalibration() {
  Preferences prefs;
  if (!prefs.begin(CALIB_NAMESPACE, true)) {
    // Namespace missing: use hardcoded defaults
    Serial.println("[CALIB] No NVS data, using defaults");
    return;
  }
  for (uint8_t i = 0; i < 5; i++) {
    char key[12];
    snprintf(key, sizeof(key), "lmin%d", i);
    line_min_values[i] = prefs.getUShort(key, CALIB_DEFAULT_MIN[i]);
    snprintf(key, sizeof(key), "lmax%d", i);
    line_max_values[i] = prefs.getUShort(key, CALIB_DEFAULT_MAX[i]);
  }
  // Optional: gyro / z-baseline loaded separately because they're not yet
  // captured at setup() time; see saveCalibration() and the auto-capture in
  // setup() after MPU calibration.
  prefs.end();
  Serial.print("[CALIB] Loaded min[0..4] = ");
  for (uint8_t i = 0; i < 5; i++) { Serial.print(line_min_values[i]); Serial.print(i < 4 ? "," : ""); }
  Serial.print(" max[0..4] = ");
  for (uint8_t i = 0; i < 5; i++) { Serial.print(line_max_values[i]); Serial.print(i < 4 ? "," : ""); }
  Serial.println();
}

void saveCalibration() {
  Preferences prefs;
  prefs.begin(CALIB_NAMESPACE, false);
  for (uint8_t i = 0; i < 5; i++) {
    char key[12];
    snprintf(key, sizeof(key), "lmin%d", i);
    prefs.putUShort(key, line_min_values[i]);
    snprintf(key, sizeof(key), "lmax%d", i);
    prefs.putUShort(key, line_max_values[i]);
  }
  prefs.putFloat("gyro_off", x_gyro_offset);
  prefs.putShort("z_base", z_accel_baseline);
  prefs.end();
  Serial.println("[CALIB] Saved to NVS");
}

void resetCalibration() {
  Preferences prefs;
  prefs.begin(CALIB_NAMESPACE, false);
  prefs.clear();
  prefs.end();
  for (uint8_t i = 0; i < 5; i++) {
    line_min_values[i] = CALIB_DEFAULT_MIN[i];
    line_max_values[i] = CALIB_DEFAULT_MAX[i];
  }
  Serial.println("[CALIB] Reset to defaults (500 / 3500)");
}

// Blocking auto-calibration: samples raw ADC for `duration_ms` while the user
// moves the sensor over the line. Only safe in IDLE; returns 409 if robot is
// moving. Outputs the collected min/max as JSON.
String runLineCalibrationBlocking(uint16_t duration_ms) {
  if (robot_state != STATE_IDLE) {
    return String("{\"error\":\"robot must be IDLE (currently state=") + String(robot_state) + "\"}";
  }
  uint16_t mn[5] = {4095, 4095, 4095, 4095, 4095};
  uint16_t mx[5] = {0, 0, 0, 0, 0};
  unsigned long t0 = millis();
  while ((unsigned long)(millis() - t0) < duration_ms) {
    // Force a fresh read (skips LINE_MODE_OFF gate)
    for (uint8_t i = 0; i < 5; i++) {
      int v = analogRead(sensor_pins[i]);
      if (v < mn[i]) mn[i] = v;
      if (v > mx[i]) mx[i] = v;
    }
    // Feed the watch dog
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  // Update globals + persist
  for (uint8_t i = 0; i < 5; i++) {
    line_min_values[i] = mn[i];
    line_max_values[i] = mx[i];
  }
  saveCalibration();
  char buf[256];
  snprintf(buf, sizeof(buf),
    "{\"line_min\":[%u,%u,%u,%u,%u],\"line_max\":[%u,%u,%u,%u,%u],\"duration_ms\":%u}",
    mn[0],mn[1],mn[2],mn[3],mn[4],
    mx[0],mx[1],mx[2],mx[3],mx[4],
    (unsigned)duration_ms);
  return String(buf);
}

String buildCalibJson() {
  char buf[256];
  snprintf(buf, sizeof(buf),
    "{\"line_min\":[%u,%u,%u,%u,%u],\"line_max\":[%u,%u,%u,%u,%u],"
    "\"gyro_offset\":%.2f,\"z_baseline\":%d}",
    line_min_values[0],line_min_values[1],line_min_values[2],line_min_values[3],line_min_values[4],
    line_max_values[0],line_max_values[1],line_max_values[2],line_max_values[3],line_max_values[4],
    (double)x_gyro_offset, (int)z_accel_baseline);
  return String(buf);
}

// ============================================================================
// WEBSOCKET HANDLING & STATUS BROADCAST
// ============================================================================

void broadcastStatus() {
  if (!ws_connected) return;
  String json = buildStatusJson();
  ws.textAll(json);
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    ws_connected = true;
    ws_client_id = client->id();
    Serial.printf("[WS] Client #%u connected\n", client->id());
    client->text(buildStatusJson());
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("[WS] Client #%u disconnected\n", client->id());
    ws_connected = false;
  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      data[len] = 0;
      String msg = (char*)data;
      int type_start = msg.indexOf("\"type\":\"");
      if (type_start != -1) {
        type_start += 8;
        int type_end = msg.indexOf('"', type_start);
        String type = msg.substring(type_start, type_end);
        if (type == "pid") {
          auto extract = [&](String key, float &var, float def) {
            int idx = msg.indexOf("\"" + key + "\":");
            if (idx != -1) {
              idx += key.length() + 3;
              int end = msg.indexOf(',', idx);
              if (end == -1) end = msg.indexOf('}', idx);
              float val = msg.substring(idx, end).toFloat();
              var = val;
            }
          };
          extract("Kp", Kp, KP_DEFAULT);
          extract("Kd", Kd, KD_DEFAULT);
          extract("Kp_thr", Kp_thr, KP_THROTTLE_DEFAULT);
          extract("Ki_thr", Ki_thr, KI_THROTTLE_DEFAULT);
          savePID();
          ws.textAll(buildCalibJson());
          return;
        } else if (type == "joystick") {
          int t_idx = msg.indexOf("\"throttle\":");
          int s_idx = msg.indexOf("\"steering\":");
          if (t_idx != -1) {
            t_idx += 11;
            int end = msg.indexOf(',', t_idx);
            if (end == -1) end = msg.indexOf('}', t_idx);
            api_manual_throttle_val = constrain(msg.substring(t_idx, end).toInt(), -200, 200);
          }
          if (s_idx != -1) {
            s_idx += 11;
            int end = msg.indexOf(',', s_idx);
            if (end == -1) end = msg.indexOf('}', s_idx);
            api_manual_steering_val = constrain(msg.substring(s_idx, end).toInt(), -100, 100);
          }
          api_manual_throttle = true;
          if (robot_state != STATE_MANUAL) robot_state = STATE_MANUAL;
          return;
        } else if (type == "cmd") {
          int cmd_idx = msg.indexOf("\"cmd\":\"");
          if (cmd_idx != -1) {
            cmd_idx += 7;
            int end = msg.indexOf('"', cmd_idx);
            String cmd = msg.substring(cmd_idx, end);
            cmd.toLowerCase();
            if (cmd == "straight") { robot_state = STATE_STRAIGHT; state_start_time = millis(); }
            else if (cmd == "line") { LineFollower_setMode(LINE_MODE_ON); robot_state = STATE_LINE_FOLLOW; }
            else if (cmd == "stop" || cmd == "idle") { robot_state = STATE_IDLE; state_start_time = millis(); throttle = steering = 0; }
            else if (cmd == "manual") { robot_state = STATE_MANUAL; }
            else if (cmd == "gripper_open")  { gripperOpen(); }
            else if (cmd == "gripper_close") { gripperClose(); }
            else if (cmd == "gripper_reset") { gripperReset(); }
          }
          return;
        }
      }
    }
  }
}

// PID NVS Persistence
static const char PID_NAMESPACE[] = "pid";

void loadPID() {
  Preferences prefs;
  if (!prefs.begin("pid", true)) return;
  Kp = prefs.getFloat("Kp", KP_DEFAULT);
  Kd = prefs.getFloat("Kd", KD_DEFAULT);
  Kp_thr = prefs.getFloat("Kp_thr", KP_THROTTLE_DEFAULT);
  Ki_thr = prefs.getFloat("Ki_thr", KI_THROTTLE_DEFAULT);
  prefs.end();
  Serial.printf("[PID] Loaded: Kp=%.3f Kd=%.3f Kp_thr=%.3f Ki_thr=%.3f\n", Kp, Kd, Kp_thr, Ki_thr);
}

void savePID() {
  Preferences prefs;
  prefs.begin("pid", false);
  prefs.putFloat("Kp", Kp);
  prefs.putFloat("Kd", Kd);
  prefs.putFloat("Kp_thr", Kp_thr);
  prefs.putFloat("Ki_thr", Ki_thr);
  prefs.end();
  Serial.printf("[PID] Saved: Kp=%.3f Kd=%.3f Kp_thr=%.3f Ki_thr=%.3f\n", Kp, Kd, Kp_thr, Ki_thr);
}

void resetPID() {
  Kp = KP_DEFAULT; Kd = KD_DEFAULT; Kp_thr = KP_THROTTLE_DEFAULT; Ki_thr = KI_THROTTLE_DEFAULT;
  Preferences prefs;
  prefs.begin("pid", false);
  prefs.clear();
  prefs.end();
  Serial.println("[PID] Reset to defaults");
}

String buildPidJson() {
  char buf[128];
  snprintf(buf, sizeof(buf),
    "{\"Kp\":%.3f,\"Kd\":%.3f,\"Kp_thr\":%.3f,\"Ki_thr\":%.3f}",
    (double)Kp, (double)Kd, (double)Kp_thr, (double)Ki_thr);
  return String(buf);
}

// Build a JSON status blob (no ArduinoJson lib needed, hand-rolled)
String buildStatusJson() {
  char buf[640];
  int n = snprintf(buf, sizeof(buf),
    "{\"state\":%u,\"stateName\":\"%s\",\"distance\":%u,\"lineError\":%d,"
    "\"angle\":%.2f,\"onSlope\":%s,\"throttle\":%d,\"speedM1\":%d,\"speedM2\":%d,"
    "\"m1\":%d,\"m2\":%d,\"lineSensors\":[%d,%d,%d,%d,%d],"
    "\"servoLeft\":%d,\"servoRight\":%d,"
    "\"lineMin\":[%u,%u,%u,%u,%u],\"lineMax\":[%u,%u,%u,%u,%u],"
    "\"gyroOffset\":%.2f,\"zBaseline\":%d,"
    "\"wifi\":\"%s\",\"heap\":%u",
    (unsigned)robot_state,
    (robot_state==STATE_IDLE)?"IDLE":(robot_state==STATE_STRAIGHT)?"STRAIGHT":
    (robot_state==STATE_LINE_FOLLOW)?"LINE":(robot_state==STATE_STOPPED)?"STOPPED":"MANUAL",
    (unsigned)distance_mm,
    (int)line_position_error,
    (double)angle_adjusted,
    on_slope?"true":"false",
    (int)throttle, (int)speed_M1, (int)speed_M2,
    (int)(control_output - steering), (int)(control_output + steering),
    (int)line_sensor_calibrated[0],(int)line_sensor_calibrated[1],(int)line_sensor_calibrated[2],
    (int)line_sensor_calibrated[3],(int)line_sensor_calibrated[4],
    servo_left_deg, servo_right_deg,
    line_min_values[0],line_min_values[1],line_min_values[2],line_min_values[3],line_min_values[4],
    line_max_values[0],line_max_values[1],line_max_values[2],line_max_values[3],line_max_values[4],
    (double)x_gyro_offset, (int)z_accel_baseline,
    (wifi_mode==WIFI_STA_OK)?"sta":(wifi_mode==WIFI_AP_MODE)?"ap":(wifi_mode==WIFI_CONNECTING)?"connecting":"off",
    (unsigned)ESP.getFreeHeap());
  return String(buf).substring(0, n);
}

void registerApiRoutes() {
  webServer.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
    req->send_P(200, "text/html", webUI);
  });
  webServer.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *req) {
    req->send(200, "application/json", buildStatusJson());
  });
  webServer.on("/api/command", HTTP_GET, [](AsyncWebServerRequest *req) {
    String cmd;
    if (req->hasParam("cmd")) cmd = req->getParam("cmd")->value();
    cmd.toLowerCase();
    if (cmd == "straight") {
      robot_state = STATE_STRAIGHT; state_start_time = millis();
    } else if (cmd == "line") {
      LineFollower_setMode(LINE_MODE_ON); robot_state = STATE_LINE_FOLLOW;
    } else if (cmd == "stop" || cmd == "idle") {
      robot_state = STATE_IDLE; state_start_time = millis();
      throttle = steering = 0;
    } else if (cmd == "manual") {
      robot_state = STATE_MANUAL;
    } else if (cmd == "gripper_open")  { gripperOpen(); }
    else if (cmd == "gripper_close") { gripperClose(); }
    else if (cmd == "gripper_reset") { gripperReset(); }
    else {
      req->send(400, "application/json", "{\"error\":\"unknown cmd\"}");
      return;
    }
    req->send(200, "application/json", buildStatusJson());
  });
  webServer.on("/api/joystick", HTTP_GET, [](AsyncWebServerRequest *req) {
    if (req->hasParam("throttle")) api_manual_throttle_val = constrain(req->getParam("throttle")->value().toInt(), -200, 200);
    if (req->hasParam("steering")) api_manual_steering_val = constrain(req->getParam("steering")->value().toInt(), -100, 100);
    api_manual_throttle = true;
    if (robot_state != STATE_MANUAL) robot_state = STATE_MANUAL;
    req->send(200, "application/json", buildStatusJson());
  });
  // One-shot config: save WiFi creds then reboot so they take effect
  webServer.on("/api/save-wifi", HTTP_GET, [](AsyncWebServerRequest *req) {
    if (!req->hasParam("ssid") || !req->hasParam("pass")) {
      req->send(400, "text/plain", "missing ssid or pass");
      return;
    }
    wifiSaveCreds(req->getParam("ssid")->value(), req->getParam("pass")->value());
    req->send(200, "text/plain", "saved, rebooting");
    delay(500);
    ESP.restart();
  });
  webServer.on("/api/reset-wifi", HTTP_GET, [](AsyncWebServerRequest *req) {
    Preferences prefs;
    prefs.begin(WIFI_NAMESPACE, false);
    prefs.clear();
    prefs.end();
    req->send(200, "text/plain", "cleared, rebooting");
    delay(500);
    ESP.restart();
  });
  // Calibration API
  webServer.on("/api/calib", HTTP_GET, [](AsyncWebServerRequest *req) {
    req->send(200, "application/json", buildCalibJson());
  });
  webServer.on("/api/calib", HTTP_POST, [](AsyncWebServerRequest *req) {
    // Expect JSON body: {"line_min":[...],"line_max":[...]}
    if (!req->hasParam("plain", true)) {
      req->send(400, "application/json", "{\"error\":\"missing JSON body\"}");
      return;
    }
    String body = req->getParam("plain", true)->value();
    // Parse JSON (very simple: look for line_min and line_max arrays)
    // Format: {"line_min":[...],"line_max":[...]}
    int min_start = body.indexOf("\"line_min\":[");
    int max_start = body.indexOf("\"line_max\":[");
    if (min_start == -1 || max_start == -1) {
      req->send(400, "application/json", "{\"error\":\"missing line_min or line_max\"}");
      return;
    }
    min_start += 11; // length of "\"line_min\":["
    max_start += 11;
    int min_end = body.indexOf(']', min_start);
    int max_end = body.indexOf(']', max_start);
    if (min_end == -1 || max_end == -1) {
      req->send(400, "application/json", "{\"error\":\"malformed arrays\"}");
      return;
    }
    String min_str = body.substring(min_start, min_end);
    String max_str = body.substring(max_start, max_end);
    // Parse comma-separated ints
    int idx = 0;
    for (int p = 0, last = 0; idx < 5; ++idx) {
      int comma = min_str.indexOf(',', last);
      if (comma == -1) comma = min_str.length();
      int val = min_str.substring(last, comma).toInt();
      if (val >= 0 && val <= 4095) line_min_values[idx] = val;
      last = comma + 1;
    }
    for (int p = 0, last = 0; idx < 5; ++idx) {
      int comma = max_str.indexOf(',', last);
      if (comma == -1) comma = max_str.length();
      int val = max_str.substring(last, comma).toInt();
      if (val >= 0 && val <= 4095) line_max_values[idx] = val;
      last = comma + 1;
    }
    saveCalibration();
    req->send(200, "application/json", buildCalibJson());
  });
  webServer.on("/api/calib/reset", HTTP_GET, [](AsyncWebServerRequest *req) {
    resetCalibration();
    req->send(200, "application/json", buildCalibJson());
  });
  // Auto-calibration endpoint (blocking, only in IDLE)
  webServer.on("/api/calib/line/run", HTTP_GET, [](AsyncWebServerRequest *req) {
    if (!req->hasParam("ms")) {
      req->send(400, "application/json", "{\"error\":\"missing ms parameter\"}");
      return;
    }
    uint16_t ms = req->getParam("ms")->value().toInt();
    if (ms < 1000) ms = 1000;
    if (ms > 30000) ms = 30000;
    String result = runLineCalibrationBlocking(ms);
    req->send(200, "application/json", result);
  });
  // PID API
  webServer.on("/api/pid", HTTP_GET, [](AsyncWebServerRequest *req) {
    req->send(200, "application/json", buildPidJson());
  });
  webServer.on("/api/pid", HTTP_POST, [](AsyncWebServerRequest *req) {
    if (!req->hasParam("plain", true)) {
      req->send(400, "application/json", "{\"error\":\"missing JSON body\"}");
      return;
    }
    String body = req->getParam("plain", true)->value();
    auto extract = [&](String key, float &var, float def) {
      int idx = body.indexOf("\"" + key + "\":");
      if (idx != -1) {
        idx += key.length() + 3;
        int end = body.indexOf(',', idx);
        if (end == -1) end = body.indexOf('}', idx);
        float val = body.substring(idx, end).toFloat();
        var = val;
      }
    };
    extract("Kp", Kp, KP_DEFAULT);
    extract("Kd", Kd, KD_DEFAULT);
    extract("Kp_thr", Kp_thr, KP_THROTTLE_DEFAULT);
    extract("Ki_thr", Ki_thr, KI_THROTTLE_DEFAULT);
    savePID();
    req->send(200, "application/json", buildPidJson());
  });
  webServer.on("/api/pid/reset", HTTP_GET, [](AsyncWebServerRequest *req) {
    resetPID();
    req->send(200, "application/json", buildPidJson());
  });
  // WebSocket
  ws.onEvent(onWsEvent);
  webServer.addHandler(&ws);
  webServer.begin();
}

void initWifiAndServer() {
  Serial.println("[WIFI] Initialising...");
  WiFi.mode(WIFI_STA);

  // 1) Use compile-time defaults from secrets.h if present
  String ssid, pass;
#if defined(WIFI_SSID) && defined(WIFI_PASSWORD)
  ssid = WIFI_SSID;
  pass = WIFI_PASSWORD;
  Serial.print("[WIFI] Using compile-time SSID: ");
  Serial.println(ssid);
#else
  // 2) Otherwise load from NVS (set via /api/save-wifi or first-boot AP)
  if (wifiLoadCreds(ssid, pass)) {
    Serial.print("[WIFI] Using stored SSID: ");
    Serial.println(ssid);
  } else {
    Serial.println("[WIFI] No creds, will start AP");
  }
#endif

  if (ssid.length() > 0) {
    WiFi.begin(ssid.c_str(), pass.c_str());
    wifi_mode = WIFI_CONNECTING;
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && (unsigned long)(millis() - t0) < WIFI_CONNECT_TIMEOUT_MS) {
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (WiFi.status() == WL_CONNECTED) {
      wifi_mode = WIFI_STA_OK;
      Serial.print("[WIFI] Connected, IP: ");
      Serial.println(WiFi.localIP());
    } else {
      Serial.println("[WIFI] STA connect failed");
    }
  }

#if WIFI_AP_FALLBACK
  if (wifi_mode != WIFI_STA_OK) {
    Serial.println("[WIFI] Starting AP for first-time config");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
    wifi_mode = WIFI_AP_MODE;
    Serial.print("[WIFI] AP SSID: ");
    Serial.print(WIFI_AP_SSID);
    Serial.print("  IP: ");
    Serial.println(WiFi.softAPIP());
  }
#endif

  registerApiRoutes();
}
#endif // ENABLE_WIFI

// ============================================================================
// SETUP & LOOP
// ============================================================================

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ESP32 Self-Balancing Robot v3.4.0 ===");

  // Motor pins
  pinMode(PIN_ENABLE_MOTORS, OUTPUT);
  digitalWrite(PIN_ENABLE_MOTORS, LOW);  // TMC2209 EN active-LOW
  pinMode(PIN_MOTOR1_DIR, OUTPUT);
  pinMode(PIN_MOTOR1_STEP, OUTPUT);
  pinMode(PIN_MOTOR2_DIR, OUTPUT);
  pinMode(PIN_MOTOR2_STEP, OUTPUT);
  pinMode(PIN_WIFI_LED, OUTPUT);
  digitalWrite(PIN_WIFI_LED, LOW);

  // I2C bus — fail-fast on hung bus
  Wire.begin(21, 22);
  Wire.setClock(I2C_CLOCK_HZ);
  Wire.setTimeout(WIRE_TIMEOUT_MS);  // ms; 0 disables timeout

  // Init sensors (MPU is critical: bail out if it never responds)
  if (!initMPU6050()) {
    Serial.println("[FATAL] MPU6050 failed!");
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
  }
  initDistanceSensor();
  LineFollower_init();
  loadCalibration();
  initTimers();
  loadPID();

  // Initialize timing globals (otherwise first dt = micros()/1e6 = huge)
  timer_value = micros();
  timer_old = timer_value;
  last_steps1 = steps1;
  last_steps2 = steps2;

  robot_state = STATE_IDLE;
  throttle = 0;
  steering = 0;
  obstacle_clear_since_ms = 0;
  Serial.println("[OK] Ready!");

  // Capture slope baseline NOW, while the chassis is still.
  // (Doing it after the motor test would include vibration noise.)
  MPU6050_read_3axis();
  if (mpu_last_error == 0) {
    z_accel_baseline = accel_t_gyro.value.z_accel;
    Serial.print("[SLOPE] Z-axis baseline captured: ");
    Serial.println(z_accel_baseline);
    saveCalibration();  // persist z_baseline + gyro_offset
  } else {
    Serial.println("[SLOPE] WARN: MPU read failed at boot, baseline left at 0 (slope detection disabled)");
  }

  // Optional setup-time motor test (skipped by default for safety on a freestanding robot)
#if SKIP_SETUP_MOTOR_TEST
  Serial.println("[TEST] Skipping 1s motor spin (SKIP_SETUP_MOTOR_TEST=1)");
#else
  Serial.println("[TEST] Motor spin for 1 second...");
  setMotorSpeedM1(100);
  setMotorSpeedM2(100);
  delay(1000);
  setMotorSpeedM1(0);
  setMotorSpeedM2(0);
  Serial.println("[TEST] Motor test complete");
#endif

#if LINE_MODE_ENABLED
  LineFollower_setMode(LINE_MODE_ON);
  Serial.println("[LineFollower] Mode: ON");
#else
  LineFollower_setMode(LINE_MODE_OFF);
  Serial.println("[LineFollower] Mode: OFF (LINE_MODE_ENABLED=0)");
#endif

#if ENABLE_WIFI
  initWifiAndServer();
#endif

  Serial.println("[BALANCE] Balancing for 5 seconds...");
  state_start_time = millis();
}

void loop() {
  static unsigned long last_control_us = 0;
  unsigned long now = micros();
  if ((unsigned long)(now - last_control_us) >= 10000) {
    last_control_us = now;
    timer_value = now;
    // Guard against dt spikes on first cycle or after long stalls
    float raw_dt = (timer_value - timer_old) / 1000000.0f;
    dt = (raw_dt > 0.05f) ? 0.01f : ((raw_dt < 0.001f) ? 0.01f : raw_dt);
    timer_old = timer_value;
    readSensors();
    controlLoop();
    cascade_control_loop_counter++;
    if (cascade_control_loop_counter >= 10) {
      cascade_control_loop_counter = 0;
      loop_counter++;
    }
  }
#if ENABLE_WIFI
  wifiLedTick();
  // WebSocket periodic status broadcast
  unsigned long now_ms = millis();
  if (now_ms - ws_last_status_ms >= WS_STATUS_INTERVAL_MS) {
    ws_last_status_ms = now_ms;
    broadcastStatus();
  }
  ws.cleanupClients();
#endif
}

// Author: Anton Petnitsky
// GitHub: https://github.com/Mukller/Balance_robot
// Last modified: 2026-08-23 18:00:00 +0300
