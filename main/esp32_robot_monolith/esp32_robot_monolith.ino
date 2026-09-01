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

#define MAX_CONTROL_OUTPUT 500
#define ITERM_MAX_ERROR 30
#define ITERM_MAX 10000
#define ANGLE_OFFSET 0.0

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
float Kp = KP, Kd = KD, Kp_thr = KP_THROTTLE, Ki_thr = KI_THROTTLE;
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
bool api_manual_throttle = false;  // joystick override flag
unsigned long wifi_led_toggle_ms = 0;
bool wifi_led_state = false;
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
<title>Robot Control</title>
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
}
header h1 { font-size: 16px; font-weight: 600; color: #e6edf3; }
main { padding: 24px; max-width: 960px; margin: 0 auto; }
h2 { font-size: 11px; color: #8b949e; text-transform: uppercase; letter-spacing: 1px; font-weight: 600; margin: 0 0 12px; }
.grid { display: grid; grid-template-columns: 1fr 1fr; gap: 16px; }
@media (max-width: 640px) { .grid { grid-template-columns: 1fr; } }
.card { background: #161b22; border: 1px solid #21262d; border-radius: 8px; padding: 18px; }
.tag { display: inline-block; padding: 2px 8px; border-radius: 10px; font-size: 11px; background: #21262d; color: #8b949e; }
.tag.green  { background: rgba(63,185,80,.15);  color: #3fb950; }
.tag.red    { background: rgba(248,81,73,.15);  color: #f85149; }
.tag.blue   { background: rgba(88,166,255,.15); color: #58a6ff; }
.joystick-wrap { display: flex; justify-content: center; margin-bottom: 18px; }
.joystick-container { width: 200px; height: 200px; background: #0d1117; border: 1px solid #21262d; border-radius: 50%; position: relative; touch-action: none; }
.joystick-ring { position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%); width: 40px; height: 40px; border: 1px solid #21262d; border-radius: 50%; pointer-events: none; }
.joystick-dot { width: 44px; height: 44px; background: #58a6ff; border-radius: 50%; position: absolute; top: 78px; left: 78px; cursor: grab; box-shadow: 0 0 12px rgba(88,166,255,.4); }
.joystick-dot:active { cursor: grabbing; box-shadow: 0 0 20px rgba(88,166,255,.6); }
.btn-row { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; margin-bottom: 14px; }
button { background: #238636; color: #fff; border: none; padding: 7px 13px; border-radius: 6px; cursor: pointer; font-size: 13px; font-family: inherit; font-weight: 500; transition: background .15s; }
button:hover { background: #2ea043; }
button.secondary { background: #21262d; color: #c9d1d9; }
button.secondary:hover { background: #30363d; }
button.danger { background: #da3633; }
button.danger:hover { background: #f85149; }
.status-row { display: flex; align-items: center; justify-content: space-between; padding: 7px 0; border-top: 1px solid #21262d; margin-top: 4px; }
.status-label { font-size: 12px; color: #8b949e; }
.kv { display: flex; justify-content: space-between; padding: 5px 0; border-bottom: 1px solid #21262d; font-size: 13px; }
.kv:last-child { border: none; }
.kv-k { color: #8b949e; }
.kv-v { font-family: monospace; color: #c9d1d9; }
.bar { height: 4px; background: #21262d; border-radius: 2px; overflow: hidden; margin: 4px 0 10px; }
.bar-fill { height: 100%; background: #58a6ff; transition: width .2s; border-radius: 2px; }
.line-bars { display: flex; gap: 6px; justify-content: center; margin: 8px 0 10px; }
.s-bar { width: 24px; height: 72px; background: #0d1117; border: 1px solid #21262d; border-radius: 3px; position: relative; overflow: hidden; }
.s-fill { width: 100%; position: absolute; bottom: 0; background: #58a6ff; transition: height .15s; border-radius: 2px; }
.section-gap { margin-top: 18px; }
</style>
</head>
<body>
<header>
  <h1>🤖 Robot Control</h1>
  <div style="display:flex;gap:6px;align-items:center">
    <span class="tag blue">ESP32</span>
    <span class="tag" id="statusTag">IDLE</span>
  </div>
</header>
<main>
  <div class="grid">
    <div class="card">
      <h2>Управление</h2>
      <div class="joystick-wrap">
        <div class="joystick-container">
          <div class="joystick-ring"></div>
          <div class="joystick-dot" id="joystick"></div>
        </div>
      </div>
      <div class="btn-row">
        <button onclick="startStraight()">▶ Прямо</button>
        <button class="secondary" onclick="startLine()">〰 Линия</button>
        <button class="danger" onclick="stopRobot()">⏹ Стоп</button>
        <button class="secondary" onclick="getStatus()">↻ Статус</button>
      </div>
      <div class="status-row">
        <span class="status-label">Режим</span>
        <span class="tag" id="statusDisplay">IDLE</span>
      </div>
    </div>
    <div class="card">
      <h2>Датчики</h2>
      <div class="kv"><span class="kv-k">Дистанция (мм)</span><span class="kv-v" id="distance">--</span></div>
      <div class="kv"><span class="kv-k">На линии</span><span class="kv-v" id="onLine">НЕТ</span></div>
      <div class="kv"><span class="kv-k">Ошибка линии</span><span class="kv-v" id="lineError">0</span></div>
      <div class="section-gap">
        <h2>Датчик линии</h2>
        <div class="line-bars" id="lineSensors">
          <div class="s-bar"><div class="s-fill" id="sensor0" style="height:0%"></div></div>
          <div class="s-bar"><div class="s-fill" id="sensor1" style="height:0%"></div></div>
          <div class="s-bar"><div class="s-fill" id="sensor2" style="height:0%"></div></div>
          <div class="s-bar"><div class="s-fill" id="sensor3" style="height:0%"></div></div>
          <div class="s-bar"><div class="s-fill" id="sensor4" style="height:0%"></div></div>
        </div>
      </div>
      <div class="section-gap">
        <h2>Параметры</h2>
        <div class="kv"><span class="kv-k">Угол</span><span class="kv-v" id="angle">0.0°</span></div>
        <div class="kv"><span class="kv-k">На уклоне</span><span class="kv-v" id="onSlope">НЕТ</span></div>
        <div class="kv"><span class="kv-k">Throttle</span><span class="kv-v" id="throttle">0</span></div>
      </div>
    </div>
  </div>
</main>
<script>
  const joystick = document.getElementById('joystick');
  const container = joystick.parentElement;
  let isPressed = false;
  function handleJoystickMove(e) {
    if (!isPressed) return;
    const rect = container.getBoundingClientRect();
    const cx = rect.width / 2, cy = rect.height / 2;
    const radius = cy - 22;
    let x = (e.touches ? e.touches[0].clientX : e.clientX) - rect.left - cx;
    let y = (e.touches ? e.touches[0].clientY : e.clientY) - rect.top - cy;
    const d = Math.sqrt(x*x + y*y);
    if (d > radius) { const a = Math.atan2(y, x); x = Math.cos(a)*radius; y = Math.sin(a)*radius; }
    joystick.style.left = (cx + x - 22) + 'px';
    joystick.style.top = (cy + y - 22) + 'px';
    sendJoystickCommand(Math.round(-y/radius*200), Math.round(x/radius*100));
  }
  joystick.addEventListener('mousedown', () => isPressed = true);
  joystick.addEventListener('touchstart', () => isPressed = true);
  document.addEventListener('mouseup', resetJoystick);
  document.addEventListener('touchend', resetJoystick);
  document.addEventListener('mousemove', handleJoystickMove);
  document.addEventListener('touchmove', handleJoystickMove);
  function resetJoystick() {
    isPressed = false;
    joystick.style.left = '78px';
    joystick.style.top = '78px';
    sendJoystickCommand(0, 0);
  }
  async function sendJoystickCommand(throttle, steering) {
    try { updateDisplay(await (await fetch(`/api/joystick?throttle=${throttle}&steering=${steering}`)).json()); }
    catch {}
  }
  async function startStraight() { try { updateDisplay(await (await fetch('/api/command?cmd=straight')).json()); } catch {} }
  async function startLine() { try { updateDisplay(await (await fetch('/api/command?cmd=line')).json()); } catch {} }
  async function stopRobot() { try { updateDisplay(await (await fetch('/api/command?cmd=stop')).json()); } catch {} }
  async function getStatus() { try { updateDisplay(await (await fetch('/api/status')).json()); } catch {} }
  function updateDisplay(data) {
    if (!data) return;
    const states = ['IDLE', 'STRAIGHT', 'LINE', 'STOPPED'];
    const classes = ['', 'green', 'blue', 'red'];
    const label = states[data.state] || 'UNKNOWN';
    const cls = classes[data.state] || '';
    ['statusDisplay', 'statusTag'].forEach(id => {
      const el = document.getElementById(id);
      el.textContent = label;
      el.className = 'tag' + (cls ? ' ' + cls : '');
    });
    document.getElementById('distance').textContent = data.distance ?? '--';
    document.getElementById('lineError').textContent = data.lineError ?? '0';
    document.getElementById('onLine').textContent = (data.lineError !== null && Math.abs(data.lineError) < 100) ? 'ДА' : 'НЕТ';
    document.getElementById('angle').textContent = (data.angle || 0).toFixed(1) + '°';
    document.getElementById('onSlope').textContent = data.slope ? 'ДА' : 'НЕТ';
    document.getElementById('throttle').textContent = data.throttle ?? '0';
    if (data.sensors) {
      for (let i = 0; i < 5; i++) {
        const f = document.getElementById('sensor' + i);
        if (f) f.style.height = ((data.sensors[i] || 0) / 200 * 100) + '%';
      }
    }
  }
  setInterval(getStatus, 1000);
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

// Build a JSON status blob (no ArduinoJson lib needed, hand-rolled)
String buildStatusJson() {
  char buf[640];
  int n = snprintf(buf, sizeof(buf),
    "{\"state\":%u,\"stateName\":\"%s\",\"distance\":%u,\"lineError\":%d,"
    "\"angle\":%.2f,\"onSlope\":%s,\"throttle\":%d,\"speedM1\":%d,\"speedM2\":%d,"
    "\"m1\":%d,\"m2\":%d,\"lineSensors\":[%d,%d,%d,%d,%d],"
    "\"servoLeft\":%d,\"servoRight\":%d,"
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
  initTimers();

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
#endif
}

// Author: Anton Petnitsky
// GitHub: https://github.com/Mukller/Balance_robot
// Last modified: 2026-08-23 18:00:00 +0300
