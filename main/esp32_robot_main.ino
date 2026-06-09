// ESP32 Self-Balancing Robot - ОСНОВНОЙ СКЕТЧ
// Робот: 2 сек прямо → детекция уклона → стоп при препятствии

#include <Wire.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "Control.h"
#include "MPU6050.h"
#include "Motors.h"
#include "defines.h"
#include "globals.h"
#include "LineFollower.h"
#include "web_interface.h"

// WiFi параметры - ОТРЕДАКТИРУЙ!
String sta_ssid = "YourWiFiSSID";
String sta_password = "YourWiFiPassword";

// Ультразвуковой датчик HY-SRF05
#define SENSOR_TRIG 4   // Триггер - GPIO 4
#define SENSOR_ECHO 5   // Эхо - GPIO 5
uint16_t distance_mm = 0;
unsigned long pulse_duration = 0;

// Состояния робота
#define STATE_IDLE 0
#define STATE_STRAIGHT 1
#define STATE_LINE_FOLLOW 2
#define STATE_STOPPED 3

volatile uint8_t robot_state = STATE_IDLE;
unsigned long state_start_time = 0;

// Склон
volatile bool on_slope = false;
int16_t z_accel_baseline = 0;
#define SLOPE_ACCEL_THRESHOLD 2000
#define DISTANCE_THRESHOLD 300
#define STRAIGHT_TIME 2000

AsyncWebServer server(80);

// ============================================================================
// ИНИЦИАЛИЗАЦИЯ
// ============================================================================

bool initMPU6050() {
  Serial.println("[MPU6050] Initializing...");
  MPU6050_setup();
  vTaskDelay(pdMS_TO_TICKS(500));
  MPU6050_calibrate();
  vTaskDelay(pdMS_TO_TICKS(500));

  int32_t z_sum = 0;
  for (int i = 0; i < 50; i++) {
    MPU6050_read_3axis();
    z_sum += accel_t_gyro.value.z_accel;
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  z_accel_baseline = z_sum / 50;
  Serial.print("[MPU6050] Z-baseline: ");
  Serial.println(z_accel_baseline);
  return true;
}

void initDistanceSensor() {
  Serial.println("[HY-SRF05] Initializing...");
  pinMode(SENSOR_TRIG, OUTPUT);
  pinMode(SENSOR_ECHO, INPUT);
  digitalWrite(SENSOR_TRIG, LOW);
  vTaskDelay(pdMS_TO_TICKS(100));
  Serial.println("[OK] HY-SRF05 initialized");
}

// Функция чтения расстояния (в мм)
uint16_t readDistanceSensor() {
  // Отправить импульс 10 микросекунд на TRIG
  digitalWrite(SENSOR_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(SENSOR_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(SENSOR_TRIG, LOW);

  // Ждём импульса на ECHO (макс 30мс для ~5 метров)
  pulse_duration = pulseIn(SENSOR_ECHO, HIGH, 30000);

  // Расстояние = время / 58 (микросекунды в см)
  // или время / 5.8 для мм
  if (pulse_duration > 0) {
    uint16_t dist_mm = (pulse_duration / 5.8);
    return constrain(dist_mm, 0, 4000);  // Макс ~4 метра
  }
  return 0;
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ESP32 Self-Balancing Robot (Main) ===");

  // GPIO инициализация
  pinMode(PIN_ENABLE_MOTORS, OUTPUT);
  digitalWrite(PIN_ENABLE_MOTORS, HIGH);
  pinMode(PIN_MOTOR1_DIR, OUTPUT);
  pinMode(PIN_MOTOR1_STEP, OUTPUT);
  pinMode(PIN_MOTOR2_DIR, OUTPUT);
  pinMode(PIN_MOTOR2_STEP, OUTPUT);

  // I2C
  Wire.begin(21, 22);
  vTaskDelay(pdMS_TO_TICKS(100));

  // Датчики
  if (!initMPU6050()) {
    Serial.println("[FATAL] MPU6050 init failed!");
    while(1) vTaskDelay(pdMS_TO_TICKS(1000));
  }

  initDistanceSensor();
  LineFollower_init();
  initTimers();

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(sta_ssid.c_str(), sta_password.c_str());
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    vTaskDelay(pdMS_TO_TICKS(500));
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("[WiFi] IP: ");
    Serial.println(WiFi.localIP());
  }

  // Web обработчики
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", web_interface);
  });

  server.on("/api/joystick", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasArg("throttle")) {
      throttle = request->arg("throttle").toInt();
    }
    if (request->hasArg("steering")) {
      steering = request->arg("steering").toInt();
    }
    sendJsonStatus(request);
  });

  server.on("/api/command", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasArg("cmd")) {
      String cmd = request->arg("cmd");
      if (cmd == "straight") {
        robot_state = STATE_STRAIGHT;
        state_start_time = millis();
        Serial.println("[CMD] STRAIGHT");
      } else if (cmd == "line") {
        robot_state = STATE_LINE_FOLLOW;
        Serial.println("[CMD] LINE_FOLLOW");
      } else if (cmd == "stop") {
        robot_state = STATE_IDLE;
        throttle = 0;
        steering = 0;
        Serial.println("[CMD] STOP");
      }
    }
    sendJsonStatus(request);
  });

  server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    sendJsonStatus(request);
  });

  server.begin();
  Serial.println("[Web] Server started at http://" + WiFi.localIP().toString());
  Serial.println("[OK] Ready!");
}

void sendJsonStatus(AsyncWebServerRequest *request) {
  char response[512];
  snprintf(response, sizeof(response),
    "{\"state\":%d,\"distance\":%d,\"slope\":%d,\"angle\":%.1f,\"throttle\":%d,\"sensors\":[%d,%d,%d,%d,%d],\"lineError\":%d}",
    robot_state, distance_mm, (int)on_slope, angle_adjusted, throttle,
    line_sensor_calibrated[0], line_sensor_calibrated[1], line_sensor_calibrated[2],
    line_sensor_calibrated[3], line_sensor_calibrated[4],
    line_position_error);

  request->send(200, "application/json", response);
}

// ============================================================================
// ДЕТЕКЦИЯ СКЛОНА
// ============================================================================

bool detectSlope() {
  int32_t current_z = accel_t_gyro.value.z_accel;
  int32_t z_delta = (current_z > z_accel_baseline) ?
                    (current_z - z_accel_baseline) :
                    (z_accel_baseline - current_z);

  return (z_delta > SLOPE_ACCEL_THRESHOLD);
}

int16_t applySlopeDamping(int16_t speed) {
  if (on_slope) {
    return (speed * 0.5);  // Замедление в 2 раза
  }
  return speed;
}

// ============================================================================
// КОНТРОЛЬ (100Hz)
// ============================================================================

void controlLoop() {
  float dt = (timer_value - timer_old) / 1000000.0;

  // Читать дистанцию с HY-SRF05
  distance_mm = readDistanceSensor();

  // Препятствие → СТОП
  if (distance_mm > 0 && distance_mm < DISTANCE_THRESHOLD) {
    robot_state = STATE_STOPPED;
    throttle = 0;
    steering = 0;
    setMotorSpeedM1(0);
    setMotorSpeedM2(0);
    if (loop_counter % 100 == 0) {
      Serial.print("[OBSTACLE] ");
      Serial.print(distance_mm);
      Serial.println(" mm");
    }
    return;
  }

  // Детекция склона
  on_slope = detectSlope();

  // PID баланс
  float balance_output = stabilityPDControl(dt, angle_adjusted, target_angle, Kp, Kd);
  float speed_output = speedPIControl(dt, actual_robot_speed, throttle, Kp_thr, Ki_thr);
  control_output = balance_output + speed_output;

  // State machine
  switch (robot_state) {
    case STATE_IDLE:
      throttle = 0;
      steering = 0;
      break;

    case STATE_STRAIGHT:
      throttle = 150;
      steering = 0;
      if ((unsigned long)(millis() - state_start_time) > STRAIGHT_TIME) {
        robot_state = STATE_LINE_FOLLOW;
        Serial.println("[AUTO] STRAIGHT → LINE_FOLLOW");
      }
      break;

    case STATE_LINE_FOLLOW:
      throttle = 200;
      if (line_mode != LINE_MODE_OFF) {
        int16_t line_steering = LineFollower_getSteering();
        steering = (line_steering * 0.4);
      } else {
        steering = 0;
      }
      break;

    case STATE_STOPPED:
      throttle = 0;
      steering = 0;
      break;
  }

  // Дамп на склоне
  if (on_slope) {
    throttle = applySlopeDamping(throttle);
    steering = applySlopeDamping(steering);
    control_output = applySlopeDamping(control_output);
  }

  // Ограничения
  control_output = constrain(control_output, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);
  steering = constrain(steering, -max_steering, max_steering);
  throttle = constrain(throttle, -max_throttle, max_throttle);

  // Отправить на моторы
  setMotorSpeedM1(control_output - steering);
  setMotorSpeedM2(control_output + steering);

  // Debug
  if (loop_counter % 100 == 0) {
    Serial.print("[Status] State:");
    Serial.print(robot_state);
    Serial.print(" Dist:");
    Serial.print(distance_mm);
    Serial.print("mm Slope:");
    Serial.print(on_slope);
    Serial.print(" Angle:");
    Serial.println(angle_adjusted, 1);
  }
}

void readSensors() {
  MPU6050_read_3axis();
  angle_adjusted = MPU6050_getAngle(dt);
  if (line_mode != LINE_MODE_OFF) {
    LineFollower_readSensors();
  }
}

void IRAM_ATTR onTimer1() {
  if (speed_M1 != 0) {
    digitalWrite(PIN_MOTOR1_STEP, HIGH);
    delayMicroseconds(50);
    digitalWrite(PIN_MOTOR1_STEP, LOW);
  }
}

void IRAM_ATTR onTimer2() {
  if (speed_M2 != 0) {
    digitalWrite(PIN_MOTOR2_STEP, HIGH);
    delayMicroseconds(50);
    digitalWrite(PIN_MOTOR2_STEP, LOW);
  }
}

void IRAM_ATTR onControlTimer() {
  timer_value = timerRead(timer0);
  readSensors();
  controlLoop();
  dt = (timer_value - timer_old) / 1000000.0;
  timer_old = timer_value;

  cascade_control_loop_counter++;
  if (cascade_control_loop_counter >= 10) {
    cascade_control_loop_counter = 0;
    loop_counter++;
  }
}

void initTimers() {
  timer1 = timerBegin(0, 40, true);
  timerAttachInterrupt(timer1, &onTimer1, true);
  timerAlarmWrite(timer1, 500000, true);
  timerAlarmEnable(timer1);

  timer2 = timerBegin(1, 40, true);
  timerAttachInterrupt(timer2, &onTimer2, true);
  timerAlarmWrite(timer2, 500000, true);
  timerAlarmEnable(timer2);

  timer0 = timerBegin(2, 80, true);
  timerAttachInterrupt(timer0, &onControlTimer, true);
  timerAlarmWrite(timer0, 10000, true);
  timerAlarmEnable(timer0);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(10));
}
