#include <Wire.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <Arduino.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// Наши модули
#include "Control.h"
#include "MPU6050.h"
#include "Motors.h"
#include "defines.h"
#include "globals.h"
#include "LineFollower.h"

// VL53L0X дистанционный датчик
#include <VL53L0X.h>

#include <stdio.h>
#include "esp_types.h"
#include "soc/timer_group_struct.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "driver/ledc.h"
#include "esp32-hal-ledc.h"

// ============================================================================
// ПАРАМЕТРЫ
// ============================================================================

const char* PARAM_FADER1 = "fader1";
const char* PARAM_FADER2 = "fader2";
const char* PARAM_PUSH1 = "push1";
const char* PARAM_PUSH2 = "push2";
const char* PARAM_PUSH3 = "push3";
const char* PARAM_PUSH4 = "push4";
const char* PARAM_TOGGLE1 = "toggle1";
const char* PARAM_LINE_MODE = "line_mode";
const char* PARAM_DISTANCE = "distance";

// WiFi параметры - ОТРЕДАКТИРУЙ ЭТО!
String sta_ssid = "YourWiFiSSID";           // Замени на свой WiFi
String sta_password = "YourWiFiPassword";   // Замени на свой пароль

// VL53L0X дистанционный датчик
VL53L0X distanceSensor;
uint16_t distance_mm = 0;
#define DISTANCE_THRESHOLD 300  // миллиметры - если меньше, робот останавливается

// Состояние робота
#define STATE_IDLE 0
#define STATE_STRAIGHT 1        // Ехать прямо 2 секунды
#define STATE_LINE_FOLLOW 2     // Следить за линией
#define STATE_STOPPED 3         // Остановлен (препятствие)

volatile uint8_t robot_state = STATE_IDLE;
unsigned long state_start_time = 0;
#define STRAIGHT_TIME 2000      // 2 секунды прямой езды

// Детекция склона/горки (по вертикальному ускорению Z-оси)
volatile bool on_slope = false;
#define SLOPE_ACCEL_THRESHOLD 2000  // Изменение Z-ускорения для детекции склона
int16_t z_accel_baseline = 0;       // Базовое значение Z на ровной поверхности
float slope_speed_factor = 0.5;     // Скорость уменьшается в 2 раза на склоне

// Остановка подсистем
volatile bool emergency_stop = false;

AsyncWebServer server(80);

unsigned long previousMillis = 0;

// ============================================================================
// ИНИЦИАЛИЗАЦИЯ
// ============================================================================

bool initMPU6050() {
  Serial.println("[MPU6050] Initializing...");
  MPU6050_setup();
  vTaskDelay(pdMS_TO_TICKS(500));

  MPU6050_calibrate();
  vTaskDelay(pdMS_TO_TICKS(500));

  // Получить базовое значение Z-ускорения (робот на ровной поверхности)
  int32_t z_sum = 0;
  for (int i = 0; i < 50; i++) {
    MPU6050_read_3axis();
    z_sum += accel_t_gyro.value.z_accel;
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  z_accel_baseline = z_sum / 50;
  Serial.print("[MPU6050] Z-accel baseline: ");
  Serial.println(z_accel_baseline);
  Serial.println("[MPU6050] OK");
  return true;
}

void initDistanceSensor() {
  Serial.println("[SENSOR] Initializing VL53L0X...");
  Wire.beginTransmission(0x29);  // VL53L0X I2C адрес
  if (Wire.endTransmission() != 0) {
    Serial.println("[ERROR] VL53L0X not found!");
    return;
  }

  distanceSensor.setTimeout(500);
  if (!distanceSensor.init()) {
    Serial.println("[ERROR] Failed to detect and initialize sensor!");
    return;
  }

  // Настроить для точности
  distanceSensor.setMeasurementTimingBudget(33000);
  Serial.println("[OK] VL53L0X initialized");
}

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

// ============================================================================
// ОСНОВНАЯ ИНИЦИАЛИЗАЦИЯ
// ============================================================================

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
  Serial.println("*ESP32 Self Balancing Robot with Line Following*");
  Serial.println("*Version 2.0 - with Distance Sensor & Flight Detection*");
  Serial.println("================================================================");

  // Инициализация пинов моторов
  pinMode(PIN_ENABLE_MOTORS, OUTPUT);
  digitalWrite(PIN_ENABLE_MOTORS, HIGH);

  pinMode(PIN_MOTOR1_DIR, OUTPUT);
  pinMode(PIN_MOTOR1_STEP, OUTPUT);
  pinMode(PIN_MOTOR2_DIR, OUTPUT);
  pinMode(PIN_MOTOR2_STEP, OUTPUT);
  pinMode(PIN_SERVO, OUTPUT);

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  pinMode(PIN_WIFI_LED, OUTPUT);
  digitalWrite(PIN_WIFI_LED, LOW);

  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);

  // Настроить сервомотор
  ledcSetup(6, 50, 16);
  ledcAttachPin(PIN_SERVO, 6);
  delay(50);

  // Инициализировать I2C
  Wire.begin(21, 22);  // SDA=21, SCL=22
  vTaskDelay(pdMS_TO_TICKS(100));

  // Инициализировать сенсоры
  if (!initMPU6050()) {
    Serial.println("[FATAL] MPU6050 init failed!");
    while(1) vTaskDelay(pdMS_TO_TICKS(1000));
  }

  initDistanceSensor();
  LineFollower_init();

  Serial.println("[OK] All sensors initialized!");

  // Инициализировать таймеры для моторов
  initTimers();

  // WiFi подключение
  WiFi.mode(WIFI_STA);
  WiFi.begin(sta_ssid.c_str(), sta_password.c_str());

  int wifi_attempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifi_attempts < 20) {
    delay(500);
    Serial.print(".");
    wifi_attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("[WiFi] Connected to: ");
    Serial.println(WiFi.SSID());
    Serial.print("[WiFi] IP address: ");
    Serial.println(WiFi.localIP());
    digitalWrite(PIN_WIFI_LED, HIGH);
  }

  // Установить Web обработчики
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Robot is running!");
  });

  server.on("/control", HTTP_GET, [](AsyncWebServerRequest *request) {
    // Обработать параметры управления
    if (request->hasArg(PARAM_PUSH1)) {
      robot_state = STATE_STRAIGHT;
      state_start_time = millis();
      Serial.println("[STATE] Starting straight line mode (2 sec)");
    }

    if (request->hasArg(PARAM_PUSH2)) {
      robot_state = STATE_LINE_FOLLOW;
      Serial.println("[STATE] Switching to line following mode");
    }

    if (request->hasArg(PARAM_PUSH3)) {
      robot_state = STATE_IDLE;
      throttle = 0;
      steering = 0;
      Serial.println("[STATE] Robot stopped");
    }

    if (request->hasArg(PARAM_LINE_MODE)) {
      uint8_t mode = request->arg(PARAM_LINE_MODE).toInt();
      LineFollower_setMode(mode);
    }

    // Отправить ответ с статусом (эффективная JSON генерация)
    char response[256];
    snprintf(response, sizeof(response),
      "{\"status\":\"ok\",\"state\":%d,\"distance\":%d,\"slope\":%d,\"throttle\":%d,\"angle\":%.1f}",
      robot_state, distance_mm, (int)on_slope, throttle, angle_adjusted);

    request->send(200, "application/json", response);
  });

  server.onNotFound(notFound);
  server.begin();
  Serial.println("[WiFi] Web server started on /control");
}

// ============================================================================
// ДЕТЕКЦИЯ СКЛОНА (ГОРКИ) - ПО ВЕРТИКАЛЬНОМУ УСКОРЕНИЮ
// ============================================================================

bool detectSlope() {
  // Вертикальное ускорение (Z-ось) показывает высоту
  // На ровной поверхности Z = baseline (примерно 16384 при 1G)
  // На склоне Z отличается от baseline

  int32_t current_z = accel_t_gyro.value.z_accel;
  int32_t z_delta = (current_z > z_accel_baseline) ?
                    (current_z - z_accel_baseline) :
                    (z_accel_baseline - current_z);

  // Если Z изменился больше чем на порог - робот на склоне
  if (z_delta > SLOPE_ACCEL_THRESHOLD) {
    return true;
  }

  return false;
}

// ============================================================================
// УПРАВЛЕНИЕ СКОРОСТЬЮ НА СКЛОНЕ
// ============================================================================

int16_t applySlopeDamping(int16_t speed) {
  if (on_slope) {
    return (speed * slope_speed_factor);  // Уменьшить в 2 раза на склоне
  }
  return speed;
}

// ============================================================================
// ОСНОВНОЙ КОНТРОЛЬ (100Hz)
// ============================================================================

void controlLoop() {
  float dt = (timer_value - timer_old) / 1000000.0;

  // Читать дистанцию от лазера
  if (distanceSensor.timeoutOccurred()) {
    distance_mm = 0;
  } else {
    distance_mm = distanceSensor.readRangeSingleMillimeters();
  }

  // Проверить препятствие - СТОП!
  if (distance_mm > 0 && distance_mm < DISTANCE_THRESHOLD) {
    robot_state = STATE_STOPPED;
    throttle = 0;
    steering = 0;
    control_output = 0;
    setMotorSpeedM1(0);
    setMotorSpeedM2(0);
    Serial.print("[OBSTACLE] Distance: ");
    Serial.print(distance_mm);
    Serial.println(" mm - STOPPING!");
    return;
  }

  // Детекция склона (горки)
  on_slope = detectSlope();

  // ===== БАЗОВЫЙ PID КОНТРОЛЬ БАЛАНСА =====
  float balance_output = stabilityPDControl(dt, angle_adjusted, target_angle, Kp, Kd);
  float speed_output = speedPIControl(dt, actual_robot_speed, throttle, Kp_thr, Ki_thr);

  control_output = balance_output + speed_output;

  // ===== КОНТРОЛЬ ПО СОСТОЯНИЯМ =====

  switch (robot_state) {

    case STATE_IDLE:
      // Ничего не делать - просто баланс
      throttle = 0;
      steering = 0;
      break;

    case STATE_STRAIGHT:
      // Ехать прямо 2 секунды
      throttle = 150;  // Умеренная скорость
      steering = 0;    // Прямо!

      // Проверить истекло ли время (safe от overflow)
      if ((unsigned long)(millis() - state_start_time) > STRAIGHT_TIME) {
        robot_state = STATE_LINE_FOLLOW;
        Serial.println("[STATE] Switching to line following");
      }
      break;

    case STATE_LINE_FOLLOW:
      // Следить за линией
      throttle = 200;  // Скорость для линии

      // Получить команду от датчика линии
      if (line_mode != LINE_MODE_OFF) {
        int16_t line_steering = LineFollower_getSteering();
        steering = (line_steering * 0.4);  // Влияние линии
      } else {
        steering = 0;
      }
      break;

    case STATE_STOPPED:
      // Остановлен
      throttle = 0;
      steering = 0;
      // Может потребоваться manual reset
      break;
  }

  // ===== ПРИМЕНИТЬ ДАМП НА СКЛОНЕ =====
  if (on_slope) {
    throttle = applySlopeDamping(throttle);
    steering = applySlopeDamping(steering);
    control_output = applySlopeDamping(control_output);

    // Опционально: сигнал на склоне
    if (loop_counter % 100 == 0) {
      tone(PIN_BUZZER, 800, 100);  // Писк на склоне
    }
  }

  // ===== ОГРАНИЧЕНИЯ =====
  control_output = constrain(control_output, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);
  steering = constrain(steering, -max_steering, max_steering);
  throttle = constrain(throttle, -max_throttle, max_throttle);

  // ===== ОТПРАВИТЬ НА МОТОРЫ =====
  setMotorSpeedM1(control_output - steering);
  setMotorSpeedM2(control_output + steering);
}

// ============================================================================
// ОБНОВЛЕНИЕ ДАТЧИКОВ (в контроллере)
// ============================================================================

void readSensors() {
  // Читать MPU6050
  MPU6050_read_3axis();
  angle_adjusted = MPU6050_getAngle(dt);

  // Читать линию (если нужна отладка)
  if (line_mode != LINE_MODE_OFF) {
    LineFollower_readSensors();
  }
}

// ============================================================================
// ОТЛАДКА (вывод в Serial)
// ============================================================================

void printDebugInfo() {
  // Выводить каждую секунду
  if (loop_counter % 100 == 0) {
    int16_t z_delta = abs(accel_t_gyro.value.z_accel - z_accel_baseline);

    Serial.print("[Status] State: ");
    Serial.print(robot_state);
    Serial.print(" | Z-accel: ");
    Serial.print(accel_t_gyro.value.z_accel);
    Serial.print(" (Δ");
    Serial.print(z_delta);
    Serial.print(") | OnSlope: ");
    Serial.print(on_slope);
    Serial.print(" | Distance: ");
    Serial.print(distance_mm);
    Serial.print(" mm | Throttle: ");
    Serial.println(throttle);
  }
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  delay(1);  // Дать другим задачам время

  // ArduinoOTA обновления
  ArduinoOTA.handle();
}

// ============================================================================
// INTERRUPT ОБРАБОТЧИКИ ТАЙМЕРОВ
// ============================================================================

void IRAM_ATTR onTimer1() {
  // Мотор 1 импульс
  if (speed_M1 != 0) {
    digitalWrite(PIN_MOTOR1_STEP, HIGH);
    delayMicroseconds(50);
    digitalWrite(PIN_MOTOR1_STEP, LOW);
  }
}

void IRAM_ATTR onTimer2() {
  // Мотор 2 импульс
  if (speed_M2 != 0) {
    digitalWrite(PIN_MOTOR2_STEP, HIGH);
    delayMicroseconds(50);
    digitalWrite(PIN_MOTOR2_STEP, LOW);
  }
}

// ===== КОНТРОЛЬНЫЙ ТАЙМЕР (100Hz) =====
void IRAM_ATTR onControlTimer() {
  timer_value = timerRead(timer0);

  // Читать датчики
  readSensors();

  // Выполнить контроль
  controlLoop();

  // Обновить дельта время
  dt = (timer_value - timer_old) / 1000000.0;
  timer_old = timer_value;

  // Счётчики для фильтрации
  cascade_control_loop_counter++;
  if (cascade_control_loop_counter >= 10) {
    cascade_control_loop_counter = 0;
    loop_counter++;
    slow_loop_counter++;
  }

  // Отладка
  if (slow_loop_counter >= 100) {
    slow_loop_counter = 0;
    printDebugInfo();
  }
}

// ============================================================================
// ИНИЦИАЛИЗАЦИЯ ТАЙМЕРОВ
// ============================================================================

void initTimers() {
  // Таймер для мотора 1 (2MHz)
  timer1 = timerBegin(0, 40, true);
  timerAttachInterrupt(timer1, &onTimer1, true);
  timerAlarmWrite(timer1, 500000, true);
  timerAlarmEnable(timer1);

  // Таймер для мотора 2 (2MHz)
  timer2 = timerBegin(1, 40, true);
  timerAttachInterrupt(timer2, &onTimer2, true);
  timerAlarmWrite(timer2, 500000, true);
  timerAlarmEnable(timer2);

  // Таймер для контроля (100Hz = 10ms)
  timer0 = timerBegin(2, 80, true);
  timerAttachInterrupt(timer0, &onControlTimer, true);
  timerAlarmWrite(timer0, 10000, true);  // 10ms
  timerAlarmEnable(timer0);

  Serial.println("[OK] Timers initialized");
}

// ============================================================================
// КОНЕЦ
// ============================================================================
