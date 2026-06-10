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
#include <VL53L0X.h>
#include "web_interface.h"

// ESP32 PWM для сервомоторов (v3.x - привязка по пину)
#define PWM_FREQ 50
#define PWM_RESOLUTION 16

// WiFi параметры - ОТРЕДАКТИРУЙ!
String sta_ssid = "XPEHBAM";
String sta_password = "+375296285943";

// Датчики
VL53L0X distanceSensor;
uint16_t distance_mm = 0;

// Захват (две серво)
// ВНИМАНИЕ: сервы MG996R НЕЛЬЗЯ питать от пина 5V платы ESP32 -
// бросок тока при движении (1-2 А) роняет плату в ресет.
// Поставь 1 ТОЛЬКО после подключения серв к отдельному 5V (BEC >= 3A, общий GND).
#define GRIPPER_ENABLED 0      // 0 = захват отключён, 1 = включён

#define SERVO_LEFT_PIN 13      // Левая серво (GPIO 13)
#define SERVO_RIGHT_PIN 33     // Правая серво (GPIO 33)
volatile uint8_t gripper_state = 0;  // 0 = открыт, 1 = закрыт
#define GRIPPER_OPEN 180
#define GRIPPER_CLOSE 90

// PWM helper - конвертировать угол в PWM значение
int angleToPWM(int angle) {
  // 0° = 1000us, 180° = 2000us
  // На 50Hz и 16-bit: 1us = 3.277 ticks
  int pulse = 1000 + (angle * 1000) / 180;
  return (int)(pulse * 3.277);
}

// Определена в Timers.cpp (шаговые таймеры моторов)
void initTimers();

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
  // Используем непрерывный режим: startRangeContinuous() + readRangeContinuousMillimeters()
  // в control loop (каждые 10 мс = 100 Гц, достаточно для датчика)
  Serial.println("[OK] VL53L0X initialized");
}

void initGripper() {
  if (!GRIPPER_ENABLED) {
    Serial.println("[GRIPPER] DISABLED (set GRIPPER_ENABLED 1 after wiring separate 5V for servos)");
    return;
  }
  Serial.println("[GRIPPER] Initializing servo PWM...");

  // Сервы запускаем ПО ОЧЕРЕДИ: бросок тока MG996R при старте 1-2 А,
  // одновременный запуск двух серв просаживает 5V и роняет плату
  ledcAttach(SERVO_LEFT_PIN, PWM_FREQ, PWM_RESOLUTION);
  setServoAngle(SERVO_LEFT_PIN, GRIPPER_OPEN);
  vTaskDelay(pdMS_TO_TICKS(700));  // даём левой доехать

  ledcAttach(SERVO_RIGHT_PIN, PWM_FREQ, PWM_RESOLUTION);
  setServoAngle(SERVO_RIGHT_PIN, GRIPPER_OPEN);
  vTaskDelay(pdMS_TO_TICKS(700));  // даём правой доехать

  gripper_state = 0;
  Serial.println("[OK] Gripper initialized (OPEN)");
}

void setServoAngle(int pin, int angle) {
  angle = constrain(angle, 0, 180);
  int pwmValue = angleToPWM(angle);
  ledcWrite(pin, pwmValue);
}

void gripperOpen() {
  if (!GRIPPER_ENABLED) { Serial.println("[GRIPPER] disabled"); return; }
  setServoAngle(SERVO_LEFT_PIN, GRIPPER_OPEN);
  vTaskDelay(pdMS_TO_TICKS(150));  // развести пусковые токи серв
  setServoAngle(SERVO_RIGHT_PIN, GRIPPER_OPEN);
  gripper_state = 0;
  Serial.println("[GRIPPER] OPEN");
}

void gripperClose() {
  if (!GRIPPER_ENABLED) { Serial.println("[GRIPPER] disabled"); return; }
  setServoAngle(SERVO_LEFT_PIN, GRIPPER_CLOSE);
  vTaskDelay(pdMS_TO_TICKS(150));  // развести пусковые токи серв
  setServoAngle(SERVO_RIGHT_PIN, GRIPPER_CLOSE);
  gripper_state = 1;
  Serial.println("[GRIPPER] CLOSE");
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ESP32 Self-Balancing Robot (Main) ===");

  // GPIO инициализация
  pinMode(PIN_ENABLE_MOTORS, OUTPUT);
  digitalWrite(PIN_ENABLE_MOTORS, LOW);  // TMC2209: EN active-LOW, LOW = драйверы ВКЛЮЧЕНЫ
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

  // WiFi (включаем ДО захвата: спайк радио не должен накладываться
  // на ток серв - вместе они просаживают питание и роняют плату)
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

  initGripper();  // Захват - после WiFi (сервы дают бросок тока при старте)

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
      } else if (cmd == "gripper_open") {
        gripperOpen();
      } else if (cmd == "gripper_close") {
        gripperClose();
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
    "{\"state\":%d,\"distance\":%d,\"slope\":%d,\"angle\":%.1f,\"throttle\":%d,\"gripper\":%d,\"sensors\":[%d,%d,%d,%d,%d],\"lineError\":%d}",
    robot_state, distance_mm, (int)on_slope, angle_adjusted, throttle, gripper_state,
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

  // Защита: робот лежит/упал (>70 град от вертикали) - моторы стоп.
  // Баланс-робот реагирует моторами на наклон ТОЛЬКО когда стоит почти вертикально!
  if (fabsf(angle_adjusted) > 70.0f) {
    setMotorSpeedM1(0);
    setMotorSpeedM2(0);
    return;
  }

  // Читать дистанцию (синхронно: ~10 мс за замер, но надёжно)
  if (!distanceSensor.timeoutOccurred()) {
    distance_mm = distanceSensor.readRangeSingleMillimeters();
  }

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
      // Ручной режим: throttle/steering приходят с джойстика (/api/joystick)
      // и НЕ затираются - иначе джойстик не работает
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

// Контрольный цикл 100 Гц в loop() (НЕ в ISR: I2C-чтение MPU в прерывании
// на ESP32 приводит к крашу). Шаги моторов генерят timer1/timer2 из Timers.cpp.
void loop() {
  static unsigned long last_control_us = 0;
  unsigned long now = micros();

  if ((unsigned long)(now - last_control_us) >= 10000) {  // 10 мс = 100 Гц
    last_control_us = now;

    timer_value = now;
    dt = (timer_value - timer_old) / 1000000.0;

    readSensors();
    controlLoop();

    timer_old = timer_value;

    cascade_control_loop_counter++;
    if (cascade_control_loop_counter >= 10) {
      cascade_control_loop_counter = 0;
      loop_counter++;
    }
  }
}
