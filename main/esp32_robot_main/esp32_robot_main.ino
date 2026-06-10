// ESP32 Self-Balancing Robot
// Чистый балансир без веб-интерфейса
// State machine: IDLE → STRAIGHT (2 сек) → LINE_FOLLOW || STOPPED (препятствие)

#include <Wire.h>
#include "Control.h"
#include "MPU6050.h"
#include "Motors.h"
#include "defines.h"
#include "globals.h"
#include "LineFollower.h"
#include <VL53L0X.h>

VL53L0X distanceSensor;
uint16_t distance_mm = 0;

// Состояния робота
#define STATE_IDLE 0
#define STATE_STRAIGHT 1
#define STATE_LINE_FOLLOW 2
#define STATE_STOPPED 3

volatile uint8_t robot_state = STATE_IDLE;
unsigned long state_start_time = 0;

volatile bool on_slope = false;
int16_t z_accel_baseline = 0;

#define SLOPE_ACCEL_THRESHOLD 2000
#define DISTANCE_THRESHOLD 300
#define STRAIGHT_TIME 2000

void initTimers();  // из Timers.cpp

// ============================================================================
// ИНИЦИАЛИЗАЦИЯ ДАТЧИКОВ
// ============================================================================

bool initMPU6050() {
  Serial.println("[MPU6050] Initializing...");
  MPU6050_setup();
  vTaskDelay(pdMS_TO_TICKS(500));

  // Калибровка с максимум 3 попытками
  uint8_t cal_attempts = 0;
  while (cal_attempts < 3) {
    cal_attempts++;
    Serial.print("[CALIB] Attempt ");
    Serial.print(cal_attempts);
    Serial.println("/3 - DONT MOVE!");
    MPU6050_calibrate();
    if (x_gyro_offset != 0) break;  // успешна
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
  Serial.println("[OK] VL53L0X ready");
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ESP32 Self-Balancing Robot ===");

  // GPIO
  pinMode(PIN_ENABLE_MOTORS, OUTPUT);
  digitalWrite(PIN_ENABLE_MOTORS, LOW);  // TMC2209: EN active-LOW

  pinMode(PIN_MOTOR1_DIR, OUTPUT);
  pinMode(PIN_MOTOR1_STEP, OUTPUT);
  pinMode(PIN_MOTOR2_DIR, OUTPUT);
  pinMode(PIN_MOTOR2_STEP, OUTPUT);

  // Инициализация датчиков и управления
  if (!initMPU6050()) {
    Serial.println("[FATAL] MPU6050 failed!");
    while(1) vTaskDelay(pdMS_TO_TICKS(1000));
  }

  initDistanceSensor();
  LineFollower_init();
  initTimers();

  // Начальное состояние: IDLE
  robot_state = STATE_IDLE;
  throttle = 0;
  steering = 0;

  Serial.println("[OK] Ready!");
  Serial.println("Robot will: IDLE → STRAIGHT (2s) → LINE_FOLLOW");
  delay(2000);
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
  if (on_slope) return (speed * 0.5);  // 2x замедление на склоне
  return speed;
}

// ============================================================================
// КОНТРОЛЬНЫЙ ЦИКЛ (100 Гц)
// ============================================================================

void controlLoop() {
  float dt = (timer_value - timer_old) / 1000000.0;

  // Защита: робот лежит (>70°) - моторы стоп
  if (fabsf(angle_adjusted) > 70.0f) {
    setMotorSpeedM1(0);
    setMotorSpeedM2(0);
    return;
  }

  // Читать дистанцию
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
      // Ручное управление: throttle/steering с джойстика (если бы он был)
      // На данный момент просто стоит
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

  // Диагностика каждые 100 циклов (~1 сек)
  if (loop_counter % 100 == 0) {
    Serial.print("[Status] State:");
    Serial.print(robot_state);
    Serial.print(" Angle:");
    Serial.print(angle_adjusted, 1);
    Serial.print("° Dist:");
    Serial.print(distance_mm);
    Serial.print("mm M1:");
    Serial.print(speed_M1);
    Serial.print(" M2:");
    Serial.println(speed_M2);
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
// LOOP - контрольный цикл без блокировок
// ============================================================================

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
