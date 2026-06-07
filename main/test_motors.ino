// TEST: Моторы (тестирование вращения)
// Проверяет работу обоих моторов с контролем скорости

#include <Arduino.h>

// GPIO моторов
#define PIN_ENABLE_MOTORS 12
#define PIN_MOTOR1_DIR 27
#define PIN_MOTOR1_STEP 14
#define PIN_MOTOR2_DIR 25
#define PIN_MOTOR2_STEP 26

// Глобальные переменные для таймеров
hw_timer_t * timer1 = NULL;
hw_timer_t * timer2 = NULL;

volatile int16_t speed_M1 = 0;
volatile int16_t speed_M2 = 0;

// Счётчики шагов
volatile uint32_t steps_M1 = 0;
volatile uint32_t steps_M2 = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== MOTOR TEST ===");
  Serial.println("Testing stepper motors");

  // GPIO инициализация
  pinMode(PIN_ENABLE_MOTORS, OUTPUT);
  digitalWrite(PIN_ENABLE_MOTORS, HIGH);

  pinMode(PIN_MOTOR1_DIR, OUTPUT);
  pinMode(PIN_MOTOR1_STEP, OUTPUT);
  pinMode(PIN_MOTOR2_DIR, OUTPUT);
  pinMode(PIN_MOTOR2_STEP, OUTPUT);

  // Инициализировать таймеры
  initTimers();

  Serial.println("[OK] Motors ready!");
  Serial.println("Testing pattern:");
  Serial.println("1. Forward slow");
  Serial.println("2. Forward fast");
  Serial.println("3. Backward slow");
  Serial.println("4. Stop");
  delay(2000);
}

void initTimers() {
  // Таймер 1
  timer1 = timerBegin(0, 40, true);
  timerAttachInterrupt(timer1, &onTimer1, true);
  timerAlarmWrite(timer1, 500000, true);
  timerAlarmEnable(timer1);

  // Таймер 2
  timer2 = timerBegin(1, 40, true);
  timerAttachInterrupt(timer2, &onTimer2, true);
  timerAlarmWrite(timer2, 500000, true);
  timerAlarmEnable(timer2);
}

void IRAM_ATTR onTimer1() {
  if (speed_M1 != 0) {
    digitalWrite(PIN_MOTOR1_STEP, HIGH);
    delayMicroseconds(50);
    digitalWrite(PIN_MOTOR1_STEP, LOW);
    if (speed_M1 > 0) steps_M1++;
    else steps_M1--;
  }
}

void IRAM_ATTR onTimer2() {
  if (speed_M2 != 0) {
    digitalWrite(PIN_MOTOR2_STEP, HIGH);
    delayMicroseconds(50);
    digitalWrite(PIN_MOTOR2_STEP, LOW);
    if (speed_M2 > 0) steps_M2++;
    else steps_M2--;
  }
}

void setMotorSpeed(int motor, int16_t speed) {
  if (motor == 1) {
    if (speed > 0) {
      digitalWrite(PIN_MOTOR1_DIR, HIGH);
    } else {
      digitalWrite(PIN_MOTOR1_DIR, LOW);
    }
    speed_M1 = abs(speed);
  } else if (motor == 2) {
    if (speed > 0) {
      digitalWrite(PIN_MOTOR2_DIR, LOW);
    } else {
      digitalWrite(PIN_MOTOR2_DIR, HIGH);
    }
    speed_M2 = abs(speed);
  }
}

void loop() {
  // Тестовая программа с разными скоростями

  // 1. Медленно вперёд (10 сек)
  Serial.println("[TEST 1] Both motors forward SLOW (50 steps/s)");
  setMotorSpeed(1, 50);
  setMotorSpeed(2, 50);
  delay(10000);
  printStats();

  // 2. Быстро вперёд (10 сек)
  Serial.println("[TEST 2] Both motors forward FAST (150 steps/s)");
  setMotorSpeed(1, 150);
  setMotorSpeed(2, 150);
  delay(10000);
  printStats();

  // 3. Назад медленно (10 сек)
  Serial.println("[TEST 3] Both motors backward SLOW (-50 steps/s)");
  setMotorSpeed(1, -50);
  setMotorSpeed(2, -50);
  delay(10000);
  printStats();

  // 4. Разные скорости (10 сек) - тест синхронизации
  Serial.println("[TEST 4] Motor1 FAST, Motor2 SLOW (тест синхронизации)");
  setMotorSpeed(1, 150);
  setMotorSpeed(2, 50);
  delay(10000);
  printStats();

  // 5. Стоп
  Serial.println("[TEST 5] STOP");
  setMotorSpeed(1, 0);
  setMotorSpeed(2, 0);
  delay(3000);

  Serial.println("\n=== TEST CYCLE COMPLETE ===\n");
  delay(2000);
}

void printStats() {
  delay(1000);  // Даём мотору время работать
  Serial.print("[Stats] M1 steps: ");
  Serial.print(steps_M1);
  Serial.print(" | M2 steps: ");
  Serial.print(steps_M2);
  Serial.print(" | Ratio: ");
  if (steps_M2 > 0) {
    Serial.print((float)steps_M1 / steps_M2, 2);
  } else {
    Serial.print("N/A");
  }
  Serial.println();

  // Сброс счётчиков
  steps_M1 = 0;
  steps_M2 = 0;
}
