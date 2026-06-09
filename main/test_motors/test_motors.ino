// TEST: Моторы (тестирование вращения)
// Проверяет работу обоих шаговых моторов с реальным контролем скорости
// Совместимо с ESP32 Arduino Core v3.x (новый Timer API)

#include <Arduino.h>

// GPIO моторов
#define PIN_ENABLE_MOTORS 12
#define PIN_MOTOR1_DIR 27
#define PIN_MOTOR1_STEP 14
#define PIN_MOTOR2_DIR 25
#define PIN_MOTOR2_STEP 26

#define TIMER_FREQ 1000000UL   // 1 МГц -> 1 тик = 1 мкс

// Таймеры
hw_timer_t *timer1 = NULL;
hw_timer_t *timer2 = NULL;

volatile bool     m1_active = false;
volatile bool     m2_active = false;
volatile bool     step1_state = false;
volatile bool     step2_state = false;
volatile uint32_t steps_M1 = 0;
volatile uint32_t steps_M2 = 0;

// ISR: переключаем STEP пин. Полный шаг = HIGH->LOW (два срабатывания).
// Считаем шаги по фронту HIGH.
void IRAM_ATTR onTimer1() {
  if (!m1_active) return;
  step1_state = !step1_state;
  digitalWrite(PIN_MOTOR1_STEP, step1_state);
  if (step1_state) steps_M1++;
}

void IRAM_ATTR onTimer2() {
  if (!m2_active) return;
  step2_state = !step2_state;
  digitalWrite(PIN_MOTOR2_STEP, step2_state);
  if (step2_state) steps_M2++;
}

void initTimers() {
  // Таймер 1 (1 МГц)
  timer1 = timerBegin(TIMER_FREQ);
  timerAttachInterrupt(timer1, &onTimer1);
  timerAlarm(timer1, 10000, true, 0);   // период по умолчанию, переопределяется скоростью

  // Таймер 2 (1 МГц)
  timer2 = timerBegin(TIMER_FREQ);
  timerAttachInterrupt(timer2, &onTimer2);
  timerAlarm(timer2, 10000, true, 0);
}

// speed = шагов/сек, знак = направление
void setMotorSpeed(int motor, int16_t speed) {
  uint16_t s = abs(speed);

  if (motor == 1) {
    digitalWrite(PIN_MOTOR1_DIR, speed >= 0 ? HIGH : LOW);
    if (s == 0) {
      m1_active = false;
    } else {
      // полупериод (мкс) = переключение пина 2 раза за шаг
      uint32_t halfPeriod = (TIMER_FREQ / 2) / s;
      timerAlarm(timer1, halfPeriod, true, 0);
      m1_active = true;
    }
  } else if (motor == 2) {
    digitalWrite(PIN_MOTOR2_DIR, speed >= 0 ? LOW : HIGH);  // M2 инвертирован
    if (s == 0) {
      m2_active = false;
    } else {
      uint32_t halfPeriod = (TIMER_FREQ / 2) / s;
      timerAlarm(timer2, halfPeriod, true, 0);
      m2_active = true;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== MOTOR TEST ===");
  Serial.println("Testing stepper motors (ESP32 Core v3.x)");

  // GPIO
  pinMode(PIN_ENABLE_MOTORS, OUTPUT);
  digitalWrite(PIN_ENABLE_MOTORS, LOW);   // LOW = драйвер включён (active-LOW).
                                          // Если моторы не крутятся - попробуй HIGH.
  pinMode(PIN_MOTOR1_DIR, OUTPUT);
  pinMode(PIN_MOTOR1_STEP, OUTPUT);
  pinMode(PIN_MOTOR2_DIR, OUTPUT);
  pinMode(PIN_MOTOR2_STEP, OUTPUT);

  initTimers();

  Serial.println("[OK] Motors ready!");
  delay(2000);
}

void loop() {
  Serial.println("[TEST 1] Both forward SLOW (50 steps/s)");
  setMotorSpeed(1, 50);
  setMotorSpeed(2, 50);
  delay(5000);
  printStats();

  Serial.println("[TEST 2] Both forward FAST (150 steps/s)");
  setMotorSpeed(1, 150);
  setMotorSpeed(2, 150);
  delay(5000);
  printStats();

  Serial.println("[TEST 3] Both backward SLOW (-50 steps/s)");
  setMotorSpeed(1, -50);
  setMotorSpeed(2, -50);
  delay(5000);
  printStats();

  Serial.println("[TEST 4] M1 FAST, M2 SLOW (sync test)");
  setMotorSpeed(1, 150);
  setMotorSpeed(2, 50);
  delay(5000);
  printStats();

  Serial.println("[TEST 5] STOP");
  setMotorSpeed(1, 0);
  setMotorSpeed(2, 0);
  delay(3000);

  Serial.println("\n=== TEST CYCLE COMPLETE ===\n");
  delay(2000);
}

void printStats() {
  Serial.print("[Stats] M1 steps: ");
  Serial.print(steps_M1);
  Serial.print(" | M2 steps: ");
  Serial.print(steps_M2);
  Serial.print(" | Ratio M1/M2: ");
  if (steps_M2 > 0) Serial.print((float)steps_M1 / steps_M2, 2);
  else              Serial.print("N/A");
  Serial.println();

  steps_M1 = 0;
  steps_M2 = 0;
}
