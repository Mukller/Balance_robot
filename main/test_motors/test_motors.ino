// TEST: Моторы + драйвер TMC2209 (простой и надёжный)
// Без таймеров и прерываний - чистый шаг через micros()
// Оба мотора крутятся непрерывно с видимой скоростью, реверс каждые 3 сек
//
// Подключение TMC2209:
//   EN   -> GPIO 12   (ВКЛ драйвера, active-LOW: LOW = включён)
//   M1: DIR -> GPIO 27, STEP -> GPIO 14
//   M2: DIR -> GPIO 25, STEP -> GPIO 26
//   VIO -> 3.3V  (ОБЯЗАТЕЛЬНО! логическое питание драйвера)
//   VM  -> 12-24V (питание мотора) + конденсатор 100мкF по VM/GND
//   GND -> общий с ESP32

#include <Arduino.h>

#define PIN_ENABLE_MOTORS 12
#define PIN_M1_DIR        27
#define PIN_M1_STEP       14
#define PIN_M2_DIR        25
#define PIN_M2_STEP       26

// Интервал переключения пина STEP, мкс.
// 500 мкс -> период шага 1000 мкс -> 1000 шагов/с (хорошо видно)
unsigned int stepInterval = 500;

unsigned long t1 = 0, t2 = 0;
bool st1 = false, st2 = false;
bool dir = true;
unsigned long lastDirChange = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== TMC2209 MOTOR TEST (simple) ===");

  pinMode(PIN_ENABLE_MOTORS, OUTPUT);
  pinMode(PIN_M1_DIR, OUTPUT);
  pinMode(PIN_M1_STEP, OUTPUT);
  pinMode(PIN_M2_DIR, OUTPUT);
  pinMode(PIN_M2_STEP, OUTPUT);

  // TMC2209: EN active-LOW. LOW = драйвер включён.
  digitalWrite(PIN_ENABLE_MOTORS, LOW);

  // Начальное направление
  digitalWrite(PIN_M1_DIR, HIGH);
  digitalWrite(PIN_M2_DIR, LOW);

  Serial.println("[OK] EN=LOW (driver enabled), both motors spinning ~1000 steps/s");
  Serial.println("If nothing moves -> check VIO=3.3V, VM=12-24V, Vref (current), wiring.");
  lastDirChange = millis();
}

void loop() {
  unsigned long now = micros();

  // Шаг мотора 1
  if (now - t1 >= stepInterval) {
    t1 = now;
    st1 = !st1;
    digitalWrite(PIN_M1_STEP, st1);
  }

  // Шаг мотора 2
  if (now - t2 >= stepInterval) {
    t2 = now;
    st2 = !st2;
    digitalWrite(PIN_M2_STEP, st2);
  }

  // Реверс направления каждые 3 секунды (без блокировки)
  if (millis() - lastDirChange >= 3000) {
    lastDirChange = millis();
    dir = !dir;
    digitalWrite(PIN_M1_DIR, dir ? HIGH : LOW);
    digitalWrite(PIN_M2_DIR, dir ? LOW : HIGH);
    Serial.print("[DIR] ");
    Serial.println(dir ? "forward" : "backward");
  }
}
