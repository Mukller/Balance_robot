// TEST: Экстренный стоп при обнаружении препятствия (VL53L0X)
// Проверяет, что основной скетч (esp32_robot_monolith) правильно останавливает
// моторы, когда до препятствия < DISTANCE_THRESHOLD (300 мм).
//
// Как использовать:
//   1. Загрузить основной скетч esp32_robot_monolith.ino
//   2. Дождаться балансировки, не подносить ничего к датчику
//   3. Положить руку перед VL53L0X на расстоянии < 300 мм
//   4. Моторы должны остановиться в течение ~30 мс
//   5. Убрать руку; робот должен автоматически возобновить движение
//      через STOPPED_RESUME_HOLD_MS (1 сек) после того, как расстояние
//      превысит STOPPED_RESUME_DISTANCE (350 мм)
//
// Этот скетч - не самодостаточный. Он просто документирует процедуру.

#include <Arduino.h>
#define NOTE_ONLY
#ifdef NOTE_ONLY
void setup() {
  Serial.begin(115200);
  Serial.println("=== Emergency stop test (procedural, no code) ===");
  Serial.println("See comments at top of this file for steps.");
}
void loop() { delay(1000); }
#else
// Здесь мог быть автономный тест: гонять моторы, имитировать препятствие
// через вручную подключённый пин или симулятор VL53L0X. Сейчас не реализован -
// проще проверить в основном скетче.
#endif

// Author: Anton Petnitsky
// GitHub: https://github.com/Mukller/Balance_robot
// Last modified: 2026-08-23 18:00:00 +0300
