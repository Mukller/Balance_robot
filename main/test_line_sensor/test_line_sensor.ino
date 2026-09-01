// TEST: Датчик линии (5 датчиков Ldabrye)
// Выводит значения всех 5 датчиков в COM порт

#include <Arduino.h>

// GPIO пины датчика линии
#define LINE_SENSOR_1 34  // Левый
#define LINE_SENSOR_2 35
#define LINE_SENSOR_3 36  // Центр
#define LINE_SENSOR_4 39
#define LINE_SENSOR_5 32  // Правый

const uint8_t sensor_pins[5] = {
  LINE_SENSOR_1, LINE_SENSOR_2, LINE_SENSOR_3, LINE_SENSOR_4, LINE_SENSOR_5
};

int16_t sensor_raw[5];
int16_t sensor_calibrated[5];

// Калибровка
uint16_t min_values[5] = {500, 500, 500, 500, 500};
uint16_t max_values[5] = {3500, 3500, 3500, 3500, 3500};

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== LINE SENSOR TEST ===");
  Serial.println("Testing 5 line sensors (Ldabrye)");

  // Настроить АДЦ (тот же набор, что и в основном скетче)
  analogSetWidth(12);
  analogSetAttenuation(ADC_11db);
  analogReadResolution(12);

  Serial.println("[OK] Ready! Place sensor over white/black surface...");
  delay(1000);
}

void loop() {
  // Читать датчики
  for (uint8_t i = 0; i < 5; i++) {
    sensor_raw[i] = analogRead(sensor_pins[i]);

    // Нормализовать (0-200)
    uint16_t min_val = min_values[i];
    uint16_t max_val = max_values[i];

    if (sensor_raw[i] < min_val) {
      sensor_calibrated[i] = 0;
    } else if (sensor_raw[i] > max_val) {
      sensor_calibrated[i] = 200;
    } else {
      sensor_calibrated[i] = (uint32_t)(sensor_raw[i] - min_val) * 200 / (max_val - min_val);
    }
  }

  // Визуализация в COM
  Serial.print("[Line Sensors] Raw: ");
  for (int i = 0; i < 5; i++) {
    Serial.print(sensor_raw[i]);
    if (i < 4) Serial.print(" ");
  }

  Serial.print(" | Cal: ");
  for (int i = 0; i < 5; i++) {
    Serial.print(sensor_calibrated[i]);
    if (i < 4) Serial.print(" ");
  }

  // График в ASCII
  Serial.print(" | Graph: ");
  for (int i = 0; i < 5; i++) {
    int bars = sensor_calibrated[i] / 20;  // 0-10 бар
    for (int j = 0; j < bars; j++) Serial.print("█");
    Serial.print(" ");
  }

  Serial.println();
  delay(100);
}

// Author: Anton Petnitsky
// GitHub: https://github.com/Mukller/Balance_robot
// Last modified: 2026-06-10 00:11:14 +0300
