// TEST: Датчик расстояния VL53L0X
// Измеряет расстояние до препятствия

#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== DISTANCE SENSOR TEST ===");
  Serial.println("Testing VL53L0X (I2C 0x29)");

  // Инициализировать I2C
  Wire.begin(21, 22);
  delay(100);

  // Инициализировать датчик
  if (!sensor.init()) {
    Serial.println("[ERROR] VL53L0X not found!");
    Serial.println("Check I2C connection (SDA=21, SCL=22)");
    while (1) delay(1000);
  }

  sensor.setTimeout(500);
  sensor.setMeasurementTimingBudget(33000);
  Serial.println("[OK] VL53L0X initialized!");
  Serial.println("Distance measurements (mm):");
}

void loop() {
  uint16_t distance = sensor.readRangeSingleMillimeters();

  // Вывести значение
  Serial.print("[Distance] ");
  Serial.print(distance);
  Serial.print(" mm");

  // Статус
  if (sensor.timeoutOccurred()) {
    Serial.print(" [TIMEOUT]");
  } else {
    // График расстояния
    Serial.print(" | ");
    int bars = constrain(distance / 50, 0, 20);
    for (int i = 0; i < bars; i++) Serial.print("█");
    Serial.print(" ");

    // Интерпретация
    if (distance < 200) {
      Serial.print("[VERY CLOSE]");
    } else if (distance < 500) {
      Serial.print("[CLOSE]");
    } else if (distance < 1000) {
      Serial.print("[MEDIUM]");
    } else {
      Serial.print("[FAR]");
    }
  }

  Serial.println();
  delay(200);
}

// Author: Anton Petnitsky
// GitHub: https://github.com/Mukller/Balance_robot
// Last modified: 2026-06-10 01:30:30 +0300
