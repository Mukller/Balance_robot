// I2C SCANNER - найти, какие устройства подключены на SDA/SCL
// SDA = GPIO 21, SCL = GPIO 22

#include <Wire.h>

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n=== I2C SCANNER ===");
  Serial.println("SDA = GPIO 21, SCL = GPIO 22");
  Serial.println("Scanning for devices...\n");

  Wire.begin(21, 22);  // SDA=21, SCL=22

  byte count = 0;
  for (byte i = 1; i < 127; i++) {
    Wire.beginTransmission(i);
    byte error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("Device found at 0x");
      if (i < 16) Serial.print("0");
      Serial.print(i, HEX);
      Serial.print(" (");
      Serial.print(i, DEC);
      Serial.println(")");
      count++;
    }
  }

  Serial.print("\nTotal devices found: ");
  Serial.println(count);

  if (count == 0) {
    Serial.println("\n[ERROR] No devices found!");
    Serial.println("Check:");
    Serial.println("  - SDA (GPIO 21) connected to datasheets SDA");
    Serial.println("  - SCL (GPIO 22) connected to datasheets SCL");
    Serial.println("  - Both devices have 3.3V power");
    Serial.println("  - Both have GND connected");
    Serial.println("  - No broken wires");
  }
}

void loop() {
  delay(1000);
}

// Author: Anton Petnitsky
// GitHub: https://github.com/Mukller/Balance_robot
// Last modified: 2026-06-10 10:49:14 +0300
