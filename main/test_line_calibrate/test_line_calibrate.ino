// Line Follower Sensor Calibration Test
// Run this sketch to calibrate your line sensors before using line following feature

#include <Wire.h>

#define LINE_SENSOR_1   34   // Far left
#define LINE_SENSOR_2   35   // Left center (input-only, may not work)
#define LINE_SENSOR_3   36   // Center (input-only, may not work)
#define LINE_SENSOR_4   39   // Right center (input-only, may not work)
#define LINE_SENSOR_5   32   // Far right

static const uint8_t sensor_pins[5] = {
  LINE_SENSOR_1, LINE_SENSOR_2, LINE_SENSOR_3, LINE_SENSOR_4, LINE_SENSOR_5
};

int16_t line_sensor_raw[5];
uint16_t line_min_values[5];
uint16_t line_max_values[5];

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n=== LINE SENSOR CALIBRATION ===");
  Serial.println("GPIO pins:");
  Serial.println("  Sensor 0 (Left):       GPIO 34");
  Serial.println("  Sensor 1 (L-center):   GPIO 35 (input-only, may not respond)");
  Serial.println("  Sensor 2 (Center):     GPIO 36 (input-only, may not respond)");
  Serial.println("  Sensor 3 (R-center):   GPIO 39 (input-only, may not respond)");
  Serial.println("  Sensor 4 (Right):      GPIO 32");
  Serial.println();

  analogSetWidth(12);
  analogSetAttenuation(ADC_11db);

  // Read raw values continuously
  Serial.println("Reading raw sensor values... Place sensor on different surfaces:");
  Serial.println("- White paper (no line)");
  Serial.println("- Black line (tape)");
  Serial.println();

  for (int i = 0; i < 5; i++) {
    line_min_values[i] = 4095;
    line_max_values[i] = 0;
  }
}

void loop() {
  // Read all sensors
  for (uint8_t i = 0; i < 5; i++) {
    line_sensor_raw[i] = analogRead(sensor_pins[i]);

    // Track min/max for each sensor
    if (line_sensor_raw[i] < line_min_values[i]) {
      line_min_values[i] = line_sensor_raw[i];
    }
    if (line_sensor_raw[i] > line_max_values[i]) {
      line_max_values[i] = line_sensor_raw[i];
    }
  }

  // Print raw values
  Serial.print("[RAW]  ");
  for (int i = 0; i < 5; i++) {
    Serial.print(line_sensor_raw[i]);
    Serial.print(" ");
  }

  // Print current min/max
  Serial.print("| [MIN] ");
  for (int i = 0; i < 5; i++) {
    Serial.print(line_min_values[i]);
    Serial.print(" ");
  }

  Serial.print("| [MAX] ");
  for (int i = 0; i < 5; i++) {
    Serial.print(line_max_values[i]);
    Serial.print(" ");
  }

  Serial.println();

  // Check for stuck sensors
  Serial.print("[INFO] ");
  for (int i = 0; i < 5; i++) {
    if (line_sensor_raw[i] == line_min_values[i] && line_sensor_raw[i] == line_max_values[i]) {
      Serial.print("Sensor");
      Serial.print(i);
      Serial.print("=STUCK ");
    }
  }
  Serial.println();

  delay(500);
}
