#include "LineFollower.h"
#include "defines.h"

// Global variables
int16_t line_sensor_raw[5];
int16_t line_sensor_calibrated[5];
int16_t line_position_error = 0;
int16_t line_steering_output = 0;
uint8_t line_mode = LINE_MODE_OFF;
uint16_t line_min_values[5] = {0, 0, 0, 0, 0};      // White calibration
uint16_t line_max_values[5] = {4095, 4095, 4095, 4095, 4095}; // Black calibration

static const uint8_t sensor_pins[5] = {
  LINE_SENSOR_1, LINE_SENSOR_2, LINE_SENSOR_3, LINE_SENSOR_4, LINE_SENSOR_5
};

// Weighted position calculation for 5 sensors
// Sensor positions: -2 (left), -1, 0 (center), +1, +2 (right)
static const int8_t sensor_weights[5] = {-2, -1, 0, 1, 2};

// ====== INITIALIZATION ======
void LineFollower_init() {
  // Set ADC resolution to 12 bits (0-4095)
  analogSetWidth(12);
  analogSetAttenuation(ADC_11db);  // Full 0-4095 range
  
  Serial.println("[LineFollower] Initialized - 5 sensor array");
  Serial.println("[LineFollower] Run calibration before use!");
  
  // Initialize calibration values to defaults
  for (uint8_t i = 0; i < 5; i++) {
    line_min_values[i] = 500;    // Expected white value
    line_max_values[i] = 3500;   // Expected black value
  }
}

// ====== READ RAW SENSOR VALUES ======
void LineFollower_readSensors() {
  static uint16_t sample_buffer[5][LINE_SENSOR_SAMPLES];
  static uint8_t sample_index = 0;
  
  // Read raw values
  for (uint8_t i = 0; i < 5; i++) {
    sample_buffer[i][sample_index] = analogRead(sensor_pins[i]);
  }
  
  sample_index++;
  if (sample_index >= LINE_SENSOR_SAMPLES) {
    sample_index = 0;
  }
  
  // Average samples
  for (uint8_t i = 0; i < 5; i++) {
    uint32_t sum = 0;
    for (uint8_t j = 0; j < LINE_SENSOR_SAMPLES; j++) {
      sum += sample_buffer[i][j];
    }
    line_sensor_raw[i] = sum / LINE_SENSOR_SAMPLES;
    
    // Normalize to 0-200 range based on calibration
    // 0 = white (no line), 200 = black (line detected)
    uint16_t min_val = line_min_values[i];
    uint16_t max_val = line_max_values[i];
    
    if (line_sensor_raw[i] < min_val) {
      line_sensor_calibrated[i] = 0;
    } else if (line_sensor_raw[i] > max_val) {
      line_sensor_calibrated[i] = 200;
    } else {
      line_sensor_calibrated[i] = (uint32_t)(line_sensor_raw[i] - min_val) * 200 / (max_val - min_val);
    }
  }
}

// ====== CALCULATE LINE POSITION ERROR ======
// Returns weighted average position: -200 (far left) to +200 (far right)
int16_t LineFollower_calculateError() {
  LineFollower_readSensors();
  
  uint32_t total_weight = 0;
  int32_t weighted_sum = 0;
  
  // Calculate center of mass of detected line
  for (uint8_t i = 0; i < 5; i++) {
    uint16_t sensor_value = line_sensor_calibrated[i];
    weighted_sum += sensor_value * sensor_weights[i];
    total_weight += sensor_value;
  }
  
  // Avoid division by zero
  if (total_weight == 0) {
    line_position_error = 0;  // Line not detected - center position
  } else {
    // Normalize to -200 to +200 range
    line_position_error = constrain((int16_t)(weighted_sum * 200 / total_weight), -200, 200);
  }
  
  return line_position_error;
}

// ====== GET STEERING COMMAND ======
// Returns steering correction based on line position error
// Positive = turn right, Negative = turn left
int16_t LineFollower_getSteering() {
  int16_t error = LineFollower_calculateError();
  
  // Simple proportional control
  // Kp_line should be tuned based on desired aggressiveness
  float Kp_line = 0.5;  // Tune this value
  
  line_steering_output = constrain((int16_t)(error * Kp_line), -LINE_MAX_ERROR, LINE_MAX_ERROR);
  
  return line_steering_output;
}

// ====== SET LINE FOLLOWING MODE ======
void LineFollower_setMode(uint8_t mode) {
  if (mode <= LINE_MODE_AUTO) {
    line_mode = mode;
    if (mode == LINE_MODE_OFF) {
      line_steering_output = 0;
      Serial.println("[LineFollower] Mode: OFF");
    } else if (mode == LINE_MODE_ON) {
      Serial.println("[LineFollower] Mode: ON");
    } else {
      Serial.println("[LineFollower] Mode: AUTO");
    }
  }
}

// ====== GET CURRENT MODE ======
uint8_t LineFollower_getMode() {
  return line_mode;
}

// ====== CALIBRATION ROUTINE ======
// User should place sensor over white surface, then black line
void LineFollower_calibrate() {
  Serial.println("\n[LineFollower] CALIBRATION START");
  Serial.println("[LineFollower] Place sensor over WHITE surface. Press button in 5 seconds...");
  delay(5000);
  
  // Calibrate white (minimum values)
  Serial.println("[LineFollower] Reading WHITE values...");
  for (uint8_t i = 0; i < 5; i++) {
    uint32_t sum = 0;
    for (uint8_t j = 0; j < 50; j++) {
      sum += analogRead(sensor_pins[i]);
      delay(20);
    }
    line_min_values[i] = sum / 50;
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(line_min_values[i]);
  }
  
  Serial.println("\n[LineFollower] Place sensor over BLACK line. Press button in 5 seconds...");
  delay(5000);
  
  // Calibrate black (maximum values)
  Serial.println("[LineFollower] Reading BLACK values...");
  for (uint8_t i = 0; i < 5; i++) {
    uint32_t sum = 0;
    for (uint8_t j = 0; j < 50; j++) {
      sum += analogRead(sensor_pins[i]);
      delay(20);
    }
    line_max_values[i] = sum / 50;
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(line_max_values[i]);
  }
  
  Serial.println("[LineFollower] Calibration complete!");
}

// ====== DEBUG OUTPUT ======
void LineFollower_printDebug() {
  Serial.print("[Line] Raw: ");
  for (uint8_t i = 0; i < 5; i++) {
    Serial.print(line_sensor_raw[i]);
    Serial.print(" ");
  }
  Serial.print("| Cal: ");
  for (uint8_t i = 0; i < 5; i++) {
    Serial.print(line_sensor_calibrated[i]);
    Serial.print(" ");
  }
  Serial.print("| Error: ");
  Serial.print(line_position_error);
  Serial.print(" | Steering: ");
  Serial.println(line_steering_output);
}
