#ifndef LINE_FOLLOWER_H_
#define LINE_FOLLOWER_H_

#include <Arduino.h>

// Ldabrye Grayscale Line Follower Tracking Sensor
// 5 sensor pins (usually analog)
// Pin configuration - adjust based on your ESP32
#define LINE_SENSOR_1 34   // GPIO 34 (ADC1_CH6) - leftmost sensor
#define LINE_SENSOR_2 35   // GPIO 35 (ADC1_CH7)
#define LINE_SENSOR_3 36   // GPIO 36 (ADC1_CH0) - center sensor
#define LINE_SENSOR_4 39   // GPIO 39 (ADC1_CH3)
#define LINE_SENSOR_5 32   // GPIO 32 (ADC1_CH4) - rightmost sensor

// Line follower configuration
#define LINE_THRESHOLD 2000      // Threshold to distinguish black from white
#define LINE_SENSOR_SAMPLES 5    // Number of samples for averaging
#define LINE_MAX_ERROR 200       // Max steering correction

// Line follower modes
#define LINE_MODE_OFF 0
#define LINE_MODE_ON 1
#define LINE_MODE_AUTO 2

// Function declarations
void LineFollower_init();
void LineFollower_readSensors();
int16_t LineFollower_calculateError();
int16_t LineFollower_getSteering();
void LineFollower_setMode(uint8_t mode);
uint8_t LineFollower_getMode();
void LineFollower_calibrate();
void LineFollower_printDebug();

// Extern variables for sensor values
extern int16_t line_sensor_raw[5];
extern int16_t line_sensor_calibrated[5];
extern int16_t line_position_error;    // -200 to +200 (left to right offset from center)
extern int16_t line_steering_output;   // Steering command based on line position
extern uint8_t line_mode;              // Current line following mode
extern uint16_t line_min_values[5];    // Calibration min values (white)
extern uint16_t line_max_values[5];    // Calibration max values (black)

#endif /* LINE_FOLLOWER_H_ */
