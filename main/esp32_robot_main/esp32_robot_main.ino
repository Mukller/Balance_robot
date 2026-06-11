// ТЕСТ DIR ПИНА ДЛЯ МОТОРА 2
#include <Arduino.h>
#include "defines.h"

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== MOTOR2 DIR PIN TEST ===");
  
  pinMode(PIN_MOTOR2_DIR, OUTPUT);
  pinMode(PIN_MOTOR2_STEP, OUTPUT);
  pinMode(PIN_ENABLE_MOTORS, OUTPUT);
  digitalWrite(PIN_ENABLE_MOTORS, LOW);
  
  Serial.println("\nTest 1: DIR = HIGH, should spin FORWARD");
  digitalWrite(PIN_MOTOR2_DIR, HIGH);
  for (int i = 0; i < 500; i++) {
    digitalWrite(PIN_MOTOR2_STEP, HIGH);
    delayMicroseconds(500);
    digitalWrite(PIN_MOTOR2_STEP, LOW);
    delayMicroseconds(500);
  }
  delay(1000);
  
  Serial.println("Test 2: DIR = LOW, should spin BACKWARD");
  digitalWrite(PIN_MOTOR2_DIR, LOW);
  for (int i = 0; i < 500; i++) {
    digitalWrite(PIN_MOTOR2_STEP, HIGH);
    delayMicroseconds(500);
    digitalWrite(PIN_MOTOR2_STEP, LOW);
    delayMicroseconds(500);
  }
  
  Serial.println("\n=== TEST COMPLETE ===");
  Serial.println("If motor spins SAME direction in both tests:");
  Serial.println("1. DIR pin not connected properly");
  Serial.println("2. DIR pin damaged");
  Serial.println("3. Driver DIR input not working");
}

void loop() {}