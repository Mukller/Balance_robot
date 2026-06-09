// TEST: Сервомоторы захвата
// Тестирует две серво на GPIO 13 и GPIO 33
// Поворачивает их по очереди: 0° → 180° → 0°

#include <Servo.h>

#define SERVO_LEFT_PIN 13
#define SERVO_RIGHT_PIN 33

Servo servoLeft, servoRight;

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== SERVO GRIPPER TEST ===");
  Serial.println("Testing two servo motors:");
  Serial.println("  Left Servo  → GPIO 13");
  Serial.println("  Right Servo → GPIO 33");

  // Инициализировать серво
  Serial.println("\n[INIT] Attaching servos...");
  servoLeft.attach(SERVO_LEFT_PIN);
  servoRight.attach(SERVO_RIGHT_PIN);
  delay(500);

  // Начальная позиция - 0°
  servoLeft.write(0);
  servoRight.write(0);
  Serial.println("[OK] Servos initialized at 0°");
}

void loop() {
  // Тест 1: Левая серво 0° → 180° → 0°
  Serial.println("\n--- TEST 1: LEFT SERVO ---");
  testServo(servoLeft, "LEFT", SERVO_LEFT_PIN);

  delay(2000);

  // Тест 2: Правая серво 0° → 180° → 0°
  Serial.println("\n--- TEST 2: RIGHT SERVO ---");
  testServo(servoRight, "RIGHT", SERVO_RIGHT_PIN);

  delay(2000);

  // Тест 3: Обе серво вместе (захват работает)
  Serial.println("\n--- TEST 3: BOTH SERVOS (OPEN/CLOSE) ---");
  Serial.println("[ACTION] Opening gripper (180°)...");
  servoLeft.write(180);
  servoRight.write(180);
  printServoAngle(180);
  delay(1000);

  Serial.println("[ACTION] Closing gripper (90°)...");
  servoLeft.write(90);
  servoRight.write(90);
  printServoAngle(90);
  delay(1000);

  Serial.println("[ACTION] Reset to 0°...");
  servoLeft.write(0);
  servoRight.write(0);
  printServoAngle(0);
  delay(2000);
}

void testServo(Servo servo, const char* name, int pin) {
  // Поворот на 180°
  Serial.print("[ACTION] ");
  Serial.print(name);
  Serial.println(" servo → 180°");
  servo.write(180);
  printServoAngle(180);
  delay(1500);

  // Возврат на 0°
  Serial.print("[ACTION] ");
  Serial.print(name);
  Serial.println(" servo → 0°");
  servo.write(0);
  printServoAngle(0);
  delay(1500);

  Serial.print("[OK] ");
  Serial.print(name);
  Serial.println(" servo test completed");
}

void printServoAngle(int angle) {
  Serial.print("         ");
  int bars = map(angle, 0, 180, 0, 20);
  for (int i = 0; i < bars; i++) Serial.print("█");
  Serial.print(" ");
  Serial.print(angle);
  Serial.println("°");
}
