// TEST: Сервомоторы захвата (ESP32 PWM)
// Тестирует две серво на GPIO 13 и GPIO 33 с использованием встроенной PWM
// Поворачивает их по очереди: 0° → 180° → 0°

#define SERVO_LEFT_PIN 13
#define SERVO_RIGHT_PIN 33
#define PWM_FREQ 50        // 50 Hz для сервомоторов
#define PWM_RESOLUTION 16  // 16-bit resolution

// Переменные для PWM каналов (для новой версии ESP32)
int pwmLeftChannel;
int pwmRightChannel;

// Функция для конвертирования углов в PWM значение
int angleToPWM(int angle) {
  // 0° = 1000us, 180° = 2000us
  // На 50Hz и 16-bit: 1us = 3.277 ticks
  int pulse = 1000 + (angle * 1000) / 180;
  return (int)(pulse * 3.277);
}

void setServoAngle(int pin, int channel, int angle) {
  angle = constrain(angle, 0, 180);
  int pwmValue = angleToPWM(angle);
  ledcWrite(channel, pwmValue);
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== SERVO GRIPPER TEST (ESP32 PWM) ===");
  Serial.println("Testing two servo motors:");
  Serial.println("  Left Servo  → GPIO 13");
  Serial.println("  Right Servo → GPIO 33");
  Serial.println("Using ESP32 built-in PWM (50Hz)");

  // Инициализировать PWM каналы для ESP32 (версия 2.0+)
  Serial.println("\n[INIT] Setting up PWM channels...");

  // Левая серво на GPIO 13
  pwmLeftChannel = ledcCreateChannel(PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(SERVO_LEFT_PIN, PWM_FREQ, PWM_RESOLUTION);

  // Правая серво на GPIO 33
  pwmRightChannel = ledcCreateChannel(PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(SERVO_RIGHT_PIN, PWM_FREQ, PWM_RESOLUTION);

  delay(500);

  // Начальная позиция - 0°
  setServoAngle(SERVO_LEFT_PIN, pwmLeftChannel, 0);
  setServoAngle(SERVO_RIGHT_PIN, pwmRightChannel, 0);
  Serial.println("[OK] Servos initialized at 0°");
  delay(1000);
}

void loop() {
  // Тест 1: Левая серво 0° → 180° → 0°
  Serial.println("\n--- TEST 1: LEFT SERVO ---");
  testServo(SERVO_LEFT_PIN, pwmLeftChannel, "LEFT");
  delay(2000);

  // Тест 2: Правая серво 0° → 180° → 0°
  Serial.println("\n--- TEST 2: RIGHT SERVO ---");
  testServo(SERVO_RIGHT_PIN, pwmRightChannel, "RIGHT");
  delay(2000);

  // Тест 3: Обе серво вместе (захват работает)
  Serial.println("\n--- TEST 3: BOTH SERVOS (OPEN/CLOSE) ---");

  Serial.println("[ACTION] Opening gripper (180°)...");
  setServoAngle(SERVO_LEFT_PIN, pwmLeftChannel, 180);
  setServoAngle(SERVO_RIGHT_PIN, pwmRightChannel, 180);
  printServoAngle(180);
  delay(1500);

  Serial.println("[ACTION] Closing gripper (90°)...");
  setServoAngle(SERVO_LEFT_PIN, pwmLeftChannel, 90);
  setServoAngle(SERVO_RIGHT_PIN, pwmRightChannel, 90);
  printServoAngle(90);
  delay(1500);

  Serial.println("[ACTION] Reset to 0°...");
  setServoAngle(SERVO_LEFT_PIN, pwmLeftChannel, 0);
  setServoAngle(SERVO_RIGHT_PIN, pwmRightChannel, 0);
  printServoAngle(0);
  delay(2000);
}

void testServo(int pin, int channel, const char* name) {
  // Поворот на 180°
  Serial.print("[ACTION] ");
  Serial.print(name);
  Serial.println(" servo → 180°");
  setServoAngle(pin, channel, 180);
  printServoAngle(180);
  delay(1500);

  // Возврат на 0°
  Serial.print("[ACTION] ");
  Serial.print(name);
  Serial.println(" servo → 0°");
  setServoAngle(pin, channel, 0);
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
