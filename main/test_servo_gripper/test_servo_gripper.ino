// TEST: Сервомоторы захвата (ESP32 PWM)
// Тестирует две серво на GPIO 13 и GPIO 33 с использованием встроенной PWM
// Поворачивает их по очереди: 0° → 180° → 0°

// ESP32 PWM параметры для сервомоторов
#define SERVO_LEFT_PIN 13
#define SERVO_RIGHT_PIN 33
#define PWM_FREQ 50        // 50 Hz для сервомоторов
#define PWM_CHANNEL_LEFT 0
#define PWM_CHANNEL_RIGHT 1
#define PWM_RESOLUTION 16  // 16-bit resolution

// Функции для конвертирования углов в PWM значения
// Servo: 0° = ~1000us, 90° = ~1500us, 180° = ~2000us
// На 50Hz и 16-bit: max value = 2^16 = 65535
// 1 период = 20ms = 65535 ticks
// 1000us = 3277, 1500us = 4915, 2000us = 6553
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180
#define SERVO_MIN_PULSE 1000  // микросекунды
#define SERVO_MAX_PULSE 2000  // микросекунды

int angleToPWM(int angle) {
  // Конвертировать угол (0-180) в PWM pulse (1000-2000us)
  // Для ESP32 50Hz PWM (20ms period, 65535 ticks):
  // 1ms = 3277 ticks, 2ms = 6553 ticks
  int pulse = SERVO_MIN_PULSE + (angle * (SERVO_MAX_PULSE - SERVO_MIN_PULSE)) / 180;
  // Конвертировать микросекунды в PWM ticks
  // 20ms = 65535 ticks, поэтому 1us = 3.277 ticks
  return pulse * 3.277;
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== SERVO GRIPPER TEST (ESP32 PWM) ===");
  Serial.println("Testing two servo motors:");
  Serial.println("  Left Servo  → GPIO 13");
  Serial.println("  Right Servo → GPIO 33");
  Serial.println("Using ESP32 built-in PWM (50Hz)");

  // Инициализировать PWM каналы для ESP32
  Serial.println("\n[INIT] Setting up PWM channels...");

  // Левая серво
  ledcSetup(PWM_CHANNEL_LEFT, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(SERVO_LEFT_PIN, PWM_CHANNEL_LEFT);

  // Правая серво
  ledcSetup(PWM_CHANNEL_RIGHT, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(SERVO_RIGHT_PIN, PWM_CHANNEL_RIGHT);

  delay(500);

  // Начальная позиция - 0°
  setServo(PWM_CHANNEL_LEFT, 0);
  setServo(PWM_CHANNEL_RIGHT, 0);
  Serial.println("[OK] Servos initialized at 0°");
  delay(1000);
}

void loop() {
  // Тест 1: Левая серво 0° → 180° → 0°
  Serial.println("\n--- TEST 1: LEFT SERVO ---");
  testServo(PWM_CHANNEL_LEFT, "LEFT");
  delay(2000);

  // Тест 2: Правая серво 0° → 180° → 0°
  Serial.println("\n--- TEST 2: RIGHT SERVO ---");
  testServo(PWM_CHANNEL_RIGHT, "RIGHT");
  delay(2000);

  // Тест 3: Обе серво вместе (захват работает)
  Serial.println("\n--- TEST 3: BOTH SERVOS (OPEN/CLOSE) ---");

  Serial.println("[ACTION] Opening gripper (180°)...");
  setServo(PWM_CHANNEL_LEFT, 180);
  setServo(PWM_CHANNEL_RIGHT, 180);
  printServoAngle(180);
  delay(1500);

  Serial.println("[ACTION] Closing gripper (90°)...");
  setServo(PWM_CHANNEL_LEFT, 90);
  setServo(PWM_CHANNEL_RIGHT, 90);
  printServoAngle(90);
  delay(1500);

  Serial.println("[ACTION] Reset to 0°...");
  setServo(PWM_CHANNEL_LEFT, 0);
  setServo(PWM_CHANNEL_RIGHT, 0);
  printServoAngle(0);
  delay(2000);
}

void setServo(int channel, int angle) {
  // Ограничить угол от 0 до 180
  angle = constrain(angle, 0, 180);
  int pwmValue = angleToPWM(angle);
  ledcWrite(channel, pwmValue);
}

void testServo(int channel, const char* name) {
  // Поворот на 180°
  Serial.print("[ACTION] ");
  Serial.print(name);
  Serial.println(" servo → 180°");
  setServo(channel, 180);
  printServoAngle(180);
  delay(1500);

  // Возврат на 0°
  Serial.print("[ACTION] ");
  Serial.print(name);
  Serial.println(" servo → 0°");
  setServo(channel, 0);
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
