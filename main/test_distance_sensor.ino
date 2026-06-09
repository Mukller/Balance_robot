// TEST: Ультразвуковой датчик расстояния HY-SRF05
// Измеряет расстояние до препятствия с использованием звуковых волн

// Пины датчика
#define SENSOR_TRIG 4   // Триггер (отправляем импульс)
#define SENSOR_ECHO 5   // Эхо (получаем отражение)

unsigned long pulse_duration = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== DISTANCE SENSOR TEST (HY-SRF05) ===");
  Serial.println("Ultrasonic sensor: TRIG=GPIO4, ECHO=GPIO5");

  // Инициализировать пины
  pinMode(SENSOR_TRIG, OUTPUT);
  pinMode(SENSOR_ECHO, INPUT);
  digitalWrite(SENSOR_TRIG, LOW);
  delay(100);

  Serial.println("[OK] HY-SRF05 initialized!");
  Serial.println("Distance measurements (mm):");
  Serial.println("---");
}

// Функция чтения расстояния
uint16_t readDistance() {
  // Отправить импульс 10 микросекунд на TRIG
  digitalWrite(SENSOR_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(SENSOR_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(SENSOR_TRIG, LOW);

  // Ждём импульса на ECHO (макс 30мс для ~5 метров)
  // pulseIn возвращает время в микросекундах
  pulse_duration = pulseIn(SENSOR_ECHO, HIGH, 30000);

  // Преобразование: расстояние = время / 5.8 (мм)
  // Формула: distance (см) = pulse_duration / 58
  //          distance (мм) = pulse_duration / 5.8
  if (pulse_duration > 0) {
    uint16_t dist_mm = (pulse_duration / 5.8);
    return constrain(dist_mm, 0, 4000);  // Макс ~4 метра
  }
  return 0;
}

void loop() {
  uint16_t distance = readDistance();

  // Вывести значение
  Serial.print("[Distance] ");
  Serial.print(distance);
  Serial.print(" mm");

  // Статус
  if (pulse_duration == 0) {
    Serial.print(" [NO ECHO - out of range]");
  } else {
    // График расстояния (каждые 50мм = один блок)
    Serial.print(" | ");
    int bars = constrain(distance / 50, 0, 20);
    for (int i = 0; i < bars; i++) Serial.print("█");
    Serial.print(" ");

    // Интерпретация расстояния
    if (distance < 200) {
      Serial.print("[VERY CLOSE - 0-20cm]");
    } else if (distance < 500) {
      Serial.print("[CLOSE - 20-50cm]");
    } else if (distance < 1000) {
      Serial.print("[MEDIUM - 50-100cm]");
    } else if (distance < 2000) {
      Serial.print("[FAR - 100-200cm]");
    } else {
      Serial.print("[VERY FAR - >200cm]");
    }
  }

  Serial.println();
  delay(200);  // 200ms между измерениями
