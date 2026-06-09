// DIAGNOSTIC: Сырое чтение 5 пинов датчика линии
// Цель: найти пин, который "висит" (не меняет значение)
// Без калибровки и усреднения - чистый analogRead
//
// Как пользоваться:
//   1. Загрузить, открыть Serial Monitor (115200)
//   2. Водить датчиком над чёрным/белым (или закрывать пальцем)
//   3. Смотреть колонку RANGE = max - min:
//        RANGE большой  -> пин живой, реагирует
//        RANGE ~0       -> пин ВИСИТ (нет подтяжки / нет контакта / мёртвый канал)
//   4. Команда 'r' + Enter в Serial Monitor -> сброс min/max

// Пины датчика линии (как в проекте)
#define S1_PIN 34   // GPIO 34 (ADC1_CH6) - край,  input-only, БЕЗ подтяжки
#define S2_PIN 35   // GPIO 35 (ADC1_CH7) -        input-only, БЕЗ подтяжки
#define S3_PIN 36   // GPIO 36 (ADC1_CH0) - центр, input-only, БЕЗ подтяжки (VP)
#define S4_PIN 39   // GPIO 39 (ADC1_CH3) -        input-only, БЕЗ подтяжки (VN)
#define S5_PIN 32   // GPIO 32 (ADC1_CH4) - край,  есть внутренняя подтяжка

const uint8_t pins[5] = { S1_PIN, S2_PIN, S3_PIN, S4_PIN, S5_PIN };
const char*   names[5] = { "S1(34)", "S2(35)", "S3(36)", "S4(39)", "S5(32)" };

int raw[5];
int minv[5];
int maxv[5];

void resetMinMax() {
  for (uint8_t i = 0; i < 5; i++) {
    minv[i] = 4095;
    maxv[i] = 0;
  }
  Serial.println("[RESET] min/max cleared");
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== LINE SENSOR PIN DIAGNOSTIC ===");
  Serial.println("Pins: S1=34 S2=35 S3=36 S4=39 S5=32");

  analogSetWidth(12);              // 0..4095
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);  // полный диапазон ~0..3.3V

  resetMinMax();
  Serial.println("Move sensor over black/white. RANGE~0 means the pin is STUCK.");
  Serial.println("Send 'r' to reset min/max.\n");
  delay(500);
}

void loop() {
  // Сброс по команде 'r'
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'r' || c == 'R') resetMinMax();
  }

  // Читать сырые значения и обновлять min/max
  for (uint8_t i = 0; i < 5; i++) {
    raw[i] = analogRead(pins[i]);
    if (raw[i] < minv[i]) minv[i] = raw[i];
    if (raw[i] > maxv[i]) maxv[i] = raw[i];
  }

  // Вывод по каждому пину
  for (uint8_t i = 0; i < 5; i++) {
    int range = maxv[i] - minv[i];

    Serial.print(names[i]);
    Serial.print(" raw=");   printPad(raw[i]);
    Serial.print(" min=");   printPad(minv[i]);
    Serial.print(" max=");   printPad(maxv[i]);
    Serial.print(" RANGE="); printPad(range);

    // Вердикт: реагирует пин или висит
    if (range < 100) Serial.print("  <-- STUCK?");
    else             Serial.print("  OK");

    Serial.print("   ");
  }
  Serial.println();

  delay(150);
}

// Выровнять число до 4 знаков для читаемости
void printPad(int v) {
  if (v < 1000) Serial.print(' ');
  if (v < 100)  Serial.print(' ');
  if (v < 10)   Serial.print(' ');
  Serial.print(v);
}
