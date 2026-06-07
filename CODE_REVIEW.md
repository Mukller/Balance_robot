# 🔍 CODE REVIEW - ESP32 Balancing Robot v2.0

**Статус:** ⚠️ **ТРЕБУЕТ ИСПРАВЛЕНИЙ** перед production

---

## 📊 ИТОГОВЫЙ ОТЧЁТ

| Категория | Статус | Заметки |
|-----------|--------|---------|
| **Безопасность** | ⚠️ | WiFi credentials hardcoded |
| **Корректность** | ⚠️ | Race conditions на volatile переменных |
| **Производительность** | ✅ | Хорошо оптимизирован |
| **Читаемость** | ✅ | Хороший код, с комментариями |
| **Управление памятью** | ⚠️ | String в JSON может быть проблематично |

---

## 🔴 КРИТИЧЕСКИЕ ПРОБЛЕМЫ

### 1. **Hardcoded WiFi credentials (SECURITY)**

**Файл:** `esp32_wifi_balancing_robot_v2.ino:42-43`
```cpp
String sta_ssid = "your_ssid";
String sta_password = "your_password";
```

**Проблема:** Credentials в исходном коде.

**Решение:**
```cpp
// Использовать #define или читать из EEPROM/Flash
#include <Preferences.h>

Preferences preferences;

void setup() {
  preferences.begin("robot", false);
  sta_ssid = preferences.getString("ssid", "default_ssid");
  sta_password = preferences.getString("pwd", "default_pwd");
  preferences.end();
}
```

---

### 2. **Race condition на volatile переменных (CORRECTNESS)**

**Файл:** `esp32_wifi_balancing_robot_v2.ino:56,61,67`
```cpp
volatile uint8_t robot_state = STATE_IDLE;
volatile bool on_slope = false;
volatile bool emergency_stop = false;
```

**Проблема:** 
- WiFi обработчик пишет в `robot_state` без синхронизации
- `controlLoop()` читает в прерывании таймера
- Может быть torn read/write

**Решение:**
```cpp
// Использовать портативный spinlock или semaphore
void handleStateChange(uint8_t new_state) {
  taskENTER_CRITICAL(&timerMux);  // ESP32 macro
  robot_state = new_state;
  taskEXIT_CRITICAL(&timerMux);
}
```

---

### 3. **String конкатенация в JSON (PERFORMANCE & MEMORY)**

**Файл:** `esp32_wifi_balancing_robot_v2.ino:219-229`
```cpp
String response = "{\"status\":\"ok\",\"state\":";
response += String(robot_state);
response += ",\"distance_mm\":";
// ... 5 раз += String(...)
```

**Проблема:** 
- Каждый `+=` может вызвать реаллокацию
- 6+ выделений памяти на один запрос
- На устройстве с 520KB RAM это проблема

**Решение:**
```cpp
// Вариант 1: Использовать snprintf()
char response[256];
snprintf(response, sizeof(response),
  "{\"status\":\"ok\",\"state\":%d,\"distance_mm\":%d,\"on_slope\":%d,\"throttle\":%d,\"angle\":%.1f}",
  robot_state, distance_mm, on_slope, throttle, angle_adjusted);
request->send(200, "application/json", response);

// Вариант 2: Использовать DynamicJsonDocument (ArduinoJson)
DynamicJsonDocument doc(256);
doc["status"] = "ok";
doc["state"] = robot_state;
String response;
serializeJson(doc, response);
request->send(200, "application/json", response);
```

---

## ⚠️ ВЫСОКИЙ ПРИОРИТЕТ

### 4. **Блокирующее задержка в инициализации (CORRECTNESS)**

**Файл:** `esp32_wifi_balancing_robot_v2.ino:85-90`
```cpp
for (int i = 0; i < 50; i++) {
  MPU6050_read_3axis();
  z_sum += accel_t_gyro.value.z_accel;
  delay(10);  // ← 500ms блокирует весь chip!
}
```

**Проблема:** 
- WiFi инициализация может быть заблокирована
- Может привести к timeout'ам

**Решение:**
```cpp
// Использовать vTaskDelay вместо delay
for (int i = 0; i < 50; i++) {
  MPU6050_read_3axis();
  z_sum += accel_t_gyro.value.z_accel;
  vTaskDelay(pdMS_TO_TICKS(10));  // yield to other tasks
}
```

---

### 5. **Отсутствует обработка ошибок инициализации (CORRECTNESS)**

**Файл:** `esp32_wifi_balancing_robot_v2.ino:177-195`

```cpp
initMPU6050();        // Может заваливаться молча
initDistanceSensor(); // Может заваливаться молча
LineFollower_init();  // Может заваливаться молча
```

**Проблема:** Функции ничего не возвращают. Если сенсор не инициализировался - код продолжает работать!

**Решение:**
```cpp
bool initMPU6050() {
  MPU6050_setup();
  // ... код ...
  return true; // или false
}

// В setup():
if (!initMPU6050()) {
  Serial.println("[FATAL] MPU6050 init failed!");
  while(1) delay(1);  // Зависнуть, чтобы дать знать об ошибке
}
```

---

### 6. **Недостаточное ограничение значений (CORRECTNESS)**

**Файл:** `esp32_wifi_balancing_robot_v2.ino:356-357`
```cpp
steering = constrain(steering, -max_steering, max_steering);
throttle = constrain(throttle, -max_throttle, max_throttle);
```

**Проблема:** Но `control_output` не ограничивается перед отправкой на моторы!

**Решение:**
```cpp
control_output = constrain(control_output, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);
steering = constrain(steering, -max_steering, max_steering);
throttle = constrain(throttle, -max_throttle, max_throttle);
```

---

## 📋 СРЕДНИЙ ПРИОРИТЕТ

### 7. **Переполнение счётчика времени (CORRECTNESS)**

**Файл:** `esp32_wifi_balancing_robot_v2.ino:323`
```cpp
if (millis() - state_start_time > STRAIGHT_TIME) {
```

**Проблема:** `millis()` переполняется каждые ~49 дней. После этого логика сломается!

**Решение:**
```cpp
// Правильная проверка
if ((unsigned long)(millis() - state_start_time) > STRAIGHT_TIME) {
```

---

### 8. **Опасное приведение типов (CORRECTNESS)**

**Файл:** `esp32_wifi_balancing_robot_v2.ino:249`
```cpp
int16_t z_delta = abs(current_z - z_accel_baseline);
```

**Проблема:** `abs()` работает с `int`, а не `int16_t`. Потенциальное переполнение!

**Решение:**
```cpp
int16_t z_delta = abs((int32_t)current_z - z_accel_baseline);
// или
int16_t z_delta = (current_z > z_accel_baseline) ? 
                   (current_z - z_accel_baseline) : 
                   (z_accel_baseline - current_z);
```

---

### 9. **Отсутствует синхронизация на глобальном состоянии (CORRECTNESS)**

**Файл:** `LineFollower.cpp:38-50`
```cpp
static uint16_t sample_buffer[5][LINE_SENSOR_SAMPLES];
static uint8_t sample_index = 0;
```

**Проблема:** Если `LineFollower_readSensors()` вызывается из разных контекстов (основной loop + WiFi обработчик), может быть race condition на `sample_index`.

**Решение:**
```cpp
// Переделать как ring buffer с atomic index
// или убедиться вызывается только из одного контекста
```

---

## ✅ ЧТО ХОРОШО

1. **Хорошая структура кода** - разделение на функции, ясные имена
2. **Хороший state machine** - четыре состояния, легко следить
3. **Комментарии** - есть где нужно, не переборщено
4. **Правильная иерархия включаемых файлов** - нет циклических зависимостей
5. **Использование volatile для ISR переменных** - в целом правильно (хотя синхронизация нужна)
6. **Калибровка Z-baseline** - умная идея для детекции уклона

---

## 🎯 ПЛАН ИСПРАВЛЕНИЙ

### Urgency 1 (CRITICAL - исправить сейчас):
- [ ] Убрать hardcoded WiFi credentials
- [ ] Добавить обработку ошибок инициализации
- [ ] Исправить race conditions

### Urgency 2 (HIGH - исправить перед production):
- [ ] Заменить String на snprintf/JSON library
- [ ] Исправить типы данных (abs на int32_t)
- [ ] Добавить ограничения на control_output
- [ ] Исправить overflow millis()

### Urgency 3 (MEDIUM - рекомендуется):
- [ ] Использовать vTaskDelay вместо delay
- [ ] Добавить logging/telemetry
- [ ] Добавить unit tests
- [ ] Документировать API

---

## 📈 МЕТРИКИ КОДА

| Метрика | Значение | Норма |
|---------|----------|-------|
| Циклическая сложность | Низкая ✅ | < 10 |
| Lines per function | ~30-40 | < 50 |
| Global variables | 15+ ⚠️ | < 5 |
| Stack depth | ? | < 8KB |
| Memory usage | ? | < 256KB (half) |

---

## 🧪 РЕКОМЕНДУЕМЫЕ ТЕСТЫ

```cpp
// Unit tests:
✓ detectSlope() при разных Z значениях
✓ LineFollower калибровка граничные значения
✓ State transitions с таймерами
✓ Distance sensor threshold

// Integration tests:
✓ WiFi + Motor control одновременно
✓ Long-running (8+ часов) на флюктуирующих значениях
✓ Emergency stop -> recovery
✓ Obstacle detection + line follow
```

---

## 📝 ВЫВОД

**Код РАБОЧИЙ, но требует исправлений для production:**

1. **Безопасность:** Убрать hardcoded credentials
2. **Надёжность:** Добавить synchronization и error handling
3. **Performance:** Оптимизировать JSON и memory allocation

**Рекомендация:** 
- ✅ Использовать для тестирования и разработки
- ⚠️ Перед продакшеном применить все исправления из CRITICAL
- 🟢 После исправлений готово для реального использования

---

**Проверено:** 2026-06-07  
**Уровень ревью:** HIGH (production-ready)  
**Статус:** ⚠️ NEEDS FIXES
