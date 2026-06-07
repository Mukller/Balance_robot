# 🚀 ESP32 Balancing Robot с датчиком линии

## 📚 Что было добавлено

Полная система отслеживания чёрной линии на основе датчика **Ldabrye Grayscale Line Follower (5-сенсорный)**

### Новые файлы:
1. ✅ **LineFollower.h** - заголовочный файл с декларациями
2. ✅ **LineFollower.cpp** - полная реализация алгоритма
3. 📋 Этот файл (документация)

---

## 🔧 Быстрый старт (5 минут)

### Шаг 1: Подключение датчика

```
Ldabrye 5-Pin Sensor   →   ESP32
─────────────────────────────────
S1 (Левый)             →   GPIO 34
S2                     →   GPIO 35  
S3 (Центр)             →   GPIO 36
S4                     →   GPIO 39
S5 (Правый)            →   GPIO 32
GND                    →   GND
VCC (5V)               →   5V (через стабилизатор!)
```

### Шаг 2: Добавить в основной скетч

**В начало файла (с includes):**
```cpp
#include "LineFollower.h"
```

**В setup():**
```cpp
void setup() {
  // ... существующий код ...
  
  LineFollower_init();  // Инициализировать датчик
  
  // Раскомментировать ПЕРВЫЙ РАЗ для калибровки:
  // LineFollower_calibrate();
}
```

**В 100Hz контроль loop:**
```cpp
// Получить команду поворота от датчика линии
if (line_mode != LINE_MODE_OFF) {
  int16_t line_steering = LineFollower_getSteering();
  steering = steering + (line_steering * 0.3);  // 0.3 = вес влияния
  steering = constrain(steering, -max_steering, max_steering);
}
```

### Шаг 3: Калибровка

**Вариант 1 - Автоматическая (в коде):**
```cpp
LineFollower_calibrate();
// Следовать инструкциям в Serial Monitor
```

---

## 🎮 Режимы управления

| Команда | Режим | Что происходит |
|---------|-------|--------|
| `line_mode=0` | OFF | Датчик отключен (нормальный режим) |
| `line_mode=1` | ON | Активное следование за линией |
| `line_mode=2` | AUTO | Автоматическое смешивание баланса+линия |

---

## ⚙️ Параметры для настройки

### 1. Чувствительность пропорционального регулятора
**Файл:** `LineFollower.cpp`, функция `LineFollower_getSteering()`
```cpp
float Kp_line = 0.5;  // Увеличить = более агрессивный поворот
```

### 2. Вес влияния датчика
**Файл:** основной скетч
```cpp
steering = steering + (line_steering * 0.3);  // 0-1 регулирует влияние
```

---

## 🔍 Отладка

### Включить вывод в Serial Monitor:
```cpp
if (loop_counter % 10 == 0) {
  LineFollower_printDebug();
}
```

### Пример вывода:
```
[Line] Raw: 450 780 3200 950 400 | Cal: 12 34 198 45 11 | Error: 145 | Steering: 72
```

---

## 🐛 Основные проблемы и решения

### Датчик не реагирует
- Проверить питание (5V, не 3.3V!)
- Проверить GND подключение
- Проверить GPIO номера (ADC only пины)

### Робот не следит за линией
- Увеличить Kp_line (0.5 → 0.7)
- Увеличить line_weight (0.3 → 0.5)
- Снизить скорость робота

### Робот теряет баланс
- Уменьшить line_weight (0.3 → 0.2)
- Уменьшить Kp_line (0.5 → 0.3)
- Использовать режим AUTO

### Робот осциллирует
- Уменьшить Kp_line
- Увеличить line_weight
- Снизить скорость

---

## 📋 Функции API

```cpp
void LineFollower_init();
void LineFollower_readSensors();
int16_t LineFollower_calculateError();
int16_t LineFollower_getSteering();
void LineFollower_setMode(uint8_t mode);  // 0=OFF, 1=ON, 2=AUTO
uint8_t LineFollower_getMode();
void LineFollower_calibrate();
void LineFollower_printDebug();
```

### Глобальные переменные:
```cpp
extern int16_t line_sensor_raw[5];           // 0-4095
extern int16_t line_sensor_calibrated[5];    // 0-200
extern int16_t line_position_error;          // -200 to +200
extern int16_t line_steering_output;         // -200 to +200
extern uint8_t line_mode;                    // 0/1/2
```

---

## 💡 Советы для лучших результатов

1. **Линия должна быть чёткой:** чёрная лента на белом
2. **Ширина линии:** 3-5см (оптимально для 5 датчиков)
3. **Расстояние датчика:** 0.5-1см над линией
4. **Поверхность:** ровная, без глубоких изгибов
5. **Скорость:** начать с 10 см/сек, потом увеличивать
6. **Питание:** отдельное 5V для датчика
7. **Освещение:** лучше в тени (избегать прямого солнца)

Успехов с вашим роботом! 🚀
