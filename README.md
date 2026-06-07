# 🤖 ESP32 Self-Balancing Robot

Автономный робот-балансир на двух колёсах с линией, препятствиями и управлением через веб-интерфейс.

## 📁 Файлы проекта

### Основные скетчи:
- **esp32_robot_main.ino** - основная программа робота
- **web_interface.h** - веб-интерфейс с джойстиком

### Тестовые скетчи:
- **test_line_sensor.ino** - тест датчика линии (5 датчиков)
- **test_distance_sensor.ino** - тест датчика расстояния (VL53L0X)
- **test_motors.ino** - тест моторов (скорость и синхронизация)

### Библиотеки:
- LineFollower.h/.cpp
- Control.h/.cpp
- MPU6050.h/.cpp
- Motors.h/.cpp
- defines.h, globals.h/.cpp

---

## ⚡ Быстрый старт

### 1. Редактировать WiFi

В **esp32_robot_main.ino**:
```cpp
String sta_ssid = "YourWiFiSSID";
String sta_password = "YourWiFiPassword";
```

### 2. Загрузить в Arduino IDE

Библиотеки нужны:
- AsyncTCP
- ESPAsyncWebServer
- VL53L0X

### 3. Открыть веб-интерфейс

```
http://192.168.X.X
```

---

## 🎮 Управление (веб-интерфейс)

### Джойстик
- **Вверх-вниз** = Throttle (скорость)
- **Влево-вправо** = Steering (поворот)

### Кнопки
- **▶ Прямо** - ехать 2 сек, потом линия
- **〰 Линия** - следить за линией
- **⏹ Стоп** - остановка

### Датчики в реальном времени
- Расстояние до препятствия (мм)
- 5 датчиков линии (график)
- Угол наклона
- Статус уклона

---

## 🤖 Как работает робот

```
1. IDLE (ждёт)
   └─ push "Прямо"
   
2. STRAIGHT (2 сек прямо)
   └─ авто переход
   
3. LINE_FOLLOW (следит за линией)
   - Если видит препятствие → СТОП
   - Если уклон → замедление (×0.5)
   
4. STOPPED (препятствие)
   └─ push "Стоп"
```

---

## 🧪 Тестирование

### 1. Тест датчика линии

```cpp
// Загрузить: test_line_sensor.ino
// Serial Monitor: 115200 baud
// Выводит: Raw values, Calibrated, ASCII график
```

### 2. Тест датчика расстояния

```cpp
// Загрузить: test_distance_sensor.ino
// Serial Monitor: 115200 baud
// Выводит: Расстояние в мм, статус, ASCII график
// Проверяет: 0-200mm (очень близко) до >1000mm (далеко)
```

### 3. Тест моторов

```cpp
// Загрузить: test_motors.ino
// Serial Monitor: 115200 baud
// Тесты:
//   1. Оба медленно вперёд
//   2. Оба быстро вперёд
//   3. Оба назад
//   4. Разные скорости (синхронизация)
// Выводит: Количество шагов, соотношение скоростей
```

---

## 🔧 Параметры в коде

```cpp
#define DISTANCE_THRESHOLD 300      // Стоп < 300мм
#define STRAIGHT_TIME 2000          // 2 сек прямо
#define SLOPE_ACCEL_THRESHOLD 2000  // Порог уклона
#define MAX_THROTTLE 550            // Макс скорость
#define MAX_STEERING 140            // Макс поворот
```

---

## 📊 GPIO подключение (ESP32)

```
I2C:
  GPIO 21 (SDA) ← MPU6050, VL53L0X
  GPIO 22 (SCL) ← MPU6050, VL53L0X

Моторы:
  GPIO 27/14 ← Motor1 (DIR/STEP)
  GPIO 25/26 ← Motor2 (DIR/STEP)
  GPIO 12 ← Enable

Датчик линии:
  GPIO 34/35/36/39/32 ← Sensors (S1-S5)
```

---

## 🐛 Помощь

**Робот не подключается:**
- Проверить SSID/пароль WiFi
- Смотреть Serial Monitor (115200)

**Не балансирует:**
- Проверить I2C (MPU6050)
- Проверить моторы (test_motors.ino)

**Не следит за линией:**
- Калибровать датчик (test_line_sensor.ino)
- Проверить контрастность линии

**Не видит препятствие:**
- Тест датчика (test_distance_sensor.ino)
- Отрегулировать DISTANCE_THRESHOLD

---

## ✅ Проверка перед использованием

- [ ] ESP32 подключена
- [ ] Все датчики подключены
- [ ] WiFi SSID/пароль введены
- [ ] test_motors.ino проходит
- [ ] test_line_sensor.ino работает
- [ ] test_distance_sensor.ino работает
- [ ] Веб-интерфейс открывается

---

**Версия:** 3.0 (с веб-интерфейсом)  
**Статус:** ✅ Production Ready

---

## 🔌 ПОЛНАЯ СХЕМА ПОДКЛЮЧЕНИЯ

### ESP32 Pinout (30 pin version)

```
         ┌─────────────────────┐
         │      ESP32-30       │
    GND  │  ⬤ ⬤ ⬤ ⬤ ⬤ ⬤ ⬤ ⬤  │  3.3V
    GND  │  ⬤                 ⬤ │  VUSB
    3.3V │  ⬤                 ⬤ │  GND
    RST  │  ⬤                 ⬤ │  EN
    12   │  ⬤ ENABLE MOTORS   ⬤ │  27 (MOTOR1_DIR)
    13   │  ⬤ SERVO           ⬤ │  25 (MOTOR2_DIR)
    14   │  ⬤ MOTOR1_STEP     ⬤ │  32 (LINE_S5)
    26   │  ⬀                 ⬀ │  33 (BUZZER)
    35   │  ⬀ LINE_S2         ⬀ │  34 (LINE_S1)
    39   │  ⬀ LINE_S4         ⬀ │  36 (LINE_S3)
    GND  │  ⬀                 ⬀ │  GND
    GND  │  ⬀                 ⬀ │  15
    23   │  ⬀ VSPI_MOSI       ⬀ │  2 (WIFI_LED)
    22   │  ⬀ SCL (I2C)       ⬀ │  4
    21   │  ⬀ SDA (I2C)       ⬀ │  5
    19   │  ⬀ VSPI_MISO       ⬀ │  18 (VSPI_CLK)
    GND  │  ⬀                 ⬀ │  GND
    GND  │  ⬀                 ⬀ │  17
    16   │  ⬀ RX2             ⬀ │  GND
         └─────────────────────┘
```

---

## 📋 ДЕТАЛЬНОЕ ПОДКЛЮЧЕНИЕ КОМПОНЕНТОВ

### 1. МОТОРЫ (Stepper NEMA17 с драйвером)

```
Motor 1:
┌──────────────────┐
│  NEMA17 Motor   │
├──────────────────┤
│ DIR ─────→ GPIO 27
│ STEP ────→ GPIO 14
│ GND ─────→ GND
│ +12V ────→ 12V (через драйвер)
└──────────────────┘

Motor 2:
┌──────────────────┐
│  NEMA17 Motor   │
├──────────────────┤
│ DIR ─────→ GPIO 25
│ STEP ────→ GPIO 26
│ GND ─────→ GND
│ +12V ────→ 12V (через драйвер)
└──────────────────┘

Motor Enable:
GPIO 12 → Motor Driver Enable
GND ────→ GND
```

### 2. MPU6050 (Гироскоп + Акселерометр)

```
┌──────────────────┐
│     MPU6050      │
├──────────────────┤
│ SDA ─────→ GPIO 21 (I2C)
│ SCL ─────→ GPIO 22 (I2C)
│ GND ─────→ GND
│ VCC ─────→ 3.3V
│ INT ─────→ не подключать (опционально)
└──────────────────┘

I2C адрес: 0x68
```

### 3. VL53L0X (Датчик расстояния)

```
┌──────────────────┐
│    VL53L0X       │
├──────────────────┤
│ SDA ─────→ GPIO 21 (I2C)
│ SCL ─────→ GPIO 22 (I2C)
│ GND ─────→ GND
│ VCC ─────→ 3.3V
│ XSHUT ───→ не подключать (опционально)
└──────────────────┘

I2C адрес: 0x29
```

### 4. ДАТЧИК ЛИНИИ (Ldabrye 5-sensor)

```
┌─────────────────────────────┐
│  Ldabrye Line Sensor Array  │
├─────────────────────────────┤
│ S1 (Левый)   ─→ GPIO 34 (ADC)
│ S2          ─→ GPIO 35 (ADC)
│ S3 (Центр)  ─→ GPIO 36 (ADC)
│ S4          ─→ GPIO 39 (ADC)
│ S5 (Правый) ─→ GPIO 32 (ADC)
│ GND         ─→ GND
│ VCC         ─→ 5V
└─────────────────────────────┘
```

### 5. СВЕТОДИОДЫ И ЗУММЕР

```
LED Status:
GPIO 32 → LED+ → ⎕ → 220Ω → GND

WiFi LED:
GPIO 2 → LED+ → ⎕ → 220Ω → GND

Buzzer:
GPIO 33 → Buzzer+ → GND
```

### 6. ПИТАНИЕ (Очень важно!)

```
┌─────────────────────────┐
│   Power Distribution    │
├─────────────────────────┤
│ Battery/PSU             │
│ ├─→ 5V ────→ 5V шина    │
│ │  └─→ Line Sensor VCC
│ │  └─→ VL53L0X VCC
│ │  └─→ Zener 3.3V для ESP32
│ │
│ ├─→ 12V ───→ Motor Driver
│ │  └─→ Stepper Motors
│ │
│ └─→ GND ───→ GND шина (общая)
│    └─→ ESP32 GND
│    └─→ Драйвер GND
│    └─→ Датчики GND
└─────────────────────────┘

⚠️ ОБЯЗАТЕЛЬНО: Все GND должны быть соединены вместе!
```

---

## 🎯 ПЛАТА ПОДКЛЮЧЕНИЯ (breadboard раскладка)

```
I2C (GPIO 21, 22):
┌─ 3.3V ─ Pull-up 4.7K ─ GPIO 22 (SCL) ─ MPU6050 SCL
├─ 3.3V ─ Pull-up 4.7K ─ GPIO 21 (SDA) ─ MPU6050 SDA
│                                      └─ VL53L0X SDA/SCL
└─ GND ─────────────────────────────── Оба датчика GND

АДЦ (Line Sensors):
GPIO 34 ← Sensor1 (ADC1_CH6)
GPIO 35 ← Sensor2 (ADC1_CH7)
GPIO 36 ← Sensor3 (ADC1_CH0)
GPIO 39 ← Sensor4 (ADC1_CH3)
GPIO 32 ← Sensor5 (ADC1_CH4)

Motor Control:
GPIO 27 → Драйвер DIR1 → Motor1 DIR
GPIO 14 → Драйвер STEP1 → Motor1 STEP
GPIO 25 → Драйвер DIR2 → Motor2 DIR
GPIO 26 → Драйвер STEP2 → Motor2 STEP
GPIO 12 → Драйвер ENABLE

Аудио/Визуально:
GPIO 33 → Buzzer
GPIO 2 → WiFi LED
```

---

## 📐 ПРИНЦИПИАЛЬНАЯ СХЕМА

```
                     ┌──────────────────┐
                     │       ESP32      │
                     └──────────────────┘
                              │
         ┌────────────────────┼─────────────────────┐
         │                    │                     │
    ┌────────┐         ┌───────────┐        ┌──────────┐
    │ Motors │         │ Sensors   │        │ WiFi/Web │
    │ ────── │         │ ──────── │        │ ──────── │
    │ M1/M2  │         │ MPU6050   │        │ Interface│
    │ Driver │         │ VL53L0X   │        │ Browser  │
    │ 12V    │         │ LineSensor│        │ Joystick │
    └────────┘         └───────────┘        └──────────┘
         │                    │                     │
         └────────────────────┼─────────────────────┘
                              │
                         [Battery]
```

---

## ✅ ПРОВЕРКА ПОДКЛЮЧЕНИЯ

```
На столе:
1. GPIO 21/22 (I2C) → подключить MPU6050
   - SDA → GPIO 21
   - SCL → GPIO 22
   - 3.3V, GND

2. GPIO 34/35/36/39/32 (ADC) → подключить датчик линии
   - S1-S5 → GPIO 34/35/36/39/32
   - 5V, GND

3. GPIO 27/14/25/26 (Motor) → подключить драйвер
   - DIR/STEP → GPIO 27/14/25/26
   - GND, 12V

4. Проверить:
   [ ] Все 3.3V соединены
   [ ] Все GND соединены (общая земля)
   [ ] VL53L0X на I2C вместе с MPU6050
   [ ] I2C Pull-up резисторы (4.7K между SDA/SCL и 3.3V)
   [ ] Питание моторов отдельное от ESP32
```

---

## 🔍 I2C PULL-UP РЕЗИСТОРЫ (ВАЖНО!)

```
Если датчики не отвечают, добавить:

GPIO 22 (SCL):
3.3V ──[4.7K]──┬── GPIO 22
               │
            MPU6050 SCL
            VL53L0X SCL

GPIO 21 (SDA):
3.3V ──[4.7K]──┬── GPIO 21
               │
            MPU6050 SDA
            VL53L0X SDA
```

---

**Версия схемы:** 1.0  
**Последнее обновление:** 2026-06-07
