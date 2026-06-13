<div align="center">

[![License: MIT](https://img.shields.io/badge/License-MIT-purple?style=flat-square)](LICENSE.md)
[![maintained](https://img.shields.io/badge/maintained%3F-yes-green?style=flat-square)](https://github.com/Mukller/REPO_NAME)
[![contributions welcome](https://img.shields.io/badge/contributions-welcome-brightgreen?style=flat-square)](CONTRIBUTING.md)

</div>

---

<input type="radio" id="lang-en" name="language" checked style="display:none;">
<input type="radio" id="lang-ru" name="language" style="display:none;">

<div style="text-align: center; margin: 20px 0;">
  <label for="lang-en" style="display: inline-block; padding: 12px 24px; background: #58A6FF; color: white; border-radius: 6px 0 0 6px; cursor: pointer; font-weight: bold; margin-right: -4px; user-select: none;">English</label><label for="lang-ru" style="display: inline-block; padding: 12px 24px; background: #30363D; color: white; border-radius: 0 6px 6px 0; cursor: pointer; font-weight: bold; user-select: none;">Русский</label>
</div>

<style>
  input[type="radio"] { display: none !important; }
  #lang-en:checked ~ div.lang-en { display: block; }
  #lang-en:checked ~ div.lang-ru { display: none; }
  #lang-ru:checked ~ div.lang-en { display: none; }
  #lang-ru:checked ~ div.lang-ru { display: block; }
  .lang-en { display: block; }
  .lang-ru { display: none; }
</style>

---

<div class="lang-en">
## English Version




# 🤖 ESP32 Self-Balancing Robot

A self-balancing robot on two wheels with line tracking, obstacle avoidance, and web-based joystick control.

## 📁 Project Files

### Main Sketches:
- **esp32_robot_main.ino** - Main robot program
- **web_interface.h** - Web interface with joystick

### Test Sketches:
- **test_line_sensor.ino** - Line sensor testing (5 sensors)
- **test_distance_sensor.ino** - Distance sensor testing (VL53L0X)
- **test_motors.ino** - Motor speed and synchronization testing

### Libraries:
- LineFollower.h/.cpp
- Control.h/.cpp
- MPU6050.h/.cpp
- Motors.h/.cpp
- defines.h, globals.h/.cpp

---

## ⚡ Quick Start

### 1. Edit WiFi Settings

In **esp32_robot_main.ino**:
```cpp
String sta_ssid = "YourWiFiSSID";
String sta_password = "YourWiFiPassword";
```

### 2. Install Libraries

Required Arduino IDE libraries:
- AsyncTCP
- ESPAsyncWebServer
- VL53L0X

### 3. Open Web Interface

```
http://192.168.X.X
```

---

## 🎮 Control (Web Interface)

### Joystick Control
- **Up/Down** = Throttle (speed)
- **Left/Right** = Steering (turn)

### Buttons
- **▶ Straight** - Drive straight for 2 sec, then line follow
- **〰 Line** - Start line following mode
- **⏹ Stop** - Emergency stop

### Real-Time Sensor Display
- Distance to obstacle (mm)
- Line sensor data (5 sensors with graph)
- Tilt angle
- Hill/slope status

---

## 🤖 How It Works

```
1. IDLE (waiting for command)
   └─ Press "Straight"
   
2. STRAIGHT (drives 2 seconds)
   └─ Auto transition
   
3. LINE_FOLLOW (tracks black line)
   - Detects obstacles → STOP
   - Detects slope → Speed reduction (2x)
   
4. STOPPED (obstacle detected)
   └─ Press "Stop" button
```

---

## 🧪 Testing

### 1. Line Sensor Test

```cpp
// Upload: test_line_sensor.ino
// Serial Monitor: 115200 baud
// Output: Raw values, calibrated values, ASCII graph
```

### 2. Distance Sensor Test

```cpp
// Upload: test_distance_sensor.ino
// Serial Monitor: 115200 baud
// Output: Distance in mm, status, ASCII graph
// Shows: 0-200mm (very close) to >1000mm (far)
```

### 3. Motor Test

```cpp
// Upload: test_motors.ino
// Serial Monitor: 115200 baud
// Tests:
//   1. Both motors forward slow
//   2. Both motors forward fast
//   3. Both motors backward
//   4. Different speeds (synchronization check)
// Output: Step counts, speed ratio
```

---

## 🔧 Configuration Parameters

```cpp
#define DISTANCE_THRESHOLD 300      // Stop if < 300mm
#define STRAIGHT_TIME 2000          // 2 seconds straight
#define SLOPE_ACCEL_THRESHOLD 2000  // Hill/slope detection
#define MAX_THROTTLE 550            // Maximum speed
#define MAX_STEERING 140            // Maximum turn
```

---

## 🔌 GPIO Pinout (ESP32)

```
I2C:
  GPIO 21 (SDA) ← MPU6050, VL53L0X
  GPIO 22 (SCL) ← MPU6050, VL53L0X

Motors:
  GPIO 27/14 ← Motor1 (DIR/STEP)
  GPIO 25/26 ← Motor2 (DIR/STEP)
  GPIO 12 ← Enable

Line Sensor:
  GPIO 34/35/36/39/32 ← Sensors (S1-S5)
```

### Detailed Wiring Diagram

See WIRING section in main README.md for complete schematic with:
- ESP32 pinout diagram
- Component connections
- Power distribution
- Pull-up resistor placement
- Breadboard layout

---

## 🐛 Troubleshooting

**Robot won't connect to WiFi:**
- Check SSID/password in code
- Check Serial Monitor (115200 baud)
- Verify ESP32 is powered on

**Robot won't balance:**
- Check I2C connection to MPU6050
- Run test_motors.ino to verify motors work
- Calibrate sensors properly

**Line following not working:**
- Calibrate line sensor (test_line_sensor.ino)
- Check GPIO 34/35/36/39/32 connections
- Verify line has high contrast
- Check sensor is 0.5-1cm above line

**Obstacle detection failing:**
- Run test_distance_sensor.ino
- Check VL53L0X I2C connection
- Adjust DISTANCE_THRESHOLD if needed

---

## ✅ Pre-Flight Checklist

- [ ] ESP32 connected to computer
- [ ] All sensors connected
- [ ] WiFi SSID/password entered
- [ ] test_motors.ino passes
- [ ] test_line_sensor.ino works
- [ ] test_distance_sensor.ino works
- [ ] Web interface loads

---

## 📊 Performance

| Metric | Value |
|--------|-------|
| Control Loop | 100Hz (10ms) |
| Web API Response | <50ms |
| Line Sensor Rate | 100Hz |
| Distance Sensor | 200Hz |
| Motor Max Speed | ~500 RPM |
| Balance Response | <50ms |

---

## 📚 Documentation

- **README.md** - Main documentation (Russian)
- **README_EN.md** - English documentation (this file)
- **CHANGELOG.md** - Version history
- **CONTRIBUTING.md** - How to contribute
- **CODE_OF_CONDUCT.md** - Community guidelines
- **LICENSE.md** - MIT License
- **RELEASE_INFO.md** - Release information

---

## 🚀 Future Features

- Machine learning obstacle avoidance
- SLAM navigation
- Multi-robot coordination
- Mobile app control
- Camera integration
- Voice control
- Advanced PID tuning interface
- Telemetry recording

---

## 📝 License

MIT License - Free for personal, educational, and commercial use.  
See LICENSE.md for details.

---

## 🤝 Contributing

Contributions welcome! See CONTRIBUTING.md for guidelines.

---

**Version:** 3.0  
**Status:** ✅ Production Ready  
**Repository:** https://github.com/Mukller/Balance_robot  
**Maintainer:** Anton (@Mukller)

---

**Last Updated:** 2026-06-07

---

</div>

<div class=lang-ru>




## Русская версия




# 🤖 ESP32 Self-Balancing Robot

<div align="center">

[![License: MIT](https://img.shields.io/badge/License-MIT-purple?style=flat-square)](LICENSE.md)
[![C++](https://img.shields.io/badge/C%2B%2B-17-blue?style=flat-square&logo=cplusplus)](https://isocpp.org/)
[![ESP32](https://img.shields.io/badge/ESP32-Platform-red?style=flat-square)](https://www.espressif.com/)
[![maintained](https://img.shields.io/badge/maintained%3F-yes-green?style=flat-square)](https://github.com/Mukller/Balance_robot)
[![contributions welcome](https://img.shields.io/badge/contributions-welcome-brightgreen?style=flat-square)](CONTRIBUTING.md)

[English](README_EN.md) • Русский

</div>

Автономный робот-балансир на двух колёсах с линией, препятствиями и управлением через веб-интерфейс.

## 📁 Файлы проекта

### Основные скетчи:
- **esp32_robot_main.ino** - основная программа робота
- **web_interface.h** - веб-интерфейс с джойстиком

### Тестовые скетчи:
- **test_line_sensor.ino** - тест датчика линии (5 датчиков)
- **test_distance_sensor.ino** - тест датчика расстояния (VL53L0X)
- **test_motors.ino** - тест моторов (скорость и синхронизация)
- **test_servo_gripper.ino** - тест сервомоторов захвата (две серво)
- **test_gyro.ino** - тест гироскопа/акселерометра (MPU6050)

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
1. IDLE (ждёт команду)
   ├─ Кнопка "Прямо" → STRAIGHT
   ├─ Кнопка "Линия" → LINE_FOLLOW
   └─ Кнопка Захвата → Открыть/Закрыть (независимо)
   
2. STRAIGHT (2 сек движение вперёд)
   └─ Автоматический переход → LINE_FOLLOW
   
3. LINE_FOLLOW (следит за линией)
   ├─ Если видит препятствие → STOPPED
   ├─ Если уклон → замедление (×0.5)
   └─ Захват управляется отдельно
   
4. STOPPED (препятствие обнаружено)
   ├─ Моторы остановлены
   └─ Кнопка "Стоп" → IDLE

Захват (независимый):
- Открыть: 180° (можно в любой момент)
- Закрыть: 90° (можно в любой момент)
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

### 4. Тест сервомоторов захвата

```cpp
// Загрузить: test_servo_gripper.ino
// Serial Monitor: 115200 baud
// Тесты:
//   1. Левая серво: 0° → 180° → 0°
//   2. Правая серво: 0° → 180° → 0°
//   3. Обе вместе (Открыть 180° → Закрыть 90° → Сброс 0°)
// Пины: GPIO 13 (левая), GPIO 33 (правая)
// Выводит: ASCII график углов поворота
```

### 5. Тест гироскопа (MPU6050)

```cpp
// Загрузить: test_gyro.ino
// Serial Monitor: 115200 baud
// Проверяет: WHO_AM_I (0x68), затем читает данные
// Выводит:
//   - Гироскоп X/Y/Z (°/сек)
//   - Акселерометр X/Y/Z (g)
//   - Углы Pitch / Roll
//   - Температуру (°C)
// Пины I2C: SDA=GPIO 21, SCL=GPIO 22
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
I2C (для датчиков):
  GPIO 21 (SDA) ← MPU6050, VL53L0X
  GPIO 22 (SCL) ← MPU6050, VL53L0X

Моторы (движение):
  GPIO 27 ← Motor1 DIR (направление)
  GPIO 14 ← Motor1 STEP (шаги)
  GPIO 25 ← Motor2 DIR (направление)
  GPIO 26 ← Motor2 STEP (шаги)
  GPIO 12 ← Enable (включение)

Захват (две серво):
  GPIO 13 ← Servo Left (левая рука)
  GPIO 33 ← Servo Right (правая рука)

Датчик линии:
  GPIO 34 ← S1 (левый край)
  GPIO 35 ← S2 (левая четверть)
  GPIO 36 ← S3 (центр)
  GPIO 39 ← S4 (правая четверть)
  GPIO 32 ← S5 (правый край)
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

### 5. ЗАХВАТ (две серво)

```
┌──────────────────────────┐
│  Servo Left (MG996R)     │
├──────────────────────────┤
│ Signal ────→ GPIO 13 (PWM)
│ GND ───────→ GND
│ VCC ───────→ 5V
└──────────────────────────┘

┌──────────────────────────┐
│  Servo Right (MG996R)    │
├──────────────────────────┤
│ Signal ────→ GPIO 33 (PWM)
│ GND ───────→ GND
│ VCC ───────→ 5V
└──────────────────────────┘

Углы:
- OPEN (180°) - захват открыт
- CLOSE (90°) - захват закрыт
```

### 6. СВЕТОДИОДЫ (опционально)

```
WiFi LED:
GPIO 2 → LED+ → ⎕ → 220Ω → GND
```

### 7. ПИТАНИЕ (Очень важно!)

```
┌──────────────────────────────┐
│   Power Distribution         │
├──────────────────────────────┤
│ Battery/PSU                  │
│ ├─→ 5V ────→ 5V шина         │
│ │  ├─→ Line Sensor VCC       │
│ │  ├─→ VL53L0X VCC           │
│ │  └─→ Servo Left + Right    │
│ │
│ ├─→ 12V ───→ Motor Driver    │
│ │  └─→ Stepper Motors        │
│ │
│ ├─→ 3.3V ──→ ESP32 (Zener)   │
│ │  └─→ MPU6050 VCC           │
│ │
│ └─→ GND ───→ GND шина (ОБЩАЯ)│
│    ├─→ ESP32 GND            │
│    ├─→ Драйвер GND          │
│    ├─→ Датчики GND          │
│    └─→ Серво GND            │
└──────────────────────────────┘

⚠️ КРИТИЧНО: 
- Все GND должны быть соединены вместе!
- Серво требуют отдельный источник питания (5V мощный!)
- Защищать сигналы серво от помех моторов
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
1. GPIO 21/22 (I2C) → датчики
   - SDA → GPIO 21 (MPU6050, VL53L0X)
   - SCL → GPIO 22 (MPU6050, VL53L0X)
   - 3.3V, GND

2. GPIO 34/35/36/39/32 (ADC) → датчик линии
   - S1-S5 → GPIO 34/35/36/39/32
   - 5V, GND

3. GPIO 27/14/25/26/12 (Motor) → драйвер
   - DIR/STEP → GPIO 27/14/25/26
   - Enable → GPIO 12
   - GND, 12V

4. GPIO 13/33 (PWM) → сервомоторы захвата
   - Servo Left  → GPIO 13
   - Servo Right → GPIO 33
   - GND, 5V (отдельный источник!)

5. Проверить:
   [ ] Все 3.3V соединены
   [ ] Все GND соединены (ОБЩАЯ земля!)
   [ ] MPU6050 + VL53L0X на I2C (GPIO 21/22)
   [ ] I2C Pull-up резисторы (4.7K)
   [ ] Датчик линии на GPIO 34/35/36/39/32
   [ ] Моторы на GPIO 27/14/25/26/12
   [ ] Сервомоторы на GPIO 13 и GPIO 33
   [ ] Питание серв отдельное и мощное!
   [ ] Питание моторов 12V отдельное
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

</div>