<div align="center">

[![License: MIT](https://img.shields.io/badge/License-MIT-purple?style=flat-square)](LICENSE.md)
[![maintained](https://img.shields.io/badge/maintained%3F-yes-green?style=flat-square)](https://github.com/Mukller/Balance_robot)
[![contributions welcome](https://img.shields.io/badge/contributions-welcome-brightgreen?style=flat-square)](CONTRIBUTING.md)

### 🌐 Язык / Language

**Нажми, чтобы развернуть нужный язык · Click to expand your language**

</div>

<details open>
<summary><b>🇬🇧 English</b></summary>

<br>

# 🤖 ESP32 Self-Balancing Robot

An autonomous two-wheeled self-balancing robot with line following, obstacle detection, and control via a web interface.

## 📁 Project files

### Main sketches:
- **esp32_robot_main.ino** - the robot's main program
- **web_interface.h** - web interface with a joystick

### Test sketches:
- **test_line_sensor.ino** - line sensor test (5 sensors)
- **test_distance_sensor.ino** - distance sensor test (VL53L0X)
- **test_motors.ino** - motor test (speed and synchronization)
- **test_servo_gripper.ino** - gripper servo test (two servos)
- **test_gyro.ino** - gyroscope/accelerometer test (MPU6050)

### Libraries:
- LineFollower.h/.cpp
- Control.h/.cpp
- MPU6050.h/.cpp
- Motors.h/.cpp
- defines.h, globals.h/.cpp

---

## ⚡ Quick start

### 1. Edit WiFi

In **esp32_robot_main.ino**:
```cpp
String sta_ssid = "YourWiFiSSID";
String sta_password = "YourWiFiPassword";
```

### 2. Upload from the Arduino IDE

Required libraries:
- AsyncTCP
- ESPAsyncWebServer
- VL53L0X

### 3. Open the web interface

```
http://192.168.X.X
```

---

## 🎮 Control (web interface)

### Joystick
- **Up-down** = Throttle (speed)
- **Left-right** = Steering (turning)

### Buttons
- **▶ Straight** - drive 2 sec, then follow the line
- **〰 Line** - follow the line
- **⏹ Stop** - stop

### Real-time sensors
- Distance to obstacle (mm)
- 5 line sensors (graph)
- Tilt angle
- Slope status

---

## 🤖 How the robot works

```
1. IDLE (waiting for a command)
   ├─ "Straight" button → STRAIGHT
   ├─ "Line" button → LINE_FOLLOW
   └─ Gripper button → Open/Close (independent)
   
2. STRAIGHT (2 sec forward motion)
   └─ Automatic transition → LINE_FOLLOW
   
3. LINE_FOLLOW (follows the line)
   ├─ If it sees an obstacle → STOPPED
   ├─ If on a slope → slow down (×0.5)
   └─ Gripper is controlled separately
   
4. STOPPED (obstacle detected)
   ├─ Motors stopped
   └─ "Stop" button → IDLE

Gripper (independent):
- Open: 180° (any time)
- Close: 90° (any time)
```

---

## 🧪 Testing

### 1. Line sensor test

```cpp
// Upload: test_line_sensor.ino
// Serial Monitor: 115200 baud
// Outputs: Raw values, Calibrated, ASCII graph
```

### 2. Distance sensor test

```cpp
// Upload: test_distance_sensor.ino
// Serial Monitor: 115200 baud
// Outputs: distance in mm, status, ASCII graph
// Checks: 0-200mm (very close) to >1000mm (far)
```

### 3. Motor test

```cpp
// Upload: test_motors.ino
// Serial Monitor: 115200 baud
// Tests:
//   1. Both slow forward
//   2. Both fast forward
//   3. Both backward
//   4. Different speeds (synchronization)
// Outputs: step count, speed ratio
```

### 4. Gripper servo test

```cpp
// Upload: test_servo_gripper.ino
// Serial Monitor: 115200 baud
// Tests:
//   1. Left servo: 0° → 180° → 0°
//   2. Right servo: 0° → 180° → 0°
//   3. Both together (Open 180° → Close 90° → Reset 0°)
// Pins: GPIO 13 (left), GPIO 33 (right)
// Outputs: ASCII graph of rotation angles
```

### 5. Gyroscope test (MPU6050)

```cpp
// Upload: test_gyro.ino
// Serial Monitor: 115200 baud
// Checks: WHO_AM_I (0x68), then reads data
// Outputs:
//   - Gyroscope X/Y/Z (°/sec)
//   - Accelerometer X/Y/Z (g)
//   - Pitch / Roll angles
//   - Temperature (°C)
// I2C pins: SDA=GPIO 21, SCL=GPIO 22
```

---

## 🔧 Parameters in code

```cpp
#define DISTANCE_THRESHOLD 300      // Stop < 300mm
#define STRAIGHT_TIME 2000          // 2 sec straight
#define SLOPE_ACCEL_THRESHOLD 2000  // Slope threshold
#define MAX_THROTTLE 550            // Max speed
#define MAX_STEERING 140            // Max steering
```

---

## 📊 GPIO wiring (ESP32)

```
I2C (for sensors):
  GPIO 21 (SDA) ← MPU6050, VL53L0X
  GPIO 22 (SCL) ← MPU6050, VL53L0X

Motors (movement):
  GPIO 27 ← Motor1 DIR (direction)
  GPIO 14 ← Motor1 STEP (steps)
  GPIO 25 ← Motor2 DIR (direction)
  GPIO 26 ← Motor2 STEP (steps)
  GPIO 12 ← Enable

Gripper (two servos):
  GPIO 13 ← Servo Left (left arm)
  GPIO 33 ← Servo Right (right arm)

Line sensor:
  GPIO 34 ← S1 (left edge)
  GPIO 35 ← S2 (left quarter)
  GPIO 36 ← S3 (center)
  GPIO 39 ← S4 (right quarter)
  GPIO 32 ← S5 (right edge)
```

---

## 🐛 Help

**The robot won't connect:**
- Check the WiFi SSID/password
- Watch the Serial Monitor (115200)

**It doesn't balance:**
- Check I2C (MPU6050)
- Check the motors (test_motors.ino)

**It doesn't follow the line:**
- Calibrate the sensor (test_line_sensor.ino)
- Check the line contrast

**It doesn't see obstacles:**
- Test the sensor (test_distance_sensor.ino)
- Adjust DISTANCE_THRESHOLD

---

## ✅ Pre-use checklist

- [ ] ESP32 connected
- [ ] All sensors connected
- [ ] WiFi SSID/password entered
- [ ] test_motors.ino passes
- [ ] test_line_sensor.ino works
- [ ] test_distance_sensor.ino works
- [ ] The web interface opens

---

**Version:** 3.0 (with web interface)  
**Status:** ✅ Production Ready

---

## 🔌 FULL WIRING DIAGRAM

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

## 📋 DETAILED COMPONENT WIRING

### 1. MOTORS (NEMA17 stepper with driver)

```
Motor 1:
┌──────────────────┐
│  NEMA17 Motor   │
├──────────────────┤
│ DIR ─────→ GPIO 27
│ STEP ────→ GPIO 14
│ GND ─────→ GND
│ +12V ────→ 12V (via driver)
└──────────────────┘

Motor 2:
┌──────────────────┐
│  NEMA17 Motor   │
├──────────────────┤
│ DIR ─────→ GPIO 25
│ STEP ────→ GPIO 26
│ GND ─────→ GND
│ +12V ────→ 12V (via driver)
└──────────────────┘

Motor Enable:
GPIO 12 → Motor Driver Enable
GND ────→ GND
```

### 2. MPU6050 (Gyroscope + Accelerometer)

```
┌──────────────────┐
│     MPU6050      │
├──────────────────┤
│ SDA ─────→ GPIO 21 (I2C)
│ SCL ─────→ GPIO 22 (I2C)
│ GND ─────→ GND
│ VCC ─────→ 3.3V
│ INT ─────→ leave unconnected (optional)
└──────────────────┘

I2C address: 0x68
```

### 3. VL53L0X (Distance sensor)

```
┌──────────────────┐
│    VL53L0X       │
├──────────────────┤
│ SDA ─────→ GPIO 21 (I2C)
│ SCL ─────→ GPIO 22 (I2C)
│ GND ─────→ GND
│ VCC ─────→ 3.3V
│ XSHUT ───→ leave unconnected (optional)
└──────────────────┘

I2C address: 0x29
```

### 4. LINE SENSOR (Ldabrye 5-sensor)

```
┌─────────────────────────────┐
│  Ldabrye Line Sensor Array  │
├─────────────────────────────┤
│ S1 (Left)   ─→ GPIO 34 (ADC)
│ S2          ─→ GPIO 35 (ADC)
│ S3 (Center) ─→ GPIO 36 (ADC)
│ S4          ─→ GPIO 39 (ADC)
│ S5 (Right)  ─→ GPIO 32 (ADC)
│ GND         ─→ GND
│ VCC         ─→ 5V
└─────────────────────────────┘
```

### 5. GRIPPER (two servos)

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

Angles:
- OPEN (180°) - gripper open
- CLOSE (90°) - gripper closed
```

### 6. LEDS (optional)

```
WiFi LED:
GPIO 2 → LED+ → ⎕ → 220Ω → GND
```

### 7. POWER (Very important!)

```
┌──────────────────────────────┐
│   Power Distribution         │
├──────────────────────────────┤
│ Battery/PSU                  │
│ ├─→ 5V ────→ 5V rail         │
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
│ └─→ GND ───→ GND rail (COMMON)│
│    ├─→ ESP32 GND            │
│    ├─→ Driver GND           │
│    ├─→ Sensors GND          │
│    └─→ Servo GND            │
└──────────────────────────────┘

⚠️ CRITICAL: 
- All GNDs must be tied together!
- Servos need a separate power source (a beefy 5V!)
- Protect servo signals from motor noise
```

---

## 🎯 WIRING BOARD (breadboard layout)

```
I2C (GPIO 21, 22):
┌─ 3.3V ─ Pull-up 4.7K ─ GPIO 22 (SCL) ─ MPU6050 SCL
├─ 3.3V ─ Pull-up 4.7K ─ GPIO 21 (SDA) ─ MPU6050 SDA
│                                      └─ VL53L0X SDA/SCL
└─ GND ─────────────────────────────── Both sensors GND

ADC (Line Sensors):
GPIO 34 ← Sensor1 (ADC1_CH6)
GPIO 35 ← Sensor2 (ADC1_CH7)
GPIO 36 ← Sensor3 (ADC1_CH0)
GPIO 39 ← Sensor4 (ADC1_CH3)
GPIO 32 ← Sensor5 (ADC1_CH4)

Motor Control:
GPIO 27 → Driver DIR1 → Motor1 DIR
GPIO 14 → Driver STEP1 → Motor1 STEP
GPIO 25 → Driver DIR2 → Motor2 DIR
GPIO 26 → Driver STEP2 → Motor2 STEP
GPIO 12 → Driver ENABLE

Audio/Visual:
GPIO 33 → Buzzer
GPIO 2 → WiFi LED
```

---

## 📐 SCHEMATIC

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

## ✅ WIRING CHECK

```
On the bench:
1. GPIO 21/22 (I2C) → sensors
   - SDA → GPIO 21 (MPU6050, VL53L0X)
   - SCL → GPIO 22 (MPU6050, VL53L0X)
   - 3.3V, GND

2. GPIO 34/35/36/39/32 (ADC) → line sensor
   - S1-S5 → GPIO 34/35/36/39/32
   - 5V, GND

3. GPIO 27/14/25/26/12 (Motor) → driver
   - DIR/STEP → GPIO 27/14/25/26
   - Enable → GPIO 12
   - GND, 12V

4. GPIO 13/33 (PWM) → gripper servos
   - Servo Left  → GPIO 13
   - Servo Right → GPIO 33
   - GND, 5V (separate source!)

5. Verify:
   [ ] All 3.3V tied together
   [ ] All GND tied together (COMMON ground!)
   [ ] MPU6050 + VL53L0X on I2C (GPIO 21/22)
   [ ] I2C pull-up resistors (4.7K)
   [ ] Line sensor on GPIO 34/35/36/39/32
   [ ] Motors on GPIO 27/14/25/26/12
   [ ] Servos on GPIO 13 and GPIO 33
   [ ] Servo power separate and beefy!
   [ ] Motor power 12V separate
```

---

## 🔍 I2C PULL-UP RESISTORS (IMPORTANT!)

```
If the sensors don't respond, add:

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

**Schematic version:** 1.0  
**Last updated:** 2026-06-07

</details>

<details>
<summary><b>🇷🇺 Русский</b></summary>

<br>

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

</details>
