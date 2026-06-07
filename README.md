# 🤖 ESP32 Self-Balancing Robot with Line Following

Автономный робот-балансир на двух колёсах с датчиком линии и дистанционным датчиком.

## ⚡ Быстрый старт

### 1. Подключить железо

**GPIO (ESP32):**
```
SDA (21) ← MPU6050/VL53L0X
SCL (22) ← MPU6050/VL53L0X
GPIO 27/14 ← Motor 1 (DIR/STEP)
GPIO 25/26 ← Motor 2 (DIR/STEP)
GPIO 34/35/36/39/32 ← Line Sensor (S1-S5)
```

### 2. Отредактировать WiFi

В файле `main/esp32_wifi_balancing_robot_v2.ino` найти:
```cpp
String sta_ssid = "YourWiFiSSID";
String sta_password = "YourWiFiPassword";
```

### 3. Загрузить в Arduino IDE

Библиотеки:
- AsyncTCP
- ESPAsyncWebServer  
- VL53L0X

### 4. Калибровка

1. Serial Monitor: 115200 baud
2. Раскомментировать `LineFollower_calibrate();`
3. Загрузить, следовать инструкциям
4. Закомментировать обратно

---

## 🎮 Управление

### WiFi API

```bash
# Ехать прямо 2 сек, потом линия
http://192.168.X.X/control?push1=1

# Линия
http://192.168.X.X/control?push2=1

# Стоп
http://192.168.X.X/control?push3=1
```

### Состояния

| № | Состояние | Действие |
|---|-----------|----------|
| 0 | IDLE | Ждёт |
| 1 | STRAIGHT | Прямо 2 сек |
| 2 | LINE_FOLLOW | Следит за линией |
| 3 | STOPPED | Препятствие |

---

## 📊 Функции

✅ Баланс (MPU6050)  
✅ Линия (5 датчиков)  
✅ Препятствия (VL53L0X)  
✅ Уклоны (Z-ось акселерометра)  
✅ WiFi управление  

---

## 📁 Файлы

```
main/
├── esp32_wifi_balancing_robot_v2.ino  ← Основной
├── LineFollower.h / .cpp              ← Линия
├── Control.h / .cpp                   ← PID
├── MPU6050.h / .cpp                   ← Гироскоп
├── Motors.h / .cpp                    ← Моторы
└── ...
```

---

## 🔧 Параметры

```cpp
#define DISTANCE_THRESHOLD 300      // Стоп < 300мм
#define STRAIGHT_TIME 2000          // 2 сек прямо
#define SLOPE_ACCEL_THRESHOLD 2000  // Уклон
float slope_speed_factor = 0.5      // Замедление
```

---

## 🐛 Помощь

**Не балансирует:**
- Проверить I2C (MPU6050)
- Калибровка
- Моторы работают?
- Отрегулировать Kp/Kd

**Не следит за линией:**
- Калибровать датчик
- GPIO 34/35/36/39/32
- Включить LINE_FOLLOW
- Линия чёрная на белом?

**Препятствие не работает:**
- I2C адрес 0x29 (VL53L0X)
- DISTANCE_THRESHOLD

---

## ✅ Тест

- [ ] Баланс (IDLE)
- [ ] Прямо 2 сек (push1)
- [ ] Линия (LINE_FOLLOW)
- [ ] Замедление на уклоне
- [ ] Стоп < 300мм
- [ ] WiFi работает

---

**Версия:** 2.0  
**Статус:** ✅ Рабочая
