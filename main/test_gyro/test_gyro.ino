// TEST: Гироскоп + акселерометр (MPU6050 / MPU6500 / MPU9250)
// Прямое чтение по I2C (без модулей проекта)
// Выводит: gyro (X/Y/Z), accel (X/Y/Z), температуру, углы pitch/roll
// I2C: SDA = GPIO 21, SCL = GPIO 22

#include <Wire.h>
#include <math.h>

#define MPU_ADDR        0x68   // I2C адрес (общий для всей серии MPU)
#define REG_PWR_MGMT_1  0x6B   // Управление питанием
#define REG_WHO_AM_I    0x75   // Идентификатор чипа
#define REG_ACCEL_XOUT  0x3B   // Начало блока данных (accel/temp/gyro)

#define I2C_SDA 21
#define I2C_SCL 22

// Чувствительность (настройки по умолчанию)
#define ACCEL_SCALE 16384.0   // ±2g  -> 16384 LSB/g
#define GYRO_SCALE  131.0     // ±250 °/s -> 131 LSB/(°/s)

// Тип чипа (определяется по WHO_AM_I) - влияет на формулу температуры
bool isMPU6050 = true;

// Сырые данные
int16_t ax, ay, az;   // акселерометр
int16_t tempRaw;      // температура
int16_t gx, gy, gz;   // гироскоп

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== MPU GYRO TEST ===");
  Serial.println("I2C: SDA=GPIO21, SCL=GPIO22");

  Wire.begin(I2C_SDA, I2C_SCL);
  delay(100);

  // Проверка наличия датчика и определение модели
  uint8_t who = readByte(REG_WHO_AM_I);
  Serial.print("[WHO_AM_I] 0x");
  Serial.println(who, HEX);

  switch (who) {
    case 0x68: Serial.println("[CHIP] MPU6050 detected");  isMPU6050 = true;  break;
    case 0x70: Serial.println("[CHIP] MPU6500 detected");  isMPU6050 = false; break;
    case 0x71: Serial.println("[CHIP] MPU9250 detected");  isMPU6050 = false; break;
    case 0x73: Serial.println("[CHIP] MPU9255 detected");  isMPU6050 = false; break;
    case 0x00:
    case 0xFF:
      Serial.println("[ERROR] No response on I2C! Check wiring (SDA=21, SCL=22, VCC=3.3V, GND)");
      while (1) delay(1000);
    default:
      Serial.println("[WARN] Unknown chip ID - trying as MPU-compatible anyway");
      isMPU6050 = false;
      break;
  }

  // Разбудить датчик (выйти из sleep) - регистр общий для всей серии
  writeByte(REG_PWR_MGMT_1, 0x00);
  delay(100);

  Serial.println("[OK] Sensor initialized!");
  Serial.println("Keep sensor still to see stable values.\n");
  delay(1000);
}

void loop() {
  readSensor();

  // Перевод в физические единицы
  float ax_g = ax / ACCEL_SCALE;
  float ay_g = ay / ACCEL_SCALE;
  float az_g = az / ACCEL_SCALE;

  float gx_dps = gx / GYRO_SCALE;
  float gy_dps = gy / GYRO_SCALE;
  float gz_dps = gz / GYRO_SCALE;

  // Формула температуры зависит от чипа
  float tempC = isMPU6050 ? (tempRaw / 340.0 + 36.53)    // MPU6050
                          : (tempRaw / 333.87 + 21.0);   // MPU6500/9250

  // Углы наклона по акселерометру
  float pitch = atan2(ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * 180.0 / PI;
  float roll  = atan2(ay_g, sqrt(ax_g * ax_g + az_g * az_g)) * 180.0 / PI;

  // Вывод
  Serial.print("[GYRO dps] X=");  Serial.print(gx_dps, 1);
  Serial.print(" Y=");            Serial.print(gy_dps, 1);
  Serial.print(" Z=");            Serial.print(gz_dps, 1);

  Serial.print("  [ACC g] X=");   Serial.print(ax_g, 2);
  Serial.print(" Y=");            Serial.print(ay_g, 2);
  Serial.print(" Z=");            Serial.print(az_g, 2);

  Serial.print("  [Angle] Pitch="); Serial.print(pitch, 1);
  Serial.print(" Roll=");            Serial.print(roll, 1);

  Serial.print("  [Temp] ");       Serial.print(tempC, 1);
  Serial.println(" C");

  delay(200);
}

// ---- I2C хелперы ----

// Чтение всего блока: accel(6) + temp(2) + gyro(6) = 14 байт от 0x3B
void readSensor() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_XOUT);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  ax      = (Wire.read() << 8) | Wire.read();
  ay      = (Wire.read() << 8) | Wire.read();
  az      = (Wire.read() << 8) | Wire.read();
  tempRaw = (Wire.read() << 8) | Wire.read();
  gx      = (Wire.read() << 8) | Wire.read();
  gy      = (Wire.read() << 8) | Wire.read();
  gz      = (Wire.read() << 8) | Wire.read();
}

uint8_t readByte(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 1, true);
  return Wire.read();
}

void writeByte(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission(true);
}

// Author: Anton Petnitsky
// GitHub: https://github.com/Mukller/Balance_robot
// Last modified: 2026-06-09 23:20:44 +0300
