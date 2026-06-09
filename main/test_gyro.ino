// TEST: Гироскоп + акселерометр MPU6050
// Прямое чтение по I2C (без модулей проекта)
// Выводит: gyro (X/Y/Z), accel (X/Y/Z), температуру, углы pitch/roll
// I2C: SDA = GPIO 21, SCL = GPIO 22

#include <Wire.h>
#include <math.h>

#define MPU_ADDR        0x68   // I2C адрес MPU6050
#define REG_PWR_MGMT_1  0x6B   // Управление питанием
#define REG_WHO_AM_I    0x75   // Идентификатор (должен вернуть 0x68)
#define REG_ACCEL_XOUT  0x3B   // Начало блока данных (accel/temp/gyro)

#define I2C_SDA 21
#define I2C_SCL 22

// Чувствительность (настройки по умолчанию)
#define ACCEL_SCALE 16384.0   // ±2g  -> 16384 LSB/g
#define GYRO_SCALE  131.0     // ±250 °/s -> 131 LSB/(°/s)

// Сырые данные
int16_t ax, ay, az;   // акселерометр
int16_t tempRaw;      // температура
int16_t gx, gy, gz;   // гироскоп

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== MPU6050 GYRO TEST ===");
  Serial.println("I2C: SDA=GPIO21, SCL=GPIO22");

  Wire.begin(I2C_SDA, I2C_SCL);
  delay(100);

  // Проверка наличия датчика
  uint8_t who = readByte(REG_WHO_AM_I);
  Serial.print("[WHO_AM_I] 0x");
  Serial.println(who, HEX);
  if (who != 0x68) {
    Serial.println("[ERROR] MPU6050 not found! Check wiring (SDA=21, SCL=22, VCC=3.3V)");
    while (1) delay(1000);
  }

  // Разбудить датчик (выйти из sleep)
  writeByte(REG_PWR_MGMT_1, 0x00);
  delay(100);

  Serial.println("[OK] MPU6050 initialized!");
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

  float tempC = tempRaw / 340.0 + 36.53;  // формула из даташита

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
