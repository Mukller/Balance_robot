#include "MPU6050.h"
#include "Wire.h"

accel_t_gyro_union accel_t_gyro;
float x_gyro_value;
float x_gyro_offset = 0.0;
float accel_angle;
float angle = 0.0;

uint8_t swap;
#define SWAP(x,y) swap = x; x = y; y = swap

float MPU6050_getAngle(float dt)
{
  // Защита от слишком большого dt
  if (dt > 0.05) dt = 0.02;
  if (dt < 0.001) dt = 0.01;
  
  // Вычисляем угол по акселерометру (только если z не ноль)
  if (accel_t_gyro.value.z_accel != 0) {
    accel_angle = atan2f((float)accel_t_gyro.value.y_accel, (float)accel_t_gyro.value.z_accel) * RAD2GRAD;
  }
  
  // Гироскоп в градусах/сек (деление на 65.5 для диапазона 500 deg/s)
  x_gyro_value = (accel_t_gyro.value.x_gyro - x_gyro_offset) / 65.5;
  
  // Комплементарный фильтр
  angle = 0.98 * (angle + x_gyro_value * dt) + 0.02 * accel_angle;
  
  // Коррекция дрейфа гироскопа
  int16_t correction = constrain(accel_t_gyro.value.x_gyro, x_gyro_offset - 10, x_gyro_offset + 10);
  x_gyro_offset = x_gyro_offset * 0.9995 + correction * 0.0005;
  
  static int diag_cnt = 0;
  diag_cnt++;
  if (diag_cnt >= 50) {
    diag_cnt = 0;
    Serial.print("[MPU] AccelAngle=");
    Serial.print(accel_angle, 2);
    Serial.print(" GyroRate=");
    Serial.print(x_gyro_value, 2);
    Serial.print(" Angle=");
    Serial.print(angle, 2);
    Serial.print(" dt=");
    Serial.println(dt, 5);
  }
  
  return angle;
}

void MPU6050_calibrate()
{
  int i;
  long value = 0;
  float dev;
  int16_t values[100];
  bool gyro_cal_ok = false;
  uint8_t attempts = 0;

  delay(500);
  while (!gyro_cal_ok && attempts < 10){
    attempts++;
    Serial.print("Gyro calibration (attempt ");
    Serial.print(attempts);
    Serial.println("/10)... DONT MOVE!");

    value = 0;
    for (i = 0; i < 100; i++)
    {
      MPU6050_read_3axis();
      values[i] = accel_t_gyro.value.x_gyro;
      value += accel_t_gyro.value.x_gyro;
      delay(25);
    }
    
    value = value / 100;
    dev = 0;
    for (i = 0; i < 100; i++)
      dev += (float)(values[i] - value) * (values[i] - value);
    dev = sqrt((1 / 100.0) * dev);
    
    Serial.print("offset: ");
    Serial.print(value);
    Serial.print("  stddev: ");
    Serial.println(dev);
    
    if (dev < 50.0)
      gyro_cal_ok = true;
  }
  
  x_gyro_offset = value;
  angle = atan2f((float)accel_t_gyro.value.y_accel, (float)accel_t_gyro.value.z_accel) * RAD2GRAD;
  
  Serial.print("[CALIB] Final offset: ");
  Serial.print(x_gyro_offset);
  Serial.print(" initial angle: ");
  Serial.println(angle);
}

void MPU6050_setup()
{
  int error;
  uint8_t c;

  error = MPU6050_read(MPU6050_WHO_AM_I, &c, 1);
  Serial.print("WHO_AM_I : ");
  Serial.print(c, HEX);
  Serial.print(", error = ");
  Serial.println(error);

  MPU6050_write_reg(MPU6050_PWR_MGMT_1, bit(MPU6050_DEVICE_RESET));
  delay(125);
  MPU6050_write_reg (MPU6050_PWR_MGMT_1, 0x01);
  
  // Config Gyro scale (500deg/seg)
  MPU6050_write_reg(MPU6050_GYRO_CONFIG, MPU6050_FS_SEL_500);
  // Config Accel scale (2g)
  MPU6050_write_reg(MPU6050_ACCEL_CONFIG, MPU6050_AFS_SEL_2G);
  // Config Digital Low Pass Filter 10Hz
  MPU6050_write_reg(MPU6050_CONFIG, MPU6050_DLPF_10HZ);
  // Set Sample Rate to 100Hz
  MPU6050_write_reg(MPU6050_SMPLRT_DIV, 9);
  // Data ready interrupt enable
  MPU6050_write_reg(MPU6050_INT_ENABLE, MPU6050_DATA_RDY_EN);
  // Clear the 'sleep' bit
  MPU6050_write_reg (MPU6050_PWR_MGMT_1, 0x01);
}

void MPU6050_read_3axis()
{
  int error;
  
  error = MPU6050_read (MPU6050_ACCEL_XOUT_H, (uint8_t *) &accel_t_gyro, sizeof(accel_t_gyro));
  
  // swap bytes
  SWAP (accel_t_gyro.reg.x_accel_h, accel_t_gyro.reg.x_accel_l);
  SWAP (accel_t_gyro.reg.y_accel_h, accel_t_gyro.reg.y_accel_l);
  SWAP (accel_t_gyro.reg.z_accel_h, accel_t_gyro.reg.z_accel_l);
  SWAP (accel_t_gyro.reg.t_h, accel_t_gyro.reg.t_l);
  SWAP (accel_t_gyro.reg.x_gyro_h, accel_t_gyro.reg.x_gyro_l);
  SWAP (accel_t_gyro.reg.y_gyro_h, accel_t_gyro.reg.y_gyro_l);
  SWAP (accel_t_gyro.reg.z_gyro_h, accel_t_gyro.reg.z_gyro_l);
}

int MPU6050_read(int start, uint8_t *buffer, int size)
{
  int i, n, error;

  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);
  if (n != 1)
    return (-10);

  n = Wire.endTransmission(false);
  if (n != 0)
    return (n);

  Wire.requestFrom(MPU6050_I2C_ADDRESS, size, true);
  i = 0;
  while (Wire.available() && i < size)
  {
    buffer[i++] = Wire.read();
  }
  if (i != size)
    return (-11);

  return (0);
}

int MPU6050_write(int start, const uint8_t *pData, int size)
{
  int n, error;

  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);
  if (n != 1)
    return (-20);

  n = Wire.write(pData, size);
  if (n != size)
    return (-21);

  error = Wire.endTransmission(true);
  if (error != 0)
    return (error);

  return (0);
}

int MPU6050_write_reg(int reg, uint8_t data)
{
  return MPU6050_write(reg, &data, 1);
}