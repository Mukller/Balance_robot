#include <Arduino.h>
#include "defines.h"
#include "globals.h"

void setMotorSpeedM1(int16_t tspeed)
{
  long timer_period;
  int16_t speed;

  if ((speed_M1 - tspeed) > MAX_ACCEL)
    speed_M1 -= MAX_ACCEL;
  else if ((speed_M1 - tspeed) < -MAX_ACCEL)
    speed_M1 += MAX_ACCEL;
  else
    speed_M1 = tspeed;

#if MICROSTEPPING==16
  speed = speed_M1 * 50;
#else
  speed = speed_M1 * 25;
#endif

  if (speed == 0) {
    timer_period = ZERO_SPEED;
    dir_M1 = 0;
  }
  else if (speed > 0) {
    timer_period = 2000000 / speed;
    dir_M1 = 1;
    digitalWrite(PIN_MOTOR1_DIR, HIGH);
  }
  else {
    timer_period = 2000000 / -speed;
    dir_M1 = -1;
    digitalWrite(PIN_MOTOR1_DIR, LOW);
  }
  
  if (timer_period > ZERO_SPEED) {
    timer_period = ZERO_SPEED;
  }

  timerAlarm(timer1, timer_period, true, 0);
}

void setMotorSpeedM2(int16_t tspeed)
{
  long timer_period;
  int16_t speed;

  if ((speed_M2 - tspeed) > MAX_ACCEL)
    speed_M2 -= MAX_ACCEL;
  else if ((speed_M2 - tspeed) < -MAX_ACCEL)
    speed_M2 += MAX_ACCEL;
  else
    speed_M2 = tspeed;

#if MICROSTEPPING==16
  speed = speed_M2 * 50;
#else
  speed = speed_M2 * 25;
#endif

  if (speed == 0) {
    timer_period = ZERO_SPEED;
    dir_M2 = 0;
    // ВАЖНО: При остановке оба пина STEP должны быть LOW
    digitalWrite(PIN_MOTOR2_STEP, LOW);
  }
  else if (speed > 0) {
    timer_period = 2000000 / speed;
    dir_M2 = 1;
    digitalWrite(PIN_MOTOR2_DIR, HIGH);  // ИСПРАВЛЕНО: было LOW
    Serial.print("M2 DIR HIGH, speed="); Serial.println(speed);  // Отладка
  }
  else {
    timer_period = 2000000 / -speed;
    dir_M2 = -1;
    digitalWrite(PIN_MOTOR2_DIR, LOW);   // ИСПРАВЛЕНО: было HIGH
    Serial.print("M2 DIR LOW, speed="); Serial.println(speed);   // Отладка
  }
  
  if (timer_period > ZERO_SPEED) {
    timer_period = ZERO_SPEED;
  }

  timerAlarm(timer2, timer_period, true, 0);
}