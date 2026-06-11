#include <Arduino.h>
#include "defines.h"
#include "globals.h"

// PD controller for balancing with higher gain
float stabilityPDControl(float DT, float input, float setPoint, float Kp, float Kd)
{
  static float prev_error = 0;
  
  float error = setPoint - input;
  float derivative = (error - prev_error) / DT;
  
  // Dead zone - игнорируем маленькие ошибки
  float output = 0;
  if (fabs(error) > 0.3) {  // Только если ошибка больше 0.3 градуса
    output = Kp * error + Kd * derivative;
  }
  
  prev_error = error;
  
  // Ограничение
  output = constrain(output, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);
  
  return output;
}

float speedPIControl(float DT, int16_t input, int16_t setPoint, float Kp, float Ki)
{
  int16_t error = setPoint - input;
  PID_errorSum += constrain(error, -ITERM_MAX_ERROR, ITERM_MAX_ERROR);
  PID_errorSum = constrain(PID_errorSum, -ITERM_MAX, ITERM_MAX);
  
  float output = Kp * error + Ki * PID_errorSum * DT;
  return output;
}

float positionPDControl(long actualPos, long setPointPos, float Kpp, float Kdp, int16_t speedM)
{
  float P = constrain(Kpp * float(setPointPos - actualPos), -115, 115);
  float output = P + Kdp * float(speedM);
  return output;
}