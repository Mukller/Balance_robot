// САМЫЙ ПРОСТОЙ ТЕСТ МОТОРА (TMC2209)
// Пины как в README:
//   EN   = GPIO 12
//   M1: DIR = 27, STEP = 14
//   M2: DIR = 25, STEP = 26
// Оба мотора просто крутятся. Всё.
//
// Замечание: DIR=HIGH на обоих моторах НЕ гарантирует движение робота вперёд -
// это зависит от того, как моторы закреплены на раме. Используй этот скетч
// только для проверки "оба мотора крутятся вообще". Для проверки направления
// используй test_motors.ino (он меняет DIR каждые 3 сек).

#define EN    12
#define DIR1  27
#define STEP1 14
#define DIR2  25
#define STEP2 26

void setup() {
  pinMode(EN, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(STEP1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(STEP2, OUTPUT);

  digitalWrite(EN, LOW);     // LOW = драйвер включён (TMC2209)
  digitalWrite(DIR1, HIGH);
  digitalWrite(DIR2, HIGH);
}

void loop() {
  // один шаг обоим моторам
  digitalWrite(STEP1, HIGH);
  digitalWrite(STEP2, HIGH);
  delayMicroseconds(500);
  digitalWrite(STEP1, LOW);
  digitalWrite(STEP2, LOW);
  delayMicroseconds(500);
}

// Author: Anton Petnitsky
// GitHub: https://github.com/Mukller/Balance_robot
// Last modified: 2026-06-10 01:57:13 +0300
