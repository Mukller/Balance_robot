#include "globals.h"
#include "defines.h"
#include "esp32-hal-timer.h"

extern "C" {

portMUX_TYPE muxer1 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE muxer2 = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR timer1ISR() {
	portENTER_CRITICAL_ISR(&muxer1);

	if (dir_M1 != 0) {
		// We generate 1us STEP pulse
		digitalWrite(PIN_MOTOR1_STEP, HIGH);

		if (dir_M1 > 0)
			steps1--;
		else
			steps1++;

		digitalWrite(PIN_MOTOR1_STEP, LOW);
	}

	portEXIT_CRITICAL_ISR(&muxer1);
}
void IRAM_ATTR timer2ISR() {
	portENTER_CRITICAL_ISR(&muxer2);

	if (dir_M2 != 0) {
		// We generate 1us STEP pulse
		digitalWrite(PIN_MOTOR2_STEP, HIGH);

		if (dir_M2 > 0)
			steps2--;
		else
			steps2++;

		digitalWrite(PIN_MOTOR2_STEP, LOW);
	}
	portEXIT_CRITICAL_ISR(&muxer2);
}
}

void initTimers() {
	// ESP32 Core v3.x API: timerBegin(частота тика).
	// 2 МГц - под расчёт периодов в Motors.cpp (2000000 / speed)
	timer1 = timerBegin(2000000);
	timerAttachInterrupt(timer1, &timer1ISR);
	timerAlarm(timer1, ZERO_SPEED, true, 0);

	timer2 = timerBegin(2000000);
	timerAttachInterrupt(timer2, &timer2ISR);
	timerAlarm(timer2, ZERO_SPEED, true, 0);
}
