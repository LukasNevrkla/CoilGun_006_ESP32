#include "AssistantFile.h"

hw_timer_t * Timers[4] = { NULL,NULL,NULL,NULL };

void PinsInit()
{
	for (byte i = 0; i < ALL_SENSORS; i++)
		pinMode(SENSOR[i], INPUT_PULLDOWN);

	for (byte i = 0; i < ALL_COILS; i++)
	{
		pinMode(COIL[i], OUTPUT);
		digitalWrite(COIL[i], LOW);
	}

	pinMode(BUTTONS_INTERRUPT_PIN, INPUT);
	pinMode(BUTTONS_READ_PIN, INPUT);


	pinMode(SHOOT_BUTTON, INPUT_PULLUP);
	pinMode(CUT_OFF_BUTTON, INPUT_PULLUP);

	pinMode(CURRENT_SENSOR, INPUT);

	pinMode(CHARGING_TRANSISTOR, OUTPUT);
	digitalWrite(CHARGING_TRANSISTOR, LOW);

	//pinMode(POTENTIOMETER, INPUT);

	pinMode(RELE_1, OUTPUT);
	pinMode(RELE_2, OUTPUT);

	pinMode(BATTERY_VOLTAGE_SENSOR, INPUT_PULLDOWN);
	pinMode(CAPACITORS_VOLTAGE_SENSOR, INPUT_PULLDOWN);
}

void SetTimer(uint8_t _timer, uint64_t time, void(*interupt)(), bool reload)
{
	//Timer interrupt  https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/
	Timers[_timer] = timerBegin(_timer, 80, true);
	timerAttachInterrupt(Timers[_timer], interupt, true);
	timerAlarmWrite(Timers[_timer], time, reload);
	timerAlarmEnable(Timers[_timer]);
}

