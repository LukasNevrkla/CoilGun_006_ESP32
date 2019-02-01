#include "AssistantFile.h"

void PinsInit()
{
	for (byte i = 0; i < ALL_SENSORS; i++)
		pinMode(SENSOR[i], INPUT_PULLUP);

	for (byte i = 0; i < ALL_COILS; i++)
	{
		pinMode(COIL[i], OUTPUT);
		digitalWrite(COIL[i], LOW);
	}

	pinMode(SHOOT_BUTTON, INPUT_PULLUP);
	pinMode(CUT_OFF_BUTTON, INPUT_PULLUP);

	pinMode(CURRENT_SENSOR, INPUT);

	pinMode(CHARGING_TRANSISTOR, OUTPUT);
	digitalWrite(CHARGING_TRANSISTOR, LOW);

	pinMode(RELE_1, OUTPUT);
	pinMode(RELE_2, OUTPUT);
}

