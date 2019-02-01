
#include "AssistantFile.h"
#include "States.h"


byte state = WAIT;


void setup() 
{
	PinsInit();
	Serial.begin(9600);
	attachInterrupt(digitalPinToInterrupt(SHOOT_BUTTON), ShootButton_Interrupt, FALLING);
	attachInterrupt(digitalPinToInterrupt(CUT_OFF_BUTTON), CutOffButton_Interrupt, FALLING);
}

void loop() 
{
	if (state == WAIT)
	{
	
	}
	else if (state == CHARGE)
	{
		digitalWrite(RELE_1, HIGH);
		digitalWrite(RELE_2, HIGH);
		delay(1000);
		state = WAIT;
	}
	else if (state == SHOOT)
	{

	}
	else if (state == EMERGENCY_CUT_OFF)
	{
		delay(100);
		state = WAIT;
	}
}

void IRAM_ATTR ShootButton_Interrupt()
{
	if (state == WAIT)
	{
		state = CHARGE;
		Serial.println("CHARGE");
	}
}

void IRAM_ATTR CutOffButton_Interrupt()
{
	digitalWrite(RELE_1, LOW);
	digitalWrite(RELE_2, LOW);

	state = EMERGENCY_CUT_OFF;

	Serial.println("EMEGRECY CUT OFF!!!");
}
