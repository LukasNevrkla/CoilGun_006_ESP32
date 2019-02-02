
#include "Sensors.h"
#include "Measurement.h"
#include "AssistantFile.h"
#include "States.h"

byte state = WAIT;
//portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;	//DOKONÈIT

void setup() 
{
	Serial.begin(9600);
	Serial.println("Init");

	PinsInit();
	SensorsInit(&SensorInt, &ShootEnd);

	attachInterrupt(digitalPinToInterrupt(SHOOT_BUTTON), ShootButton_Interrupt, FALLING);
	attachInterrupt(digitalPinToInterrupt(CUT_OFF_BUTTON), CutOffButton_Interrupt, FALLING);

	ledcSetup(1, CHARGE_FREQUENCY,8);
	ledcAttachPin(CHARGING_TRANSISTOR, 1);

#if  !START_BY_BUTTON
	SensorsStart();
#endif //  !START_BY_BUTTON

}


void loop()
{
	if (state == WAIT)
	{/*
		Serial.print(MeasurePin(CAPACITORS_VOLTAGE_SENSOR, CAPACITORS_DIVIDER, false));
		Serial.print(" \t ");
		Serial.println(MeasurePin(BATTERY_VOLTAGE_SENSOR, BATTERY_DIVIDER, false));

		delay(1000);*/
	}
	else if (state == CHARGE_START)
	{
		Serial.print("Starting charging at");
		Serial.print(VOLTAGE_TO_CHARGE);
		Serial.println(" V");

		digitalWrite(RELE_1, HIGH);
		digitalWrite(RELE_2, HIGH);
		delay(500);
		state = CHARGING;
	}
	else if (state == CHARGING)
	{
		double voltage = MeasurePin(CAPACITORS_VOLTAGE_SENSOR, CAPACITORS_DIVIDER, true);
		/*Serial.print(voltage);
		Serial.print(" \t ");
		Serial.println(MeasurePin(BATTERY_VOLTAGE_SENSOR, BATTERY_DIVIDER, true));

		delay(500);*/

		if (voltage <= VOLTAGE_TO_CHARGE)
		{
			//Serial.println("charging");
			ledcWrite(1, CHARGE_PWM_ALTERNATE);
		}
		else
		{
			Serial.println("CHARGE COMPLETE");
			ledcWrite(1, 0);

			digitalWrite(RELE_1, LOW);
			digitalWrite(RELE_2, LOW);

			state = CHARGE_DONE;
		}
	}
	else if (state == CHARGE_DONE)
	{

	}
	else if (state == SHOOT_START)
	{
		SetTimer(1, MAX_TIME_FOR_SENSORS, ShootEnd,false);
	}
	else if (state == SHOOT)
	{

	}
	else if (state == EMERGENCY_CUT_OFF)
	{
		state = WAIT;
		Serial.println("EMERGENCY_CUT_OFF");
		delay(100);
	}
}

void IRAM_ATTR ShootButton_Interrupt()
{
	if (state == WAIT)
	{
		state = CHARGE_START;
	}
	else if (state == CHARGE_DONE)
	{
		state = SHOOT_START;
	}
}

void IRAM_ATTR CutOffButton_Interrupt()
{
	digitalWrite(RELE_1, LOW);
	digitalWrite(RELE_2, LOW);

	ledcWrite(1, 0);

	state = EMERGENCY_CUT_OFF;
}

void IRAM_ATTR SensorInt(byte _sensor)
{
#if !START_BY_BUTTON 
	if (_sensor==0)
		SetTimer(1, MAX_TIME_FOR_SENSORS, ShootEnd,false);
#endif

}

void IRAM_ATTR ShootEnd()
{
	SensorsEnd();
}
