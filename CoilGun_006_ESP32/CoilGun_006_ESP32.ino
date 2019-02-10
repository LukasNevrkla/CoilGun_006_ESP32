
#include "NextionDisplay.h"
#include "Sensors.h"
#include "Measurement.h"
#include "AssistantFile.h"
#include "States.h"

byte state = WAIT;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
double VoltageToCharge = VOLTAGE_TO_CHARGE;

void setup() 
{
	Serial.begin(9600);
	Serial.println("Init");

	PinsInit();
	SensorsInit(&SensorInt, mux);

	attachInterrupt(digitalPinToInterrupt(SHOOT_BUTTON), ShootButton_Interrupt, FALLING);
	attachInterrupt(digitalPinToInterrupt(CUT_OFF_BUTTON), CutOffButton_Interrupt, FALLING);

	ledcSetup(0, CHARGE_FREQUENCY,8);
	ledcAttachPin(CHARGING_TRANSISTOR, 0);

#if  !START_BY_BUTTON
	SensorsStart();
#endif //  !START_BY_BUTTON

	//digitalWrite(COIL[1], HIGH);
}

void loop()
{
	if (state == WAIT)
	{
		/*double voltage = 0.0;

		for (int i = 0; i < MEASUREMENTS_SAMPLES; i++)
			voltage += MeasurePin(CAPACITORS_VOLTAGE_SENSOR, CAPACITORS_DIVIDER, false);

		if (MEASUREMENTS_SAMPLES != 0)
			voltage /= MEASUREMENTS_SAMPLES;
		Serial.print("my: ");
		Serial.println(voltage);

		voltage = 0.0;

		for (int i = 0; i < MEASUREMENTS_SAMPLES; i++)
			voltage += GetVoltage(ExactMeasurement(), CAPACITORS_DIVIDER, false);

		if (MEASUREMENTS_SAMPLES != 0)
			voltage /= MEASUREMENTS_SAMPLES;
		Serial.print("exact: ");
		Serial.println(voltage);*/
		
		/*
		Serial.print(" \t ");
		Serial.println(MeasurePin(BATTERY_VOLTAGE_SENSOR, BATTERY_DIVIDER, false));
		*/
		//delay(2000);
	}
	else if (state == CHARGE_START)
	{
		Serial.print("Starting charging at");
		Serial.print(VoltageToCharge);
		Serial.println(" V");

		digitalWrite(RELE_1, HIGH);
		digitalWrite(RELE_2, LOW);
		delay(700);
		state = CHARGING;
	}
	else if (state == CHARGING)
	{
		double voltage=0.0;

		for (int i = 0; i < MEASUREMENTS_SAMPLES; i++)
			voltage+= MeasurePin(CAPACITORS_VOLTAGE_SENSOR, CAPACITORS_DIVIDER, true);

		if (MEASUREMENTS_SAMPLES != 0)
			voltage /= MEASUREMENTS_SAMPLES;

		Serial.println(voltage);

		delay(100);

		if (voltage <= VoltageToCharge)
		{
			ledcWrite(0, CHARGE_PWM_ALTERNATE);
		}
		else
		{
			Serial.println("CHARGE COMPLETE");
			ledcWrite(0, 0);

			digitalWrite(RELE_1, LOW);
			digitalWrite(RELE_2, HIGH);

			portENTER_CRITICAL(&mux);
			state = CHARGE_DONE;
			portEXIT_CRITICAL(&mux);
		}
	}
	else if (state == CHARGE_DONE)
	{

	}
	else if (state == SHOOT_START)
	{
		Serial.println("Shoot");
#if START_BY_BUTTON
		SensorsStart();		
#endif
		portENTER_CRITICAL(&mux);
		state = SHOOT;
		portEXIT_CRITICAL(&mux);
			
		SetTimer(1, MAX_TIME_FOR_SENSORS, ShootEndInterrupt,false);
		SetTimer(0, MaxCoilTimes[0], CoilsTimerInterrupt, false);
		//SetTimer
		
		if (CoilSequence[0] != COILS_OFF)
			digitalWrite(COIL[0], HIGH);
	}
	else if (state == SHOOT)
	{

	}
	else if (state == SHOOT_END)
	{	
		for (int i = 0; i < USED_COILS; i++)
			digitalWrite(COIL[i], LOW);

		SensorsEnd();

		for (int i = 0; i < USED_COILS; i++)
			digitalWrite(COIL[i], LOW);

		digitalWrite(RELE_2, LOW);
		state = WAIT;
	}
	else if (state == EMERGENCY_CUT_OFF)
	{
		portENTER_CRITICAL(&mux);
		state = WAIT;
		portEXIT_CRITICAL(&mux);

		for (int i = 0; i < USED_COILS; i++)
			digitalWrite(COIL[i], LOW);

		ledcWrite(0, 0);

		Serial.println("EMERGENCY_CUT_OFF");
		delay(100);
	}
}

void IRAM_ATTR ShootButton_Interrupt()
{
	portENTER_CRITICAL_ISR(&mux);
	if (state == WAIT)
	{
		state = CHARGE_START;
	}
	else if (state == CHARGE_DONE)
	{
		state = SHOOT_START;
	}
	portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR CutOffButton_Interrupt()
{
	portENTER_CRITICAL_ISR(&mux);

	digitalWrite(RELE_1, LOW);
	digitalWrite(RELE_2, LOW);

	for (int i = 0; i < USED_COILS; i++)
		digitalWrite(COIL[i], LOW);

	ledcWrite(0, 0);

	state = EMERGENCY_CUT_OFF;
	portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR SensorInt(byte _sensor)
{
	portENTER_CRITICAL_ISR(&mux);

#if !START_BY_BUTTON 
	if (_sensor == 0)
		SetTimer(1, MAX_TIME_FOR_SENSORS, ShootEnd, false);
#endif

	//Serial.println("coils int");

	for (int i = 0; i < USED_COILS; i++)
		digitalWrite(COIL[i], LOW);
	if (CoilSequence[_sensor + 1] != COILS_OFF && MaxCoilTimes[_sensor + 1] != COILS_OFF)
	{
		digitalWrite(COIL[_sensor + 1], HIGH);
		//Serial.println(_sensor);
	}

	SetTimer(0, MaxCoilTimes[_sensor + 1], CoilsTimerInterrupt, false);

	portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR ShootEndInterrupt()
{
	portENTER_CRITICAL_ISR(&mux);

	state = SHOOT_END;

	portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR CoilsTimerInterrupt()
{
	portENTER_CRITICAL_ISR(&mux);

	Serial.println("COILS_OFF");
	
	for (int i = 0; i < USED_COILS; i++)
		digitalWrite(COIL[i], LOW);

	portEXIT_CRITICAL_ISR(&mux);
}
