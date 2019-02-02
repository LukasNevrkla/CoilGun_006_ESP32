
#include "Sensors.h"

void(*ToCall_interrupt)(byte);
//void(*ToCall_end)();

void IRAM_ATTR SInt_0() { SensorInterrupt(0); }
void IRAM_ATTR SInt_1() { SensorInterrupt(1); }
void IRAM_ATTR SInt_2() { SensorInterrupt(2); }
void IRAM_ATTR SInt_3() { SensorInterrupt(3); }
void IRAM_ATTR SInt_4() { SensorInterrupt(4); }
void IRAM_ATTR SInt_5() { SensorInterrupt(5); }

void (*sensorInterrupts[ALL_SENSORS])() = { SInt_0,SInt_1,SInt_2,SInt_3,SInt_4, SInt_5 };
//template<byte _sensor> void IRAM_ATTR SensorInterrupt() { sensorInterrupt(_sensor); }


byte expectedSensor=0;
unsigned volatile long t = 0;
unsigned volatile long rawTimes[ALL_SENSORS * 2];
volatile double resultTimes[ALL_SENSORS * 2];
//volatile double *allSpeeds;


void SensorsInit(void(*_toCall_interrupt)(byte _sensor), void(*_toCall_end)())
{
	ToCall_interrupt = _toCall_interrupt;
	//ToCall_end = _toCall_end;
}

void SensorsStart()
{
	expectedSensor = 0;

	for (byte i = 0; i < USED_SENSORS; i++)
		attachInterrupt(digitalPinToInterrupt(SENSOR[i]), sensorInterrupts[i], RISING);

	t = micros();
}

void SensorsEnd()
{
#if START_BY_BUTTON
	for (byte i = 0; i < USED_SENSORS; i++)
		detachInterrupt(digitalPinToInterrupt(SENSOR[i]));
#endif

	//for (byte i = 0; i < USED_SENSORS*2; i++)
		//Serial.println(rawTimes[i]);

	CalculateTimes();

#if !START_BY_BUTTON
	SensorsStart();
#endif
}

void SensorInterrupt(byte _sensor)
{
	//Serial.print("sensor ");
	//Serial.println(_sensor);

	ToCall_interrupt(_sensor);

	if (_sensor == expectedSensor)
	{
		rawTimes[_sensor * 2] = micros();	//Rising edge is even time

		attachInterrupt(digitalPinToInterrupt(SENSOR[_sensor]), sensorInterrupts[_sensor], FALLING);
		expectedSensor++;

		if (expectedSensor >= USED_SENSORS)
		{
		}
	}
	else if (_sensor > expectedSensor)
		Serial.println("Sensor error");
	//This sensor has been at least one time triggered -> falling edge
	else
	{
		rawTimes[_sensor * 2 + 1] = micros();	//Falling edge is odd time
	}
}

void CalculateTimes()
{
	Serial.println("\n\tTimes...");
	for (int i = 0; i < USED_SENSORS * 2; i++)
	{
		double time;

	#if START_BY_BUTTON
		time = (double)(rawTimes[i] - t);
	#else 
		time = (double)(rawTimes[i] - rawTimes[0]);	//First time ll be 0
	#endif
		time /= 1000000.0;	//To seconds

		resultTimes[i] = time;
		Serial.println(resultTimes[i],9);
	}
}
