
#include "Sensors.h"

void(*ToCall_interrupt)(byte);
//void(*ToCall_end)();

//portMUX_TYPE mux_second = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR SInt_0() { SensorInterrupt(0); }
void IRAM_ATTR SInt_1() { SensorInterrupt(1); }
void IRAM_ATTR SInt_2() { SensorInterrupt(2); }
void IRAM_ATTR SInt_3() { SensorInterrupt(3); }
void IRAM_ATTR SInt_4() { SensorInterrupt(4); }
void IRAM_ATTR SInt_5() { SensorInterrupt(5); }

void (*sensorInterrupts[ALL_SENSORS])() = { SInt_0,SInt_1,SInt_2,SInt_3,SInt_4, SInt_5 };
//template<byte _sensor> void IRAM_ATTR SensorInterrupt() { sensorInterrupt(_sensor); }


volatile byte expectedSensor=0;
unsigned volatile long t = 0;
unsigned volatile long rawTimes[ALL_SENSORS * 2];
volatile double resultTimes[ALL_SENSORS * 2];
volatile double allSpeeds[ALL_SENSORS * 2];

volatile bool isSecondTime = false;



void SensorsInit(void(*_toCall_interrupt)(byte _sensor), portMUX_TYPE _mux)
{
	ToCall_interrupt = _toCall_interrupt;
	//mux_second = _mux;
}

void SensorsStart()
{
	portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;

	portENTER_CRITICAL(&_mux);

	expectedSensor = 0;

	for (byte i = 0; i < USED_SENSORS; i++)
		attachInterrupt(digitalPinToInterrupt(SENSOR[i]), sensorInterrupts[i], RISING);

	t = micros();

	portEXIT_CRITICAL(&_mux);
}

void SensorsEnd()
{
#if START_BY_BUTTON
	for (byte i = 0; i < USED_SENSORS; i++)
		detachInterrupt(digitalPinToInterrupt(SENSOR[i]));
#endif

	Serial.println("//////////");
	Serial.println("//Shoot//");
	Serial.println("//////////");

	CalculateTimes();
	CalculateSpeeds();

	isSecondTime = false;

#if !START_BY_BUTTON
	SensorsStart();
#endif
}

void IRAM_ATTR SensorInterrupt(byte _sensor)
{
	//unsigned long time = micros();
	//ToCall_interrupt(_sensor);
	
	portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;
	portENTER_CRITICAL_ISR(&_mux);

	Serial.println(_sensor);

	if (_sensor == expectedSensor)
	{
		if ((_sensor > 0 && isSecondTime) || _sensor == 0)
		{
			ToCall_interrupt(_sensor);
			rawTimes[_sensor * 2] = micros();	//Rising edge is even time

			//!!!!!!!!xTaskGetTickCount(); //version of millis() that works from interrupt!!!!!!!!!!!!!!!!!!!

			attachInterrupt(digitalPinToInterrupt(SENSOR[_sensor]), sensorInterrupts[_sensor], FALLING);
			expectedSensor++;

			if (expectedSensor >= USED_SENSORS)
			{
			}
			
			isSecondTime = false;
		}
		else if (_sensor > 0 && !isSecondTime)
			isSecondTime = true;
	}
	else if (_sensor > expectedSensor)
		Serial.println("Sensor error");
	//This sensor has been at least one time triggered -> falling edge
	else
	{
		rawTimes[_sensor * 2 + 1] = micros();	//Falling edge is odd time
	}

	portEXIT_CRITICAL_ISR(&_mux);
	//Serial.println(micros() - time);	//max 10 us
}

void CalculateTimes()
{
	portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;
	
	for (int i = 0; i < USED_SENSORS * 2; i++)
	{
		Serial.println(rawTimes[i], 9);
	}

	Serial.println("\n\tTimes...");
	for (int i = 0; i < USED_SENSORS * 2; i++)
	{
		double time;

		portENTER_CRITICAL(&_mux);
	#if START_BY_BUTTON
		time = (double)(rawTimes[i] - t);
	#else 
		time = (double)(rawTimes[i] - rawTimes[0]);	//First time ll be 0
	#endif
		portEXIT_CRITICAL(&_mux);
		time /= 1000000.0;	//To seconds

		resultTimes[i] = time;
		Serial.println(resultTimes[i],9);
	}
}

void CalculateSpeeds()
{
	Serial.println("\nSpeeds...\n");

	for (int i = 0; i < USED_SENSORS * 2; i++)
	{
		double time_1, time_2, deltaTime, distance, speed;

		if (i == 0)
			time_1 = 0;
		else
			time_1 = resultTimes[i - 1];

		time_2 = resultTimes[i];

		distance = Distances[i];
		deltaTime= time_2 - time_1;

		//Serial.println();
		//Serial.println(deltaTime,10);

		if (deltaTime != 0)
			speed = distance / (time_2 - time_1);
		else
			speed = 0;

		//speed /= 100.0; //to m/s from cm/s
		allSpeeds[i] = speed;
		Serial.println(allSpeeds[i],10);
	}

	Serial.println();
	Serial.println();
}
