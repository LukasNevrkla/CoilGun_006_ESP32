
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
double resultTimes[ALL_SENSORS * 2];
double allSpeeds[SpeedCalcCNT];

volatile bool isSecondTime = false;


void SensorsInit(void(*_toCall_interrupt)(byte _sensor), portMUX_TYPE _mux)
{
	ToCall_interrupt = _toCall_interrupt;
	//mux_second = _mux;

	for (int i = 0; i < USED_SENSORS; i++)
	{
		if (digitalRead(SENSOR[i]) == HIGH)
		{
			Serial.print("Error on ");
			Serial.print(i);
			Serial.println(". sensor!!");
		}
	}
}

void SensorsStart()
{
	portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;

	portENTER_CRITICAL(&_mux);

	for (int i = 0; i < USED_SENSORS; i++)
	{
		if (digitalRead(SENSOR[i]) == HIGH)
		{
			Serial.print("Error on ");
			Serial.print(i);
			Serial.println(". sensor!!");
		}
	}

	expectedSensor = 0;

	for (byte i = 0; i < USED_SENSORS; i++)
		attachInterrupt(digitalPinToInterrupt(SENSOR[i]), sensorInterrupts[i], RISING);

	t = micros();

	portEXIT_CRITICAL(&_mux);
}

void SensorsEnd()
{
	portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;

#if START_BY_BUTTON
	for (byte i = 0; i < USED_SENSORS; i++)
		detachInterrupt(digitalPinToInterrupt(SENSOR[i]));
#endif

	Serial.println("//////////");
	Serial.println("//Shoot//");
	Serial.println("//////////");

	CalculateTimes();
	CalculateSpeeds();

	portENTER_CRITICAL(&_mux);
	isSecondTime = false;
	portEXIT_CRITICAL(&_mux);

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
	{
		Serial.print("S err ");
		Serial.println(_sensor);
	}
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
	
	portENTER_CRITICAL(&_mux);
	Serial.println(t, 9);
	Serial.println();
	portEXIT_CRITICAL(&_mux);

	for (int i = 0; i < USED_SENSORS * 2; i++)
	{
		portENTER_CRITICAL(&_mux);
		//Serial.println(rawTimes[i], 9);
		portEXIT_CRITICAL(&_mux);
	}

	Serial.println("\n\tTimes...");
	//BluetoothPrintTxt("Times...");

	for (int i = 0; i < USED_SENSORS * 2; i++)
	{
		double time;

		portENTER_CRITICAL(&_mux);
	#if START_BY_BUTTON
		time = (double)(rawTimes[i] - t);
	#else 
		time = (double)(rawTimes[i] - rawTimes[0]);	//First time ll be 0
	#endif

		rawTimes[i] = 0;
		portEXIT_CRITICAL(&_mux);

		time /= 1000000.0;	//To seconds
		resultTimes[i] = time;

		Serial.println(resultTimes[i],9);
		BluetoothPrintTxt(String(resultTimes[i], 6)); //+ " ms");
		delay(20);
	}

	BluetoothPrintTxt("");
}

void CalculateSpeeds()
{
	Serial.println("\nSpeeds between times ...\n");
	//BluetoothPrintTxt("Speeds...");

	for (int i = 0; i < SpeedCalcCNT; i++)
	{
		Serial.print("Speed ");
		Serial.print(i);
		Serial.print(": ");
		Serial.print(SpeedCalculations[i].distance);
		Serial.print(" / time ");
		Serial.print(SpeedCalculations[i].time_2);
		Serial.print(" - time ");
		Serial.println(SpeedCalculations[i].time_1);
	}
	Serial.println();

	for (int i = 0; i < SpeedCalcCNT ; i++)
	{
		double speed;
		double deltaTime = resultTimes[SpeedCalculations[i].time_2]
			- resultTimes[SpeedCalculations[i].time_1];

		if (deltaTime != 0)
			speed = SpeedCalculations[i].distance / deltaTime;
		else
			speed = 0;

		allSpeeds[i] = speed;
		Serial.println(allSpeeds[i], 10);

		BluetoothPrintTxt(String(allSpeeds[i], 6)); //+ " m/s");
		delay(30);
	}

	Serial.println();
	Serial.println();

	BluetoothPrintTxt("");
	BluetoothPrintTxt("");
}
