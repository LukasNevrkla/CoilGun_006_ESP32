
#include "Bluetooth.h"
#include "NextionDisplay.h"
#include "Sensors.h"
#include "Measurement.h"
#include "AssistantFile.h"
#include "States.h"


byte state = WAIT;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
byte shiftRegister_1 = 0;
bool projectileCharged = false;

void setup() 
{
	Serial.begin(9600);
	Serial.println("Init");

	PinsInit();
	EEPROM_Init();
	PWM_Init();

	SensorsInit(&SensorInt, mux);
	BluetoothInit();

	attachInterrupt(digitalPinToInterrupt(BUTTONS_INTERRUPT_PIN), ButtonInterrupt, FALLING);

#if  !START_BY_BUTTON
	SensorsStart();
#endif //  !START_BY_BUTTON
}

void loop()
{
	BluetoothHandle();

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
		Serial.println("RAW");

		Serial.print(GetDividerVoltage(analogRead(CAPACITORS_VOLTAGE_SENSOR)));
		Serial.print("  \t "); 
		Serial.println(GetDividerVoltage(analogRead(BATTERY_VOLTAGE_SENSOR)));
		*/
		/*
		double voltage = 0.0;

		for (int i = 0; i < MEASUREMENTS_SAMPLES; i++)
			voltage += MeasurePin(CAPACITORS_VOLTAGE_SENSOR, CAPACITORS_DIVIDER, true);

		if (MEASUREMENTS_SAMPLES != 0)
			voltage /= MEASUREMENTS_SAMPLES;

		Serial.println(voltage);*/
		
		//Serial.print(MeasurePin(CAPACITORS_VOLTAGE_SENSOR, CAPACITORS_DIVIDER, false));
		//Serial.print(" \t ");
		//Serial.println(MeasurePin(BATTERY_VOLTAGE_SENSOR, BATTERY_DIVIDER, false));
		
		//delay(1000);
	}
	else if (state == CHARGE_START)
	{
		String txt = "Starting charging at " + String(EEPROM.read(EEPROM_VOLTAGE_ADRESS)) + " V";
		Serial.println(txt);
		BluetoothPrintTxt(txt);

		shiftRegister_1 |= 0b11000100;	//Rele on and motor sleep off
		shiftRegister_1 |= 0b00001000;	//Motor microstepping
		
		if (STEPPER_START_DIRECTION)
			shiftRegister_1 |= 0b00010000;
		else
			shiftRegister_1 &= ~0b00010000;

		shiftOut(SHIFT_REG_1_DATA, SHIFT_REG_1_CLC, LSBFIRST, shiftRegister_1);

		delay(200);	//wait for rele

		portENTER_CRITICAL(&mux);
		state = CHARGING;
		portEXIT_CRITICAL(&mux);

		if (!projectileCharged)
		{
			ledcWrite(STEPPER_MOTOR_CHANNEL, 126);
			SetTimer(STEPPER_TIMER, (uint64_t)(((double)1 / STEP_FREQUENCY) * 1000000 * STEP_CNT),
				StepperTimerInterrupt, false);
		}

		ledcWrite(CAPACITOR_CHARGER_PWM_CHANNEL, CHARGE_PWM_ALTERNATE);
	}
	else if (state == CHARGING)
	{
		double voltage=0.0;

		for (int i = 0; i < MEASUREMENTS_SAMPLES; i++)
			voltage+= MeasurePin(CAPACITORS_VOLTAGE_SENSOR, CAPACITORS_DIVIDER, true);

		if (MEASUREMENTS_SAMPLES != 0)
			voltage /= MEASUREMENTS_SAMPLES;

		String txt = String (voltage)+ " V";
		Serial.println(txt);
		BluetoothPrintTxt(txt);

		delay(300);

		if (voltage <= EEPROM.read(EEPROM_VOLTAGE_ADRESS))
		{
			//ledcWrite(CAPACITOR_CHARGER_PWM_CHANNEL, CHARGE_PWM_ALTERNATE);
		}
		else
		{
			String txt = "capacitors charging complete";
			Serial.println(txt);
			BluetoothPrintTxt(txt);

			ledcWrite(CAPACITOR_CHARGER_PWM_CHANNEL, 0);

			shiftRegister_1 &= ~0b11000000;		//Turn rele off
			shiftOut(SHIFT_REG_1_DATA, SHIFT_REG_1_CLC, LSBFIRST, shiftRegister_1);

			portENTER_CRITICAL(&mux);
			state = CHARGE_ALMOST_DONE;
			portEXIT_CRITICAL(&mux);
		}
	}
	else if (state == CHARGE_ALMOST_DONE)
	{
		if (projectileCharged)
		{
			portENTER_CRITICAL(&mux);
			state = CHARGE_DONE;
			portEXIT_CRITICAL(&mux);

			String txt = "CHARGING COMPLETE";
			Serial.println(txt);
			BluetoothPrintTxt(txt);
		}
	}
	else if (state == CHARGE_DONE)
	{

	}
	else if (state == SHOOT_START)
	{
		String txt = "Shooting...\n";
		Serial.println(txt);
		BluetoothPrintTxt(txt);

#if START_BY_BUTTON
		SensorsStart();		
#endif

		shiftRegister_1 &= ~0b11000000;
		shiftOut(SHIFT_REG_1_DATA, SHIFT_REG_1_CLC, MSBFIRST, shiftRegister_1);

		portENTER_CRITICAL(&mux);
		state = SHOOT;
		portEXIT_CRITICAL(&mux);

		digitalWrite(SHIFT_REG_0_MR, HIGH);
		digitalWrite(SHIFT_REG_0_DATA, HIGH);
			
		SetTimer(SHOOT_TIMER, MAX_TIME_FOR_SENSORS, ShootTimerInterrupt,false);
		SetTimer(COILS_TIMER, MaxCoilTimes[0], CoilsTimerInterrupt, false);
		
		//Coil I... on
		if (CoilSequence[0] != COILS_OFF)
			digitalWrite(SHIFT_REG_0_CLC, HIGH);

		digitalWrite(SHIFT_REG_0_CLC, LOW);
		digitalWrite(SHIFT_REG_0_DATA, LOW);
	}
	else if (state == SHOOT)
	{
	}
	else if (state == SHOOT_END)
	{
		digitalWrite(SHIFT_REG_0_MR, LOW);

		if (projectileCharged)
		{
			if (!STEPPER_START_DIRECTION)	//Motor dir.
				shiftRegister_1 |= 0b00010000;
			else
				shiftRegister_1 &= ~0b00010000;
			//shiftRegister_1 &= ~(STEPPER_START_DIRECTION << 4);
			shiftOut(SHIFT_REG_1_DATA, SHIFT_REG_1_CLC, LSBFIRST, shiftRegister_1);
			ledcWrite(STEPPER_MOTOR_CHANNEL, 126);

			SetTimer(STEPPER_TIMER, (uint64_t)(((double)1 / STEP_FREQUENCY) * 1000000 * STEP_CNT),
				StepperTimerInterrupt, false);

			portENTER_CRITICAL(&mux);
			state = MOTOR_MOVING;
			portEXIT_CRITICAL(&mux);
		}
		else
		{
			portENTER_CRITICAL(&mux);
			state = WAIT;
			portEXIT_CRITICAL(&mux);

			shiftRegister_1 &= ~0b00000100;	//Motor sleep on
			shiftOut(SHIFT_REG_1_DATA, SHIFT_REG_1_CLC, LSBFIRST, shiftRegister_1);
		}

		SensorsEnd();
	}
	else if (MOTOR_MOVING)
	{
		if (!projectileCharged)
		{
			portENTER_CRITICAL(&mux);
			state = WAIT;
			portEXIT_CRITICAL(&mux);

			shiftRegister_1 &= ~0b00000100;	//Motor sleep on
			shiftOut(SHIFT_REG_1_DATA, SHIFT_REG_1_CLC, LSBFIRST, shiftRegister_1);
		}
	}
	else if (state == EMERGENCY_CUT_OFF)
	{
		digitalWrite(SHIFT_REG_0_MR, LOW);
		ledcWrite(CAPACITOR_CHARGER_PWM_CHANNEL, 0);
		ledcWrite(STEPPER_MOTOR_CHANNEL, 0);

		portENTER_CRITICAL(&mux);
		state = WAIT;
		portEXIT_CRITICAL(&mux);

		shiftRegister_1 = 0;
		shiftOut(SHIFT_REG_1_DATA, SHIFT_REG_1_CLC, LSBFIRST, shiftRegister_1);

		String txt = "EMERGENCY CUT OFF!!";
		Serial.println(txt);
		BluetoothPrintTxt(txt);
		delay(100);
	}
}

void IRAM_ATTR ButtonInterrupt()
{

	pinMode(BUTTONS_INTERRUPT_PIN, OUTPUT);
	detachInterrupt(digitalPinToInterrupt(BUTTONS_INTERRUPT_PIN));

	digitalWrite(BUTTONS_INTERRUPT_PIN, HIGH);

	uint16_t raw = 0;

	for (int i = 0; i < 3; i++)
		raw += analogRead(BUTTONS_READ_PIN);

	raw /= 3;

	portENTER_CRITICAL_ISR(&mux);
	if (raw > 900 && raw < 1300)
	{
		if (state==WAIT)
			state = CHARGE_START;
		else if (state==CHARGE_DONE)
			state = SHOOT_START;
	}
	else if (raw > 1600 && raw < 2000)
		state = EMERGENCY_CUT_OFF;
	else if (raw > 2300 && raw < 2600)	//Stepper end stop
	{/*
		ledcWrite(STEPPER_MOTOR_CHANNEL, 0);

		shiftRegister_1 |= 0b00000100;	//Motor sleep
		shiftOut(SHIFT_REG_1_DATA, SHIFT_REG_1_CLC, LSBFIRST, shiftRegister_1);

		projectileCharged = true;

		String txt = "Projectile charged";
		Serial.println(txt);
		BluetoothPrintTxt(txt);*/
	}
	else if (raw > 3000 && raw < 3400)
		Serial.println("4");
	else if (raw > 3500 && raw < 3900)
		Serial.println("5");
	else
		Serial.print("");

	portEXIT_CRITICAL_ISR(&mux);

	pinMode(BUTTONS_INTERRUPT_PIN, INPUT);
	attachInterrupt(digitalPinToInterrupt(BUTTONS_INTERRUPT_PIN), ButtonInterrupt, FALLING);
}
/*
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
*/
void IRAM_ATTR SensorInt(byte _sensor)
{
	portENTER_CRITICAL_ISR(&mux);

#if !START_BY_BUTTON 
	if (_sensor == 0)
		SetTimer(SHOOT_TIMER, MAX_TIME_FOR_SENSORS, ShootEnd, false);
#endif

	//Serial.println("coils int");

	//for (int i = 0; i < USED_COILS; i++)
		//digitalWrite(COIL[i], LOW);
	if (CoilSequence[_sensor + 1] != COILS_OFF && MaxCoilTimes[_sensor + 1] != COILS_OFF)
	{
		//digitalWrite(COIL[_sensor + 1], HIGH);
		digitalWrite(SHIFT_REG_0_CLC, HIGH);
		digitalWrite(SHIFT_REG_0_CLC, LOW);
	}

	SetTimer(COILS_TIMER, MaxCoilTimes[_sensor + 1], CoilsTimerInterrupt, false);

	portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR ShootTimerInterrupt()
{
	portENTER_CRITICAL_ISR(&mux);

	state = SHOOT_END;

	portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR CoilsTimerInterrupt()
{
	portENTER_CRITICAL_ISR(&mux);

	Serial.println("COILS_OFF");
	
	digitalWrite(SHIFT_REG_0_MR, LOW);

	//for (int i = 0; i < USED_COILS; i++)
		//digitalWrite(COIL[i], LOW);

	portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR StepperTimerInterrupt()		//Stop motor
{
	ledcWrite(STEPPER_MOTOR_CHANNEL,0);
	projectileCharged = !projectileCharged;

	if (projectileCharged)
	{
		String txt = "Projectile charging complete";
		Serial.println(txt);
		BluetoothPrintTxt(txt);
	}
}


