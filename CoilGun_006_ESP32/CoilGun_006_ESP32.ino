
#include "Bluetooth.h"
#include "NextionDisplay.h"
#include "Sensors.h"
#include "Measurement.h"
#include "AssistantFile.h"
#include "States.h"

//1110


byte state = WAIT;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
byte shiftRegister_1 = 0;
bool projectileCharged = false;
//unsigned volatile long t = 0;

void setup() 
{
	Serial.begin(115200);
	Serial.println("Init");

	PinsInit();
	EEPROM_Init();
	PWM_Init();
	MotorInit(shiftRegister_1);

	SensorsInit(&SensorInt, mux);
	BluetoothInit();

	attachInterrupt(digitalPinToInterrupt(BUTTONS_INTERRUPT_PIN), ButtonInterrupt, FALLING);
	attachInterrupt(digitalPinToInterrupt(PRESSED_BUTTON_SIG), PressedButtonInterrupt, FALLING);

	if (digitalRead(PRESSED_BUTTON_SIG)==HIGH)//(!IsButtonPressed())	//Motor is not in base position
	{
		String txt = "Motor calibration";
		Serial.println(txt);
		BluetoothPrintTxt(txt);

		MotorStart(BACKWARD, shiftRegister_1, STEP_SLOW_FREQUENCY);

		portENTER_CRITICAL(&mux);
		state = MOTOR_CALIBRATION;
		portEXIT_CRITICAL(&mux);
	}
	else
	{
		String txt = "Motor calibrated";
		Serial.println(txt);
		BluetoothPrintTxt(txt);
	}

#if  !START_BY_BUTTON
	SensorsStart();
#endif //  !START_BY_BUTTON
}

void loop()
{
	if (state!=SHOOT)
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

		shiftRegister_1 |= 0b00000011;	//Rele on
		shiftOut(SHIFT_REG_1_DATA, SHIFT_REG_1_CLC, LSBFIRST, shiftRegister_1);

		delay(200);	//wait for rele

		if (!projectileCharged)
		{
			MotorStart(FORWARD,shiftRegister_1,STEP_FREQUENCY);
			SetTimer(STEPPER_TIMER, (uint64_t)(((double)1 / STEP_FREQUENCY) * 1000000 * STEP_CNT),
				StepperTimerInterrupt, false);
		}

		ledcWrite(CAPACITOR_CHARGER_PWM_CHANNEL, EEPROM.read(EEPROM_CHARGE_PWM_ALTERNATE));//CHARGE_PWM_ALTERNATE);

		portENTER_CRITICAL(&mux);
		state = CHARGING;
		portEXIT_CRITICAL(&mux);
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

		delay(200);

		if (voltage <= EEPROM.read(EEPROM_VOLTAGE_ADRESS))
		{
			ledcWrite(CAPACITOR_CHARGER_PWM_CHANNEL, EEPROM.read(EEPROM_CHARGE_PWM_ALTERNATE));
		}
		else
		{
			String txt = "capacitors charging complete";
			Serial.println(txt);
			BluetoothPrintTxt(txt);

			ledcWrite(CAPACITOR_CHARGER_PWM_CHANNEL, 0);

			shiftRegister_1 &= ~0b00000011;		//Turn rele off
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

		shiftRegister_1 &= ~0b00000011;
		shiftOut(SHIFT_REG_1_DATA, SHIFT_REG_1_CLC, LSBFIRST, shiftRegister_1);

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
			MotorStart(BACKWARD, shiftRegister_1, STEP_FREQUENCY);

			portENTER_CRITICAL(&mux);
			state = MOTOR_CALIBRATION;
			portEXIT_CRITICAL(&mux);
		}
		else
		{
			portENTER_CRITICAL(&mux);
			state = WAIT;
			portEXIT_CRITICAL(&mux);
		}

		SensorsEnd();

		delay(10);
	}
	else if (state == EMERGENCY_CUT_OFF)
	{
		digitalWrite(SHIFT_REG_0_MR, LOW);
		ledcWrite(CAPACITOR_CHARGER_PWM_CHANNEL, 0);
		//ledcWrite(STEPPER_MOTOR_CHANNEL, 0);
		MotorStop(shiftRegister_1);

		shiftRegister_1 = 0;
		shiftOut(SHIFT_REG_1_DATA, SHIFT_REG_1_CLC, MSBFIRST, shiftRegister_1);

		String txt = "EMERGENCY CUT OFF!!";
		Serial.println(txt);
		BluetoothPrintTxt(txt);
		delay(100);

		portENTER_CRITICAL(&mux);
		state = WAIT;
		portEXIT_CRITICAL(&mux);
	}
	else if (state == MOTOR_CALIBRATION)
	{
	}
}

void IRAM_ATTR ButtonInterrupt()
{
	pinMode(BUTTONS_INTERRUPT_PIN, OUTPUT);
	detachInterrupt(digitalPinToInterrupt(BUTTONS_INTERRUPT_PIN));

	digitalWrite(BUTTONS_INTERRUPT_PIN, HIGH);

	uint16_t raw = 0;
	byte samples = 1;

	for (int i = 0; i < samples; i++)
		raw += analogRead(BUTTONS_READ_PIN);

	raw /= samples;

	//Serial.println(raw);

	portENTER_CRITICAL_ISR(&mux);

	if (raw > 900 && raw < 1300)
	{
		if (state==WAIT)
			state = CHARGE_START;
		else if (state==CHARGE_DONE)
			state = SHOOT_START;
	}
	else if (raw > 1600 && raw < 2000)
	{
		//Serial.println("CUTT OFF");
		state = EMERGENCY_CUT_OFF;
	}
	else if (raw > 2300 && raw < 3300)	//Stepper end-stop
	{
	}
	else if (raw > 3300 && raw < 3400)
		Serial.println("4");
	else if (raw > 3500 && raw < 3900)
		Serial.println("5");
	else
	{
	}

	portEXIT_CRITICAL_ISR(&mux);

	//digitalWrite(BUTTONS_INTERRUPT_PIN, LOW);
	pinMode(BUTTONS_INTERRUPT_PIN, INPUT);
	attachInterrupt(digitalPinToInterrupt(BUTTONS_INTERRUPT_PIN), ButtonInterrupt, FALLING);
}

void IRAM_ATTR PressedButtonInterrupt()
{
	if (state == MOTOR_CALIBRATION)
	{
		MotorStop(shiftRegister_1);

		String txt = "Motor calibrated";
		Serial.println(txt);
		BluetoothPrintTxt(txt);

		projectileCharged = false;

		portENTER_CRITICAL_ISR(&mux);
		state = WAIT;
		portEXIT_CRITICAL_ISR(&mux);
	}
}

void IRAM_ATTR SensorInt(byte _sensor)
{
	portENTER_CRITICAL_ISR(&mux);

#if !START_BY_BUTTON 
	if (_sensor == 0)
		SetTimer(SHOOT_TIMER, MAX_TIME_FOR_SENSORS, ShootEnd, false);
#endif

	if (CoilSequence[_sensor + 1] != COILS_OFF && MaxCoilTimes[_sensor + 1] != COILS_OFF)
	{
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
	
	digitalWrite(SHIFT_REG_0_MR, LOW);	//TurnNextCoil

	portEXIT_CRITICAL_ISR(&mux);
}	//Warning 

void IRAM_ATTR StepperTimerInterrupt()		//Stop motor
{
	MotorStop(shiftRegister_1);
	projectileCharged = true;//!projectileCharged;

	//if (projectileCharged)
	//{
		String txt = "Projectile charging complete";
		Serial.println(txt);
		BluetoothPrintTxt(txt);
	//}
}
/*
bool IsButtonPressed()
{
	bool result = false;

	pinMode(BUTTONS_INTERRUPT_PIN, OUTPUT);
	detachInterrupt(digitalPinToInterrupt(BUTTONS_INTERRUPT_PIN));
	digitalWrite(BUTTONS_INTERRUPT_PIN, HIGH);

	uint16_t data = analogRead(BUTTONS_READ_PIN);
	//Serial.println(data);

	if (data > 2300 && data < 3300)	//Stepper end-stop
		result = true;
	else
		result = false;

	pinMode(BUTTONS_INTERRUPT_PIN, INPUT);
	attachInterrupt(digitalPinToInterrupt(BUTTONS_INTERRUPT_PIN), ButtonInterrupt, FALLING);

	return result;
}
*/

