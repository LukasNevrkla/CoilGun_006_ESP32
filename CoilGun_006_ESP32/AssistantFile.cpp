#include "AssistantFile.h"

hw_timer_t * Timers[4] = { NULL,NULL,NULL,NULL };
//double VoltageToCharge = PREDEFINED_VOLTAGE_TO_CHARGE;

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

	pinMode(SHIFT_REG_0_CLC, OUTPUT);
	pinMode(SHIFT_REG_0_DATA, OUTPUT);
	pinMode(SHIFT_REG_0_MR, OUTPUT);

	pinMode(SHIFT_REG_1_CLC, OUTPUT);
	pinMode(SHIFT_REG_1_DATA, OUTPUT);
	pinMode(SHIFT_REG_1_MR, OUTPUT);

	digitalWrite(SHIFT_REG_0_MR, LOW);
	digitalWrite(SHIFT_REG_1_MR, HIGH);

	pinMode(CURRENT_SENSOR, INPUT);
	pinMode(STEPPER_MOTOR, OUTPUT);

	pinMode(CHARGING_TRANSISTOR, OUTPUT);
	digitalWrite(CHARGING_TRANSISTOR, LOW);

	pinMode(BATTERY_VOLTAGE_SENSOR, INPUT_PULLDOWN);
	pinMode(CAPACITORS_VOLTAGE_SENSOR, INPUT_PULLDOWN);

	pinMode(PRESSED_BUTTON_SIG, INPUT_PULLUP);
	pinMode(PRESSED_BUTTON_GND, OUTPUT);
	digitalWrite(PRESSED_BUTTON_GND, LOW);
}

void EEPROM_Init()
{
	EEPROM.begin(EEPROM_SIZE);

	byte voltage = EEPROM.read(EEPROM_VOLTAGE_ADRESS);

	if (!(voltage > 0 && voltage < 80))
	{
		EEPROM.write(EEPROM_VOLTAGE_ADRESS, PREDEFINED_VOLTAGE_TO_CHARGE);
		EEPROM.commit();
	}
}

void PWM_Init()
{
	ledcSetup(CAPACITOR_CHARGER_PWM_CHANNEL, CHARGE_FREQUENCY, 8);
	ledcAttachPin(CHARGING_TRANSISTOR, CAPACITOR_CHARGER_PWM_CHANNEL);
}

bool MotorInit(byte &shiftRegister)
{
	shiftRegister |= 1 << 4;	//Motor microstepping
	shiftRegister &= ~(1 << 5);	//Motor sleep on

	shiftOut(SHIFT_REG_1_DATA, SHIFT_REG_1_CLC, MSBFIRST, shiftRegister);

	ledcSetup(STEPPER_MOTOR_CHANNEL, STEP_FREQUENCY, 8);
	ledcAttachPin(STEPPER_MOTOR, STEPPER_MOTOR_CHANNEL);

	return true;
	/*
	pinMode(BUTTONS_INTERRUPT_PIN, OUTPUT);
	//detachInterrupt(digitalPinToInterrupt(BUTTONS_INTERRUPT_PIN));
	digitalWrite(BUTTONS_INTERRUPT_PIN, HIGH);
	 
	int data = analogRead(BUTTONS_READ_PIN);
	if (data > 2300 && data < 3300)
		return false;
	else
		return true;

	Serial.println(analogRead(BUTTONS_READ_PIN));

	pinMode(BUTTONS_INTERRUPT_PIN, INPUT);*/
	//attachInterrupt(digitalPinToInterrupt(BUTTONS_INTERRUPT_PIN), ButtonInterrupt, FALLING);
}

void SetTimer(uint8_t _timer, uint64_t time, void(*interupt)(), bool reload)
{
	//Timer interrupt  https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/
	Timers[_timer] = timerBegin(_timer, 80, true);
	timerAttachInterrupt(Timers[_timer], interupt, true);
	timerAlarmWrite(Timers[_timer], time, reload);
	timerAlarmEnable(Timers[_timer]);
}

void MotorStart(bool _direction, byte &shiftRegister, double stepFrequency)
{
	shiftRegister |= 1 << 5;	//Motor sleep off

	if (_direction == FORWARD)	//Motor dir
		shiftRegister |= 1 << 3;
	else
		shiftRegister &= ~(1 << 3);

	shiftOut(SHIFT_REG_1_DATA, SHIFT_REG_1_CLC, MSBFIRST, shiftRegister);

	ledcSetup(STEPPER_MOTOR_CHANNEL, stepFrequency, 8);
	ledcAttachPin(STEPPER_MOTOR, STEPPER_MOTOR_CHANNEL);
	ledcWrite(STEPPER_MOTOR_CHANNEL, 126);
}
void MotorStop(byte &shiftRegister)
{
	shiftRegister &= ~(1 << 5);	//Motor sleep on

	shiftOut(SHIFT_REG_1_DATA, SHIFT_REG_1_CLC, MSBFIRST, shiftRegister);
	ledcWrite(STEPPER_MOTOR_CHANNEL, 0);
}

