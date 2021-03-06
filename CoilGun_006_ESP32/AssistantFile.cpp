#include "AssistantFile.h"

hw_timer_t * Timers[4] = { NULL,NULL,NULL,NULL };
//double VoltageToCharge = PREDEFINED_VOLTAGE_TO_CHARGE;

void PinsInit()
{
	for (byte i = 0; i < ALL_SENSORS; i++)
		pinMode(SENSOR[i], INPUT_PULLUP);

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

	byte isLoaded = EEPROM.read(EEPROM_IS_LOADED_ADRESS);

	if (isLoaded != 1 && isLoaded != 0)
	{
		EEPROM.write(EEPROM_IS_LOADED_ADRESS, false);
		EEPROM.commit();
	}

	int f = EEPROM.read(EEPROM_CHARGE_PWM_FREQUENCY);

	if (f < 1)
	{
		EEPROM.write(EEPROM_CHARGE_PWM_FREQUENCY, CHARGE_FREQUENCY);
		EEPROM.commit();
	}

	byte p = EEPROM.read(EEPROM_PRINT_ALL_TIMES);

	if (p != 0 && p != 1)
	{
		EEPROM.write(EEPROM_PRINT_ALL_TIMES, 0);
		EEPROM.commit();
	}
}

void PWM_Init()
{
	int f = EEPROM.read(EEPROM_CHARGE_PWM_FREQUENCY);
	if (f < 1)
	{
		EEPROM.write(EEPROM_CHARGE_PWM_FREQUENCY, CHARGE_FREQUENCY);
		EEPROM.commit();
		f = CHARGE_FREQUENCY;
	}
	ledcSetup(CAPACITOR_CHARGER_PWM_CHANNEL, (double)f * 1000.0, 8);
	//ledcSetup(CAPACITOR_CHARGER_PWM_CHANNEL, CHARGE_FREQUENCY, 8);
	ledcAttachPin(CHARGING_TRANSISTOR, CAPACITOR_CHARGER_PWM_CHANNEL);
}

void MotorInit(byte &shiftRegister)
{
	shiftRegister |= 1 << 4;	//Motor microstepping
	shiftRegister &= ~(1 << 5);	//Motor sleep on

	shiftOut(SHIFT_REG_1_DATA, SHIFT_REG_1_CLC, MSBFIRST, shiftRegister);

	ledcSetup(STEPPER_MOTOR_CHANNEL, STEP_FREQUENCY, 8);
	ledcAttachPin(STEPPER_MOTOR, STEPPER_MOTOR_CHANNEL);
}

void SetTimer(uint8_t _timer, uint64_t time, void(*interupt)(), bool reload)		//time in us
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

byte CheckBatteryVoltage(double voltage, byte cells)
{
	if (voltage <= 4.3*cells && voltage>5)
	{
		if (voltage < BATTERY_MIN_VOLTAGE_PER_CELL*cells)
			return LOW;
		else
			return OK;
	}
	return UNKNOWN;
}

