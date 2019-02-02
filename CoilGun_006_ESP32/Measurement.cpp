// 
// 
// 

#include "Measurement.h"

double MeasurePin(byte pin,byte divider,bool isReleOpen)
{
	double voltage = (double)analogRead(pin);
	voltage = GetDividerVoltage(voltage);
	voltage = GetVoltage(voltage, divider, isReleOpen);

	return voltage;
}

double GetDividerVoltage(uint16_t raw)
{
	return raw * (3.25 / 4095.0);
}

double GetVoltage(double dividerVoltage, byte divider,bool isReleOpen)
{
	switch (divider)
	{
		case BATTERY_DIVIDER:
		{
			double voltage = (double)(dividerVoltage*(BATTERY_DIVIDER_RESISTOR_1 + BATTERY_DIVIDER_RESISTOR_2)) / BATTERY_DIVIDER_RESISTOR_2;
			voltage = voltage * BATTERY_CONSTANT_A + BATTERY_CONSTANT_B;

			if (isReleOpen)
				voltage += 0.1;

			if (voltage == BATTERY_CONSTANT_B)
				return 0.0;
			else
				return voltage;
		}
		break;

	case CAPACITORS_DIVIDER:
		{
			double voltage = ((double)dividerVoltage)*(CAPACITORS_DIVIDER_RESISTOR_1 + CAPACITORS_DIVIDER_RESISTOR_2) / CAPACITORS_DIVIDER_RESISTOR_2;
			voltage = voltage * CAPACITORS_CONSTANT_A + CAPACITORS_CONSTANT_B;

			if (isReleOpen)
				voltage -= 2;//4.54;

			if (voltage == CAPACITORS_CONSTANT_B)
				return 0.0;
			else
				return voltage;
		}
		break;
	}

	return 0.0;
}
