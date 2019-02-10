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
			voltage = voltage * voltage * CAPACITORS_CONSTANT_A + voltage * CAPACITORS_CONSTANT_B + CAPACITORS_CONSTANT_C;
			//voltage -= 6;//1.0;
			//if (isReleOpen)
				//voltage -= 2;//4.54;

			if (voltage == CAPACITORS_CONSTANT_C)
				return 0.0;
			else
				return voltage;
		}
		break;
	}

	return 0.0;
}

double ExactMeasurement()
{
#define V_REF 1100  // ADC reference voltage

	// Configure ADC
	adc1_config_width(ADC_WIDTH_12Bit);
	adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_11db);

	// Calculate ADC characteristics i.e. gain and offset factors
	esp_adc_cal_characteristics_t characteristics;
	esp_adc_cal_get_characteristics(V_REF, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, &characteristics);

	// Read ADC and obtain result in mV
	uint32_t voltage = adc1_to_voltage(ADC1_CHANNEL_6, &characteristics);
	//printf("%d mV\n", voltage);

	double v = (double) voltage / 1000.0;
	return v;
}
