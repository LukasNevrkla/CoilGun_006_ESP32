

#include "Bluetooth.h"

BluetoothSerial SerialBT;
#define LAST_DATA_SIZE 4
int lastData[LAST_DATA_SIZE];
//extern double VoltageToCharge;

void BluetoothInit()
{
	SerialBT.begin("CoilGun Bluetooth");
}

void BluetoothPrintTxt(String txt)
{
	SerialBT.println(txt);
}

void BluetoothHandle()
{
	if (SerialBT.available()) //Check if we receive anything from Bluetooth
	{
		int data = SerialBT.read();

		if (data == 'v')
		{
			int value = BluetoothIntTxtToNumber(lastData,LAST_DATA_SIZE);

			EEPROM.write(EEPROM_VOLTAGE_ADRESS, value);
			EEPROM.commit();

			String txt = "Voltage to charge: " + String(value) + " V";
			Serial.println(txt);
			BluetoothPrintTxt(txt);
		}
		else if (data == 's')
		{
			int value = BluetoothIntTxtToNumber(lastData, LAST_DATA_SIZE);

			String txt = "PWM charging alternate is set to: " + String(value) + " %";
			Serial.println(txt);
			BluetoothPrintTxt(txt);

			value = ((double)value / 100) * 256;

			EEPROM.write(EEPROM_CHARGE_PWM_ALTERNATE, value);
			EEPROM.commit();

			txt = "(" + String(value) + ")\n";
			Serial.println(txt);
			BluetoothPrintTxt(txt);
		}

		for (int i = 0; i < LAST_DATA_SIZE - 1; i++)
			lastData[i] = lastData[i + 1];

		lastData[LAST_DATA_SIZE - 1] = data;
	}
}

int BluetoothIntTxtToNumber(int txt[], int length)
{
	String t = "";

	for (int i = 0;i < length; i++)
	{
		if (isDigit(txt[i]))
			t += (char)txt[i];

		Serial.println(txt[i]);
	}

	return t.toInt();
}
