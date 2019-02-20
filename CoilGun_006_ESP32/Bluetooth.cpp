

#include "Bluetooth.h"

BluetoothSerial SerialBT;
int lastData[2];
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
		//Serial.println(data);

		if (data == 'v')	//Format: xxv
		{
			String value = "";

			if (isDigit(lastData[0]) && isDigit(lastData[1])) 
			{
				//Cant be like this: value += (char)lastData[0] + (char)lastData[1];
				value += (char)lastData[0];
				value += (char)lastData[1];

				int data = value.toInt();

				EEPROM.write(EEPROM_VOLTAGE_ADRESS, data);
				EEPROM.commit();

				String txt = "Voltage to charge: " + String(data) + " V";
				Serial.println(txt);
				BluetoothPrintTxt(txt);
			}		
		}

		lastData[0] = lastData[1];
		lastData[1] = data;
	}
}
