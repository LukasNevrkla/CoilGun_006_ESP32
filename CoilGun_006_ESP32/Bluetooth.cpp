

#include "Bluetooth.h"



/*
void Callbacks::OnWrite(BLECharacteristic * pCharacteristic)
{
	receivedMessage = pCharacteristic->getValue();

	if (receivedMessage.length() > 0)
	{
		Serial.println("Prijata zprava: ");
		for (int i = 0; i < receivedMessage.length(); i++) 
			Serial.print(receivedMessage[i]);

		Serial.println();
	}
}

void BluetoothInit()
{
	BLEDevice::init("CoilGun Bluetooth");
	BLEServer *pServer = BLEDevice::createServer();
	pServer->setCallbacks(new ServerCallbacks());
	BLEService *pService = pServer->createService(SERVICE_UUID);
	pCharacteristic = pService->createCharacteristic(
		CHARACTERISTIC_UUID_TX,
		BLECharacteristic::PROPERTY_NOTIFY
	);
	pCharacteristic->addDescriptor(new BLE2902());
	BLECharacteristic *pCharacteristic = pService->createCharacteristic(
		CHARACTERISTIC_UUID_RX,
		BLECharacteristic::PROPERTY_WRITE
	);
	pCharacteristic->setCallbacks(new Callbacks());
	pService->start();

	pServer->getAdvertising()->start();
	Serial.println("Bluetooth ready...");
}

void BluetoothSendTxt(String txt)
{
	if (deviceConnected == true)
	{
		char *txtChar;// [txt.length() + 1];
		txt.toCharArray(txtChar, txt.length() + 1);
		pCharacteristic->setValue(txtChar);

		pCharacteristic->notify();
		Serial.print("*** Odeslana zprava: ");
		Serial.print(txtChar);
	}
}
*/

BluetoothSerial SerialBT;

void BluetoothInit()
{
	if (!SerialBT.begin("CoilGun Bluetooth")) 
		Serial.println("An error occurred initializing Bluetooth");
	else 
		Serial.println("Bluetooth initialized");
}

void BluetoothPrintTxt()
{
}

void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
	if (event == ESP_SPP_SRV_OPEN_EVT) {
		Serial.println("Client Connected");
	}
}
