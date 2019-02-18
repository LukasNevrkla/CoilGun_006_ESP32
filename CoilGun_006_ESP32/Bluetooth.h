#pragma once

#include "arduino.h"

#include <BluetoothSerial.h>

//BluetoothSerial SerialBT;

void BluetoothInit();
void BluetoothPrintTxt();
void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);

//void BluetoothPrint()
//{
//	SerialBT.println("DFdf");
//}
/*
#include "arduino.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
String receivedMessage;

// https://www.uuidgenerator.net/
#define SERVICE_UUID           "46ef0173-7c07-4b90-a36e-9b427188a2fe"
#define CHARACTERISTIC_UUID_RX "46ef0173-7c07-4b90-a36e-9b427188a2fe"
#define CHARACTERISTIC_UUID_TX "46ef0173-7c07-4b90-a36e-9b427188a2fe"

class ServerCallbacks : public BLEServerCallbacks 
{
	void onConnect(BLEServer* pServer) 
	{
		deviceConnected = true;
	};

	void onDisconnect(BLEServer* pServer) 
	{
		deviceConnected = false;
	}
};

class Callbacks : public BLECharacteristicCallbacks 
{
	void OnWrite(BLECharacteristic *pCharacteristic);
};

void BluetoothInit();
void BluetoothSendTxt(String txt);
*/
//gabi
