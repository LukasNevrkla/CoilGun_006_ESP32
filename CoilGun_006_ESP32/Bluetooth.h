#pragma once

#include "arduino.h"
#include "AssistantFile.h"
#include <BluetoothSerial.h>

extern double VoltageToCharge;

void BluetoothInit();
void BluetoothPrintTxt(String txt);
void BluetoothHandle();


////gabi
