#pragma once

#include "arduino.h"


/*
NexText VoltToCharge = NexText(0, 2, "ToChargeTxt");
NexText CurrentVoltage = NexText(0, 5, "CurrTxt");
NexButton Plus = NexButton(0, 7, "plus");
NexButton Minus = NexButton(0, 8, "minus");*/

#define TO_CHARGE_TXT "ToChargeTxt"
#define CURR_TXT "CurrTxt"

void SetComponentTxt(String component, double data);