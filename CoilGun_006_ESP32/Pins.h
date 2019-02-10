#pragma once

#include "Configuration.h"

//Sellect you device... (just one)
#define CG_v1

#ifdef CG_v1

#define SHOOT_BUTTON 23
#define CUT_OFF_BUTTON 22

#define CHARGING_TRANSISTOR 12
#define POTENTIOMETER 13
#define CURRENT_SENSOR 27

#define RELE_1 32
#define RELE_2 33

#define BATTERY_VOLTAGE_SENSOR 25
#define CAPACITORS_VOLTAGE_SENSOR 34

const byte COIL[USED_COILS] = { 4,5}; 
const byte SENSOR[USED_SENSORS] = { 21,19 };

#endif // OLD_CG