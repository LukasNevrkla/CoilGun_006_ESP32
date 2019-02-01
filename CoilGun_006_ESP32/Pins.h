#pragma once

#include "Configuration.h"

//Sellect you device... (just one)
#define CG_v1

#ifdef CG_v1

#define SHOOT_BUTTON 25
#define CUT_OFF_BUTTON 26

#define CHARGING_TRANSISTOR 34
#define CURRENT_SENSOR 27

#define RELE_1 32
#define RELE_2 33

const byte COIL[ALL_COILS] = { 32 }; //D2-D7
const byte SENSOR[ALL_SENSORS] = { 30,31 };

#endif // OLD_CG