#pragma once

#include "Configuration.h"

//Sellect you device... (just one)
#define CG_v1

#ifdef CG_v1

const byte COIL[ALL_COILS] = { 4,4,4,4,4,4};
const byte SENSOR[ALL_SENSORS] = { 23,22,1,3,21,19 };

#define BUTTONS_INTERRUPT_PIN 32
#define BUTTONS_READ_PIN 35

#define SHOOT_BUTTON 23
#define CUT_OFF_BUTTON 22

#define CHARGING_TRANSISTOR 12
//#define POTENTIOMETER 13

#define RELE_1 33
#define RELE_2 33

#define BATTERY_VOLTAGE_SENSOR 25
#define CAPACITORS_VOLTAGE_SENSOR 34
#define CURRENT_SENSOR 27

#endif // OLD_CG