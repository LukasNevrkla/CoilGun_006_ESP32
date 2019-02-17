#pragma once

#include "Configuration.h"

//Sellect you device... (just one)
#define CG_v1

#ifdef CG_v1

const byte COIL[ALL_COILS] = { 4,4,4,4,4,4};
const byte SENSOR[ALL_SENSORS] = { 23,22,1,3,21,19 };

#define BUTTONS_INTERRUPT_PIN 32
#define BUTTONS_READ_PIN 35

//COILS
#define SHIFT_REG_0_DATA 38
#define SHIFT_REG_0_MR 25
#define SHIFT_REG_0_CLC 26

#define SHIFT_REG_1_DATA 27
#define SHIFT_REG_1_MR 14
#define SHIFT_REG_1_CLC 13

//#define SHOOT_BUTTON 23
//#define CUT_OFF_BUTTON 22

#define CHARGING_TRANSISTOR 2
//#define POTENTIOMETER 13

#define RELE_1 4
#define RELE_2 4

#define BATTERY_VOLTAGE_SENSOR 34
#define CAPACITORS_VOLTAGE_SENSOR 39
#define CURRENT_SENSOR 4

#endif // OLD_CG