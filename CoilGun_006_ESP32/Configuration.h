#pragma once

#include "States.h"


/////////
//Shoot//
/////////

#define USED_COILS 2
#define USED_SENSORS 2

#define ALL_COILS 2
#define ALL_SENSORS 6

#define START_BY_BUTTON true

#define MAX_TIME_FOR_SENSORS 3000000
//#define MAX_COILS_ON_TIME 80000

const unsigned long MaxCoilTimes[USED_SENSORS + 1] =
{ 80000,100000,COILS_OFF };

const byte CoilSequence[USED_SENSORS+1] =
{ 1,1,COILS_OFF}; 

#define LENGTH  0.039 //0.07
//If START_BY_BUTTON... first time ll be 0
const double Distances[USED_SENSORS * 2] =
{ 0, LENGTH, 0.2 - LENGTH, LENGTH };	//in meters

//////////
//CHARGE//
//////////

#define VOLTAGE_TO_CHARGE 10.0
#define CHARGE_FREQUENCY 31000	//HZ
#define CHARGE_PWM_ALTERNATE 100

///////////////
//MEASUREMENT//
///////////////

#define MEASUREMENTS_SAMPLES 10

#define CAPACITORS_DIVIDER_RESISTOR_1 10000000.0
#define CAPACITORS_DIVIDER_RESISTOR_2 470000.0

#define CAPACITORS_CONSTANT_A 0.0//-0.000133285	//1.23625
#define CAPACITORS_CONSTANT_B 1.0574 //1.06349	//3.18795
#define CAPACITORS_CONSTANT_C -0.148//2.1976

#define BATTERY_DIVIDER_RESISTOR_1 460000.0
#define BATTERY_DIVIDER_RESISTOR_2 85000.0

#define BATTERY_CONSTANT_A 1.02287
#define BATTERY_CONSTANT_B 0.109502

