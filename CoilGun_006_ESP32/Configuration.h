#pragma once

#include "States.h"


/////////
//Shoot//
/////////

#define USED_COILS 2
#define USED_SENSORS 2

#define ALL_COILS 6
#define ALL_SENSORS 6

#define START_BY_BUTTON true

#define MAX_TIME_FOR_SENSORS 1000000

const unsigned long MaxCoilTimes[USED_SENSORS + 1] =
{ 70000,30000,COILS_OFF };

const byte CoilSequence[USED_SENSORS+1] =
{ 1,1,COILS_OFF}; 

#define LENGTH  0.039 //0.07
struct SpeedCalculation
{
	byte time_1;
	byte time_2;
	double distance;
};

#define SpeedCalcCNT 3

const SpeedCalculation SpeedCalculations[SpeedCalcCNT] = {
	{0,1,LENGTH},{2,3,LENGTH},{0,2,0.08} };

//////////
//CHARGE//
//////////

#define PREDEFINED_VOLTAGE_TO_CHARGE 23.0
#define CHARGE_FREQUENCY 20000//31000//60000	//HZ//nej 20-31khz
#define CHARGE_PWM_ALTERNATE 130//110

/////////////////
//STEPPER MOTOR//
/////////////////

#define STEP_CNT 150
#define STEPPER_START_DIRECTION FORWARD
#define STEP_FREQUENCY 700 //Hz (max. 250kHz)
#define STEP_SLOW_FREQUENCY 400 //Hz (max. 250kHz)


///////////////
//MEASUREMENT//
///////////////

#define MEASUREMENTS_SAMPLES 10

#define CAPACITORS_DIVIDER_RESISTOR_1 10000000.0
#define CAPACITORS_DIVIDER_RESISTOR_2 470000.0

#define CAPACITORS_CONSTANT_A 0.0//-0.000133285	//1.23625
#define CAPACITORS_CONSTANT_B 1.13924//1.0574 //1.06349	//3.18795
#define CAPACITORS_CONSTANT_C 3.60759//-0.148//2.1976

#define BATTERY_DIVIDER_RESISTOR_1 467000.0
#define BATTERY_DIVIDER_RESISTOR_2 93700.0

#define BATTERY_CONSTANT_A 1.02287
#define BATTERY_CONSTANT_B 0.109502

