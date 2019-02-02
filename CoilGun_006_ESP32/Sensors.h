#ifndef _SENSORS_h
#define _SENSORS_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "Configuration.h"
#include "Pins.h"

#include "Arduino.h"
#include "Configuration.h"
#include "Pins.h"


void SensorsInit(void(*_toCall_interrupt)(byte _sensor), void(*_toCall_end)());
void SensorsStart();
void SensorsEnd();
void SensorInterrupt(byte _sensor);

void CalculateTimes();

//void(*ToCall)(byte);	//in some mysterious way, can not be declared here

void IRAM_ATTR SInt_0();
void IRAM_ATTR SInt_1();
void IRAM_ATTR SInt_2();
void IRAM_ATTR SInt_3();
void IRAM_ATTR SInt_4();
void IRAM_ATTR SInt_5();


#endif

