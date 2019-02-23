// AssistantFile.h

#ifndef _ASSISTANTFILE_h
#define _ASSISTANTFILE_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif


#include <EEPROM.h>
#include "Configuration.h"
#include "Pins.h"

//extern double VoltageToCharge;

void PinsInit();
void EEPROM_Init();
void PWM_Init();
bool MotorInit(byte &shiftRegister);
void SetTimer(uint8_t _timer, uint64_t time, void(*interupt)(), bool reload);
void MotorStart(bool _direction, byte &shiftRegister, double stepFrequency);
void MotorStop(byte &shiftRegister);


#endif

