// AssistantFile.h

#ifndef _ASSISTANTFILE_h
#define _ASSISTANTFILE_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "Configuration.h"
#include "Pins.h"


void PinsInit();

#endif

