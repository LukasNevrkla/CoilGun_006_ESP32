// Measurement.h

#ifndef _MEASUREMENT_h
#define _MEASUREMENT_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "Configuration.h"
#include "States.h"

double MeasurePin(byte pin, byte divider, bool isReleOpen);
double GetDividerVoltage(uint16_t raw);
double GetVoltage(double dividerVoltage, byte divider, bool isReleOpen);

#endif

