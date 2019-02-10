// 
// 
// 

#include "NextionDisplay.h"


void endNextionCommand()
{
	Serial.write(0xff);
	Serial.write(0xff);
	Serial.write(0xff);
}

void SetComponentTxt(String component, double data)
{
	String command = component+".txt=\"" + String(data, 1) + "\"";
	Serial.print(command);
	endNextionCommand();
}
