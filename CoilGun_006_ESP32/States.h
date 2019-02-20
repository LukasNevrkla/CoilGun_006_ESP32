#pragma once

///////////////
//Main states//
///////////////

#define WAIT 0
#define CHARGE_START 1
#define CHARGING 2
#define CHARGE_ALMOST_DONE 3	//Waiting for motor
#define CHARGE_DONE 4
#define SHOOT_START 5
#define SHOOT 6
#define SHOOT_END 7
#define MOTOR_MOVING 8
#define EMERGENCY_CUT_OFF 9

////////////
//DIVIDERS//
////////////

#define CAPACITORS_DIVIDER 0
#define BATTERY_DIVIDER 1

#define COILS_OFF 200

//////////
//EEPROM//
//////////

#define EEPROM_SIZE 2

#define EEPROM_VOLTAGE_ADRESS 0
#define EEPROM_IS_LOADED_ADRESS 1

/////////////////
//STEPPER MOTOR//
/////////////////

#define CAPACITOR_CHARGER_PWM_CHANNEL 0
#define STEPPER_MOTOR_CHANNEL 1

#define BACKWARD 0
#define FORWARD 1

//////////
//TIMERS//
//////////

#define SHOOT_TIMER 0
#define COILS_TIMER 1
#define STEPPER_TIMER 2
