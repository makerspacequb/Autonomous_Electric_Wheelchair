//------------------------------------------------------------------------------------------------------------------
// NAME: pins.h
// AUTH: Ryan McCartney
// DATE: 22nd July 2019
// DESC: Pin Defintiions for Autonomous Wheelchair
// NOTE: All Rights Reserved, 2019, QLab Makerspace
//------------------------------------------------------------------------------------------------------------------

#ifndef PINS_H
#define PINS_H
#include "Arduino.h"

//Right Motor
#define RightMotorDirection 22 //DIR Input on Board
#define RightMotorSleep 23 //SLP Input on Board
#define RightMotorEnergise 24 //Relay Isolating Supply to Right Motor

#define RightMotorFault 25 //FLT Pin on Board
#define RightMotorCurrent 1 //CS Pin on Board

#define RightMotorSpeed 8 //PWM Input on Board

#define RightEncoderA 18
#define RightEncoderB 19

//Left Motor
#define LeftMotorDirection 26 //DIR Input on Board
#define LeftMotorSleep 27  //SLP Input on Board
#define LeftMotorEnergise 28  //Relay Isolating Supply to Left Motor

#define LeftMotorFault 29  //FLT Pin on Board
#define LeftMotorCurrent 2  //CS Pin on Board

#define LeftMotorSpeed 9 //PWM Input on Board

#define LeftEncoderA 20
#define LeftEncoderB 21

//Other
#define MotorBrakes 52 //Relay to apply mechanical brake
#define WarningLight 53 //Relay to apply mechanical brake
#define Reset 42 //Pin attached to reset for instrinsic program reset
#define voltageSensor 0  //Voltage Sensor
#define ESTOP 2

#endif
