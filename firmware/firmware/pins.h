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
#define RIGHT_MOTOR_DIR 22 //DIR Input on Board
#define RIGHT_MOTOR_SLEEP 23 //SLP Input on Board
#define RIGHT_MOTOR_ENERGISE 24 //Relay Isolating Supply to Right Motor

#define RIGHT_MOTOR_FAULT 25 //FLT Pin on Board
#define RIGHT_MOTOR_CURRENT 1 //CS Pin on Board

#define RIGHT_MOTOR_SPEED 8 //PWM Input on Board

#define RIGHT_ENCODER_A 18
#define RIGHT_ENCODER_B 19

//Left Motor
#define LEFT_MOTOR_DIR 26 //DIR Input on Board
#define LEFT_MOTOR_SLEEP 27  //SLP Input on Board
#define LEFT_MOTOR_ENERGISE 28  //Relay Isolating Supply to Left Motor

#define LEFT_MOTOR_FAULT 29  //FLT Pin on Board
#define LEFT_MOTOR_CURRENT 2  //CS Pin on Board

#define LEFT_MOTOR_SPEED 9 //PWM Input on Board

#define LEFT_ENCODER_A 20
#define LEFT_ENCODER_B 21

//Other
#define MOTOR_BRAKES 52 //Relay to apply mechanical brake
#define WARNING_LIGHT 53 //Relay to apply mechanical brake
#define VOLTAGE_SENSOR 0  //Voltage Sensor
#define ESTOP 2
#define ESTOP_POWER 3

#endif
