//------------------------------------------------------------------------------------------------------------------
// NAME: config.h
// AUTH: Ryan McCartney
// DATE: 22nd July 2019
// DESC: Config Details for Autonomous Wheelchair
// NOTE: All Rights Reserved, 2019, QLab Makerspace
//------------------------------------------------------------------------------------------------------------------

#ifndef CONFIG_H
#define CONFIG_H

#define STATUS_FREQ 1
#define BAUD_RATE 115200
#define MAX_PARAM_LENGTH 10 //length of instruction array
#define MAX_PARAMS 3 //Maximum number of variables in an instruction
#define INTERRUPT_TIME 50000 //Polling interupt time (microseconds)
#define PWM_SCALER 1 //Changes PWM Frequency (Values 1-6, Freuqencies, 31KHz-<20Hz)
#define MAX_SPEED 1700

#define ENCODER_DIAMETER 83 //IN MILIMETERS
#define WHEEL_DIAMETER 230 //IN MILIMETERS
#define PULSES_PER_REV 600

#define LEFT_ACCEL_RATE 750.0
#define RIGHT_ACCEL_RATE 750.0

//PID CONTROL for Speed Sync
#define KP 0.05
#define KI 0.15
#define KD 0.01

#endif
