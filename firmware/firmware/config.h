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
#define MAX_PARAM_LENGTH 6 //length of instruction array
#define MAX_PARAMS 3 //Maximum number of variables in an instruction
#define INTERRUPT_TIME 200 //Polling interupt time (microseconds)
#define PWM_SCALER 1 //Changes PWM Frequency (Values 1-6, Freuqencies, 31KHz-<20Hz)

#define ENCODER_DIAMETER 68 //IN MILIMETERS
#define WHEEL_DIAMETER 230 //IN MILIMETERS
#define PULSES_PER_REV 600

//PID CONTROL for Speed Sync
#define KP 2
#define KI 0
#define KD 0

#endif
