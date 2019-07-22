//------------------------------------------------------------------------------------------------------------------
// NAME: firmware.ino
// AUTH: Ryan McCartney
// DATE: 22nd July 2019
// DESC: Pin Defintiions for Autonomous Wheelchair
// NOTE: All Rights Reserved, 2019, QLab Makerspace
//------------------------------------------------------------------------------------------------------------------

#include "Motor.h"
#include "pins.h"
#include "Config.h"
#include "TimerOne.h"

//initialize some variables
unsigned long statusTime = millis();
const int statusDelay = (1/STATUS_FREQ)*1000;
volatile bool eStopActivated = false;
char instruction[INST_ARRAY_LEN];
int instIndex = 0;

//Declare Motor objects
Motor leftMotor = Motor();
Motor rightMotor = Motor();

void leftMotorPhaseA(void){

}

void leftMotorPhaseB(void){

}

void rightMotorPhaseA(void){

}

void rightMotorPhaseB(void){
  
}


bool interruptBusy = false;
void interrupt(void){
  if(!interruptBusy){
    interruptBusy = true;
    //handle motor movement by interrupt
    for (int i = 0; i < TOTAL_JOINTS; i++)
      joints[i].update(INTERRUPT_TIME);
      interruptBusy = false;
  }
}

void setup() { 
  for (int i = 0; i < TOTAL_JOINTS; i++)
    joints[i].begin();
   
  //Setup Main Serial
  Serial.begin(BAUD_RATE);
  Serial.println("INFO: Starting up...");
  
  //Setup Hand
  pinMode(END_EFFECTOR_2, OUTPUT);
  hand.attach(END_EFFECTOR_2); 

  //Setup Tool Communication line
  Serial1.begin(BAUD_RATE);
  Serial1.println("INFO: Arm Setup Complete.");

  //Load data from EEPROM
  loadPositions();

  //set timer interrupt for motor and encoder control
  Timer1.attachInterrupt(interrupt);
  Timer1.initialize(INTERRUPT_TIME);
  Timer1.start();

  //Set Hardware Interupt for EStop
  pinMode(ESTOP, INPUT);
  attachInterrupt(digitalPinToInterrupt(ESTOP), eStop, FALLING);
  
  Serial.println("INFO: Setup Complete.");
}

void loop() {
  readSerial();
  //Send Status Message at Configured Frequency
  if((millis()-statusTime) > statusDelay){
    sendStatus();
  }
}

void readSerial(){
  if (Serial.available() > 0){
    char nextChar = Serial.read();
    //if char is new line last instruction complete, process instruction
    if(nextChar == '\n'){
      if(instIndex > 0) {
        //make sure rest of instruction is cleared
        for (int i = instIndex; i < INST_ARRAY_LEN; i++)
          instruction[i] = NULL;
        //send instruction for processing
        processInstruction(instruction);
        instIndex = 0;
      }
    }
    //add to instruction string
    else {
      if(instIndex >= INST_ARRAY_LEN)
        Serial.println("ERROR: Instruction parser: Instruction index out of bounds.");
      else{
        instruction[instIndex] = nextChar;
        instIndex++;
      }
    }
  }
}

void processInstruction(char *input){
  //check first byte
  switch(toLowerCase(input[0])){
    case 's': 
      joints[input[1] - '0'].setSpeed(atol(input+2));
      Serial.print("INFO: Set motor: ");
      Serial.print(input[1]);
      Serial.print(" to speed: ");
      Serial.print(atol(input+2));
      Serial.println("deg/s");
      break;
    case 'd': 
      joints[input[1] - '0'].setMinSpeed(atol(input+2));
      Serial.print("INFO: Set motor: ");
      Serial.print(input[1]);
      Serial.print(" min speed to: ");
      Serial.print(atol(input+2));
      Serial.println("deg/s");
      break;
    case 'z': 
      joints[input[1] - '0'].setAccelRate(atol(input+2));
      Serial.print("INFO: Set motor: ");
      Serial.print(input[1]);
      Serial.print(" acceleration steps: ");
      Serial.print(atol(input+2));
      Serial.println("us/step");
      break;
    case 'm': 
      moveJoint(input[1] - '0',atol(input+2)); 
      break;
    case 'q': 
      quit();
      break;
    case 'r':
      eStopActivated = false;
      Serial.println("INFO: Emergency stop reset.");
      break;
    case 'i':
      //Return information about positions
      printPositions();
      break;
    default: 
      Serial.println("WARNING: Command not found");
  }
}

void printMovemetStates(){
  String outputString = "STATUS: MOVEMENT";
  for(int i = 0; i < TOTAL_JOINTS; i++) {
    outputString += ","+(String)(joints[i].checkMovement());
  }
  Serial.println(outputString);
}

void printPositions(){
  String outputString = "STATUS: POSITION";
  for(int i = 0; i < TOTAL_JOINTS; i++) {
    outputString += ","+(String)(joints[i].getPosDegrees());
  }
  Serial.println(outputString);
}

void printCalibration(){
  String outputString = "STATUS: CALIBRATION";
  for(int i = 0; i < TOTAL_JOINTS; i++) {
    outputString += ","+(String)(joints[i].checkCalibration());
  }
  Serial.println(outputString);
}

void moveJoint(int jointIndex, int value){
  if(!eStopActivated){
    if(!armCalibrated)
      Serial.println("WARNING: Motors are not calibrated. Calibrate with 'c' command.");
      Serial.print("INFO: Moving motor ");
      Serial.print(jointIndex);
      Serial.print(", ");
      Serial.print(value);
      Serial.println(" degrees");
      joints[jointIndex].move(value);
    }
  else{
    Serial.println("WARNING: Movement Disabled. Reset with 'r' to continue.");
  }
}

void moveJointTo(int jointIndex, int value){
  if(!eStopActivated){
    if(joints[jointIndex].checkCalibration()){
      Serial.print("INFO: Moving motor ");
      Serial.print(jointIndex);
      Serial.print(" to position ");
      Serial.print(value);
      Serial.println(" degrees");
      joints[jointIndex].moveTo(value);
    } 
    else{
      Serial.print("WARNING: Joint ");
      Serial.print(jointIndex);
      Serial.print(" is not calibrated. Calibrate with 'c");
      Serial.print(jointIndex);
      Serial.println("' command.");
    }
  }
  else{
    Serial.println("WARNING: Movement Disabled. Reset with 'r' to continue.");
  }
}

void sendStatus(){
  printPositions();
  printCalibration();
  printMovemetStates();
  statusTime = millis();
}

void quit(){
  for (int i = 0; i < TOTAL_JOINTS; i++){
    joints[i].move(0);
  }
  Serial.println("INFO: Arm Stopped");
}

void eStop(){
  for (int i = 0; i < TOTAL_JOINTS; i++){
    joints[i].move(0);
  }
  if(!eStopActivated){
    eStopActivated = true;
    Serial.println("INFO: Emergency Stop Pressed. Release button and reset with 'r' to continue.");
  }
}
