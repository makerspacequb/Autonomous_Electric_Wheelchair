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
String instruction[MAX_PARAMETERS];
int instChar = 0;
int instIndex = 0;

//Declare Motor objects
Motor leftMotor = Motor(LEFT_MOTOR_SLLEP, LEFT_MOTOR_FAULT, LEFT_MOTOR_SPEED, LEFT_MOTOR_DIR, LEFT_MOTOR_CURRENT, LEFT_MOTOR_ENERGISE);
Motor rightMotor = Motor(RIGHT_MOTOR_SLLEP, RIGHT_MOTOR_FAULT, RIGHT_MOTOR_SPEED, RIGHT_MOTOR_DIR, RIGHT_MOTOR_CURRENT, RIGHT_MOTOR_ENERGISE);

//Iniitialise Encoder Variables
volatile int leftPulses = 0;
volatile int rightPulses = 0;
double encoderMultiplier = 0.00;

//------------------------------------------------------------------------------------------------------------------
//Encoder Interupts
//------------------------------------------------------------------------------------------------------------------
void rightEncoderPulse(){
  if((digitalRead(rightEncoderA) == HIGH)&&(digitalRead(rightEncoderB) == LOW)){
    //Moving Forwards
    rightPulses++;
  }
  else if((digitalRead(rightEncoderA) == LOW)&&(digitalRead(rightEncoderB) == HIGH)){
    //Moving Backwards
    rightPulses--;
  }
}

void leftEncoderPulse() {
    if((digitalRead(leftEncoderA) == HIGH)&&(digitalRead(leftEncoderB) == LOW)){
    //Moving Forwards
    leftPulses++;
  }
  else if((digitalRead(leftEncoderA) == LOW)&&(digitalRead(leftEncoderB) == HIGH)){
    //Moving Backwards
    leftPulses--;
  }
}

//------------------------------------------------------------------------------------------------------------------
//TIMER INTERUPT ROUTINE
//------------------------------------------------------------------------------------------------------------------
bool interruptBusy = false;
void interrupt(void){
  if(!interruptBusy){
    interruptBusy = true;
    leftMotor.update(INTERRUPT_TIME,distanceTravelled(leftPulses));
    rightMotor.update(INTERRUPT_TIME,distanceTravelled(rightPulses));
    leftPulses = 0;
    rightPulses = 0;
    checkMovement();
    interruptBusy = false;
  }
}

//------------------------------------------------------------------------------------------------------------------
//SETUP FUNCTION
//------------------------------------------------------------------------------------------------------------------
void setup() { 

  leftMotor.begin();
  leftMotor.begin();
   
  //Setup Main Serial
  Serial.begin(BAUD_RATE);
  Serial.println("INFO: Wheelchair starting up...");

  //Setup Encoder Pins
  pinMode(rightEncoderA, INPUT_PULLUP);
  pinMode(rightEncoderB, INPUT_PULLUP);
  pinMode(leftEncoderA, INPUT_PULLUP);
  pinMode(leftEncoderB, INPUT_PULLUP);
  
  //Attach Interupts
  attachInterrupt(digitalPinToInterrupt(rightEncoderA), rightEncoderPulse, RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncoderB), rightEncoderPulse, RISING);
  attachInterrupt(digitalPinToInterrupt(leftEncoderA), leftEncoderPulse, RISING);
  attachInterrupt(digitalPinToInterrupt(leftEncoderB), leftEncoderPulse, RISING);

  //Set Hardware Interupt for EStop
  pinMode(ESTOP_POWER, OUTPUT);
  digitalWrite(ESTOP_POWER,HIGH);
  pinMode(ESTOP, INPUT);
  attachInterrupt(digitalPinToInterrupt(ESTOP), eStop, FALLING);

  //Setup Brakes and Light
  pinMode(WARNING_LIGHT,OUTPUT);
  pinMode(MOTOR_BRAKES,OUTPUT);

  //Setup Voltage Sensor
  pinMode(VOLTAGE_SENSOR, INPUT_PULLUP);
  
  //Calculate Distance Constant
  encoderMultiplier = (ENCODER_DIAMETER*PI)/PULSES_PER_REV;
     
  //set timer interrupt for motor control
  Timer1.attachInterrupt(interrupt);
  Timer1.initialize(INTERRUPT_TIME);
  Timer1.start();

  Serial.println("INFO: Setup Complete.");
}

//------------------------------------------------------------------------------------------------------------------
//MAIN LOOP
//------------------------------------------------------------------------------------------------------------------
void loop() {
  readSerial();
  //Send Status Message at Configured Frequency
  if((millis()-statusTime) > statusDelay){
    sendStatus();
  }
}

//------------------------------------------------------------------------------------------------------------------
//Read Serial Line
//------------------------------------------------------------------------------------------------------------------
void readSerial(){
  if (Serial.available() > 0){
    char nextChar = Serial.read();
    //if char is new line last instruction complete, process instruction
    if(nextChar == '\n')&&(instIndex > 0){
      //Send instruction for exucution
      executeInstruction(instruction);
      instChar = 0;
      instIndex = 0;
    }
    //add to instruction string
    else {
      if(instChar == 0){
        instruction[instIndex] = null;
      }
      if(instIndex >= MAX_PARAMS){
        Serial.println("ERROR: Too many instruction variables passed.");
      }
      else if(instChar >= MAX_PARAM_LENGTH){
        Serial.println("ERROR: Instruction variable too long.");
      }
      else if(nextChar = ','){
        instIndex++;
        instruction[instIndex] = null;
      }
      else{
        instruction[instIndex] += nextChar;
        instChar++;
      }
    }      
  }
}

//------------------------------------------------------------------------------------------------------------------
//Process Incstruction
//------------------------------------------------------------------------------------------------------------------
void executeInstruction(String instruction){
  //check first byte
  switch(toLowerCase(instruction[0])){
    case 's': 
      if(instruction[1] == 'l'){
        leftMotor.setSpeed(atol(instruction[2]));
        Serial.print("INFO: Left motor speed set to ");
        Serial.print(instruction[2]);
        Serial.println("mm/s.");
      }
      else if(instruction[1] == 'r'){
        rightMotor.setSpeed(atol(instruction[2]));
        Serial.print("INFO: Right motor speed set to ");
        Serial.print(instruction[2]);
        Serial.println("mm/s.");
      }
      else if(instruction[1] == 'b'){
        leftMotor.setSpeed(atol(instruction[2]));
        rightMotor.setSpeed(atol(instruction[2]));
        Serial.print("INFO: Base motor speed set to ");
        Serial.print(instruction[2]);
        Serial.println("mm/s.");
      }
      break;
    case 'z': 
      if(instruction[1] == 'l'){
        leftMotor.setBaseAccel(atol(instruction[2]));
        Serial.print("INFO: Left motor accelleration set to ");
        Serial.print(instruction[2]);
        Serial.println("mm/s^2.");
      }
      else if(instruction[1] == 'r'){
        rightMotor.setBaseAccel(atol(instruction[2]));
        Serial.print("INFO: Right motor accelleration set to ");
        Serial.print(instruction[2]);
        Serial.println("mm/s^2.");
      }
      else if(instruction[1] == 'b'){
        leftMotor.setBaseAccel(atol(instruction[2]));
        rightMotor.setBaseAccel(atol(instruction[2]));
        Serial.print("INFO: Base motor accelleration set to ");
        Serial.print(instruction[2]);
        Serial.println("mm/s^2.");
      }
      break;
    case 't':
      if(instruction[1] == 'l'){
        leftMotor.setArbitarySpeed(atol(instruction[2]));
        Serial.print("INFO: Left motor speed throttled to ");
        Serial.print(instruction[2]);
        Serial.println("%.");
      }
      else if(instruction[1] == 'r'){
        rightMotor.setArbitarySpeed(atol(instruction[2]));
        Serial.print("INFO: Right motor speed throttled to ");
        Serial.print(instruction[2]);
        Serial.println("%.");
      }
      break;
    case 'i':
      sendStatus();
      break;
    case 'r':
      reset();
      break;
    case 'q': 
      eStop();
      break;
    case 'm':
      Serial.print("INFO: Moving wheelchair as defined."); 
      move(atol(instruction[2]),atol(instruction[3])); 
      break;
    default: 
      Serial.println("WARNING: Command not found");
  }
}

//------------------------------------------------------------------------------------------------------------------
//Status Printing Functions
//------------------------------------------------------------------------------------------------------------------
void printMovemetStates(){
  String outputString = "STATUS: MOVEMENT";
  outputString += ","+leftMotor.getMoveState();
  outputString += ","+rightMotor.getMoveState();
  Serial.println(outputString);
}

void printPositions(){
  String outputString = "STATUS: DISTANCE"
  outputString += ","+leftMotor.getDistance();
  outputString += ","+rightMotor.getDistance();
  Serial.println(outputString);
}

//------------------------------------------------------------------------------------------------------------------
//Get Voltage
//------------------------------------------------------------------------------------------------------------------
void checkMovement(){
  if((leftMotor.getMoveState() == 0)&&(rightMotor.getMoveState() == 0)){
    brakes(true);
  } 
}

//------------------------------------------------------------------------------------------------------------------
//Send Status Messages to Serial Line
//------------------------------------------------------------------------------------------------------------------
void sendStatus(){
  printPositions();
  printMovemetStates();
  statusTime = millis();
}

//------------------------------------------------------------------------------------------------------------------
//Stop Wheelchair
//------------------------------------------------------------------------------------------------------------------
void stop(){
  leftMotor.stop();
  rightMotor.stop();
  Serial.println("INFO: Wheelchair Stopped.");
}

//------------------------------------------------------------------------------------------------------------------
//Get Voltage
//------------------------------------------------------------------------------------------------------------------
float getVoltage(void){
   float voltageFactor = 0.02932551; //Voltage in Volts
   float voltageOffset = 1.65;
   batteryVoltage = (analogRead(VOLTAGE_SENSOR)*voltageFactor)-voltageOffset;
   return batteryVoltage;
  }
  
//------------------------------------------------------------------------------------------------------------------
//Apply or Disengage Brakes
//------------------------------------------------------------------------------------------------------------------
void brakes(bool state){
  digitalWrite(MOTOR_BRAKE,state);
  digitalWrite(WARNING_LIGHT,!state);
  }

//------------------------------------------------------------------------------------------------------------------
//Calculate Distance
//------------------------------------------------------------------------------------------------------------------
double distanceTravelled(int pulses){
  double movement = encoderMultiplier*pulses;
  return movement;
  }
  
//------------------------------------------------------------------------------------------------------------------
//ESTOP ROUTINE
//------------------------------------------------------------------------------------------------------------------
void eStop(){
  leftMotor.softStop();
  rightMotor.hardStop();
  if(!eStopActivated){
    eStopActivated = true;
    Serial.println("INFO: Emergency Stop Pressed. Release button and reset with 'r' to continue.");
  }
}

//------------------------------------------------------------------------------------------------------------------
//RESET FUNCTION
//------------------------------------------------------------------------------------------------------------------
void(* reset) (void) = 0;
