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
    leftMotor.update(INTERRUPT_TIME);
    rightMotor.update(INTERRUPT_TIME);
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

//------------------------------------------------------------------------------------------------------------------
//Process Incstruction
//------------------------------------------------------------------------------------------------------------------
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
      reset();
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
//Apply of Disengage Brakes
//------------------------------------------------------------------------------------------------------------------
void brakes(bool state){
  digitalWrite(MOTOR_BRAKE,state);
  digitalWrite(WARNING_LIGHT,!state);
  }

//------------------------------------------------------------------------------------------------------------------
//ESTOP ROUTINE
//------------------------------------------------------------------------------------------------------------------
void eStop(){
  leftMotor.stop();
  rightMotor.stop();
  if(!eStopActivated){
    eStopActivated = true;
    Serial.println("INFO: Emergency Stop Pressed. Release button and reset with 'r' to continue.");
  }
}

//------------------------------------------------------------------------------------------------------------------
//RESET FUNCTION
//------------------------------------------------------------------------------------------------------------------
void(* reset) (void) = 0;
