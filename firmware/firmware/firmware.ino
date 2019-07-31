//------------------------------------------------------------------------------------------------------------------
// NAME: firmware.ino
// AUTH: Ryan McCartney
// DATE: 26th July 2019
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
String instruction[MAX_PARAMS] = {};
int instChar,instIndex = 0;

//Declare Motor objects
Motor leftMotor = Motor(LEFT_MOTOR_SLEEP, LEFT_MOTOR_FAULT, LEFT_MOTOR_SPEED, PWM_SCALER, LEFT_MOTOR_DIR, LEFT_MOTOR_CURRENT, LEFT_MOTOR_ENERGISE);
Motor rightMotor = Motor(RIGHT_MOTOR_SLEEP, RIGHT_MOTOR_FAULT, RIGHT_MOTOR_SPEED, PWM_SCALER, RIGHT_MOTOR_DIR, RIGHT_MOTOR_CURRENT, RIGHT_MOTOR_ENERGISE);

//Iniitialise Encoder Variables
volatile int leftPulses = 0;
volatile int rightPulses = 0;
double encoderMultiplier = 0.00;

//------------------------------------------------------------------------------------------------------------------
//Encoder Interupts
//------------------------------------------------------------------------------------------------------------------
void rightEncoderPulse(){
  if((digitalRead(RIGHT_ENCODER_A) == HIGH)&&(digitalRead(RIGHT_ENCODER_B) == LOW)){
    //Moving Forwards
    rightPulses++;
  }
  else if((digitalRead(RIGHT_ENCODER_A) == LOW)&&(digitalRead(RIGHT_ENCODER_B) == HIGH)){
    //Moving Backwards
    rightPulses--;
  }
}

void leftEncoderPulse() {
    if((digitalRead(LEFT_ENCODER_A) == HIGH)&&(digitalRead(LEFT_ENCODER_B) == LOW)){
    //Moving Forwards
    leftPulses++;
  }
  else if((digitalRead(LEFT_ENCODER_A) == LOW)&&(digitalRead(LEFT_ENCODER_B) == HIGH)){
    //Moving Backwards
    leftPulses--;
  }
}

//------------------------------------------------------------------------------------------------------------------
//RESET FUNCTION
//------------------------------------------------------------------------------------------------------------------
void(* reset) (void) = 0;

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
  pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_B, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_A, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_B, INPUT_PULLUP);
  
  //Attach Interupts
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), rightEncoderPulse, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_B), rightEncoderPulse, RISING);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), leftEncoderPulse, RISING);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_B), leftEncoderPulse, RISING);

  //Set Hardware Interupt for EStop
  pinMode(ESTOP_POWER, OUTPUT);
  digitalWrite(ESTOP_POWER,HIGH);
  pinMode(ESTOP, INPUT);
  attachInterrupt(digitalPinToInterrupt(ESTOP), eStop, LOW);

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
    nextChar = toLowerCase(nextChar);
    //if char is new line last instruction complete, process instruction
    if((nextChar == '\n')&&(instIndex >= 0)){
      //Send instruction for exucution
      instChar = 0;
      instIndex = 0;
      executeInstruction();
      Serial.print("PARMA 0:");
      Serial.print(instruction[0]);
      Serial.print("PARMA 1:");
      Serial.print(instruction[1]);
      Serial.print("PARMA 2:");
      Serial.println(instruction[2]);
    }
    //add to instruction string
    else {
      if(instChar == 0){
        instruction[instIndex] = "";
      }
      if(instIndex >= MAX_PARAMS){
        Serial.println("ERROR: Too many instruction variables passed.");
      }
      else if(instChar >= MAX_PARAM_LENGTH){
        Serial.print("DEBUG: ");
        Serial.println(instChar);
        Serial.println("ERROR: Instruction variable too long.");
      }
      else if(nextChar == ','){
        instIndex++;
        instruction[instIndex] = "";
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
void executeInstruction(){
  //check first byte
  switch(instruction[0].charAt(0)){
    case 's': 
      if(instruction[1].charAt(0) == 'l'){
        leftMotor.setSpeed(instruction[2].toFloat());
        Serial.print("INFO: Left motor speed set to ");
        Serial.print(instruction[2]);
        Serial.println("mm/s.");
      }
      else if(instruction[1].charAt(0) == 'r'){
        rightMotor.setSpeed(instruction[2].toFloat());
        Serial.print("INFO: Right motor speed set to ");
        Serial.print(instruction[2]);
        Serial.println("mm/s.");
      }
      else if(instruction[1].charAt(0) == 'b'){
        leftMotor.setSpeed(instruction[2].toFloat());
        rightMotor.setSpeed(instruction[2].toFloat());
        Serial.print("INFO: Base motor speed set to ");
        Serial.print(instruction[2]);
        Serial.println("mm/s.");
      }
      break;
    case 'z': 
      if(instruction[1].charAt(0) == 'l'){
        leftMotor.setAccelRate(instruction[2].toFloat());
        Serial.print("INFO: Left motor accelleration set to ");
        Serial.print(instruction[2]);
        Serial.println("mm/s^2.");
      }
      else if(instruction[1].charAt(0) == 'r'){
        rightMotor.setAccelRate(instruction[2].toFloat());
        Serial.print("INFO: Right motor accelleration set to ");
        Serial.print(instruction[2]);
        Serial.println("mm/s^2.");
      }
      else if(instruction[1].charAt(0) == 'b'){
        leftMotor.setAccelRate(instruction[2].toFloat());
        rightMotor.setAccelRate(instruction[2].toFloat());
        Serial.print("INFO: Base motor accelleration set to ");
        Serial.print(instruction[2]);
        Serial.println("mm/s^2.");
      }
      break;
    case 't':
      brakes(false);
      if(instruction[1].charAt(0) == 'l'){
        leftMotor.setMotorSpeed(instruction[2].toInt());
        Serial.print("INFO: Left motor speed throttled to ");
        Serial.print(instruction[2]);
        Serial.println(".");
      }
      else if(instruction[1].charAt(0) == 'r'){
        rightMotor.setMotorSpeed(instruction[2].toInt());
        Serial.print("INFO: Right motor speed throttled to ");
        Serial.print(instruction[2]);
        Serial.println(".");
      }
      break;
    case 'i':
      Serial.println("INFO: Status Information as Follows.");
      sendStatus();
      break;
    case 'r':
      Serial.println("INFO: Reseting System.");
      reset();
      break;
    case 'q': 
      eStop();
      break;
    case 'm':
      Serial.print("INFO: Moving wheelchair as defined."); 
      move(instruction[2].toFloat(),instruction[3].toFloat()); 
      break;
    default: 
      Serial.println("WARNING: Command not found");
  }
}

//------------------------------------------------------------------------------------------------------------------
//Status Printing Functions
//------------------------------------------------------------------------------------------------------------------
void sendStatus(){
  printPositions();
  printMovemetStates();
  printTelemetry();
  statusTime = millis();
}

void printMovemetStates(){
  String outputString = "STATUS: MOVEMENT";
  outputString += ","+leftMotor.getMoveState();
  outputString += ","+rightMotor.getMoveState();
  Serial.println(outputString);
}

void printPositions(){
  String outputString = "STATUS: DISTANCE";
  outputString += ","+String(leftMotor.getDistance());
  outputString += ","+String(rightMotor.getDistance());
  Serial.println(outputString);
}

void printTelemetry(){
  String outputString = "STATUS: TELEMETRY";
  outputString += ","+String(getVoltage(),3)+"V";
  outputString += ","+String(leftMotor.getCurrent(),3)+"A";
  outputString += ","+String(rightMotor.getCurrent(),3)+"A";
  Serial.println(outputString);
}

//------------------------------------------------------------------------------------------------------------------
//Check Movement and Change Brakes and Lights
//------------------------------------------------------------------------------------------------------------------
void checkMovement(){
  if((leftMotor.getMoveState() == 0)&&(rightMotor.getMoveState() == 0)){
    brakes(true);
  } 
  else{
     brakes(false);
  }
}

//------------------------------------------------------------------------------------------------------------------
//Stop Wheelchair
//------------------------------------------------------------------------------------------------------------------
void stop(){
  leftMotor.softStop();
  rightMotor.softStop();
  Serial.println("INFO: Wheelchair Stopped.");
}

//------------------------------------------------------------------------------------------------------------------
//Get Voltage
//------------------------------------------------------------------------------------------------------------------
float getVoltage(void){
   float voltageFactor = 0.02932551; //Voltage in Volts
   float voltageOffset = 1.65;
   float batteryVoltage = (analogRead(VOLTAGE_SENSOR)*voltageFactor)-voltageOffset;
   return batteryVoltage;
  }
  
//------------------------------------------------------------------------------------------------------------------
//Apply or Disengage Brakes
//------------------------------------------------------------------------------------------------------------------
void brakes(bool state){
  digitalWrite(MOTOR_BRAKES,!state);
  digitalWrite(WARNING_LIGHT,state);
  }

//------------------------------------------------------------------------------------------------------------------
//Calculate Distance
//------------------------------------------------------------------------------------------------------------------
double distanceTravelled(int pulses){
  double movement = encoderMultiplier*pulses;
  return movement;
  }


//------------------------------------------------------------------------------------------------------------------
//Move As Described
//------------------------------------------------------------------------------------------------------------------
void move(float radius,float degrees){
  //TO DO 
}
  
//------------------------------------------------------------------------------------------------------------------
//ESTOP ROUTINE
//------------------------------------------------------------------------------------------------------------------
void eStop(){
  leftMotor.hardStop();
  rightMotor.hardStop();
  if(!eStopActivated){
    eStopActivated = true;
    Serial.println("INFO: Emergency Stop Pressed. Release button and reset with 'r' to continue.");
  }
}
