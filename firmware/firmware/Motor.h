#ifndef MOTOR_H
#define MOTOR_H
#include "Config.h"

class Motor{
  public:
    Motor();
    Motor(int iSleepPin, int iFaultPin, int speedPin, int pwmScaler, volatile uint8_t *timerPort, int dirPin, int currentPin, int energisePin);

    //PUBLIC - Actions
    void update(unsigned int elapsedMicros,int pulses);
    void hardStop(void);
    void softStop(void);
    float determineSpeed(float distance,unsigned int elapsedMicros);
    //PUBLIC - Getters
    bool getMoveState(void);
    double getDistance(void);
      
    //PUBLIC - Setters
    void setSpeed(float newSpeed);
    void moveMetricSpeed(float speed);
    void moveArbitarySpeed(int arbitarySpeed);
    void setAccelRate(float rate);
    void begin(void);
    float getCurrent(void);
  
  private:
    //PRIVATE - Getters
    bool getFaultState(void);

    //PRIVATE - Functions
    void updateAccel(unsigned int elapsedMicros);
    void updateAccelParams();
    void throttle(int elapsedMicros);
    void adjustSpeed(int speed);
    void energise(bool state);
    void sleep(bool state);
    double calculateDistance(int pulses);
    
    //PRIVATE - Pin Definitions
    int iSleepPin, iFaultPin, speedPin, pwmScaler, dirPin, currentPin, energisePin;

    //PRIVATE - Other Variables
    volatile bool stopped,manualControl;
    double encoderMultiplier,distanceTravelled;
    volatile int desiredMovement;
    float accelIncrement;
    volatile bool fault;
    volatile float current,accelRate;
    volatile uint8_t *timerPort;

    //PRIVATE - Speed Variables
    volatile int motorSpeed,motorDirection;
    double currentSpeed, goalSpeed, topSpeed;
    
    //PRIVATE - PID Variables
    double cumulativeError,lastError;
};

Motor::Motor(int iSleepPin, int iFaultPin, int speedPin, int pwmScaler, volatile uint8_t *timerPort, int dirPin, int currentPin, int energisePin){ 
  this->iSleepPin = iSleepPin;
  this->iFaultPin = iFaultPin;
  this->speedPin = speedPin;
  this->dirPin = dirPin;
  this->currentPin = currentPin;
  this->energisePin = energisePin; 
  this->stopped = true;
  this->pwmScaler = pwmScaler;
  this->timerPort = timerPort;
}

//Needs to be called in setup to initialise pins
void Motor::begin(){
  //Pin Mode Setup
  pinMode(energisePin,OUTPUT);
  pinMode(iSleepPin,OUTPUT);
  pinMode(speedPin,OUTPUT);
  pinMode(dirPin,OUTPUT);
  pinMode(iFaultPin,INPUT);
  pinMode(currentPin,INPUT);

  //Adjust PWM Frequency
  *timerPort &= ~7; //Clear Bits
  *timerPort |= pwmScaler;

  goalSpeed = 0.00;
  topSpeed = 0.00;
  
  //Calculate Distance Constant
  encoderMultiplier = (ENCODER_DIAMETER*PI)/PULSES_PER_REV;
  
  //Make sure wheelchair is stopped
  hardStop();
  distanceTravelled = 0.0;
}

void Motor::update(unsigned int elapsedMicros,int pulses){
  fault = getFaultState();
  float distance = calculateDistance(pulses);
  distanceTravelled +=distance;
  currentSpeed = determineSpeed(distance, elapsedMicros);
  if((!manualControl)&&(!stopped)){
    updateAccel(elapsedMicros);
    throttle(elapsedMicros);
  }
}

void Motor::updateAccel(unsigned int elapsedMicros){
  if(goalSpeed != topSpeed){
    if(goalSpeed < topSpeed){
      goalSpeed += accelIncrement;
    }
    else if(goalSpeed > topSpeed){
      goalSpeed -= accelIncrement;
    }
  }
}
    
void Motor::throttle(int elapsedMicros){
  if(currentSpeed != goalSpeed){
    double error = goalSpeed - currentSpeed;                              
    cumulativeError += error * (elapsedMicros*1000000);             
    double rateError = (error - lastError)/elapsedMicros;
    int deltaSpeed = (KP*error) + (KI*cumulativeError) + (KD*rateError); 
    int newMotorSpeed = motorSpeed + deltaSpeed;                        
    adjustSpeed(newMotorSpeed);
    lastError = error; 
  }
}

float Motor::determineSpeed(float distance,unsigned int elapsedMicros){
  //Speed in mm/s 
  float speed = distance/0.05;//(double(elapsedMicros)/10000000);
  return speed;
}

//setters
void Motor::adjustSpeed(int speed){
  if(speed == 0){
    sleep(true);
    energise(false);
    stopped = true;
  }
  else{
    motorDirection = speed/abs(speed);
    motorSpeed = abs(speed);
    //Write the Pins
    energise(true);
    sleep(false);
    if(motorDirection == -1){
      digitalWrite(dirPin,1);
    }
    else{
      digitalWrite(dirPin,0);
      }
    analogWrite(speedPin,motorSpeed);
    stopped = false;
  }
}

void Motor::updateAccelParams(){
  accelIncrement = accelRate*(INTERRUPT_TIME/1000000);
  //decelDistance = topSpeed
}

void Motor::setAccelRate(float rate){
  this->accelRate = rate;
  updateAccelParams();
}


double Motor::calculateDistance(int pulses){
  double movement = encoderMultiplier*pulses;
  return movement;
  }

//PRIVATE - SETTER
void Motor::energise(bool state){
  digitalWrite(energisePin,!state);
}


//PUBLIC - SETTER
void Motor::setSpeed(float newSpeed){
  manualControl = false;
  this->topSpeed = newSpeed;
}

//PUBLIC - SETTER
void Motor::moveMetricSpeed(float speed){
  manualControl = false;
  topSpeed = speed;
}

//PUBLIC - SETTER
void Motor::moveArbitarySpeed(int arbitarySpeed){
  if((arbitarySpeed<= 255)&&(arbitarySpeed>= -255)){
    distanceTravelled = 0.0;
    manualControl = true;
    adjustSpeed(arbitarySpeed);
  }
}

//PUBLIC - GETTER
bool Motor::getMoveState(){
  bool state = !stopped;
  return state;
  }

//PRIVATE - Getter
bool Motor::getFaultState(){ 
  bool status = !digitalRead(iFaultPin);
  return status;
  }

//PUBLIC - Getter
float Motor::getCurrent(){
  float currentFactor = 0.244379;
  float currentOffset = 0.00;
  //Current in Amperes
  float newCurrent= (analogRead(currentPin)-currentOffset)*currentFactor;
  return newCurrent;
  }

//PUBLIC - Getter
double Motor::getDistance(void){
  return distanceTravelled;
}
//PUBLIC - Action
void Motor::hardStop(void){
  goalSpeed = 0.0;
  topSpeed = 0.0;
  adjustSpeed(0);
  energise(false);
  }

//PUBLIC - Action
void Motor::softStop(void){
  topSpeed = 0.0;
  }

//PRIVATE - Setter
void Motor::sleep(bool state){
  digitalWrite(iSleepPin,!state);
}
 
#endif
