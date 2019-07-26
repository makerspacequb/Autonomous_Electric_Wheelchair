#ifndef MOTOR_H
#define MOTOR_H
#include "Config.h"

class Motor{
  public:
    Motor();
    Motor(int iSleepPin, int iFaultPin, int speedPin, int dirPin, int currentPin, int energisePin);

    //PUBLIC - Actions
    void update(unsigned int elapsedMicros,double distance);
    void move(int mmToMove);
    void hardStop();
    void softStop();
    void rampSpeed(int desiredSpeed);

    //PUBLIC - Getters
    bool getMoveState();
      
    //PUBLIC - Setters
    void setSpeed(int speed);
    void setAccelRate(int rate);
    void begin();

  private:
    int iSleepPin, iFaultPin, speedPin, dirPin, currentPin, energisePin;
    bool stopped;
    double distanceTravelled,actualSpeed,desiredSpeed;
    volatile int motorSpeed, desiredMovement;
    volatile bool fault;
    volatile float current;

    //PID Variables
    double cumulativeError,lastError
 
    //PRIVATE - Getters
    float getCurrent();
    bool getFaultState();
};

Motor::Motor(int iSleepPin, int iFaultPin, int speedPin, int dirPin, int currentPin, int energisePin){ 
  this->iSleepPin = iSleepPin;
  this->iFaultPin = iFaultPin;
  this->speedPin = speedPin;
  this->dirPin = dirPin;
  this->currentPin = currentPin;
  this->energisePin = energisePin; 
  this->stopped = false;
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
  //Make sure wheelchair is stopped
  hardStop();
  distanceTravelled = 0.0;

}

void Motor::update(unsigned int elapsedMicros,double distance){
  fault = getFaultState();
  distanceTravelled =+ distance;
  determineSpeed(distanceTravelled, elapsedMicros);
  updateSpeed();
}

void Motor::updateAccel(){

  //If not going to reach top speed
  if((stepsTarget-accelLength)<0){
    accelLength = (stepsTarget/2);
  }

  //Acceleration Region
  if(steps > (stepsTarget-accelLength)){
    currentSpeed += accelRate;
  }
  //Deceleration Region
  else if(steps < accelLength){
    currentSpeed -= accelRate;
  }
  //Deceleration for Mid Move Speed Changes
  else if(currentSpeed > speed){
    currentSpeed -= accelRate;
  }

  currentStepDelayDuration = (long)1000000 / (long)currentSpeed;    
}

void Motor::move(int distance){

  desiredMovement = distance;
  distanceTravelled = 0.0;

}

void Motor::throttle(int elapsedMicros){
  if(actualSpeed != desiredSpeed){
    double error = desiredSpeed - actualSpeed;                              
    cumulativeError += error * elapsedMicros;             
    double rateError = (error - lastError)/elapsedMicros;
    int outputSpeed = KP*error + KI*cumulativeError + KD*rateError;                           
    setSpeed(outputSpeed);
    lastError = error; 
  }
}

void Motor:determineSpeed(double distance,unsigned int elapsedMicros){
  //Speed in mm/s 
  actualSpeed = distance/(elapsedMicros/10000000);
}

//setters
void Motor::setSpeed(int speed){
  this->motorSpeed = speed;
  digitalWrite(speedPin,speed);
}

void Motor::rampSpeed(int desiredSpeed){
  rampedSpeed = speed
  if(desiredSpeed>speed){
    while(speed != desiredSpeed){
      rampedSpeed++;
      setSpeed(rampedSpeed)
      delayMicroseconds(accelDelay)
    }
   }
   else if(desiredSpeed<speed){
    while(speed != desiredSpeed){
      rampedSpeed--;
      setSpeed(rampedSpeed)
      delayMicroseconds(accelDelay)
    }
   }
}

void Motor::setAccelRate(int rate){
  accelRate = rate;
  updateAccelParams();
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

//PRIVATE - Getter
float Motor::getCurrent(){
  float currentFactor = 0.244379;
  float currentOffset = 0.00;
  //Current in Amperes
  float newCurrent= (analogRead(currentPin)-currentOffset)*currentFactor;
  return newCurrent;
  }

//PUBLIC - Action
void Motor::hardStop(void){
  setSpeed(0);
  digitalWrite(energisePin,LOW);
  stopped = false;
  }

//PUBLIC - Action
void Motor::softStop(void){
  rampSpeed(0);
  digitalWrite(energisePin,LOW);
  stopped = false;
  }
  
#endif
