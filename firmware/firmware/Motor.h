#ifndef MOTOR_H
#define MOTOR_H
#include "Config.h"

class Motor{
  public:
    Motor();
    Motor(int iSleepPin, int iFaultPin, int speedPin, int dirPin, int currentPin, int energisePin);

    //PUBLIC - Actions
    void update(unsigned int elapsedMicros,double distance);
    void hardStop(void);
    void softStop(void);
    void rampSpeed(int desiredSpeed);
    double determineSpeed(double distance,unsigned int elapsedMicros);
    //PUBLIC - Getters
    bool getMoveState(void);
    double getDistance(void);
      
    //PUBLIC - Setters
    void hold(bool state);
    void setSpeed(int speed);
    void setAccelRate(int rate);
    void setMotorSpeed(int arbitarySpeed);
    void begin(void);
    float getCurrent(void);
  
  private:
    //PRIVATE - Getters
    bool getFaultState(void);

    //PRIVATE - Functions
    void updateAccel(unsigned int elapsedMicros);
    void updateAccelParams();
    void throttle(int elapsedMicros);
    void energise(bool state);
    void sleep(bool state);
    
    //PRIVATE - Pin Definitions
    int iSleepPin, iFaultPin, speedPin, dirPin, currentPin, energisePin;

    //PRIVATE - Other Variables
    bool stopped,manualControl;
    double distanceTravelled,actualSpeed,desiredSpeed;
    volatile int desiredMovement,accellerationDelay;
    volatile bool fault;
    volatile float current,accelRate;

    //PRIVATE - Speed Variables
    volatile int motorSpeed,motorDirection;
    double currentSpeed, goalSpeed, topSpeed;
    
    //PRIVATE - PID Variables
    double cumulativeError,lastError;
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
  actualSpeed = determineSpeed(distanceTravelled, elapsedMicros);
  if(manualControl){
    updateAccel(elapsedMicros);
    throttle(elapsedMicros);
  }
}

void Motor::updateAccel(unsigned int elapsedMicros){
  if(elapsedMicros>accellerationDelay){
    if(goalSpeed < topSpeed){
      goalSpeed++;
    }
    else if(goalSpeed > topSpeed){
      goalSpeed--;
    }
  }
}
    
void Motor::throttle(int elapsedMicros){
  if(currentSpeed != goalSpeed){
    double error = goalSpeed - currentSpeed;                              
    cumulativeError += error * elapsedMicros;             
    double rateError = (error - lastError)/elapsedMicros;
    int newMotorSpeed = (KP*error) + (KI*cumulativeError) + (KD*rateError);                           
    setSpeed(newMotorSpeed);
    lastError = error; 
  }
}

double Motor::determineSpeed(double distance,unsigned int elapsedMicros){
  //Speed in mm/s 
  double speed = distance/(elapsedMicros/10000000);
  return speed;
}

//setters
void Motor::setSpeed(int speed){
  this->motorDirection = speed/abs(speed);
  this->motorSpeed = abs(speed);
  if(speed == 0){
    hold(true);
    energise(false);
    sleep(true);
    stopped = true;
  }
  else{
    //Write the Pins
    energise(true);
    sleep(false);
    digitalWrite(speedPin,motorSpeed);
    digitalWrite(dirPin,motorDirection);
    stopped = false;
  }
}

void Motor::updateAccelParams(){
  accellerationDelay = 1/accelRate;
}

void Motor::setAccelRate(int rate){
  accelRate = rate;
  updateAccelParams();
}

//PRIVATE - SETTER
void Motor::energise(bool state){
  digitalWrite(energisePin,!state);
}

//PUBLIC - SETTER
void Motor::setMotorSpeed(int arbitarySpeed){
  if((arbitarySpeed<= 255)&&(arbitarySpeed>= -255)){
    manualControl = true;
    setSpeed(arbitarySpeed);
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
  setSpeed(0);
  energise(false);
  }

//PUBLIC - Action
void Motor::softStop(void){
  goalSpeed = 0.0;
  }

//PUBLIC - Action
void Motor::hold(bool state){
  digitalWrite(iSleepPin,!state);
}

//PRIVATE - Setter
void Motor::sleep(bool state){
  digitalWrite(iSleepPin,!state);
}
 
#endif
