#ifndef MOTOR_H
#define MOTOR_H

class Motor{
  public:
    Motor();
    Motor(int iSleepPin, int iFaultPin, int speedPin, int dirPin, int currentPin, int energisePin);
    
    void move(int mmToMove);
    
    //setters
    void setSpeed(int speed);
    void setAccelRate(int rate);
    void begin();

    //getters
    int getSpeed(){ return speed; };

  private:
    int iSleepPin, iFaultPin, speedPin, dirPin, currentPin, energisePin;
    
    volatile int steps, currentSpeed, stepsTarget, currentStepDelayDuration, maxStepDelayDuration, speed, minSpeed, accelRate, accelLength;
    unsigned int stepRunTime;
    bool stepDelay, enableHIGH, motorInvert;
    volatile uint8_t *stepPort;
    uint8_t stepByte;
    void updateAccel();
    void updateAccelParams();
};

Motor::Motor(int iSleepPin, int iFaultPin, int speedPin, int dirPin, int currentPin, int energisePin){
  this->stepPin = stepPin;
  this->dirPin = dirPin;
  this->enablePin = enablePin;
  this->accelRate = accelRate;
  this->enableHIGH = enableHIGH;
  this->motorInvert = motorInvert;
  this->stepPort = stepPort;
  this->stepByte = stepByte;
  setSpeed(speed);
  setMinSpeed(minSpeed);
  steps = 0;
  stepRunTime = 0;
  currentStepDelayDuration = maxStepDelayDuration;
  
}

//Needs to be called in setup to initialise pins
void Motor::begin(){
  //Pin Mode Setup
  pinMode(energisePin,OUTPUT);
  pinMode(iSleepPin,OUTPUT);
  pinMode(speedPin,OUTPUT);
  pinMode(dirPin,OUTPUT);
  pinMode(faultPin,INPUT);
  pinMode(currentPin,INPUT);

}

bool Motor::step(unsigned int elapsedMicros, bool contMove){
  bool stepped = false;
  stepRunTime += elapsedMicros;
  if (steps > 0){
    if(!stepDelay){
      *stepPort = *stepPort & (~stepByte);
      delayMicroseconds(3);
      *stepPort = *stepPort | stepByte;
      stepped = true;
      stepDelay = true;
      if(!contMove){
        steps--;
        }
    }
    else{
      if(stepRunTime > currentStepDelayDuration){
        stepRunTime = 0;
        updateAccel();  
        stepDelay = false;
      }
   }
  }
  return stepped;
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

void Motor::move(int stepsToMove){
  steps = stepsTarget = abs(stepsToMove);
  
  //Set Direction of Motor
  digitalWrite(dirPin, (stepsToMove > 0) ^ motorInvert);
  //Set speed to min
  currentSpeed = minSpeed;
}

void Motor::updateAccelParams(){
  accelLength = (speed-minSpeed)/accelRate;
  stepsTarget = steps;
}

//setters
void Motor::setSpeed(int speed){
  this->speed = speed;
  digitalWrite(speedPin,
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

void Motor::hardStop(void){
  setSpeed(0);
  digitalWrite(energisePin,LOW);
  }

void Motor::softStop(void){
  rampSpeed(0);
  digitalWrite(energisePin,LOW);
  }
  
#endif
