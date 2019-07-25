//------------------------------------------------------------------------------------------------------------------
// NAME: Encoder Testing
// AUTH: Ryan McCartney
// DATE: 25th July 2019
// DESC: Left, Right Motor Encoders for distance measurement and angle
// NOTE: All Rights Reserved, 2018, Queen's University Belfast
//------------------------------------------------------------------------------------------------------------------

//Pin Definitions
#define rightEncoderA 18
#define rightEncoderB 19
#define leftEncoderA 20
#define leftEncoderB 21

//Constant Definitions
#define PI 3.1415926535897932384626433832795
#define ENCODER_DIAMETER 68 //IN MILIMETERS
#define WHEEL_DIAMETER 230 //IN MILIMETERS
#define PULSES_PER_REV 600

//Global Variables
volatile int leftPulses = 0;
volatile int rightPulses = 0;
double leftWheelTravel = 0.00;
double rightWheelTravel = 0.00;
double encoderMultiplier = 0.00;

//INTERUPT SERVICE ROUTINES
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

void setup() {

  //Setup Pins
  pinMode(rightEncoderA, INPUT_PULLUP);
  pinMode(rightEncoderB, INPUT_PULLUP);
  pinMode(leftEncoderA, INPUT_PULLUP);
  pinMode(leftEncoderB, INPUT_PULLUP);

  //Serial Communications Setup
  Serial.begin(115200);

  //Attach Interupts
  attachInterrupt(digitalPinToInterrupt(rightEncoderA), rightEncoderPulse, RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncoderB), rightEncoderPulse, RISING);
  attachInterrupt(digitalPinToInterrupt(leftEncoderA), leftEncoderPulse, RISING);
  attachInterrupt(digitalPinToInterrupt(leftEncoderB), leftEncoderPulse, RISING);

  //Calculate Distance Constant
  encoderMultiplier = (ENCODER_DIAMETER*PI)/PULSES_PER_REV;
  
  Serial.println("Encoder Testing");
}

void printPulses(void){ 
  Serial.print("Left Pulses: ");
  Serial.print(leftPulses);
  Serial.print(", Right Pulses: ");
  Serial.println(rightPulses);
}

void plotPulses(void){ 
  Serial.print(leftPulses);
  Serial.print("\t");
  Serial.println(rightPulses);
}

//Turn Angle
float angle(float leftDistance, float rightDistance){

  float wheelbase = 0.4;
  float angleRotation = atan((leftDistance-rightDistance)/wheelbase);

  //Convert to Degrees
  angleRotation = (angleRotation/(2*PI))*360;
  
  return angleRotation;
  }

//Calculate Distance
double distanceTravelled(int pulses){
  double wheelMovement = encoderMultiplier*pulses;
  return wheelMovement;
  }

void loop() {

  double rightWheelTravelNew = distanceTravelled(rightPulses);
  double leftWheelTravelNew = distanceTravelled(leftPulses);
  //float currentAngle = angle(leftWheelTravel,rightWheelTravel);

  //Print Indicators
  if ((rightWheelTravelNew != rightWheelTravel) || (leftWheelTravelNew != leftWheelTravel)){
    Serial.print("Left wheel has travelled ");
    Serial.print(leftWheelTravel,6);
    Serial.print("mm and the right ");
    Serial.print(rightWheelTravel,6);
    Serial.println("mm.");
    rightWheelTravel = rightWheelTravelNew;
    leftWheelTravel = leftWheelTravelNew;
  }

  //plotPulses();
  //printPulses();
  
  delay(0.1);
}
