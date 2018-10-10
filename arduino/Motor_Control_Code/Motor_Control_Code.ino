//------------------------------------------------------------------------------------------------------------------
// NAME: Electric Wheelchair Motor Control Alogtrithm
// AUTH: Ryan McCartney
// DATE: 9th October 2018
// DESC: Recieves serial communications containing control commands for the electric wheelchair
// NOTE: All Rights Reserved, 2018, Queen's University Belfast
//------------------------------------------------------------------------------------------------------------------

//Pin Definitions
#define RightMotorDirection 24 //DIR Input on Board
#define LeftMotorDirection 22 //DIR Input on Board
#define RightMotorCoast 28 //SLP Input on Board
#define LeftMotorCoast 26  //SLP Input on Board
#define RightMotorFault 32 //FLT Pin on Board
#define LeftMotorFault 30  //FLT Pin on Board
#define RightMotorSpeed 3 //PWM Input on Board
#define LeftMotorSpeed 2 //PWM Input on Board
#define RightMotorCurrent 1 //CS Pin on Board
#define LeftMotorCurrent 0  //CS Pin on Board

#define MotorBrakes 34 //Relay to apply mechanical brake
#define BatteryIndication 2 //Voltage Sensor

//Receive Data Variables
float maxCurrent = 3.00;

//Receive Data Variables
String inputString = "";
String command = ""; //Override commands to STOP, indicate ERRORs, etc.
int setSpeed = 0; // -100 to +100 (Reverse to Forward)
int setAngle = 0; // -100 to +100 (Left to Right)
int serialInputs = 1; //Commands Recieved for Processing
bool stringComplete = false;

//Transmittable Data
float batteryVoltage = 0.00;
float rightMotorCurrent = 0.00;
float leftMotorCurrent = 0.00;
int rightMotorFault = 0;
int leftMotorFault = 0;
String statusMessage = "";

//Motor Driver Output Variables
int leftMotorSpeed = 0; // 0 to 255
int rightMotorSpeed = 0; // 0 to 255
String direction = "Forward";

//------------------------------------------------------------------------------------------------------------------
//Main Programme operating loop
//------------------------------------------------------------------------------------------------------------------
void setup() {
  
  //Initialise Ourput Pins
  pinMode(RightMotorDirection, OUTPUT);
  pinMode(LeftMotorDirection, OUTPUT);
  pinMode(RightMotorSpeed, OUTPUT);
  pinMode(LeftMotorSpeed, OUTPUT);
  pinMode(MotorBrakes, OUTPUT);

  //Set Initial State
  digitalWrite(RightMotorDirection, LOW);
  digitalWrite(LeftMotorDirection, LOW);
  digitalWrite(RightMotorSpeed, LOW);
  digitalWrite(LeftMotorSpeed, LOW);
  digitalWrite(MotorBrakes, LOW);
  
  //Setup Input Pins
  pinMode(BatteryIndication, INPUT_PULLUP);
  pinMode(RightMotorCurrent, INPUT_PULLUP);
  pinMode(LeftMotorCurrent, INPUT_PULLUP);
  pinMode(RightMotorFault, OUTPUT);
  pinMode(LeftMotorFault, OUTPUT);

  //Serial Communications Setup
  Serial.begin(115200);
  statusMessage = "STATUS = System restarting.";
  sendData();
  //Reserve Memory Sapce for Incomming Bytes
  inputString.reserve(200);

}

//------------------------------------------------------------------------------------------------------------------
//Maps Speed and Direction for Output to Motor Driver
//------------------------------------------------------------------------------------------------------------------
bool mapOutputs() {

  bool status = false;

  //Check the range of speed data
  if((setSpeed <= 100 && setSpeed >= -100) || setSpeed == 0){

     //Check the range of angle data
    if((setAngle <= 100 && setAngle >= -100) || setAngle == 0){
    
      if(setSpeed < 0){
        setSpeed = -setSpeed;
        direction = "Reverse";
      }
      else{
        direction = "Forward";
      }
    
      //Map Inputs to motor PWM
      map(leftMotorSpeed, 0, 100, 0, 255);
      map(rightMotorSpeed, 0, 100, 0, 255);
    
      if (setAngle < 0) {
        setAngle = -setAngle;
        map(setAngle, 0, 100, 1, 0);
        leftMotorSpeed = leftMotorSpeed*setAngle;
        status = true;
      }
      else if (setAngle > 0){
        map(setAngle, 0, 100, 1, 0);
        rightMotorSpeed = rightMotorSpeed*setAngle;
        status = true;
        }
      else{
        status = true;
      }
    }
  }
  
    if(status == true){
      statusMessage = "STATUS = Data Mapped Succesfully";
    }
    else{
      statusMessage = "ERROR = Failed to map sent data to output. Check range of sent data.";
    }
    return status;
  }

//------------------------------------------------------------------------------------------------------------------
//When Serial input is made this function parses the contents
//------------------------------------------------------------------------------------------------------------------
bool proccessInput() {

  bool status = false;
  char delimiter = ',';
  char newline = '\r';
  int variableNumber = 1;
  String variable = "";

  // Length (with one extra character for the null terminator)
  int stringLength = inputString.length() + 1; 

  // Prepare the character array (the buffer) 
  char charArray[stringLength];

  // Copy it over 
  inputString.toCharArray(charArray, stringLength);

  
  
  //Parse char array derved from input string from serial line
  for(int i = 0; i < stringLength; i++){

   // if the incoming character is a comma then it must indicate the end of a variable
   if (charArray[i] == delimiter) {

      if (variableNumber == 1){
         setSpeed = variable.toInt();
         }
      else if (variableNumber == 2){
         setAngle = variable.toInt();
         }
      else{
        command = "ERROR";   
      }
      variableNumber++;
      variable = "";
    }

  else if(charArray[i] == newline){
    
    //Assign Command Variable
    command = variable;        
    
    //Function performed correctly if here, set status
    if(variableNumber == 3){
      status = true;
      break;
      }
    else{
      status = false;
      command = "ERROR";
      }
    }
  else{
  
  //Input Sterilisation 
  String currentChar = "";
  currentChar =+ charArray[i];
  int inputSterilise = currentChar.toInt();
     
    if(variableNumber == 1 || variableNumber == 2){
      if((inputSterilise == 45) || ((inputSterilise >= 45) && (inputSterilise <= 57))){
          variable += charArray[i];
          }
      else{
          status = false;
          command = "ERROR";
          break;
          }
      }
    else if(variableNumber == 3){
      if((inputSterilise >= 65) && (inputSterilise <= 90)){
          variable += charArray[i];
      }
      else{
          status = false;
          command = "ERROR";
          break;
          }
      }
    }
  }

  //Debug Indicators
  /*Serial.print("Data Sequence = ");
  Serial.println(serialInputs); 
  Serial.print("Raw Input = ");
  Serial.print(inputString);
  
  if(status == true){
      Serial.print("Speed Read from Serial Port = ");
      Serial.println(setSpeed);
      Serial.print("Angle Read from Serial Port = ");
      Serial.println(setAngle);
      }
      
  Serial.print("Command Read from Serial Port = ");
  Serial.println(command);
  Serial.println("");
  */
 
  //Reset Serial
  inputString = "";
  stringComplete = false;

  //Increment Commmands Processed
  serialInputs++;

  if(status == true){
      statusMessage = "STATUS = Data succesfully received.";
    }
    else{
      statusMessage = "ERROR = Data not received. Check the format of your data.";
    }
    
  return status;
}

//------------------------------------------------------------------------------------------------------------------
//When Serial input is made this function parses the contents
//------------------------------------------------------------------------------------------------------------------
bool executeCommands() {

  //Function error variable
  bool status = false;

  if(command == "ERROR"){
    stopWheelchair();
    sendData();
    status = true;
    
  }
  else if(command == "SEND"){
    sendData();
    status = true;

  }
  else if(command == "STOP"){
    stopWheelchair();
    status = true;
  }
  else if(command == "RUN"){
    status = true;    
  }
  else if(command == "BRAKEOFF"){

   //Allow wheelchair to be stopped
   setSpeed = 0;
   setAngle = 0;
  
   //Stop Wheelchair Coasting
   digitalWrite(RightMotorCoast, 1);
   digitalWrite(LeftMotorCoast, 1);

   //Turn off Brakes
   digitalWrite(MotorBrakes, 0);
    
   status = true;
  }
  else if(command == "RESET"){
    sendData();
    status = true;
  }
  
  else{
     status = false;
     statusMessage = "ERROR = No valid command to execute. Declaring Error.";
     command = "ERROR";
     sendData();
     stopWheelchair();
  }
  return status; 
}

//------------------------------------------------------------------------------------------------------------------
//Stop the Wheelchair
//------------------------------------------------------------------------------------------------------------------
void stopWheelchair() {

   //Set Variables to Appropriate Values to stop chair further moving.
   setSpeed = 0;
   setAngle = 0;
   command = "STOP";
   
   //Speed to Zero
   analogWrite(RightMotorSpeed, 0);
   analogWrite(LeftMotorSpeed, 0);
 
   //Stop Wheelchair Coasting
   digitalWrite(RightMotorCoast, 1);
   digitalWrite(LeftMotorCoast, 1);

   //Apply Brakes
   digitalWrite(MotorBrakes, 1);

   return;
}

//------------------------------------------------------------------------------------------------------------------
//Reads input ariables
//------------------------------------------------------------------------------------------------------------------
void readInputs() {

   //Contants for variable conversion
   float voltageFactor = 0.024438; //Voltage in Volts
   float voltageOffset = 0.00;
   float currentFactor = 0.244379; //Current in Amps
   float currentOffset = 0.00;

   //Read sensor data to update variables 
   batteryVoltage = (analogRead(BatteryIndication)*voltageFactor)-voltageOffset;
   rightMotorCurrent = (analogRead(RightMotorCurrent)*currentFactor)-currentOffset;
   leftMotorCurrent = (analogRead(LeftMotorCurrent)*currentFactor)-currentOffset;

   rightMotorFault = digitalRead(RightMotorFault);
   leftMotorFault = digitalRead(LeftMotorFault);

   return;
}

//------------------------------------------------------------------------------------------------------------------
//Send input variables and stauts over serial
//------------------------------------------------------------------------------------------------------------------
void sendData() {

   readInputs();
   
   if(rightMotorFault == 1){
      statusMessage = "ERROR = Right motor has developed a fault.";
   }
   else if(leftMotorFault == 1){
      statusMessage = "ERROR = Left motor has developed a fault.";
   }
   
   //Send Serial Information
   Serial.print(rightMotorCurrent);
   Serial.print(",");
   Serial.print(leftMotorCurrent);
   Serial.print(",");
   Serial.print(batteryVoltage);
   Serial.print(",");
   Serial.println(statusMessage);    
   
   return; 
}

//------------------------------------------------------------------------------------------------------------------
//Main Programme operating loop
//------------------------------------------------------------------------------------------------------------------
void loop() {

  //Function Variables
  bool status = true;
  int DIR = 0;
 
  //When new Serial Data Arrives
  if (stringComplete == true) {

      if (proccessInput() == true){

        if (mapOutputs() == true){
        
          if (executeCommands() == true){

            //Set Direction to DIR pin
            if(direction =="Forward"){
              DIR = 0;
              digitalWrite(RightMotorDirection, DIR);
              digitalWrite(LeftMotorDirection, DIR);
            }
            else if(direction =="Reverse"){
              DIR = 1;
              digitalWrite(RightMotorDirection, DIR);
              digitalWrite(LeftMotorDirection, DIR);
            }
            else{
              status = false;
            }
   
            analogWrite(RightMotorSpeed, rightMotorSpeed);
            analogWrite(LeftMotorSpeed, leftMotorSpeed);
            status = true;
          }
          else{
            stopWheelchair();
            sendData();
            }
        }
        else{
          stopWheelchair();
          sendData();
          }
      }
      else{
        stopWheelchair();
        sendData();
        }
    }

  readInputs();

  //Current limiting code
  if(rightMotorCurrent > maxCurrent || rightMotorCurrent > maxCurrent){
  
  if(setSpeed > 0){
  setSpeed = setSpeed - 1;
  }
  else if(setSpeed < 0){
  setSpeed = setSpeed + 1;
  }

  inputString = setSpeed;
  inputString += ",";
  inputString += setAngle;
  inputString += ",RUN";
  
  statusMessage = "WARNING = Overcurrent Warning, recitfying issue.";
  stringComplete = true;
  sendData;
  }
  
  if(status == false){
  
  inputString = "0,0,ERROR";
  statusMessage = "ERROR = Main Loop program error.";
  stringComplete = true;
  }
}

//------------------------------------------------------------------------------------------------------------------
//Rertieve Serial Data
//------------------------------------------------------------------------------------------------------------------
void serialEvent() {
  
  while (Serial.available()){

    //Gets the next byte
    char inputChar = (char)Serial.read();
    
    //Add latest char to string
    inputString += inputChar;
      
    if (inputChar == '\n') {
      stringComplete = true;
    }
  }
}