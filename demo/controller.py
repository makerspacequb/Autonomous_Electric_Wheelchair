#NAME: mapping.py
#DATE: 05/08/2019
#AUTH: Ryan McCartney
#DESC: Python function to control a robotic wheelchair with an xbox controller.
#COPY: Copyright 2019, All Rights Reserved, Ryan McCartney

from wheelchair import Wheelchair
import pygame
import time
import json

class Controller:

    debug = False
    logFilePath = "logs/log.txt"
    deadzone = 0.3
    #COPY: Copyright 2019, All Rights Reserved, Ryan McCartney

    def __init__(self,config):

        self.logging = True
        self.gamepadConnected = False
        self.gamepadToUse = 0
        self.deadzone = 0.1

        #Create instance of Arm class
        self.wheelchair = Wheelchair(config)
        self.wheelchair.resetSerial()

        #Start Pygame
        pygame.init()
        pygame.joystick.init()

        #Setup Variables for Buttons
        self.axisTotal = 5
        self.axisPositions = [0]*self.axisTotal
        self.buttonTotal = 10
        self.lastButtonState = [0]*self.buttonTotal
   
        self.lastLeftSpeed = 0
        self.lastRightSpeed = 0
        self.topSpeed = config["topSpeed"]

    #Logging Function
    def log(self, entry):
        
        currentDateTime = time.strftime("%d/%m/%Y %H:%M:%S")
        logEntry = currentDateTime + ": " + entry

        if self.logging == True:
            #open a txt file to use for logging
            logFile = open(self.logFilePath,"a+")
            logFile.write(logEntry+"\n")
            logFile.close()

        print(logEntry)
    
    def mapLeftJoystick(self):
        status = True
        try:
            #Capture Button States
            for i in range(0,self.axisTotal):
                self.axisPositions[i] = self.gamepad.get_axis(i)
                self.mapWheelSpeeds(self.axisPositions[0],self.axisPositions[1])
        except:
            self.log("ERROR: Failed to map left joystick.")
            status = False
        return status

    def mapButtons(self):
        buttonState = [0]*self.buttonTotal
        status = True
        try:
            #Capture Button States
            for i in range(0,self.buttonTotal):
                buttonState[i] = self.gamepad.get_button(i)

            #A BUTTON - RESET
            if(buttonState[0] and (self.lastButtonState[0] == 0)):
                self.wheelchair.resetSerial()
            #B BUTTON - STOP
            if(buttonState[1] and (self.lastButtonState[1] == 0)):
                self.wheelchair.eStop()
            #Y BUTTON - INCREASE SPEED
            if(buttonState[3] and (self.lastButtonState[3] == 0)):
                if (self.topSpeed+1) > self.wheelchair.maxSpeed:
                    self.topSpeed = self.wheelchair.maxSpeed
                else:
                    self.topSpeed = self.topSpeed + 1
            #X BUTTON - DECREASE SPEED
            if(buttonState[2] and (self.lastButtonState[2] == 0)):
                if (self.topSpeed-1) < 0:
                    self.topSpeed = 0
                else:
                    self.topSpeed = self.topSpeed - 1
            #START BUTTON - CALIBRATE ARM
            if(buttonState[7] and (self.lastButtonState[7] == 0)):
                self.log("INFO: START button pressed.")
            #LEFT THUMB BUTTON - CHANGE JOINT
            if(buttonState[8] and (self.lastButtonState[8] == 0)):
                self.log("INFO: LEFT THUMB button pressed.")
                self.wheelchair.stop()
            #RIGHT THUMB BUTTON - CHANGE JOINT
            if(buttonState[9] and (self.lastButtonState[9] == 0)):  
                self.log("INFO: RIGHT THUMB button pressed.")
                self.wheelchair.stop()  
            self.lastButtonState = buttonState
        except:
            self.log("ERROR: Mapping Buttons Error.")
            status = False
        return status
    
    def gamepads(self):
        gamepads = pygame.joystick.get_count()
        self.log("INFO: There are "+str(gamepads)+" connected to the PC.")
        return gamepads
    
    def getGamepadData(self):
        status = True
        try:
            #Get Current Data
            pygame.event.get()
        except:
            status = False
        return status

    def connectGamepad(self):
        #Initialise first gamepad
        self.gamepad = pygame.joystick.Joystick(self.gamepadToUse)
        self.gamepad.init()
        self.log("INFO: Gamepad Connected Succesfully.")
    
    def mapWheelSpeeds(self,xAxisPos,yAxisPos):
        
        #Converting to Discrete Speed Control
        xAxisPos = round(xAxisPos,2)
        yAxisPos = round(yAxisPos,2)
        #self.log("STATUS: AXIS "+str(yAxisPos))
        if (abs(xAxisPos)>self.deadzone) and (abs(yAxisPos)>self.deadzone):
            mappedLeftSpeed = self.mapToRange(-yAxisPos,-1,1,-self.wheelchair.maxSpeed,self.wheelchair.maxSpeed)
            mappedRightSpeed = self.mapToRange(-yAxisPos,-1,1,-self.wheelchair.maxSpeed,self.wheelchair.maxSpeed)
            
            if mappedLeftSpeed != self.lastLeftSpeed:
                pass
                #self.wheelchair.leftThrottle(mappedLeftSpeed)
            if mappedRightSpeed != self.lastRightSpeed:
                pass
                #self.wheelchair.rightThrottle(mappedRightSpeed)

            self.lastLeftSpeed = mappedLeftSpeed
            self.lastRightSpeed = mappedRightSpeed
        else:
            #In Deadzone
            self.wheelchair.softStop()
            self.lastLeftSpeed = 1
            self.lastRightSpeed = 1

    
    @staticmethod
    def mapToRange(raw,rawMin,rawMax,mapMin,mapMax):
        mapped = (raw - rawMin) * (mapMax - mapMin) / (rawMax - rawMin) + mapMin
        return mapped