#NAME: wheelchair.py
#DATE: 05/08/2019
#AUTH: Ryan McCartney
#DESC: A python class for moving an entity in real-time via and http API
#COPY: Copyright 2019, All Rights Reserved, Ryan McCartney

import threading
import time
import json
import requests
import random
from requests import Session

#define threading wrapper
def threaded(fn):
    def wrapper(*args, **kwargs):
        thread = threading.Thread(target=fn, args=args, kwargs=kwargs)
        thread.start()
        return thread
    return wrapper
    
class Wheelchair:

    debug = False
    logFilePath = "logs/log.txt"
    header = {'User-Agent':'Mozilla/5.0 (Windows NT 6.1; WOW64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/41.0.2272.101 Safari/537.36'}
    
    def __init__(self,ipAddress,port,config):

        self.joints = 6
        self.logging = True
        self.baseURL = "http://"+ipAddress+":"+str(self.port)
        self.error = False
        self.timeout = 2 #Seconds
        self.pollingStatus = False

        #Values loaded from 'config.json'
        for joint in config["joints"]:
            self.jointMaxRotation.append(joint["maxRotation"])
            self.jointMaxSpeed.append(joint["maxSpeed"])
            self.jointMinSpeed.append(joint["minSpeed"])
            self.jointPosDefault.append(joint["defaultPosition"])
            self.jointSpeedDefault.append(joint["defaultSpeed"])
            self.jointAccelDefault.append(joint["defaultAccel"])

        #Status Flags
        self.jointPosition = [None]*self.joints
        self.switchState = [0]*self.joints
        self.calibrationState = [0]*self.joints
        self.movementFlag = [0]*self.joints

        try:
            self.session = requests.session()
            self.clearLogs()
            self.connected = True
        except:
            self.log("ERROR: Cannot create a session.")
            self.connected = False

        #Open a solver for kinematics
        self.kin = Kinematic()
        #Start capturing status packets
        self.getStatus()

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
    
    #Send and Receive Messages with implemented logging
    def sendCommand(self, command):
        
        #Start Timing
        start = time.time()
        #combine with host address
        message = self.baseURL + "send?command=" + command
        message = message.encode('ascii')
        if self.pollingStatus == False:
            self.getStatus()

        try:
            if self.debug == True:
                response = self.session.get(message,timeout=self.timeout)
                status = response.content.decode("utf-8").split("\n")
                self.log("INFO: Transmission response code is "+str(response.status_code))
                end = time.time()
                self.log("STATUS: Sending '"+str(command)+"' took "+str(round((end-start),2))+" seconds.")
                self.log(status[0])
            else:
                self.session.get(message,timeout=self.timeout)
            self.connected = True
        except:
            self.log("ERROR: Could not access API.")
            self.connected = False

    @threaded
    def getStatus(self):
          
        while self.connected:
            self.pollingStatus = True
            try:
                message = self.baseURL + "getLatest"
                response = self.session.get(message,timeout=self.timeout)
                status = response.content.split("\n")
            
                #Extract Joint Positions
                if(status[0].find("STATUS:")!=-1):
                    if(status[0].find("MOVEMENT") != -1):
                        data = status[0].split(",")
                        self.movementFlag = list(map(int,data[1:]))
                    elif(status[0].find("CALIBRATION") != -1):
                        data = status[0].split(",")
                        self.calibrationState = list(map(int,data[1:]))
                    elif(status[0].find("POSITION") != -1):
                        data = status[0].split(",")
                        try:
                            self.jointPosition = list(map(float,data[1:]))
                        except:
                            pass
                    elif(status[0].find("SWITCH") != -1):
                        data = status[0].split(",")
                        self.switchState = list(map(int,data[1:]))
                    else:
                        self.log("FAILED TO PARSE: "+status[0])
                elif(status[0] !=""):
                    self.log(status[0])

            except:     
                self.log("INFO: Did not receive status response from API.")
        
        self.pollingStatus = False

    def leftThrottle(self,speed):
        if (speed <= 255) and (speed >= -255):
            command = "t,l,"+int(speed)
            self.sendCommand(command)
            self.log("INFO: Left wheel speed set to "+speed+".")
        else:
            self.log("ERROR: Speed out of range.")
    
    def rightThrottle(self,speed):
        if (speed <= 255) and (speed >= -255):
            command = "t,r,"+int(speed)
            self.sendCommand(command)
            self.log("INFO: Right wheel speed set to "+speed+".")
        else:
            self.log("ERROR: Speed out of range.")

    def getPose(self):
        pose = self.kin.forwardKinematics(self.jointPosition)
        return pose

    def getWheelMovement(self):
        return self.wheelMovement 
    
    def getWheel(self,wheel):
        try:
            if wheel == "left":
                wheelID = "l"
            elif wheel == "right":
                wheelID = "r"
            elif wheel == "both":
                wheelID = "b"
            else:
                wheeelID = "error"
        except:
            wheeelID = "e"
        return wheeelID

    def setAccel(self,wheel,accel):
        wheelID = self.getWheel(wheel)
        if wheelID != "error":
            command = "z"+str(wheelID)+str(accel,2)
            self.sendCommand(command)
            self.log("INFO: "+str(wheel)+ " wheel acceleration rate adjusted to "+str(accel,2)+" mm per second squared.")
    
    def setBaseSpeed(self,speed):
        command = "s,b,"+str(spped,2)
        self.sendCommand(command)
        self.log("INFO: Base wheel speed adjusted to "+str(speed,2)+" mm/s.")
    
    def moveAtSped(self,wheel,speed):
        wheelID = self.getWheel(wheel)
        if (wheelID == "l") or (wheelID == "r"):
            command = "s"+str(wheelID)+str(speed,2)
            self.sendCommand(command)
            self.log("INFO: "+str(wheel)+ " wheel speed set to "+str(accel,2)+" mm/s.")
    
    def stop(self):
        self.sendCommand("q")
        self.log("INFO: Arm Emergency Stopped.")

    def checkConnection(self):
        self.sendCommand("test")
        self.log("INFO: Testing the connection.")
               
    def clearLogs(self):
        url = self.baseURL + "clearLogs"
        response = self.session.get(url,timeout=self.timeout)
        if response.content.decode("utf-8"):
            self.log(response.content.decode("utf-8"))

    def reset(self):
        messages = ["disconnect","connect"]
        for message in messages:
            url = self.baseURL + message
            response = self.session.get(url,timeout=self.timeout)
            if response.content.decode("utf-8"):
                self.log(response.content.decode("utf-8"))

        time.sleep(1.5)
        self.log("INFO: Arm Reset.")
    
    def eStopReset(self):
        self.sendCommand("r")
        time.sleep(1)
        self.log("INFO: Emergency Stop Latch Reset.")