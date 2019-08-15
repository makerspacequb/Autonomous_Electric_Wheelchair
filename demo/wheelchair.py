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
    
    def __init__(self,config):

        self.logging = True
        self.stopped = False
        self.wheels = config["wheels"]
        self.port = config["port"]
        self.ipAddress = config["ipAddress"]
        self.baseURL = "http://"+str(self.ipAddress)+":"+str(self.port)+"/"
        self.error = False
        self.timeout = 2 #Seconds
        self.pollingStatus = False

        self.topSpeed = config["topSpeed"]
        self.maxSpeed = config["maxSpeed"]

        #Status Flags and Telemetry Variables
        self.movementFlag = [0]*self.wheels
        self.distance = [0]*self.wheels
        self.current = [0]*self.wheels
        self.voltage = 0.0

        try:
            self.session = requests.session()
            self.clearLogs()
            self.connected = True
        except:
            self.log("ERROR: Cannot create a session.")
            self.connected = False

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
                        self.movementFlag = data
                    elif(status[0].find("DISTANCE") != -1):
                        data = status[0].split(",")
                        self.distance = data
                    elif(status[0].find("TELEMETRY") != -1):
                        data = status[0].split(",")
                        self.voltage = data.pop(0)
                        self.current = data
                    else:
                        self.log("FAILED TO PARSE: "+status[0])
                elif(status[0] !=""):
                    if self.debug:
                        self.log(status[0])
            except:     
                self.log("INFO: Did not receive status response from API.")
        
        self.pollingStatus = False

    def leftThrottle(self,speed):
        self.stopped = False
        if (speed <= 255) and (speed >= -255):
            command = "t,l,"+str(speed)
            self.sendCommand(command)
            if self.debug:
                self.log("INFO: Left wheel speed set to "+str(speed)+".")
        else:
            self.log("ERROR: Speed out of range.")
    
    def rightThrottle(self,speed):
        self.stopped = False
        if (speed <= 255) and (speed >= -255):
            command = "t,r,"+str(speed)
            self.sendCommand(command)
            if self.debug:
                self.log("INFO: Right wheel speed set to "+str(speed)+".")
        else:
            self.log("ERROR: Speed out of range.")

    def getWheelMovement(self):
        return self.movementFlag 
    
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
    
    def moveAtSpeed(self,wheel,speed):
        wheelID = self.getWheel(wheel)
        if (wheelID == "l") or (wheelID == "r"):
            command = "s"+str(wheelID)+str(speed,2)
            self.sendCommand(command)
            self.log("INFO: "+str(wheel)+ " wheel speed set to "+str(accel,2)+" mm/s.")
    
    def stop(self):
        self.leftThrottle(0)
        self.rightThrottle(0)
        self.log("INFO: Wheelchair stopped and brakes applied.")
    
    def softStop(self):
        if not self.stopped:
            self.leftThrottle(1)
            self.rightThrottle(1)
            self.stopped = True
            if self.debug:
                self.log("INFO: Wheelchair stopped, brakes not applied.")
    
    def eStop(self):
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

    def resetSerial(self):
        messages = ["disconnect","connect"]
        for message in messages:
            url = self.baseURL + message
            response = self.session.get(url,timeout=self.timeout)
            if response.content.decode("utf-8"):
                self.log(response.content.decode("utf-8"))

        time.sleep(1.5)
        self.log("INFO: Arm Reset.")
    
    def resetArduino(self):
        self.sendCommand("r")
        time.sleep(1)
        self.log("INFO: Emergency Stop Latch Reset.")