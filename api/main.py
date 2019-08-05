#NAME:  main.py
#DATE:  Wednesday 5th August 2019
#AUTH:  Ryan McCartney, EEE Undergraduate, Queen's University Belfast
#DESC:  A python script for running a cherrpi API as a serial passthrough
#COPY:  Copyright 2018, All Rights Reserved, Ryan McCartney

import threading
import cherrypy
import serial
import time
import json
import os

cofigFilePath =  "api/settings.json"

#define threading wrapper
def threaded(fn):
    def wrapper(*args, **kwargs):
        thread = threading.Thread(target=fn, args=args, kwargs=kwargs)
        thread.start()
        return thread
    return wrapper

try:
    
    class API(object):

        def __init__(self,cofigFilePath):
            
            self.loadConfig(cofigFilePath)
            
            #Initiialise other Variables
            self.connected = False
            self.serialMonitorData = ["-,-"]*self.serialMonitorLines
            self.latestMessage = ""
            self.previousMessage = ""
            self.indexPrepared = False

            #Update Server Port
            cherrypy.config.update(
                {'server.socket_host': '0.0.0.0',
                 'server.socket_port': self.serverPort}
            )   

            #On startup try to connect to serial
            self.connect()
            self.startXboxControl()

        def loadConfig(self,configFilePath):
            with open(configFilePath) as configFile:  
                config = json.load(configFile)
                self.serverName  = config["serverName"]
                self.serverPort  = config["serverPort"]
                self.serialPort  = config["serialPort"]
                self.baudrate  = config["baudrate"]
                self.serialMonitorLines  = config["serialMonitorLines"]
                self.hostname  = config["hostname"]

        @cherrypy.expose
        def index(self):
            if not self.indexPrepared:
                self.prepareIndex()
            #On index try to connect to serial
            self.connect()

            with open ("api/index.html", "r") as webPage:
                contents=webPage.readlines()
            return contents
        
        def prepareIndex(self):
            contents = ""
            with open("api/baseIndex.html", "rt") as webPageIn:
                for line in webPageIn:
                    contents += line.replace('SERVERNAMEFEILD',self.serverName)
            with open("api/index.html", "wt") as webPageOut:
                    webPageOut.write(contents)
                    self.indexPrepared = True

        @cherrypy.expose
        def startXboxControl(self):
            try:
                status = "Xbox Controller succesfully connected."
            except:
                status = "Xbox Controller coulf not be connected."
            return status

        @cherrypy.expose
        def joystick(self):
            self.disconnect()
            self.connect()
            with open ("api/joystick.html", "r") as webPage:
                contents=webPage.readlines()
            return contents

        @cherrypy.expose
        def clearLogs(self):

            currentDateTime = time.strftime("%d/%m/%Y %H:%M:%S")

            #Clear Transmit Log
            log = open("api/public/transmitLog.csv","w")
            log.write("Date and Time,Command String Passed\n")
            log.close()

            #Clear Receive Log
            log = open("api/public/receiveLog.csv","w")
            log.write("Date and Time,"+self.serverName+" Response\n")
            log.close()
            
            #Clear serial monitor
            self.serialMonitorData = ["-,-"]*self.serialMonitorLines

            #Return Message
            status = currentDateTime + " - INFO: Transmit and Receive Logs have been cleared."
            print(status)

            return status

        @cherrypy.expose
        def send(self,command="this"):
            
            #Get Current Date and Time for Logging
            currentDateTime = time.strftime("%d/%m/%Y %H:%M:%S")
            
            if(self.connected == False):
                status = self.connect()
    
            try:
                #Add command to transmit log
                with open ("api/public/transmitLog.csv", "a+") as log:
                    log.write(currentDateTime+","+command+"\n")

                #Write Command Passed to Serial Port
                payload = (command+'\n').encode('ascii')
                self.serial.write(payload)
                time.sleep(0.008)

                status = currentDateTime + " - INFO: '" + command + "' sent succesfully."

            except:
                status = currentDateTime + " - ERROR: Could not send '"+ command +"' to serial port. Check connection."
                self.connected = False

            print(status)
            return status

        @threaded
        def receive(self):
            
            #Initialise array to store data serial monitor data
            self.serialMonitorData = ["-,-"]*self.serialMonitorLines

            while self.connected == True:
                
                #Get Current Date and Time for Logging
                currentDateTime = time.strftime("%d/%m/%Y %H:%M:%S")
                #Read Response if Avalible
                response = "VOID"
                
                try:
                    if self.serial.in_waiting > 0:

                        response = self.serial.readline().decode('utf-8')
                    
                        response = response.strip()
                        logLine = currentDateTime+","+str(response)
                        self.latestMessage = response

                        #Add response to receive log
                        with open ("api/public/receiveLog.csv", "a+") as log:
                            log.write(logLine+"\n")
                                                
                        #Add received data to serial monitor array
                        self.serialMonitorData.pop(0)
                        self.serialMonitorData.append(logLine)        
                        #print(logLine)
                    if self.serial.in_waiting > 200:
                        self.serial.reset_input_buffer()
                        dump = self.serial.readline().decode('utf-8')
                        currentDateTime = time.strftime("%d/%m/%Y %H:%M:%S")
                        status = currentDateTime + " - ERROR: Buffer full dumping '"+str(dump)+"'."
                        print(status)
                except:
                    self.connected = False
                    currentDateTime = time.strftime("%d/%m/%Y %H:%M:%S")
                    status = currentDateTime + " - ERROR: Cannot read serial line."
                    print(status)

        @cherrypy.expose
        def serialMonitor(self):
            
            tableContent = ""
            columns = 0
            headers = ["Timestamp","Data 1","Data 2","Data 3","Data 4","Data 5","Data 6","Data 7","Data 8"]

            columns = 8 
        
            #Get table contents
            for row in self.serialMonitorData:
                tableContent += "<tr><td width='30%'>"
                tableContent += row.replace(",", "</td><td width='70%'>")
                tableContent += "</td></tr>"

            #Add Correct number of Headers
            headerRow = "<tr>"
            for i in range(0,columns):
                headerRow += "<th>"+headers[i]+"</th>"
            headerRow = "</tr>"

            #Form Table
            table = "<table>"
            table += headerRow
            table += tableContent
            table +="</table>"

            return table

        @cherrypy.expose
        def getLast(self):
            return self.latestMessage

        @cherrypy.expose
        def getLatest(self):

            if self.previousMessage == self.latestMessage:
                message = "" 
            else:
                message = self.latestMessage

            self.previousMessage = self.latestMessage

            return message

        @cherrypy.expose
        def connect(self):

            currentDateTime = time.strftime("%d/%m/%Y %H:%M:%S")
            status = currentDateTime + " - INFO: Motor control box arduino already connected."

            if(self.connected == False):
                
                try:
                    self.disconnect()
                    #Open Serial Connection
                    self.serial = serial.Serial(
                        port= self.serialPort,
                        baudrate=self.baudrate,
                        parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE,
                        bytesize=serial.EIGHTBITS,
                        )
                    time.sleep(1)
                    self.connected = True
                    self.receive()
                    status = "INFO: "+self.serverName+" connected to "+self.serial.name+"."
                except:
                    status = "ERROR: Could not establish a connection with "+self.serverName+"."
      
            print(status)

            return status   

        @cherrypy.expose
        def disconnect(self):

            try:
                self.serial.close()
                self.connected = False
                status = "INFO: "+self.serverName+" disconnected."
            except:
                status = "INFO: "+self.serverName+" is not connected."

            print(status)
            return status   

        @cherrypy.expose
        def getImage(self):

            image = "NOT YET OPERATIONAL"
            
            return image

    if __name__ == '__main__':

        cherrypy.config.update(
            {'server.socket_host': '0.0.0.0',
             'server.socket_port': 8080}
        )     
        cherrypy.quickstart(API(cofigFilePath), '/',
            {
                'favicon.ico':
                {
                    'tools.staticfile.on': True,
                    'tools.staticfile.filename': os.path.join(os.getcwd(),'api/public/favicon.ico')
                },
                '/public': {
                    'tools.staticdir.on'    : True,
                    'tools.staticdir.dir'   : os.path.join(os.getcwd(),'api/public'),
                    'tools.staticdir.index' : 'index.html',
                    'tools.gzip.on'         : True
                }
            }
        )        
except:
    print("ERROR: Main sequence error.")
