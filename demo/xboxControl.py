#NAME: xboxControl.py
#DATE: 25/07/2019
#AUTH: Ryan McCartney
#DESC: Python function to control a 6DOF robotic arm with an xbox controller.
#COPY: Copyright 2019, All Rights Reserved, Ryan McCartney

from controller import Controller
import json
import time

status = True

#Clear Log File
open('logs/log.txt', 'w').close()

with open('config/wheelchair1.json') as json_file:  
    config = json.load(json_file)

#Create instance of Arm class
control = Controller(config)

while 1:
    while control.wheelchair.connected:
        while control.gamepads > 0:
            control.connectGamepad()
            while status:
                #Get Current Data
                status = control.getGamepadData()
                status = control.mapButtons()
                status = control.mapLeftJoystick()
        time.sleep(1)
        
    if control.wheelchair.connected == False:
        #Poll connecion
        control.wheelchair.checkConnection()
        print("ERROR: Connection lost with robotic arm. Waiting for reconnect...")
        time.sleep(1)