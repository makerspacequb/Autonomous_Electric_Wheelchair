#NAME:  install.py
#DATE:  Monday 5th August 2019
#AUTH:  Ryan McCartney
#DESC:  A python script for installing wheelchair API utility
#COPY:  Copyright 2019, All Rights Reserved, Ryan McCartney

import os
import json
settingFilePath = "settings.json"

def getDependencies():
    #Install Dependencies
    os.system("sudo pip3 install --upgrade pip")
    os.system("sudo pip3 install CherryPy")
    os.system("sudo pip3 install pyserial")
    os.system("sudo pip3 install pygame")
    
def setupPythonStartup():
    
    cwd = os.getcwd()
    startupFile = "/etc/rc.local"
    startupFileContents = None
    cdCommand ="cd "+str(cwd)
    startCommand="sudo python3 "+str(cwd)+"/api/start.py"
        
    with open(startupFile, 'r') as file:
        startupFileContents = file.readlines()
    
    lineNumber = len(startupFileContents)-1
    startupFileContents.insert(lineNumber,cdCommand + "\n")
    lineNumber = len(startupFileContents)-1
    startupFileContents.insert(lineNumber,startCommand + "\n")
    
    with open(startupFile,'w') as file:
        file.writelines(startupFileContents)
        
def changeHostname():
    #Change Hostname
    hostname = settings["hostname"]
    os.system("sudo hostname "+hostname)
  
if __name__ == '__main__':
    
    #Load Settings File
    settingsFile = open(settingFilePath,"r")
    settings = json.load(settingsFile)
    settingsFile.close()

    print("INFO: Installing pyapi")
    print("INFO: Getting dependencies for operation")
    getDependencies()
    print("INFO: Setting up Raspberry Pi to run pyapi on startup")
    setupPythonStartup()
    print("INFO: Changing hostname")
    changeHostname()
    print("INFO: Installation complete. Please reboot.")