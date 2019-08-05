#NAME:  start.py
#DATE:  Monday 5th August 2019
#AUTH:  Ryan McCartney
#DESC:  A python script for starting wheelchair API utility
#COPY:  Copyright 2019, All Rights Reserved, Ryan McCartney

import os

if __name__ == '__main__':
    
    print("Starting Wheelchair API")
    cwd = os.getcwd()
    mainPath = cwd+"/api/main.py"
    os.system("sudo python3 "+mainPath)