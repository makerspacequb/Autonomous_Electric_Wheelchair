# Wheelchair Original Firmware

Arduino Mega Firmware for current control of the Wheelchair Base

## Usage

1. Arduino Mega Serial line at 115200 BAUD with NL and CR
2. Commands Passed as follows; `SPEED,ANGLE,COMMAND`
3. Speed and Angle use values between -100 and +100 for full range of movement
4. Possible commands include, `RUN`, `SEND`, `STOP`, `RESET`, `BRAKEOFF`
