# Wheel Base Movement Firmware

Arduino Mega Firmware for current control of the Wheelchair Base

## Usage

1. Arduino Mega Serial line at 115200 BAUD with NL and CR
2. Commands Passed as follows; `SPEED,ANGLE,COMMAND`
3. Speed and Angle use values between -100 and +100 for full range of movement
4. Possible commands include, `RUN`, `SEND`, `STOP`, `RESET`, `BRAKEOFF`

## Commands Usage

Serial commands avalible for operating the wheelchair are as follows;

* `i` - Information, returns status information with positioning, movement flags and telemetry.
* `r` - Reset, required after `q`, this performs a hardware reset of the Arduino/
* `q` - Quit, calls an emergency stop function ceasing all activity.
* `s,"MOTOR","SPEED MM/S"` - Sets motor speeds, either on an individual basis or as a group. `MOTOR` options include, left hand motor - `l`, right hand motor - `r` or a base value for both `b`.
* `z,"MOTOR","ACCELERATION MM/S^2"` - Sets motor acceleration rates, either on an individual basis or as a group. `MOTOR` options include, left hand motor - `l`, right hand motor - `r` or a base value for both `b`.
* `t` - Throttle, 
* `m` - Move,

## Documentation

### `firmware.ino` Functions

### `Motor` Class Functions

* `Motor::begin` - Called when class is constructed. Sets up pins for each motor and set default values.
* `Motor::softStop` - Stops the wheelchair, applying brakes with a deceleration profile matching the set `accelRate`.
* `Motor::hardStop ` - Immediately stops the wheelchair, applying brakes without an decceleration profile.
