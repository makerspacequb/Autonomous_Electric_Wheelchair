# Autonomous Wheelchair Firmware

New improved, object-orientated firwmare for the QLab autonomous wheelchairs. To be used with an Arduino Mega.

## Implemented Features

* PID Speed Control using Rotary Encoders
* Latching Emergency Stop
* 1Hz Status Messages with position updates
* Position Tracking
* Acceleration/Deceleration Profiles
* Manual Throttle Contol for Teleprecence

## Future Firmware Features

Some features that may be useful to implement in the future.

* Movements at predetermined speeds, angles or angular velocity
* Move to x,y cartesian position
* Manual Brake control, for exmaple `b,0` or `b,1`.

## Commands

* `t,l,NUMBER`, throtles left motor to a `NUMBER` whose value is betweem -255 and 255
* `t,r,NUMBER`, throtles right motor to a `NUMBER` whose value is betweem -255 and 255
* `s,l,NUMBER`, throtles left motor speed to a `NUMBER` whose value is a float in mm/s. PID controlled.
* `s,r,NUMBER`, throtles right motor speed to a `NUMBER` whose value is a float in mm/s. PID controlled.
* `r` resets the arduino. Required if emergency stop has been pressed.
* `q` quit, ceases all activity.
* `s,b,NUMBER`, sets the base speed of the system to a `NUMBER` in mm/s. Used for making specific distance moves.
* `z,l,NUMBER`, sets the left motor acceleration to `NUMBER` in milimeters per second squared.
* `z,r,NUMBER`, sets the right motor acceleration to `NUMBER` in milimeters per second squared.
* `z,b,NUMBER`, sets the base motor acceleration to `NUMBER` in milimeters per second squared. For moves requiring the motors to be used syncronously.
* `i`, returns status update on the wheelchair. Information includes distances moved. Movement flags for the left and right motor and telemery. System voltage is given in volts and left and right motor current in Amperes.

## Throttle Command for Teleprescence

Using the `t` command throttles the wheelchair's left or right motor independently. Whilst the rotary encoder will continue to report the distance each wheel has moved as part of the status messages this data is not used to manage speed. I.E. the system is open loop. Speed is set arbitarily within an 8 bit range 0 to 255. 0 represents the wheel being stationary and 255 at full speed which is approximately 8km/h. The polarity of this value determines direction. +255 results in full speed forward, -255 results in full speed reverse.

Additionally, the firmware manages control of the drivers in this mode. Driver power supplies are isolated when not in use automatically. Brakes are automatically applied when both left and right motors are stationary. The flashing beacon is turned on via a relay when the firmware initiates any movement.

## Usage

1. Arduino Mega Serial line at 115200 BAUD with NL and CR
2. Commands Passed as follows; `t,l,100`

### `Motor` Class Functions

* `Motor::begin` - Called when class is constructed. Sets up pins for each motor and set default values.
* `Motor::softStop` - Stops the wheelchair, applying brakes with a deceleration profile matching the set `accelRate`.
* `Motor::hardStop ` - Immediately stops the wheelchair, applying brakes without an decceleration profile.
