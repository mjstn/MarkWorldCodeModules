This repository contains code modules from Mark-World.com, also called Mark-Toys.com.


C source as well as Arduino IDE code folders come from projects presented now or in the past at Mark-World.com.  To be honest, this is presented to give people a head start with assorted devices.  The Arduino code stands on it's own as long as the user has pulled in support for the stated processors type.  The C code came from environments of my projects and these files should serve as a good starting out point or as examples for others so I have left it as public.  Use at your own risk and it can give a nice head start on projects.

Visit Mark-World.com (also called mark-toys.com) to see many projects where many of them use this code or modified versions of this cade due to time marching on.

Below is a description of the Code posted to this repository

Esp32_Code
Many drivers for displays and some sensors that live in the esp-idf type of dev environment are in this folder.  The source files usually sit in the 'main' folder and under that is usually the 'include' folder where the .h files will reside.  Some of these drivers will require the mark-toys i2c_driver.  I have done my best to either use the same code I use OR code that I tried to clean off non-required parts of header files from so it is usable in other projects.   Consider this code generally a good head start but you may have to resolve make issues in a few cases.

  - I2C_Driver:     Wrappers for the Esp32 sdk low level drivers
  - LSM6DS3:        Support for ST micro popular accelerometer and Gyro chip
  - VL53L0X:        Support for multiple ST micro time of flight range sensors on one I2C bus
                    This VL53L0X driver is greatly simplified effort and lacks full calibration.
  - ST7735:         Driver for TFT displays based on the ST7735 chip.  Nice but small color TFT
  - SSD1306_SH1106: Support for popular small OLED display driver chips over I2C bus
  - PCD8544:        Driver for low cost Nokia 5110 type LCD display that uses the PCD8544 chip.  
 

ProxSensors
Use an Arduino nano to monitor up to 8 VL53L0X IR time of flight distance measurement units.
Respond to a host as to what ranges are current for each unit in either polling or automatic periodic sending of the values for the sensors.

ProximityBuzzer
Use an Arduino Nano and an HC-SR04 sonar unit to form a proximity sensor with adjustable threshold.  If an object is seen closer than the threshold set with a potentiometer then we light an LED and also output a PWM signal that can be connected to a piezo driver or other driver to signal something is within the proximity alert distance.

SonarHC-SR04WithLCDDisp
Use an Arduino nano to control an HC-SR04 'sonar' ultrasonic range finding device.   We then control a Parallax 2 line by 16 character display over a serial line so that we can have a handheld rangefinder project. 

SteelBallTable
An Arduino sketch that executes two control loops that allow a steel ball to be balanced on top of a plate that has two axis of tilt controlled by two servos.   The table is a resistive pressure sensitive touch plate and is not a capacitive touch plate.  This is a very simplistic piece of code (for a control loop) and is not very fancy although understanding something of proportional control is of value because your hardware will vary and so the control coefficients will for sure be different in your usage. The steel ball table has marched on to use the Mark-Toys Esp32 dev board with C in a FreeRTOS environment so the arduino code is from an earlier model.  Visit http://mark-toys.com/Steel_Ball_Table.html to see the Steel Ball Table in action.
:

ServoTestWithLcdDisplay
Use an Arduino Nano to read a potentiometer and then set a servo.  Very basic program to just test if the servo is operating.

Adafruit_L3GD20_PollingI2C-master
A modified lib of the Adafruit_L3GD20-master lib which used the Wire lib.
This lib allow the Adafruit code to operate in an Arduino ISR and not use ISRs.
At this time the submission of Dec 15 2014 has an ifdef defined for I2CMOD.
Define I2CMOD and the I2C library by Wayne Truchsess is used (you must supply).   
You may #undef I2CMOD so this code reverts to the original Dec 2014 Adafruit lib.
Future mode could make this code use either Wire or I2C through the .begin() method.
The API to the library is identical to the Adafruit usage even though it may be nice 
to have better error reporting if the API were modified but it remains compatible

Adafruit_MMA8451_PollingI2C-master
A modified version of Adafruit_MMA8451_Library-master lib which used the Wire lib.
This lib allow the Adafruit code to operate in an Arduino ISR and not use ISRs.
At this time the submission of Dec 15 2014 has an ifdef defined for I2CMOD.
Define I2CMOD and the I2C library by Wayne Truchsess is used (you must supply).   
You may #undef I2CMOD so this code reverts to the original Dec 2014 Adafruit lib.
Future mode could make this code use either Wire or I2C through the .begin() method.
The API to the library is identical to the Adafruit usage even though it may be nice 
to have better error reporting if the API were modified but it remains compatible
