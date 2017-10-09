This repository contains code modules from Mark-World.com, also called Mark-Toys.com.


Arduino IDE compatible code folders are here as well as C source.  To be honest, this is here to make it safe for my own usage but may serve as a good starting out point or as examples for others so I have left it as public.  Use at your own risk and it can give a nice head start on projects.

Visit Mark-World.com (also called mark-toys.com) to see many tech projects

Below is a description of the Code posted to this repository

SonarHC-SR04WithLCDDisp
Use an Arduino nano to control an HC-SR04 'sonar' ultrasonic range finding device.   We then control a Parallax 2 line by 16 character display over a serial line so that we can have a handheld rangefinder project.  

ProxSensors
Use an Arduino nano to monitor up to 8 VL53L0X IR time of flight distance measurement units.
Respond to a host as to what ranges are current for each unit in either polling or automatic periodic sending of the values for the sensors.

SteelBallTable
An Arduino sketch that executes two control loops that allow a steel ball to be balanced on top of a plate that has two axis of tilt controlled by two servos.   The table is a resistive pressure sensitive touch plate and is not a capacitive touch plate.  This is a very simplistic piece of code (for a control loop) and is not very fancy although understanding something of proportional control is of value because your hardware will vary and so the control coefficients will for sure be different in your usage.
:
Esp32_VL53L0X
C source for support of the VL53L0X time-of-flight proximity sensors by ST Micro.   
This is fairly rough and minimal code in the form of a .c and a .h file.
This code does NOT do the complex calibrations required for precise measurements so it is 'as is' but I have found it useful.
The code assumes use in Esp32 esp-idf environment using FreeRTOS and also the Mark-World I2C wrapper routines also in this repository.  

Esp32_I2C
This is mostly a wrapper around some esp-idf i2c calls that make I2C calls a little bit higher level than the low level esp-idf calls.  This code is used for example in the Esp32_VL53L0X module

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
