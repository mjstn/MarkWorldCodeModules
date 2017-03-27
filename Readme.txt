This repository contains Arduino IDE compatible code to be shared from Mark-World.com

Visit Mark-World.com (also called mark-toys.com) to see many tech projects

Below is a description of the Code posted to this repository

SonarHC-SR04WithLCDDisp
Use an Arduino nano to control an HC-SR04 'sonar' ultrasonic range finding device.   We then control a Parallax 2 line by 16 character display over a serial line so that we can have a handheld rangefinder project.  

ProxSensors
Use an Arduino nano to monitor up to 8 VL53L0X IR time of flight distance measurement units.
Respond to a host as to what ranges are current for each unit in either polling or automatic periodic sending of the values for the sensors.

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
