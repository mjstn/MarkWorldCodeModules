This repository contains libs for Arudino developed on Arduino UNO unless specified otherwise.

Below is a description of the libraries present

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