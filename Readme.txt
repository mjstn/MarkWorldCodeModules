This repository contains libs for Arudino developed on Arduino UNO unless specified otherwise.

Below is a description of the libraries present

Adafruit_L3GD20_PollingI2C-master
This is a modified version of Adafruit_L3GD20-master lib which exclusively used the Wire lib.
The purpose is to allow the Adafruit lib to operate in an Arduino ISR which is rather key in some uses.
At this time the submission of Dec 15 2014 has an ifdef defined for I2CMOD which then causes this
lib to use the ISR safe I2C library by Wayne Truchsess (which you must supply).   
The purpose of this lib is to have support for Wire or I2C interfaces by changing the I2CMOD define.
The intent is that if one uses   undef for I2CMOD in the .cpp file then this reverts to Adafruit code of Dec 2014
It would be a fairly easy modification to make this code use either one per a parameter for the .begin() method.
The API to the library is identical to the Adafruit usage even though there was room to have better error reporting.


Adafruit_MMA8451_PollingI2C-master
This is a modified version of Adafruit_MMA8451_Library-master lib which exclusively used the Wire lib.
The purpose is to allow the Adafruit lib to operate in an Arduino ISR which is rather key in some uses.
At this time the submission of Dec 15 2014 has an ifdef defined for I2CMOD which then causes this
lib to use the ISR safe I2C library by Wayne Truchsess (which you must supply).   
The purpose of this lib is to have support for Wire or I2C interfaces by changing the I2CMOD define.
The intent is that if one uses   undef for I2CMOD in the .cpp file then this reverts to Adafruit code of Dec 2014
It would be a fairly easy modification to make this code use either one per a parameter for the .begin() method.
The API to the library is identical to the Adafruit usage even though there was room to have better error reporting.