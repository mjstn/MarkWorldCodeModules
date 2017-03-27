
// Tester for HC-SR04 Sonar with readout on an optional Parallax 2x16 LCD display
//
// Use an HC-SR04 'sonar' unit as a distance meter with an LCD display 
// This is coded for use with an Arduino Nano
// 
// Created by Mark Johnston at Mark-World.com on 20170336 from some other test code
//

// Define the pin numbers to be used for functions
const int LedPin    = 13;  // Arduino Uno and Nano use 13

const int TrigPin1  = 3;
const int EchoPin1  = 4;
const int TrigPin2  = 5;
const int EchoPin2  = 7;

#define   AnalogPot A1     // Not really required, just shows how to read an analog input 


// We use the SoftwareSerial library to drive the LCD display
#include <SoftwareSerial.h>
const int DispTxPin = 2;   // Display Serial Transmit pin for LCD display 
// Define a 'soft' serial port for the LCD display so the serial monitor can be used for debug or extra details as desired
SoftwareSerial displaySerial = SoftwareSerial(255, DispTxPin);

long duration;
int  cm;
int  potSetpoint;

int  loopIdx = 0;


// Parallax 2x16  unique display control is here.
// To simplify port for other display we have these routines all here
// For the actual prints to the display they are in the code so look for   displaySerial.print
//
void display_reset()
{
  displaySerial.write(12); 
  delay(50);
}
void display_clearScreen()
{
    displaySerial.write(17);                 // Turn backlight on (redundant)
    displaySerial.write(128);                // place cursor at first char of top line
}
void display_cursorToSecondRowStart(void)
{
    displaySerial.write(148);                // place cursor at first char of second row
}
void display_cursorToSecondRowEnd(void)
{
    displaySerial.write(161);                // place cursor at first char of second row
}


// Helper to make code more readable when we start a reading
void sonar_sendPing(void)
{
    digitalWrite(TrigPin1, LOW);
    digitalWrite(TrigPin2, LOW);
    delayMicroseconds(4);           // No lower than 2
    digitalWrite(TrigPin1, HIGH);
    digitalWrite(TrigPin2, HIGH);
    delayMicroseconds(6);           // No lower than 5
    digitalWrite(TrigPin1, LOW);
    digitalWrite(TrigPin2, LOW);
}

// A helper to blink the pin specified (normally led) for some number of blinks of a given duration
void led_blink(int LedPin, int blinks, int milliSec)
{
  int idx;

  pinMode(LedPin, OUTPUT);    // Assure the pin is set as an output

  for (idx=0 ; idx < blinks ; idx++) {
    digitalWrite(LedPin, HIGH);
    delay(milliSec);
    digitalWrite(LedPin, LOW);
    delay(milliSec);
  }
}

void setup() {
  Serial.begin(9600);               // This is the arduino serial monitor that can be used for debug or if no LCD display
 
  pinMode(DispTxPin, OUTPUT);       // Used so for software serial output to LCD display we set the Tx pin for output
  pinMode(AnalogPot, INPUT);        // analog input to show how this is done (completely optional)

  led_blink(LedPin, 3, 200);        // Blink the led a few times
  
  // Trigger pins to Sonar device(s)
  pinMode(TrigPin1, OUTPUT);
  pinMode(TrigPin2, OUTPUT);
  
  // Echo pins back from Sonar device(s)
  pinMode(EchoPin1, INPUT_PULLUP);
  pinMode(EchoPin2, INPUT_PULLUP);

  displaySerial.begin(9600);
  delay(10);
  
  // Clear display then be sure it is lit
  display_reset();
  display_clearScreen();
  
  loopIdx = 0;

  delay(100);     
}

void loop() {
  uint16_t rPanel_x, rPanel_y;
  uint8_t rPanel_z;

  Serial.println("Starting main loop.\n");

  loopIdx += 1;
  if (loopIdx & 1) {
    digitalWrite(LedPin, HIGH);
  } else {
    digitalWrite(LedPin, LOW);
  }
  
  // Initially write out to clear full display
  displaySerial.print("                                ");
  
  while (1) {
    duration = 0.0;
  
    potSetpoint = analogRead(AnalogPot);    // We don't really use this, this is just a demo for analog read

    Serial.println("Starting main while loop.\n");

    //   start out the sonar by doing a 'ping' on the transmitter
    sonar_sendPing();             // Transmits a ping on the sonar transmitter

    // Turn on LED while we wait for echo
    //digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
  
    // Get the echo by waiting for the sonar unit with a timeout of 200 milliseconds
    duration = pulseIn(EchoPin1, HIGH, 200000);
  
    //digitalWrite(13, LOW);    // turn the LED off after we got the echo result
  
    // convert microsec to cm
    cm = duration / 58;
  
    display_clearScreen();
    
    if (duration == 0.0) {
      //              12345678901234561234567890123456
      displaySerial.print(" No Sonar Echo!                 ");
    } else {
      //              12345678901234561234567890123456
      displaySerial.print("Sonar Distance                  ");   // Print with flushe for full display
    }
    delay(20);
    display_cursorToSecondRowStart();
    displaySerial.print(cm);
    displaySerial.print(" cm");
    display_cursorToSecondRowEnd();
 
    delay(1000);
  }
}
