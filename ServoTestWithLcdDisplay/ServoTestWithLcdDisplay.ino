
// Tester for Servos where we set via a potentiometer and show setting on a Parallax 2x16 LCD display
//
// This is very simple minded and basic.  It is meant to show A/D readback, PWM setting and some serial but not be exact
//
// Created by Mark Johnston at Mark-World.com on 20170336 from some other test code


const int LedPin   = 13;   // STM Maple Mini is 33    Arduino Uno and Nano and Teensy 3.2 is 13

const int servoPin = 10;   // This is the PWM control signal to the Servo
#define   AnalogPot  A1    // Used to read servo setpoint from a potentiometer

#include <Servo.h>
Servo testServo;           // create servo object

// We use the SoftwareSerial library to drive the LCD display
#include <SoftwareSerial.h>
const int DispTxPin = 2;   // Display Serial Transmit pin for LCD display 
// Define a 'soft' serial port for the LCD display so the serial monitor can be used for debug or extra details as desired
SoftwareSerial displaySerial = SoftwareSerial(255, DispTxPin);

int  loopIdx = 0;

//   Values used relating to the servo test itself
int  potSetpoint;
int  pwmPercent;
int  servoDegrees = 10;
int  lastServoDegrees = -1;

// -------------------------------------------------------------------------------------
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

void display_cursorToFirstRowStart(void)
{
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
// -------------------------------------------------------------------------------------

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

  Serial.begin(9600);
 
  pinMode(DispTxPin, OUTPUT);

  pinMode(AnalogPot, INPUT);     // analog input for PWM setting
  testServo.attach(servoPin);   // Attach to D11 pwm pin

  led_blink(LedPin, 3, 200);        // Blink the led a few times
  
  displaySerial.begin(9600);
  delay(10);
  // Clear display then be sure it is lit
  display_reset();
  display_clearScreen();
  
  loopIdx = 0;

  delay(100);     
}

void loop() {
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
    
    potSetpoint = analogRead(AnalogPot);          // Get the setting for the servo from the analog potentiometer

    Serial.println("Starting main while loop.\n");

    //
    //   Servo Tester Mode
    //
    servoDegrees = map(potSetpoint, 0, 1023, 0, 180);   // Map the 0-1023 A/D input to 0-180 degrees

    // Servo tester mode
    display_cursorToFirstRowStart();          // place cursor at first char of top line
    displaySerial.print("Servo Tester                    ");   // Print with flushe for full display
    
    display_cursorToSecondRowStart();        // place cursor at first char of second row
    displaySerial.print("Servo Deg: ");
    displaySerial.print(servoDegrees);
    
    display_cursorToSecondRowStart();        // place cursor near end of second row
    
    pwmPercent = servoDegrees / 3;
    if (pwmPercent > 100) {
      pwmPercent = 100;
    }

    if (lastServoDegrees != servoDegrees) {
      Serial.print("Setting servo to ");Serial.println(servoDegrees);
      testServo.write(servoDegrees);      // Update servo hardware.  Every time this jolts the servo 
      lastServoDegrees = servoDegrees;
      delay(2000);                        // Increase delay to not have the servo jolt for each setting
    }

    delay(1000);    // We go through this loop and the servo glitches each time.  Unresolved issue.
  }
}
