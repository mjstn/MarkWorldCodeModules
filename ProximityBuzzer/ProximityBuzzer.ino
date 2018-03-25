//
// Proximity buzzer using HC-SR04 Sonar and a low cost PWM driven buzzer
//
// Intended for use with Arduino Nano with the following wiring
//   HC-SR04  is soldered with GND in the Nano GND pin and then other leads into D2, D3, D4 then solder wire from +5 to D4
//   Led      is soldered into D5 and D6 with D5 also tied to D4 and +5.  Diode must have internal resistor and substrate node is on D6
//   Piezo    Piezo passive buzzer goes from GND to D10
//   Adjust   Optional adjustment perhaps for frequency or maybe sensitivity is 10k pot across leads A1, A2, A3
// 
// 20180324    Created from Sonar test skitch by Mark Johnston of Mark-Toys.com
//
// Suggested options depending on usage:
// Look for going from no detect to detect and then output coded beeps using long and short or modulate frequency
// So you could set a parameter for a given sensor with some switch settings and then use long/short beep patterns for 'which sensor'
// Frequency changes are a function of PWM frequency.  This info is 'out there' but research is required for frequency changes
//

#define   PROXIMITY_ZONE_CM   180   // Default cm till object will trigger this alarm
#define   USE_ZONE_POT        1     // Define as 1 to read analog pot for setting proximity zone with trim pot

const int LedPin    = 13;  // STM Maple Mini is 33    Arduino Uno and Nano and Teensy 3.2 is 13
const int NearLed   =  6;  // Lights when object is near enough to be in range of the warning

const int DispTxPin = 7;   // Display Serial Transmit pin.  

const int BuzzerPin = 10;  // Buzzer will be on a PWM capable output pin

const int EchoPin1  = 2;   // Sonar unit echo readback
const int TrigPin1  = 3;   // Sonar unit trigger signal

const int vcc1Pin   = 4;   // We mount sonar and LED in digital inputs but tie those to 5V 
const int vcc2Pin   = 5;

const int AnalogPotGnd = A3;
const int AnalogPot1   = A4;  // Optional reading we may use for sensitivity 
const int AnalogPotVCC = A5;

int proxSetpoint = 150;   // Default cm for trigger of the alarm zone


#include <Wire.h>

// Define optional serial display (must setup Arduino to use SoftwareSerial Lib)
#undef DISPLAY_MODULE
#ifdef DISPLAY_MODULE
#include <SoftwareSerial.h>
SoftwareSerial displaySerial = SoftwareSerial(255, DispTxPin);
#endif

long duration;
int  cm;
int  pwmPercent;

int  loopIdx = 0;

int  tofRange = 0;

void setup() {

  pinMode(vcc1Pin, INPUT);      // We tie this to VCC so have it be an input
  pinMode(vcc2Pin, INPUT);      // We tie this to VCC so have it be an input

  Serial.begin(9600);

  Wire.begin();

  pinMode(DispTxPin, OUTPUT);   // Optional serial display Tx line

  // Adjustment pot will be on A1,A2,A3
  

  pinMode(BuzzerPin, OUTPUT);   // Audio buzzer driver

  pinMode(AnalogPotGnd, OUTPUT);
  pinMode(AnalogPotVCC, OUTPUT);
  digitalWrite(AnalogPotGnd, LOW);   // To ease pot wiring we supply GND on this pin
  digitalWrite(AnalogPotVCC, HIGH);  // To ease pot wiring we supply V+ on this pin
  pinMode(AnalogPot1, INPUT);   // analog input for PWM setting

  pinMode(NearLed, OUTPUT);     // Configure LED that will indicate object is close
  digitalWrite(LedPin, HIGH);
    
  pinMode(LedPin, OUTPUT);
  digitalWrite(LedPin, HIGH);
  delay(200);
  digitalWrite(LedPin, LOW);
  delay(200);
  digitalWrite(LedPin, HIGH);
  delay(200);
  digitalWrite(LedPin, LOW);
  delay(200);
  digitalWrite(LedPin, HIGH);
  delay(200);
  digitalWrite(LedPin, LOW);
  
  // Trigger pins to Sonar device(s)
  pinMode(TrigPin1, OUTPUT);
  
  // Echo pins back from Sonar device(s)
  pinMode(EchoPin1, INPUT_PULLUP);

  // Start and then Clear optional display then be sure it is lit
  #ifdef DISPLAY_MODULE
  displaySerial.begin(9600);
  delay(10);
  displaySerial.write(12); 
  delay(50);
  displaySerial.write(128);                // place cursor at first char of top line
  #endif
  
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
  #ifdef DISPLAY_MODULE
  displaySerial.print("                                ");
  displaySerial.print(208);     // I attempt to shorten the anoying starup tune with 1/64th notes
  #endif
  
  while (1) {
    
    duration = 0.0;
    proxSetpoint = PROXIMITY_ZONE_CM;
    // Optionally later have pot to set a trigger distance 
    if (USE_ZONE_POT) {
      // The A/D returns values of 0 - 1024 so make this relate to center point -1 meter to +2 meters
      proxSetpoint = PROXIMITY_ZONE_CM - (((analogRead(AnalogPot1) - 50) * 2) / 2  );
      Serial.print("Threshold in cmd = "); Serial.println(proxSetpoint);
    }
  
    Serial.println("Starting main while loop.\n");
  
    // Get an echo reading
    digitalWrite(TrigPin1, LOW);
    delayMicroseconds(4);           // No lower than 2
    digitalWrite(TrigPin1, HIGH);
    delayMicroseconds(6);           // No lower than 5
    digitalWrite(TrigPin1, LOW);
    
    // Turn on LED while we wait for echo
    //digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
    
    // Get the echo
    duration = pulseIn(EchoPin1, HIGH, 200000);
    
    //digitalWrite(13, LOW);    // turn the LED off after we got the echo result
    
    // convert microsec to cm
    cm = duration / 58;

    if ((cm != 0) && (cm <= proxSetpoint)) {
      digitalWrite(NearLed, LOW);         // Turn ON  'near' led LED
      analogWrite(BuzzerPin, 120);        // Turn ON piezo buzzer
    } else {
      digitalWrite(NearLed, HIGH);        // Turn OFF 'near' led LED
      analogWrite(BuzzerPin, 0);          // 
    }

    #ifdef DISPLAY_MODULE
    displaySerial.write(17);                 // Turn backlight on (redundant)
    displaySerial.write(128);                // place cursor at first char of top line
    if (duration == 0.0) {
      //              12345678901234561234567890123456
      displaySerial.print(" No Sonar Echo!                 ");
    } else {
      //              12345678901234561234567890123456
      displaySerial.print("Sonar Distance                  ");   // Print with flush for full display
    }
    delay(20);
    displaySerial.write(148);                // place cursor at first char of second row
    displaySerial.print(cm);
    displaySerial.print(" cm");
    displaySerial.write(161);                // place cursor near end of second row
    #endif
  
    delay(500);
  }
}
