
// Proximity Sensor Array using VL53L0X TOF dist sensor
//
// Have a unit monitor up to 8 VL53L0X units and respond to a host as to what ranges are current for each 
// First implementation will hook to host via simple serial protocol but will use a crc for validation on host side.
//
// The VL53L0X driver:   https://github.com/pololu/vl53l0x-arduino
//
// Attempts have been made to use CUSTOMIZE: in this code to help find areas that are key to update for different hardware.
//
// Using Teensy 3.2 I wanted to control sensors with one I2C and be slave on other but that is not happening yet.
// So we will return the sensor values using RS232 I guess.
//
// On startup all units are held in reset then one by one they are released and new address is set.
// There is no non-volitile way on these devices so each power-down requires reset of addresses
//
// I2C goes to all VL53L0X units and each unit has one reset line that is a digital output
// Arduino Nano I2C: SCL A5 (blue), SDA A4 (yellow)   Teensy I2C:  SCL0 pin 19, SDA0 pin 18. 
// Arduino sensor reset lines D5-D12 for 8 sensors max with D5 for 1st sensor at 0 degrees. 
//
// Serial Host Interface  (a very simple protocol)
// Recognized commands (very simple 2-char commands and 3rd char is a carrage return to terminate serial input
// CMD  Reply                   Description
// a0                           Turns off automatic output mode so results must be asked of the unit by the host
// a1   Periodic: <qa:10,20:30> Enable automatic output for periodic updates.  Uses same format as qa for the reply.
// fv   <fv:1,3:4>              Returns firmware version with a major then minor rev and the normal simple sum after 2nd colin
// qa   <qa:10,20:30>           Return readings for all sensors.   In this example 2 sensors with readings of 10 and 20 are shown
// scX  <sc:X:S>                X is sensor count of 1 to 8 to setup for different sensor count than default.  S is the simple sum of data values
//
// 20170302    mjstn2011@gmail.com   Created from the breakboard code for servo and other sonar device tests
// 20170315    mjstn2011@gmail.com   1st usable form. Has simple serial protocol for asking and getting the results.
// 20170318    mjstn2011@gmail.com   Have fully wired 8-sensor hub of sensors at 45 deg all working and automatic update on/off also now available
//
// Suggested improvements: (in order of priority)
// - Accept a command to read back a single sensor like  Q1 up to Q8 for example
// - Port this to a small processor like Teensy 3.2 and support the queries to the sensor also over I2C
//

#define FW_MAJOR_VERSION                 1
#define FW_MINOR_VERSION                 3

#undef  USE_TEENSY_3.2         // NON FUNCTIONAL!  Work in progress to have this module available as an I2C slave.  (Teensy 3.2 has 2 I2C ports)

// Serial output is 'redirected' here with use of simple defines.  Normally ship with just HOSTSERIAL as  'Serial'
#define HOSTSERIAL                 Serial      // Serial port for host requests for data
#define DBGSERIAL              softSerial1     // You can use 'Serial' for normal Arduino debug ONLY if not trying to use host to read info
#define LCDSERIAL              softSerial1     // Port to be used for LCD display (not really supported for this product, just left over)

#undef  LCD_DISPLAY                            // Define to send special serial chars to go to next line or light backlight

#define HOST_SERIAL_BAUD              38400    // Baud rate in standard serial port for host query
#define DBG_SERIAL_BAUD               38400    // Baud rate for debug message serial (soft serial lib)
#define LCD_SERIAL_BAUD                9600

#define DEFAULT_VL53L01X_I2C_ADDR      0x29    // 0x29 is default from factory upon each powerup

#define SENSOR_FIRST_I2C_ADDR          0x51    // First programmed I2C addr of the sensors

#define SERIAL_ECHO                    0

// Here are the commands we recognize in this simple minded sensor subsystem
// A command is evaluated and executed on receipt of newline char
#define CMD_CHECK_ONLINE          ""      // An empty buffer (just a carrage return)
#define CMD_AUTO_UPDATE_ON        "a1"    // Enables auto-update for periodic updates with no query
#define CMD_AUTO_UPDATE_OFF       "a0"    // Disable auto-update mode so must ask for a reply
#define CMD_QUERY_ALL_SENSORS     "qa"
#define CMD_FIRMWARE_VERSION      "fv"
#define CMD_SET_SENSOR_COUNT      "sc"    // Must follow this with NO space the number of sensors like  sc4  to use 4 sensors

// This would be ideal to use a 74HC138 and just 3 pins but lets do it with just the Nano for now
// This array has the digital lines used for each of the sensors
#define SENSOR_COUNT          8      // Number of sensors to be used.  CUSTOMIZE: for your max number of sensors hooked up

#define MAX_SENSORS           8      // Maximum number of sensors suppored.  You are on your own to increase this please RTFS

int sensorCount = SENSOR_COUNT;      // Global for the number of sensors at any point in time.   The scX command changes this value.

//  Each sensor has an XSHUT pin that is held low at startup and one at a time released so a unique I2C addr can be programmed.
// The I2C address is volatile so it is gone after power down.  The pins below match my hardware
// The SENSOR_COUNT value which can change using RS-232 can be 1-MAX_SENSORS and will use values in this array which is always larger or same as SENSOR_COUNT
int sensorXshutPins[MAX_SENSORS] = { 10, 9, 8, 7, 6, 5, 12, 11 };    // CUSTOMIZE: THESE PINS Arduino Digital IO pin numbers FOR YOUR USAGE

int sensorRangesInMm[MAX_SENSORS];   // Sensor data that can be read back from host

int sensorI2cAddr[MAX_SENSORS];

const int LedPin    = 13;  // STM Maple Mini is 33    Arduino Uno and Nano and Teensy 3.2 is 13

const int DispTxPin = 2;   // Display Serial Transmit pin
const int ReadSensorPin = 4;

String  serialRxBuf = "";        // a string to hold incoming data
String  cmdString = "";
boolean rxCommandReady = false;  // whether the string is complete

int   autoUpdate = 0;            // If 1 we output every half second or so new sets of readings 
int   autoUpdateLoopCount = 2;   // 2 is max speed, larger number send out updates less frequently

#define I2C_SLAVE_ADDRESS     0x40

#define SENSOR_DATA_REPLY_BYTES  16
byte  sensorData[SENSOR_DATA_REPLY_BYTES];      // Holds 16-bit words that are sensor readings with bytes 0,1 as MSB and LSB of sensor 0 readings

#ifndef  USE_TEENSY_3.2
#include <SoftwareSerial.h>
SoftwareSerial softSerial1 = SoftwareSerial(255, DispTxPin);
#endif

#ifdef  USE_TEENSY_3.2
// #include <i2c_t3.h>      // i2c_t3 is improved lib for Teensy for multiple I2C busses instead of <Wire.h>
#include <i2c_t3wire.h>
#endif

#include <Wire.h>
#include <VL53L0X.h>     // Pololu board 7-bit addr 29 default.    https://github.com/pololu/vl53l0x-arduino

// Instantiate multiple controls.
VL53L0X VL53L0X[MAX_SENSORS];  

long duration;
int  cm;
int  loopIdx = 0;
int  i2cReceived  = 0;
int  i2cRequested = 0;

int idx = 0;

int  tofRange = 0;

// When serial input comes in on Rx line we get to here
// When we get newline we set a flag so main loop can interprit the input buffer
#ifdef USE_SERIAL_OUT_OF_LOOP
void serialEvent() {
  DBGSERIAL.println("Got ser byte!\n");
  while (HOSTSERIAL.available()) {
    // get the new byte:
    char inChar = (char)HOSTSERIAL.read();
    // add it to the inputString:
    serialRxBuf += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      rxCommandReady = true;
    }
  }
}
#endif

#ifdef  USE_TEENSY_3.2
// i2c_t3 i2c1(1);
// function that executes whenever data is received from I2C master
// this function is registered as an event, see setup()
void receiveEvent(size_t count) {
  size_t idx=0;
    i2cReceived = 1;
    while(idx < count)
    {
        if(idx < SENSOR_DATA_REPLY_BYTES)                     // drop data beyond mem boundary
            sensorData[idx++] = T3Wire1.readByte(); // copy data to mem
        else
            T3Wire1.readByte();                  // drop data if mem full
    }
  //Serial.println("i2c Receive\n");
  //while (1 < T3Wire1.available()) { // loop through all but the last
  //  char c = T3Wire1.read(); // receive byte as a character
  //  Serial.print(c);         // print the character
  //}
 // int x = T3Wire1.read();    // receive byte as an integer
  //Serial.println(x);         // print the integer
  
}

// Readback of sensor data array
void requestEvent(void)
{
     i2cRequested = 1;
     //Serial.println("i2c Request\n");
     //T3Wire1.write(sensorData, SENSOR_DATA_REPLY_BYTES);  //Set the buffer up to send all bytes of data
}
#endif

void scanI2cBus(void)
{
  DBGSERIAL.println ("I2C scanner. Scanning ...");
  byte count = 0;


  for (byte i = 1; i < 120; i++)
  {

    Wire.beginTransmission (i);
    if (Wire.endTransmission () == 0)
    {
      DBGSERIAL.print ("Found address: ");
      DBGSERIAL.print (i, DEC);
      DBGSERIAL.print (" (0x");
      DBGSERIAL.print (i, HEX);
      DBGSERIAL.println (")");
      count++;
      delay (1);  // maybe unneeded?
    } // end of good response
  } // end of for loop
  DBGSERIAL.println ("Done.");
  DBGSERIAL.print ("Found ");
  DBGSERIAL.print (count, DEC);
  DBGSERIAL.println (" device(s).");
}

// Setup sensor pins to control the sensors that we choose later to control
void initSensorControl(int sensor_count) {
  for (idx=0; idx<sensor_count ; idx++) {
    sensorI2cAddr[idx] = idx + SENSOR_FIRST_I2C_ADDR; 
    digitalWrite(sensorXshutPins[idx], LOW);        // Setup for LOW ONLY and NEVER drive it HIGH or destroy the unit!
    pinMode(sensorXshutPins[idx], OUTPUT);          // This will start with all XSHUT lines low so the chip is shutdown
  }
}

// Initialize all sensors from a reset state and set their addresses to unique values
// This call can happen in runtime if sensor count command is received
void initSensors(int sensor_count) {
  int vl1Addr;
  for (idx=0; idx<sensor_count ; idx++) {
    pinMode(sensorXshutPins[idx], INPUT);           // Start this chip. releases active low line thus emulating release of an open collector
    delay(50);                                      // Let the chip startup
    VL53L0X[idx].init(); // init time of flight chip
    VL53L0X[idx].setTimeout(200);

    VL53L0X[idx].setAddress(sensorI2cAddr[idx]);
    delay(10);
    VL53L0X[idx].setMeasurementTimingBudget(21000);   // Optionally set 21ms timing budget for measurement, default is about 33ms
    VL53L0X[idx].startContinuous();
    
    vl1Addr = VL53L0X[idx].getAddress();
    DBGSERIAL.print("VL53L0X %d I2C address is currently: "); DBGSERIAL.println(vl1Addr);
  }
  return;
}

// Send out the readings for the sensors
void sendAllReadings()
{
    int sum = 0;
    // Print sensor readings and a rough sum at end in this format    <S:value1,value2,...:16_bit_sum>
    HOSTSERIAL.print("<"); HOSTSERIAL.print(CMD_QUERY_ALL_SENSORS); HOSTSERIAL.print(":"); 
    int x;
    for (x=0; x<sensorCount ; x++) {
      sum += sensorRangesInMm[x];
      HOSTSERIAL.print(sensorRangesInMm[x]); 
      if (x < (sensorCount-1)) {
        HOSTSERIAL.print(",");
      } else {
        HOSTSERIAL.print(":");
      }
    }
    // Poor mans simple 16-bit end of the running sum
    HOSTSERIAL.print((sum & 0xffff)); HOSTSERIAL.println(">");
}

// Re-initialize the number of sensors we will use from now on
void setSensorCount(int count)
{
  int sum = 0;
  if ((count > 0) && (count <= MAX_SENSORS)) {
    sensorCount = count;

    initSensorControl(MAX_SENSORS);
    delay(100);
    initSensors(sensorCount);

    // Reply with the OK indication which is value of 1 in our standard format
    sum = sensorCount;
    HOSTSERIAL.println("<"); HOSTSERIAL.print(CMD_SET_SENSOR_COUNT); HOSTSERIAL.print(":");
    HOSTSERIAL.print(sensorCount); HOSTSERIAL.print(":"); 
    HOSTSERIAL.print((sum & 0xffff)); HOSTSERIAL.println(">");
  } else {
    // Error condition is -1 for the count
    HOSTSERIAL.println("<"); HOSTSERIAL.print(CMD_SET_SENSOR_COUNT); HOSTSERIAL.print(-1); HOSTSERIAL.print(":-1>"); 
  }
}

void setup() {

  HOSTSERIAL.begin(HOST_SERIAL_BAUD);
  serialRxBuf.reserve(200);

  DBGSERIAL.begin(DBG_SERIAL_BAUD);

  // Initialize sensor data to dummy values
  for (idx=0; idx < SENSOR_DATA_REPLY_BYTES ; idx++) {
    sensorData[idx] = idx + 0x30;
  }

  // Setup I2C addresss and chip enable lines we will control for ALL that this module can support
  initSensorControl(MAX_SENSORS);

  // Setup our I2C master port for access to sensors
  Wire.begin();   // I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 100000);

#ifdef  USE_TEENSY_3.2
  // Setup slave I2C port so a host can access results from the sensors
  T3Wire1.begin(I2C_SLAVE, I2C_SLAVE_ADDRESS, I2C_PINS_29_30, I2C_PULLUP_EXT, 100000);         // Slave I2C bus
  T3Wire1.onReceive(receiveEvent);     // When we get data from a master go handle it here
  T3Wire1.onRequest(requestEvent);     // When we get request for I2C data handle it here
#endif

  i2cReceived = 0;
  i2cRequested = 0;

  pinMode(DispTxPin, OUTPUT);
  pinMode(ReadSensorPin, OUTPUT);

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

  // for each sensor to be used we release it from shutdown and program new address
  // Setup I2C address we will program for the sensors to be used  
  initSensors(sensorCount);

  delay(10);

#ifdef   LCD_DISPLAY
  // Clear display then be sure it is lit
  LCDSERIAL.write(12); 
  delay(50);
  LCDSERIAL.write(128);                // place cursor at first char of top line
#endif 

  loopIdx = 0;

  delay(100);     
}

void loop() {

  DBGSERIAL.println("Starting main loop.\n");

  scanI2cBus();

  #ifdef   LCD_DISPLAY
  // Initially write out to clear full display
  LCDSERIAL.print("                                ");
  LCDSERIAL.print(208);     // I attempt to shorten the anoying starup tune with 1/64th notes
  #endif

  while (1) {

  // Since serial is only read between loops anyway just do it here for easier architecture
  if (HOSTSERIAL.available()) {
    DBGSERIAL.println("Got char\n");
    while (HOSTSERIAL.available()) {
      // Guard against rampent characters far too big for a command with no end of line
      if (serialRxBuf.length() > 10) {
        serialRxBuf = "";    // nuke um, the host is a moron sending junk
      }   
      
      // get the new byte:
      char inChar = (char)HOSTSERIAL.read();
      if (SERIAL_ECHO > 0) {
        HOSTSERIAL.print(inChar);
      }
      serialRxBuf += inChar;   // add it to the inputString:
   
      // if the incoming character is a newline, set a flag
      // so the main loop can do something about it:
      if (inChar == '\r') {
        rxCommandReady = true;
      }
    }
  }

  if (rxCommandReady) {
    rxCommandReady = false;

    // HOSTSERIAL.print("Got cmd: ");HOSTSERIAL.println(serialRxBuf);
    int sum = 0;

    if (serialRxBuf.length() == 1) {
      HOSTSERIAL.println("<ok:1:1>");
    }

    if (serialRxBuf.substring(0,2) == CMD_AUTO_UPDATE_ON) {
      autoUpdate = 1;
    }
    if (serialRxBuf.substring(0,2) == CMD_AUTO_UPDATE_OFF) {
      autoUpdate = 0;
    }
    
    if (serialRxBuf.substring(0,2) == CMD_QUERY_ALL_SENSORS) {
      // Send out the readings we hold at this time
      sendAllReadings();
    }

    if (serialRxBuf.substring(0,2) == CMD_FIRMWARE_VERSION) {
      // Reply with firmware version    <FV:major_version,minor_version:16_bit_sum>
      HOSTSERIAL.print("<"); HOSTSERIAL.print(CMD_FIRMWARE_VERSION); HOSTSERIAL.print(":"); 
      HOSTSERIAL.print(FW_MAJOR_VERSION); 
      HOSTSERIAL.print(",");
      HOSTSERIAL.print(FW_MINOR_VERSION); 
      HOSTSERIAL.print(":");
      sum = FW_MAJOR_VERSION + FW_MINOR_VERSION;
      HOSTSERIAL.print((sum & 0xffff)); HOSTSERIAL.println(">");
    }

    if (serialRxBuf.substring(0,2) == CMD_SET_SENSOR_COUNT) {
      int count = 0;
      if (serialRxBuf.length() > 2) {   // Only do this if we have a character for the sensor count
        count = serialRxBuf.substring(2,3).c_str()[0] - 0x30;    // Poor mans single digit numeric conversion
      }
      setSensorCount(count);    // Reset unit for new number of sensors to be used
    }

    // No matter what we re-initialize our input buffer used to receive new commands
    serialRxBuf = "";    // clear input buffer for next command
  }

  loopIdx += 1;
  if (((loopIdx % 10) > 5)) {
    digitalWrite(LedPin, HIGH);
  } else {
    digitalWrite(LedPin, LOW);
  }
  if (loopIdx > 200000) loopIdx = 0;

  if (i2cReceived > 0) {
    DBGSERIAL.println("Received I2C\n");
    i2cReceived = 0;
  }
  if (i2cRequested > 0) {
    DBGSERIAL.println("Request on I2C\n");
    i2cRequested = 0;
  }
    
  duration = 0.0;

  //Serial.println("Starting main while loop.\n");

  // If little switch with a 'g' on it is to the 'g' we stay in this custom loop for testing assorted gizmos
  // Serial.println("loop start: \n");

  #ifdef   LCD_DISPLAY
  LCDSERIAL.write(17);                 // Turn backlight on (redundant)
  LCDSERIAL.write(128);                // place cursor at first char of top line
  LCDSERIAL.print("Read VL53L0X");
  LCDSERIAL.write(148);                // place cursor at first char of second row
  #endif

  // DBGSERIAL.print(sensorCount); DBGSERIAL.print(" sensors. Loop "); DBGSERIAL.println(loopIdx);

  if ((loopIdx % 10) == 1) {
      DBGSERIAL.print("Sensors: ");
  }
  for (idx=0; idx<sensorCount ; idx++) {
    digitalWrite(ReadSensorPin, HIGH);
    tofRange = VL53L0X[idx].readRangeSingleMillimeters();
    digitalWrite(ReadSensorPin, LOW
    );

    sensorData[0] = (tofRange & 0xff00) >> 8;
    sensorData[1] = tofRange & 0xff;

    sensorRangesInMm[idx] = tofRange;     // Save copy for query by host

    if ((loopIdx % 10) == 1) {
      DBGSERIAL.print(tofRange); DBGSERIAL.print(" "); 
    }
  }
  if ((loopIdx % 2) == 1) {
      DBGSERIAL.println(" ");
  }

  if ((autoUpdate == 1) && ((loopIdx % autoUpdateLoopCount) == 1)) {
      // Send out the readings we hold at this time
      sendAllReadings();
  }

  digitalWrite(ReadSensorPin, LOW);
  delay(5);    // serialEvent is only called between loop passes so keep this super short
  digitalWrite(ReadSensorPin, HIGH);
  delay(2);
  digitalWrite(ReadSensorPin, LOW);
  delay(5);    // serialEvent is only called between loop passes so keep this super short

  }
}
