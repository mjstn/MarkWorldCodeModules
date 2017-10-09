
//
// Steel Ball Positional Table
//
// Control a flat table that a steel ball rolls around on on the top.
// 
// M. Johnston created 20170101
// 20170102  1st version that uses PD control and sort of works but needs tuning and perhaps I term
// 20170103  X and Y closed loop with setpoints from a couple pots all works and is fairly smooth control
// 20170106  Add automatic moving of setpoints to make ball roll in a square pattern
// 20170706  Convert to run on the Nano OR my own Esp32 board if USE_ESP32 is defined
// 20170722  Runs on new nicer looking Rev2 hardware still on Arduino Nano (Esp32 cannot do x-y panel yet)
//

#undef  USE_ESP32             // If CPU is Mark-Toys Esp32 board define this (still in dev here)
#define SERVO_TEST     0      // Set to 0 for operation.  Set to 1 to just test servos as sainity check
#define DRIVE_SERVOS   1      // Set non-zero for operation.  If zero we do NOT actually drive the servos
#define  AUTO_MOVE_BALL        // Automatic movement of the steel ball
#define PID_LOOP_DELAY_MS  20 // Delay of the PID loop for each pass

#ifndef  USE_ESP32
#include <Wire.h>
#include <Servo.h>
#endif

#include <TouchScreen.h>

// Define only one of these to control our custom test to be run

int hw_revision = 2;          // HW Rev 1 is 8x6 panel on plywood.  Rev 2 is plastic box with 4x3.5" panel

int debug_level = 0;                // Set to non-zero for assorted debug modes.   2 = print a lot of stuff

#ifdef USE_ESP32

// Hardware pin numbers used for this set of hardware
const int LedPin    = 2;              // The Stat LED
const int xServoPin = 21;             // X Servo PWM output pin
const int yServoPin = 19;             // Y Servo PWM output pin
const int DispTxPin = 13;             // Display Serial Transmit pin

TouchScreen   rPanel(32, 33, 25, 26);   // X is GPO32/ADC1_4 and GPO33/ADC1_5.  Y is GPIO25/ADC2_8 and GPIO26/ADC2_9

#define   xPosition    4              // X Position reading ADC2_0 on QEA_0
#define   xOffsetPot  34              // X Offset Pot on    ADC1_6 on QEB_0
#define   yPosition   27              // Y Position reading ADC2_7 on QEA_1
#define   yOffsetPot  35              // Y Offset Pot on    ADC1_7 on QEB_1
#define   AXIS_OFFSET_MIDPOINT   750  // Sets where offset control midpoint will be in ADC reading
#define   AXIS_OFFSET_DIVISOR    3    // sensitivity of offset control, higher = less effect of control
#define   SERVO_DEG_TO_DUTY      1    // Converts degrees of correction to Servo PWM percent setting
#define   SERVO_X_DUTY_OFFSET   50    // Sets the center point for mid-servo travel for X
#define   SERVO_Y_DUTY_OFFSET   50    // Sets the center point for mid-servo travel for X

int  xServoLevelDegs   = 64;       // Point where servo is level     
int  xMinServo         = 40;       // hard limit for our hardware setup
int  xMaxServo         = 60;      // hard limit for our hardware setup
int  yServoLevelDegs   = 83;       // Point where servo is level     
int  yMinServo         = 40;       // hard limit for our hardware setup  
int  yMaxServo         = 60;      // hard limit for our hardware setup

#else     // Arduino Nano

// Hardware pin numbers used for this set of hardware
const int LedPin    = 0;
const int xServoPin = 10;             // X Servo PWM output pin
const int yServoPin = 11;             // Y Servo PWM output pin
const int DispTxPin = 13;             // Display Serial Transmit pin

TouchScreen   rPanel(3, 1, 0, 2);       // X is A3 and A1.    Y  is   A0 and A2

#define   xOffsetPot   6              // X Offset Pot ADC input pin
#define   yOffsetPot   7              // Y Offset Pot ADC input pin
#define   AXIS_OFFSET_MIDPOINT   500  // Sets where offset control midpoint will be in ADC reading
#define   AXIS_OFFSET_DIVISOR    5    // sensitivity of offset control, higher = less effect of control
#define   SERVO_DEG_TO_DUTY      2    // Converts degrees of correction to Servo PWM percent setting
#define   SERVO_X_DUTY_OFFSET   105   // Sets the center point for mid-servo travel for X
#define   SERVO_Y_DUTY_OFFSET    85    // Sets the center point for mid-servo travel for X

int  xServoLevelDegs   = 0;        // Point in degrees where servo is level     
int  xMinServo         = 80;       // hard limit for our hardware setup
int  xMaxServo         = 130;      // hard limit for our hardware setup
int  yServoLevelDegs   = 0;        // Point in degrees where servo is level     
int  yMinServo         = 70;       // hard limit for our hardware setup  
int  yMaxServo         = 105;      // hard limit for our hardware setup

#endif

int  xAxisOffset;                  // Offset for X axis servo
int  xLastServoSetting = -1;
int  xServoDegrees     = 64;  

// Defines polarity for error correction that varies per hardware servo mechanical config
int  x_err_polarity    = 1;       
int  y_err_polarity    = 1; 

int  yAxisOffset;                  // Offset for X axis servo
int  yLastServoSetting = -1;
int  yServoDegrees     = 83;  


// The touch panel in use is defined by hardware version. Early proto was big 8x6 and later 4.5x3.5 inch
int rPanelCoords[2];
int rPanelXMin = 100;
int rPanelXMax = 900;
int rPanelXMid = 500;
int rPanelYMin = 100;
int rPanelYMax = 900;
int rPanelYMid = 500;


// Define serial ports for a second serial port driven by software bit-bang uart
// #include <SoftwareSerial.h>
// SoftwareSerial displaySerial = SoftwareSerial(255, DispTxPin);   // Instantiate before setup loop
//
// Setup code
// pinMode(DispTxPin, OUTPUT);
// digitalWrite(DispTxPin, HIGH);
//
// Main code for 2x16 line serial LCD display
// displaySerial.write(12);        // some sort of display init or clear
// displaySerial.begin(9600);      // set baud rate
// displaySerial.write(17);        // Turn backlight on (redundant)
// delay(50);                      // I found a delay may be required after the control sort of commands
// displaySerial.write(128);       // place cursor at first char of top line
// displaySerial.print("Hello");   // Write characters which advances cursor as well
// displaySerial.write(161);       // place cursor near end of second row


int  pwmDutyCycle      = 0;        // A 0 to 255 value for duty cycle
int  xServoSetting     = 0;        // The current X Servo setting that goes to driver
int  yServoSetting     = 0;        // The current Y Servo setting that goes to driver

//     Control parameters 

//     X:  0.05, 0.0, -0.4  works well with minimal oscillation
int    loopDelay        = PID_LOOP_DELAY_MS;
int    loopIdx          = 0;
int    xBallSetpoint    = 470;       // X Setpoint for table control in units of X panel reading
int    x_Error          = 0;         // Current position error
int    x_LastErr        = 0;
double xP_Gain          = 0.04;      // proportional control gain
double xP_Correction    = 0;         // Correction from proportional control
int    xI_Sum           = 0;         // Integration error
int    xI_Cap           = 80;        // Integration sum cap
int    xI_Close         = 20;        // When really close to target, integration sum is zeroed
int    xI_Thresh        = 200;       // Only use integration term if error is below this level
double xI_Gain          = 0;  // .02;
double xI_Correction    = 0;
int    xD_Error         = 0;         // delta position error from last pass
double xD_Gain          = -0.6;      // derivitive control gain
double xD_Correction    = 0;         // Correction from derivitive control
double xT_Correction    = 0;         // Total error correction

int    yBallSetpoint    = 470;       // X Setpoint for table control in units of X panel reading
int    y_Error          = 0;         // Current position error   
int    y_LastErr        = 0;
double yP_Gain          = 0.04;      // proportional control gain
double yP_Correction    = 0;         // Correction from proportional control
int    yI_Sum           = 0;         // Integration error
int    yI_Cap           = 80;        // Integration sum cap
int    yI_Close         = 20;        // When really close to target, integration sum is zeroed
int    yI_Thresh        = 200;       // Only use integration term if error is below this level
double yI_Gain          = 0;  // .02;
double yI_Correction    = 0;
int    yD_Error         = 0;         // delta position error from last pass
double yD_Gain          = -0.6;      // derivitive control gain
double yD_Correction    = 0;         // Correction from derivitive control
double yT_Correction    = 0;         // Total error correction

// Ball automation is done by slowly moving the ball setpoint over time
double xMoveOffset      = -90;         // This offset controls automatic movement of the ball
double yMoveOffset      = -90;         // y axis automatic movement variable
int    moveSpeed        = 1;         // Controls speed of automatic ball movements  
int    movePhase        = 0;         // Controls type of movement direction       
int    moveIndex        = 0;
int    moveInc          = 1;
int    moveMin          = -180;
int    moveMax          = 180;

#ifdef USE_ESP32
// PWM pins for servo control or motor control on mark-toys Esp32 dev board
#define PWM_X_GPIO           21
#define PWM_X_LEDC_CHANNEL    1
#define PWM_Y_GPIO           19
#define PWM_Y_LEDC_CHANNEL    2
#define PWM_FREQUENCY        50     // In Hz so 50 is 20ms period
#define PWM_RESOLUTION        8
#else
Servo xAxisServo;                  // create x table axis servo object
Servo yAxisServo;                  // create y table axis servo object
#endif

void setup() {

  pinMode(LedPin, OUTPUT);
  digitalWrite(LedPin, HIGH);
  
  // pinMode(testModeSelect, INPUT_PULLUP);        // Ground for GPS mode or open sonar or servo, ground for gps mode
  
  pinMode(xOffsetPot, INPUT);    // analog input for x table offset PWM setting
  pinMode(yOffsetPot, INPUT);    // analog input for y table offset PWM setting

  rPanelXMid = rPanelXMin + ((rPanelXMax-rPanelXMin)/2);
  rPanelYMid = rPanelXMin + ((rPanelYMax-rPanelYMin)/2);

  #ifdef USE_ESP32
  ledcSetup(PWM_X_LEDC_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(xServoPin, PWM_X_LEDC_CHANNEL);
  ledcSetup(PWM_Y_LEDC_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(yServoPin, PWM_Y_LEDC_CHANNEL);
  #else
  xAxisServo.attach(xServoPin);   // Attach to x servo pwm pin
  yAxisServo.attach(yServoPin);   // Attach to y servo pwm pin
  #endif
  
  loopIdx = 0;
    
  pinMode(DispTxPin, OUTPUT);
  digitalWrite(DispTxPin, HIGH);

  Serial.begin(38400);
  Serial.println("Steel Ball TouchPanel.");

  #ifndef USE_ESP32
  Wire.begin();
  #endif

  delay(100);     
}

void loop() {
  uint16_t rPanel_x, rPanel_y;
  uint8_t rPanel_z;

  if (debug_level > 1) {  Serial.println("Starting loop."); }

  // Account for different versions of servo and table hardware configurations
  x_err_polarity = 1;
  y_err_polarity = 1;
  if (hw_revision == 2) {
    x_err_polarity = -1;
    y_err_polarity = -1;
  }
  
  while (1) {

  // If little switch with a 'g' on it is to the 'g' we stay in this custom loop for testing assorted gizmos
  while (1) {

    loopIdx += 1;

    digitalWrite(LedPin, ((loopIdx & 0x7C) == 0x04) ? HIGH : LOW);   // Brief blip of light at low duty cycle

    #ifdef AUTO_MOVE_BALL
    
  if ((loopIdx % moveSpeed) == 0) {    // Controls speed of ball auto-movement
    
    moveIndex += 1;
    switch(movePhase) {
      case 0:
          xMoveOffset += moveInc;
          if (moveIndex > moveMax) {
              movePhase = 1;
              moveIndex = 0;
          }
          break;
     case 1:
          yMoveOffset += moveInc;
          if (moveIndex > moveMax) {
              movePhase = 2;
              moveIndex = 0;
          }
          break;
     case 2:
          xMoveOffset -= moveInc;
          if (moveIndex > moveMax) {
              movePhase = 3;
              moveIndex = 0;
          }
          break;
     case 3:
          yMoveOffset -= moveInc;
          if (moveIndex > moveMax) {
              movePhase = 0;
              moveIndex = 0;
          }
          break;
      default:
          break;
    }
  }

  #endif   // End of mechanism to move setpoint for the ball on it's own

    
    // Direct Arduino Resistive Touch Panel library code
    rPanel.read(&rPanelCoords[0]);

    rPanel_x = rPanelCoords[0];
    rPanel_y = rPanelCoords[1];

    // Servo control is then based on current settings
    xAxisOffset = (analogRead(xOffsetPot) - AXIS_OFFSET_MIDPOINT)/AXIS_OFFSET_DIVISOR;
    xBallSetpoint = xAxisOffset + xMoveOffset + (((rPanelXMax - rPanelXMin)/2) + rPanelXMin);  
                   // old code? map(xAxisOffset, 0, 1023, rPanelXMin, rPanelXMax);   // Map the 0-1023 A/D input to 0-180 degrees
    // Do control logic first by getting updating last error and calculating current error
    x_LastErr = x_Error;
    x_Error = (rPanel_x - xBallSetpoint) * x_err_polarity;
    // Compute the PID corrections
    xP_Correction = (double)x_Error * (double)xP_Gain;        //  correction from proportional control
    if ((abs(x_Error) < xI_Close) || (abs(x_Error) < xI_Thresh)){
      xI_Sum = 0; // When super small error or far away, zero the integration sum
    }  else {
      if (abs(xI_Sum) < xI_Cap) {
        xI_Sum += x_Error;
      }
    }
    xI_Correction = (double)xI_Sum * xI_Gain;
    xD_Error = (double)x_LastErr - (double)x_Error;
    xD_Correction = xD_Error * (double)xD_Gain;               //  correction from derivative control
    xT_Correction = xP_Correction + xI_Correction + xD_Correction;   //  Total correction from PD loop logic

    yAxisOffset = (analogRead(yOffsetPot) - AXIS_OFFSET_MIDPOINT)/AXIS_OFFSET_DIVISOR;
    yBallSetpoint = yAxisOffset + yMoveOffset + (((rPanelYMax - rPanelYMin)/2) + rPanelYMin);  
                   // old code? map(yAxisOffset, 0, 1023, rPanelYMin, rPanelYMax);   // Map the 0-1023 A/D input to 0-180 degrees
    // Do control logic first by getting updating last error and calculating current error
    y_LastErr = y_Error;
    y_Error = (rPanel_y - yBallSetpoint) * y_err_polarity;
    if ((abs(y_Error) < yI_Close) || (abs(y_Error) < yI_Thresh)){
      yI_Sum = 0; // When super small error or far away, zero the integration sum
    }  else {
      if (abs(yI_Sum) < yI_Cap) {
        yI_Sum += y_Error;
      }
    }
    // Compute the PID corrections
    yP_Correction = (double)y_Error * (double)yP_Gain;        //  correction from proportional control
    yI_Correction = (double)yI_Sum * yI_Gain;
    yD_Error = (double)y_LastErr - (double)y_Error;
    yD_Correction = yD_Error * (double)yD_Gain;               //  correction from derivative control
    yT_Correction = yP_Correction + yI_Correction + yD_Correction;  //  Total correction from PD loop logic

    // Now apply correction if we are not very close to the setpoint (deadband uses this logic)
    //if ((x_Error > 2) || (x_Error < -2)) {
      xServoDegrees = xServoLevelDegs + (int)xT_Correction;
    //}
    // Now apply correction if we are not very close to the setpoint (deadband uses this logic)
    //if ((y_Error > 2) || (y_Error < -2)) {
      yServoDegrees = yServoLevelDegs + (int)yT_Correction;
    //}

    xServoSetting = (xServoDegrees / SERVO_DEG_TO_DUTY) + SERVO_X_DUTY_OFFSET;  
    yServoSetting = (yServoDegrees / SERVO_DEG_TO_DUTY) + SERVO_Y_DUTY_OFFSET; 

     if (SERVO_TEST) {
      xServoSetting = (xAxisOffset) + SERVO_X_DUTY_OFFSET;
      yServoSetting = (yAxisOffset) + SERVO_Y_DUTY_OFFSET;
    }

    if (debug_level > 1) {
      Serial.print(" X Offset "); Serial.print(xAxisOffset); Serial.print(" Pos "); Serial.print(rPanel_x); 
      Serial.print(" xSp "); Serial.print(xBallSetpoint); Serial.print(" xT "); Serial.print(xT_Correction); 
      Serial.print(" xDeg "); Serial.print(xServoDegrees); Serial.print(" xSet "); Serial.print(xServoSetting);
      Serial.print(" Y Offset "); Serial.print(yAxisOffset); ; Serial.print(" Pos "); Serial.print(rPanel_y); 
      Serial.print(" ySp "); Serial.print(yBallSetpoint); Serial.print(" yT "); Serial.print(yT_Correction); 
      Serial.print(" yDeg "); Serial.print(yServoDegrees); Serial.print(" ySet "); Serial.print(yServoSetting);
      Serial.println("");
    }

    // Now drive the servos with currently computed settings in degrees and cap off for hardware limits
    if ((DRIVE_SERVOS == 1) && (xLastServoSetting != xServoSetting)) {
      if (xServoSetting < xMinServo) { xServoSetting = xMinServo; }
      if (xServoSetting > xMaxServo) { xServoSetting = xMaxServo; }

      #ifdef USE_ESP32
      pwmDutyCycle =  xServoSetting;  // Adjust for Esp32 PWM hardware
      ledcWrite(PWM_X_LEDC_CHANNEL, pwmDutyCycle);
      #else
      xAxisServo.write(xServoSetting);      // Update servo hardware
      #endif
      xLastServoSetting = xServoSetting;
    }
    if ((DRIVE_SERVOS == 1) && (yLastServoSetting != yServoSetting)) {
      if (yServoSetting < yMinServo) { yServoSetting = yMinServo; }
      if (yServoSetting > yMaxServo) { yServoSetting = yMaxServo; }
      #ifdef USE_ESP32
      pwmDutyCycle = yServoSetting;   // Adjust for Esp32 PWM hardware
      ledcWrite(PWM_Y_LEDC_CHANNEL, pwmDutyCycle);
      #else
      yAxisServo.write(yServoSetting);      // Update servo hardware
      #endif
      yLastServoSetting = yServoSetting;
    }

    if (loopDelay > 200) {
      Serial.print("pX: ");  Serial.print(rPanel_x); Serial.print(" sX: ");Serial.print(xBallSetpoint);
      Serial.print(" eX: "); Serial.print(x_Error); Serial.print(" deX: "); Serial.print(xD_Error); Serial.print(" tcX: "); Serial.print(xT_Correction);
      Serial.print(" pcX: "); Serial.print(xP_Correction); Serial.print(" dcX: "); Serial.print(xD_Correction);
      Serial.print(" :  X ofst "); Serial.print(xAxisOffset);Serial.print(" xDeg "); Serial.print(xServoDegrees); Serial.println(".");

      Serial.print("pY: ");  Serial.print(rPanel_y); Serial.print(" sY: ");Serial.print(yBallSetpoint);
      Serial.print(" eY: "); Serial.print(y_Error); Serial.print(" deY: "); Serial.print(yD_Error); Serial.print(" tcY: "); Serial.print(yT_Correction);
      Serial.print(" pcY: "); Serial.print(yP_Correction); Serial.print(" dcY: "); Serial.print(yD_Correction);
      Serial.print(" :  Y ofst "); Serial.print(yAxisOffset);Serial.print(" yDeg "); Serial.print(yServoDegrees); Serial.println(".");
    }
    
    delay(loopDelay);
    continue;
  }
  

    delay(1000);
  
  }
}
