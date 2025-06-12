/*
   -- HTL-Car Steuerung Handy --
   
   This source code of graphical user interface 
   has been generated automatically by RemoteXY editor.
   To compile this code using RemoteXY library 3.1.13 or later version 
   download by link http://remotexy.com/en/library/
   To connect using RemoteXY mobile app by link http://remotexy.com/en/download/                   
     - for ANDROID 4.15.01 or later version;
     - for iOS 1.12.1 or later version;
    
   This source code is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.    
*/
// Include motor control classes
#include "CMotor.h"
#include "CAntrieb.h"

//////////////////////////////////////////////
//        RemoteXY include library          //
//////////////////////////////////////////////
// Enable debug logging to Serial at 115200
//#define REMOTEXY__DEBUGLOG    

// RemoteXY select connection mode and include library 
#define REMOTEXY_MODE__HARDSERIAL


// RemoteXY connection settings 
#define REMOTEXY_SERIAL Serial
#define REMOTEXY_SERIAL_SPEED 9600


#include <RemoteXY.h>

// RemoteXY GUI configuration  
#pragma pack(push, 1)  
uint8_t RemoteXY_CONF[] =   // 126 bytes
  { 255,13,0,3,0,119,0,19,0,0,0,0,16,1,200,84,1,1,5,0,
  9,94,33,8,8,0,24,74,84,42,29,4,0,2,24,64,65,99,99,101,
  108,101,114,111,109,101,116,101,114,0,74,43,7,109,11,0,12,12,64,72,
  84,76,45,67,97,114,32,74,97,110,32,80,97,118,108,111,118,105,99,0,
  2,14,30,44,22,0,36,24,31,31,79,78,0,79,70,70,0,74,16,56,
  38,4,0,2,37,64,69,105,110,32,117,110,100,32,65,117,115,115,99,104,
  97,108,116,101,110,0 };
  
// this structure defines all the variables and events of your control interface 
struct {

    // input variables
  float accelerometer_01_x;
  float accelerometer_01_y;
  float accelerometer_01_z;
  uint8_t switch_01; // =1 if switch ON and =0 if OFF

    // output variables
  uint8_t strings_03; // from 0 to 1
  uint8_t strings_04; // from 0 to 1
  uint8_t strings_01; // from 0 to 1

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0

} RemoteXY;   
#pragma pack(pop)
 
/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////


// ===============================================
// HARDWARE CONFIGURATION
// ===============================================

// Motor pin definitions - adjust according to your hardware
#define MOTOR_LEFT_PWM 5
#define MOTOR_LEFT_DIR 4
#define MOTOR_RIGHT_PWM 6
#define MOTOR_RIGHT_DIR 7

// Status LED pin (optional)
#define STATUS_LED 13

// ===============================================
// CONTROL PARAMETERS
// ===============================================

const float MAX_TILT_ANGLE = 45.0;    // Maximum tilt angle in degrees
const int MAX_SPEED = 200;             // Maximum speed (0-255)
const int MIN_SPEED = 50;              // Minimum speed to overcome motor friction
const float DEAD_ZONE = 0.1;          // Dead zone for accelerometer (prevent jitter)
const int LOOP_DELAY = 50;             // Main loop delay in ms (20Hz)

// Debug settings
const bool ENABLE_DETAILED_DEBUG = true;
const bool ENABLE_MOTOR_DEBUG = true;
const bool ENABLE_ACCEL_DEBUG = true;
const int DEBUG_INTERVAL = 500;        // Debug output every 500ms

// ===============================================
// GLOBAL VARIABLES
// ===============================================

// Create motor objects
CMotor motorLeft(MOTOR_LEFT_PWM, MOTOR_LEFT_DIR);
CMotor motorRight(MOTOR_RIGHT_PWM, MOTOR_RIGHT_DIR);

// Create drive system
CAntrieb antrieb(motorLeft, motorRight);

// Debug timing
unsigned long lastDebugTime = 0;
unsigned long lastLoopTime = 0;
int loopCounter = 0;

// Previous values for change detection
float prevGx = 0, prevGy = 0, prevGz = 0;
int prevSpeed = 0;
float prevCurveParam = 0;
bool prevSwitchState = false;
bool prevConnectState = false;

// ===============================================
// UTILITY FUNCTIONS
// ===============================================

void printSeparatorLine() {
  Serial.println(F("================================================"));
}

void printDebugHeader() {
  printSeparatorLine();
  Serial.println(F("HTL-CAR DEBUG OUTPUT"));
  Serial.print(F("Compiled: "));
  Serial.print(F(__DATE__));
  Serial.print(F(" "));
  Serial.println(F(__TIME__));
  printSeparatorLine();
}

void blinkStatusLED(int times, int delayMs = 200) {
  for (int i = 0; i < times; i++) {
    digitalWrite(STATUS_LED, HIGH);
    delay(delayMs);
    digitalWrite(STATUS_LED, LOW);
    delay(delayMs);
  }
}

// ===============================================
// MATHEMATICAL FUNCTIONS
// ===============================================

// Function to calculate angle from accelerometer data
float calculateTiltAngle(float gy, float g) {
  // Avoid division by zero
  if (abs(g) < 0.01) {
    Serial.println(F("WARNING: g_total too small, using default"));
    g = 9.8;
  }
  
  // Calculate angle: alpha = arcsin(gy/g)
  float ratio = gy / g;
  
  // Constrain ratio to valid range for arcsin
  if (ratio > 1.0 || ratio < -1.0) {
    Serial.print(F("WARNING: Invalid ratio for arcsin: "));
    Serial.println(ratio);
    ratio = constrain(ratio, -1.0, 1.0);
  }
  
  // Convert to degrees
  float angle = asin(ratio) * 180.0 / PI;
  
  if (ENABLE_DETAILED_DEBUG && (millis() - lastDebugTime > DEBUG_INTERVAL)) {
    Serial.print(F("Angle calc: gy="));
    Serial.print(gy, 3);
    Serial.print(F(" g="));
    Serial.print(g, 3);
    Serial.print(F(" ratio="));
    Serial.print(ratio, 3);
    Serial.print(F(" angle="));
    Serial.println(angle, 2);
  }
  
  return angle;
}

// Function to convert angle to curve parameter
float angleToCurveParameter(float angle) {
  // Constrain angle to maximum range
  angle = constrain(angle, -MAX_TILT_ANGLE, MAX_TILT_ANGLE);
  
  // Apply dead zone
  if (abs(angle) < DEAD_ZONE * MAX_TILT_ANGLE) {
    angle = 0;
  }
  
  // Map to curve parameter range (-4 to +4)
  float curveParam = map(angle * 100, -MAX_TILT_ANGLE * 100, MAX_TILT_ANGLE * 100, -400, 400) / 100.0;
  
  return curveParam;
}

// Function to calculate speed from forward/backward tilt
int calculateSpeed(float gx, float g) {
  // Calculate forward/backward angle
  if (abs(g) < 0.01) {
    g = 9.8;
  }
  
  float ratio = gx / g;
  ratio = constrain(ratio, -1.0, 1.0);
  float forwardAngle = asin(ratio) * 180.0 / PI;
  
  // Apply dead zone
  if (abs(forwardAngle) < DEAD_ZONE * MAX_TILT_ANGLE) {
    return 0;
  }
  
  // Map angle to speed
  int speed = map(abs(forwardAngle) * 100, DEAD_ZONE * MAX_TILT_ANGLE * 100, MAX_TILT_ANGLE * 100, MIN_SPEED, MAX_SPEED);
  
  return constrain(speed, 0, MAX_SPEED);
}

// Function to determine direction from forward/backward tilt
int calculateDirection(float gx, float g) {
  if (abs(g) < 0.01) {
    g = 9.8;
  }
  
  float ratio = gx / g;
  ratio = constrain(ratio, -1.0, 1.0);
  float forwardAngle = asin(ratio) * 180.0 / PI;
  
  if (abs(forwardAngle) < DEAD_ZONE * MAX_TILT_ANGLE) {
    return 0; // No movement
  }
  
  return (forwardAngle > 0) ? 1 : -1; // 1 for forward, -1 for backward
}

// ===============================================
// DEBUG FUNCTIONS
// ===============================================

void printAccelerometerData(float gx, float gy, float gz, float g_total) {
  if (!ENABLE_ACCEL_DEBUG) return;
  
  Serial.println(F("--- ACCELEROMETER DATA ---"));
  Serial.print(F("gx (Forward/Back): "));
  Serial.print(gx, 4);
  Serial.println(F(" m/s²"));
  
  Serial.print(F("gy (Left/Right):   "));
  Serial.print(gy, 4);
  Serial.println(F(" m/s²"));
  
  Serial.print(F("gz (Up/Down):      "));
  Serial.print(gz, 4);
  Serial.println(F(" m/s²"));
  
  Serial.print(F("g_total:           "));
  Serial.print(g_total, 4);
  Serial.println(F(" m/s²"));
  
  // Check for significant changes
  float deltaGx = abs(gx - prevGx);
  float deltaGy = abs(gy - prevGy);
  float deltaGz = abs(gz - prevGz);
  
  if (deltaGx > 0.5 || deltaGy > 0.5 || deltaGz > 0.5) {
    Serial.println(F("SIGNIFICANT ACCELERATION CHANGE DETECTED!"));
  }
}

void printControlData(float steerAngle, float curveParameter, int speed, int direction) {
  Serial.println(F("--- CONTROL CALCULATIONS ---"));
  Serial.print(F("Steering Angle:     "));
  Serial.print(steerAngle, 2);
  Serial.println(F("°"));
  
  Serial.print(F("Curve Parameter:    "));
  Serial.println(curveParameter, 3);
  
  Serial.print(F("Speed:              "));
  Serial.println(speed);
  
  Serial.print(F("Direction:          "));
  switch(direction) {
    case 1:  Serial.println(F("FORWARD")); break;
    case -1: Serial.println(F("BACKWARD")); break;
    case 0:  Serial.println(F("STOP")); break;
    default: Serial.println(F("UNKNOWN")); break;
  }
}

void printMotorData(int speed, float curveParameter) {
  if (!ENABLE_MOTOR_DEBUG) return;
  
  int pwmLeft, pwmRight;
  
  // Calculate what the PWM values would be (same logic as CAntrieb::berechnePWM)
  if (curveParameter >= 0) {
    pwmRight = speed;
    pwmLeft = (int)(speed * pow(2, -curveParameter));
  } else {
    pwmLeft = speed;
    pwmRight = (int)(speed * pow(2, curveParameter));
  }
  
  pwmLeft = constrain(pwmLeft, 0, 255);
  pwmRight = constrain(pwmRight, 0, 255);
  
  Serial.println(F("--- MOTOR CONTROL ---"));
  Serial.print(F("Left Motor PWM:     "));
  Serial.println(pwmLeft);
  
  Serial.print(F("Right Motor PWM:    "));
  Serial.println(pwmRight);
  
  Serial.print(F("Speed Difference:   "));
  Serial.println(abs(pwmLeft - pwmRight));
  
  if (curveParameter > 0.1) {
    Serial.println(F("→ LEFT TURN (Right motor faster)"));
  } else if (curveParameter < -0.1) {
    Serial.println(F("← RIGHT TURN (Left motor faster)"));
  } else {
    Serial.println(F("↑ STRAIGHT"));
  }
}

void printSystemStatus() {
  Serial.println(F("--- SYSTEM STATUS ---"));
  
  Serial.print(F("RemoteXY Connected: "));
  Serial.println(RemoteXY.connect_flag ? F("YES") : F("NO"));
  
  Serial.print(F("System Switch:      "));
  Serial.println(RemoteXY.switch_01 ? F("ON") : F("OFF"));
  
  Serial.print(F("Status LED (01):    "));
  Serial.println(RemoteXY.strings_01 ? F("ON") : F("OFF"));
  
  Serial.print(F("Speed LED (03):     "));
  Serial.println(RemoteXY.strings_03 ? F("ON") : F("OFF"));
  
  Serial.print(F("Curve LED (04):     "));
  Serial.println(RemoteXY.strings_04 ? F("ON") : F("OFF"));
  
  Serial.print(F("Loop Frequency:     "));
  Serial.print(1000.0 / (millis() - lastLoopTime));
  Serial.println(F(" Hz"));
  
  Serial.print(F("Free RAM:           "));
  Serial.print(freeRam());
  Serial.println(F(" bytes"));
}

// Function to calculate free RAM
int freeRam() {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

void printChangeDetection() {
  bool hasChanges = false;
  
  if (RemoteXY.switch_01 != prevSwitchState) {
    Serial.print(F("SWITCH CHANGED: "));
    Serial.println(RemoteXY.switch_01 ? F("OFF→ON") : F("ON→OFF"));
    hasChanges = true;
  }
  
  if (RemoteXY.connect_flag != prevConnectState) {
    Serial.print(F("CONNECTION CHANGED: "));
    Serial.println(RemoteXY.connect_flag ? F("DISCONNECTED→CONNECTED") : F("CONNECTED→DISCONNECTED"));
    hasChanges = true;
    
    // Blink LED on connection change
    blinkStatusLED(RemoteXY.connect_flag ? 3 : 1, 100);
  }
  
  if (hasChanges) {
    Serial.println();
  }
}

// ===============================================
// MAIN SETUP FUNCTION
// ===============================================

void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  delay(1000); // Give serial time to initialize
  
  printDebugHeader();
  
  // Initialize RemoteXY
  Serial.println(F("Initializing RemoteXY..."));
  RemoteXY_Init();
  Serial.println(F("RemoteXY initialized successfully"));
  
  // Initialize status LED
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);
  
  // Initialize motors
  Serial.println(F("Initializing motors..."));
  motorLeft.init();
  motorRight.init();
  Serial.println(F("Motors initialized"));
  
  // Initialize output strings
  RemoteXY.strings_01 = 0; // Status indicator
  RemoteXY.strings_03 = 0; // Speed indicator
  RemoteXY.strings_04 = 0; // Curve indicator
  
  // Startup blink sequence
  Serial.println(F("System ready - performing startup sequence"));
  blinkStatusLED(5, 150);
  
  Serial.println(F("=== STARTING MAIN LOOP ==="));
  Serial.println();
  
  lastDebugTime = millis();
  lastLoopTime = millis();
}

// ===============================================
// MAIN LOOP FUNCTION
// ===============================================

void loop() {
  unsigned long loopStartTime = millis();
  loopCounter++;
  
  // Handle RemoteXY communication
  RemoteXY_Handler();
  
  // Check for state changes
  printChangeDetection();
  
  // Update previous states
  prevSwitchState = RemoteXY.switch_01;
  prevConnectState = RemoteXY.connect_flag;
  
  // Check if system is enabled via switch
  if (RemoteXY.switch_01 == 1 && RemoteXY.connect_flag == 1) {
    // System is ON and connected
    RemoteXY.strings_01 = 1; // Status ON
    digitalWrite(STATUS_LED, HIGH);
    
    // Get accelerometer values
    float gx = RemoteXY.accelerometer_01_x;  // Forward/Backward
    float gy = RemoteXY.accelerometer_01_y;  // Left/Right
    float gz = RemoteXY.accelerometer_01_z;  // Up/Down (gravity reference)
    
    // Calculate total gravity (should be ~9.8, but we normalize)
    float g_total = sqrt(gx*gx + gy*gy + gz*gz);
    
    // Avoid division by zero
    if (g_total < 0.1) {
      Serial.println(F("WARNING: g_total too low, using default"));
      g_total = 9.8; // Default gravity value
    }
    
    // Calculate steering angle from left/right tilt
    float steerAngle = calculateTiltAngle(gy, g_total);
    float curveParameter = angleToCurveParameter(steerAngle);
    
    // Calculate speed and direction from forward/backward tilt
    int speed = calculateSpeed(gx, g_total);
    int direction = calculateDirection(gx, g_total);
    
    // Update display indicators
    RemoteXY.strings_03 = map(speed, 0, MAX_SPEED, 0, 1); // Speed indicator
    RemoteXY.strings_04 = (abs(curveParameter) > 0.1) ? 1 : 0; // Curve indicator
    
    // Control the car
    if (speed > 0) {
      antrieb.kurve(curveParameter, speed, direction);
    } else {
      antrieb.stop();
    }
    
    // Debug output (time-controlled)
    if (millis() - lastDebugTime >= DEBUG_INTERVAL) {
      printAccelerometerData(gx, gy, gz, g_total);
      printControlData(steerAngle, curveParameter, speed, direction);
      printMotorData(speed, curveParameter);
      printSystemStatus();
      printSeparatorLine();
      Serial.println();
      
      lastDebugTime = millis();
    }
    
    // Store previous values
    prevGx = gx;
    prevGy = gy; 
    prevGz = gz;
    prevSpeed = speed;
    prevCurveParam = curveParameter;
    
  } else {
    // System is OFF or disconnected
    RemoteXY.strings_01 = 0; // Status OFF
    RemoteXY.strings_03 = 0; // Speed OFF
    RemoteXY.strings_04 = 0; // Curve OFF
    
    digitalWrite(STATUS_LED, LOW);
    
    // Stop all motors
    antrieb.stop();
    
    // Debug output for off state (less frequent)
    if (millis() - lastDebugTime >= DEBUG_INTERVAL * 2) {
      Serial.println(F("System OFF or disconnected"));
      printSystemStatus();
      lastDebugTime = millis();
    }
  }
  
  // Loop timing control
  lastLoopTime = loopStartTime;
  
  // Maintain consistent loop timing
  unsigned long loopDuration = millis() - loopStartTime;
  if (loopDuration < LOOP_DELAY) {
    RemoteXY_delay(LOOP_DELAY - loopDuration);
  } else if (loopDuration > LOOP_DELAY * 2) {
    Serial.print(F("WARNING: Loop took too long: "));
    Serial.print(loopDuration);
    Serial.println(F("ms"));
  }
}