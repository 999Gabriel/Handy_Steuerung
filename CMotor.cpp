#include "CMotor.h"
#include <Arduino.h>

CMotor::CMotor(int pwmPin, int dirPin) {
  pinPWM = pwmPin;
  pinDIR = dirPin;
}

void CMotor::init() {
  pinMode(pinPWM, OUTPUT);
  pinMode(pinDIR, OUTPUT);
  
  // Initialize with motor stopped
  stop();
}

void CMotor::bewegen(int geschwindigkeit, int richtung) {
  // Constrain speed value between 0 and 255
  geschwindigkeit = constrain(geschwindigkeit, 0, 255);
  
  if (richtung > 0) {
    // Forward
    digitalWrite(pinDIR, HIGH);
    analogWrite(pinPWM, geschwindigkeit);
  } 
  else if (richtung < 0) {
    // Backward
    digitalWrite(pinDIR, LOW);
    analogWrite(pinPWM, geschwindigkeit);
  } 
  else {
    // Stop
    stop();
  }
}

void CMotor::stop() {
  analogWrite(pinPWM, 0);
  // Direction pin state doesn't matter when PWM is 0
}