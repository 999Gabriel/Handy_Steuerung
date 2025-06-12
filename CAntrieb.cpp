#include "CAntrieb.h"
#include <Arduino.h>
#include <math.h>  // For pow() function

CAntrieb::CAntrieb(CMotor &leftMotor, CMotor &rightMotor)
  : motorLi(leftMotor), motorRe(rightMotor) {
}

void CAntrieb::berechnePWM(float kurvenparameter, int geschwindigkeit, int &pwmLi, int &pwmRe) {
  // Based on: pwmRe = pwmLi * 2^kurvenparameter
  // For left curve (positive parameter): Right motor is faster
  // For right curve (negative parameter): Left motor is faster
  
  if (kurvenparameter >= 0) {
    // Left curve - right motor at full speed
    pwmRe = geschwindigkeit;
    pwmLi = (int)(geschwindigkeit * pow(2, -kurvenparameter));
  } else {
    // Right curve - left motor at full speed
    pwmLi = geschwindigkeit;
    pwmRe = (int)(geschwindigkeit * pow(2, kurvenparameter));
  }
  
  // Constrain PWM values
  pwmLi = constrain(pwmLi, 0, 255);
  pwmRe = constrain(pwmRe, 0, 255);
}

void CAntrieb::kurve(float kurvenparameter, int geschwindigkeit, int richtung) {
  int pwmLi, pwmRe;
  
  berechnePWM(kurvenparameter, geschwindigkeit, pwmLi, pwmRe);
  
  motorLi.bewegen(pwmLi, richtung);
  motorRe.bewegen(pwmRe, richtung);
}

void CAntrieb::vorwaerts(int geschwindigkeit) {
  kurve(0, geschwindigkeit, 1);  // No curve, forward direction
}

void CAntrieb::rueckwaerts(int geschwindigkeit) {
  kurve(0, geschwindigkeit, -1);  // No curve, backward direction
}

void CAntrieb::drehung(int geschwindigkeit, int richtung) {
  if (richtung > 0) {
    // Clockwise - left motor forward, right motor backward
    motorLi.bewegen(geschwindigkeit, 1);
    motorRe.bewegen(geschwindigkeit, -1);
  } else {
    // Counter-clockwise - left motor backward, right motor forward
    motorLi.bewegen(geschwindigkeit, -1);
    motorRe.bewegen(geschwindigkeit, 1);
  }
}

void CAntrieb::stop() {
  motorLi.stop();
  motorRe.stop();
}