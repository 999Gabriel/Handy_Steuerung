#ifndef CANTRIEB_H
#define CANTRIEB_H

#include "CMotor.h"

class CAntrieb {
  private:
    CMotor &motorLi;  // Left motor reference
    CMotor &motorRe;  // Right motor reference
    
    // Calculate PWM values based on curve parameter
    void berechnePWM(float kurvenparameter, int geschwindigkeit, int &pwmLi, int &pwmRe);
    
  public:
    // Constructor with motor references
    CAntrieb(CMotor &leftMotor, CMotor &rightMotor);
    
    // Move in a curve with given parameters
    // kurvenparameter: Curve sharpness (negative for right curve, positive for left curve)
    // geschwindigkeit: Speed value (0-255)
    // richtung: 1 for forward, -1 for backward
    void kurve(float kurvenparameter, int geschwindigkeit, int richtung);
    
    // Move straight forward
    void vorwaerts(int geschwindigkeit);
    
    // Move straight backward
    void rueckwaerts(int geschwindigkeit);
    
    // Turn in place (one motor forward, one backward)
    // richtung: 1 for clockwise, -1 for counter-clockwise
    void drehung(int geschwindigkeit, int richtung);
    
    // Stop both motors
    void stop();
};

#endif // CANTRIEB_H