#ifndef CMOTOR_H
#define CMOTOR_H

class CMotor {
  private:
    int pinPWM;    // PWM speed control pin
    int pinDIR;    // Direction control pin
    
  public:
    // Constructor with pin configuration
    CMotor(int pwmPin, int dirPin);
    
    // Initialize pins
    void init();
    
    // Move motor with given speed and direction
    // speed: 0-255
    // direction: 1 for forward, -1 for backward, 0 for stop
    void bewegen(int geschwindigkeit, int richtung);
    
    // Stop the motor
    void stop();
    
    // Getter for pins (if needed)
    int getPWMPin() const { return pinPWM; }
    int getDirPin() const { return pinDIR; }
};

#endif // CMOTOR_H