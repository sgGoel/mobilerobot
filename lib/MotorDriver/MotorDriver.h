#ifndef MOTORDRIVER_H
#define MOTORDRIVER_H

#include <Arduino.h>

/**
 * @class MotorDriver
 * @brief Class to control a motor using PWM.
 */
class MotorDriver {
public:
    /**
     * @brief Constructor for MotorDriver class.
     * 
     * @param dirPin Digital pin number connected to the motor's direction control.
     * @param pwmPin Digital pin number connected to the motor's PWM control.
     * @param ledcChannel LEDC channel used for PWM.
     * @param pwmFreq Frequency of PWM signal (default 20kHz).
     * @param pwmBits Resolution of PWM signal (default 10 bits).
     */
    MotorDriver(int dirPin, int pwmPin, int ledcChannel, int pwmFreq = 20000, int pwmBits = 10);
    
    /**
     * @brief Sets up the motor driver by initializing pins and PWM.
     */
    void setup();

    /**
     * @brief Drives the motor by setting the PWM duty cycle.
     * 
     * @param dutyCycle A value between -1.0 (full reverse) and 1.0 (full forward).
     */
    void drive(double dutyCycle);

    /**
     * @brief Gets the current PWM duty cycle value.
     * 
     * @return double The duty cycle last set by the drive method.
     */
    double getCurrentDutyCycle();

private:
    int _dirPin;          ///< Pin for motor direction control
    int _pwmPin;          ///< Pin for motor PWM control
    int _ledcChannel;     ///< LEDC channel for PWM
    int _pwmFreq;         ///< Frequency of PWM signal
    int _pwmBits;         ///< Resolution of PWM signal
    double _currentDutyCycle; ///< Current duty cycle for the motor
};

#endif // MOTORDRIVER_H
