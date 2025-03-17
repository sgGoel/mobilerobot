#include "MotorDriver.h"

MotorDriver::MotorDriver(int dirPin, int pwmPin, int ledcChannel, int pwmFreq, int pwmBits)
: _dirPin(dirPin), _pwmPin(pwmPin), _ledcChannel(ledcChannel), _pwmFreq(pwmFreq), _pwmBits(pwmBits), _currentDutyCycle(0) {
    // Constructor initializes member variables with provided values
}

void MotorDriver::setup() {
    // Configure motor control pins as outputs
    pinMode(_dirPin, OUTPUT);
    pinMode(_pwmPin, OUTPUT);

    // Setup PWM on the specified channel, frequency, and resolution
    ledcSetup(_ledcChannel, _pwmFreq, _pwmBits);

    // Attach the PWM pin to the LEDC channel
    ledcAttachPin(_pwmPin, _ledcChannel);
}

void MotorDriver::drive(double dutyCycle) {
    // Constrain the duty cycle to valid range (-1.0 to 1.0)
    dutyCycle = constrain(dutyCycle, -0.999, 0.999);

    // Set the motor direction based on the sign of duty cycle
    digitalWrite(_dirPin, dutyCycle > 0);

    // Calculate and set the PWM duty cycle
    _currentDutyCycle = abs(dutyCycle) * (pow(2, _pwmBits) - 1);
    ledcWrite(_ledcChannel, _currentDutyCycle);
}

double MotorDriver::getCurrentDutyCycle() {
    // Return the current duty cycle value
    return _currentDutyCycle;
}
