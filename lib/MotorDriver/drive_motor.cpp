#include "MotorDriver.h"
#define DIR_PIN 2
#define PWM_PIN 4
#define LEDC_CHANNEL 0
// Initialize the motor driver object with DIR and PWM pin numbers and LEDC channel
MotorDriver motor(DIR_PIN, PWM_PIN, LEDC_CHANNEL);

void setup() {
    // Setup the motor driver
    motor.setup();
}

void loop() {
    // Drive the motor at 50% duty cycle (forward)
    motor.drive(0.5);

    // Retrieve the current duty cycle
    double dutyCycle = motor.getCurrentDutyCycle();

    // The dutyCycle variable can be used for monitoring or further control
    delay(1000);
}
