#include <Arduino.h>
#include "robot_pinout.h"
#include "MotorDriver.h"
#include "EncoderVelocity.h"
#include "PID.h"
#include "util.h"

#define Kp 0.25
#define Ki 0.01
#define Kd 0
#define pidTau 0.1

MotorDriver motor(A_DIR1, A_PWM1, 0);
EncoderVelocity encoder(ENCODER1_A_PIN, ENCODER1_B_PIN, CPR_312_RPM, 0.2);
PID pid(Kp, Ki, Kd, 0, pidTau, false);

double setpoint = 0;
double velocity = 0;
double controlEffort = 0;

void setup() {
    // Setup code here, such as initializing serial communication or motor drivers
    Serial.begin();

    motor.setup();
}

void loop() {
    // Update setpoint at 50Hz
    EVERY_N_MILLIS(20) {
        // Change this to anything you want! For example, setpoint can be a sinusoidal wave.
        setpoint = 2;
    }

    //update PID at 200Hz
    EVERY_N_MILLIS(5) {
        velocity = encoder.getVelocity(); 
        controlEffort = pid.calculateParallel(velocity, setpoint);
        motor.drive(controlEffort);
    }

    // Print values at 20Hz
    EVERY_N_MILLIS(50) {
        Serial.printf("SP: %.2f   VEL: %.2f   CE: %.2f\n", setpoint, velocity, controlEffort);
    }
}