#include <Arduino.h>
#include "robot_pinout.h"
#include "MotorDriver.h"

#define NUM_SPEEDS 8

#define DELAY 500 // Delay between motor movements in milliseconds

// Create an instance of the MotorDriver class
MotorDriver motors[NUM_MOTORS] = { {A_DIR1, A_PWM1, 0}, {A_DIR2, A_PWM2, 1},
                                   {B_DIR1, B_PWM1, 2}, {B_DIR2, B_PWM2, 3} };

const double speeds[NUM_SPEEDS] = {1.0, 0.0, 0.5, 0.0, -1.0, 0.0, -0.5, 0.0};
const char* description[NUM_SPEEDS] = {"Moving motor %d forward at full speed", "Stopping %d",
                                       "Moving motor %d forward at half speed", "Stopping %d",
                                       "Moving motor %d backward at full speed", "Stopping %d",
                                       "Moving motor %d backward at half speed", "Stopping %d",};

void setup() {
    // Initialize serial communication
    Serial.begin();

    // Setup the motor driver
    for (uint8_t i = 0; i < NUM_MOTORS; i++)
        motors[i].setup();
}

void loop() {
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        for (uint8_t j = 0; j < NUM_SPEEDS; j++) {
            Serial.printf(description[j], i+1);
            Serial.println();
            motors[i].drive(speeds[j]);
            delay(DELAY);
        }
    }
}