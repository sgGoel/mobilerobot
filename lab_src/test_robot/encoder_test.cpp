#include "EncoderVelocity.h"
#include "robot_pinout.h"
#include "util.h"

#define PRINT_DELAY 50

// Encoder velocity readers
EncoderVelocity encoders[NUM_MOTORS] = { {ENCODER1_A_PIN, ENCODER1_B_PIN, CPR_312_RPM, 0.2},
                                         {ENCODER2_A_PIN, ENCODER2_B_PIN, CPR_312_RPM, 0.2},
                                         {ENCODER3_A_PIN, ENCODER3_B_PIN, CPR_312_RPM, 0.2}, 
                                         {ENCODER4_A_PIN, ENCODER4_B_PIN, CPR_312_RPM, 0.2} };

void setup() {
    Serial.begin();
}

void loop(){
    // Print encoder readings every PRINT_DELAY milliseconds
    EVERY_N_MILLIS(PRINT_DELAY) {
        Serial.printf("POS, VEL: ");
        for (uint8_t i = 0; i < NUM_MOTORS; i++) 
            Serial.printf("E%d %.2f, %.2f   ", i+1, 
                           encoders[i].getPosition(), encoders[i].getVelocity());        
        Serial.println();
    }
}