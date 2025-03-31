#include <Arduino.h>
#include <ESP32Encoder.h>
#include "robot_pinout.h"
#include "util.h"

#define PRINT_DELAY 50

ESP32Encoder encoders[NUM_MOTORS];
uint8_t encoderPins[NUM_MOTORS*2] = {ENCODER1_A_PIN, ENCODER1_B_PIN,
                                     ENCODER2_A_PIN, ENCODER2_B_PIN,
                                     ENCODER3_A_PIN, ENCODER3_B_PIN, 
                                     ENCODER4_A_PIN, ENCODER4_B_PIN};

void setup(){	
	Serial.begin();

    ESP32Encoder::useInternalWeakPullResistors=UP;

	for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        // Enable the weak pull down resistors
		encoders[i].attachFullQuad(encoderPins[i*2], encoderPins[i*2+1]);
        encoders[i].setCount(0);
    }
}

void loop(){
	// Loop and read the count
	EVERY_N_MILLIS(PRINT_DELAY) {
        Serial.printf("COUNTS: ");
        for (uint8_t i = 0; i < NUM_MOTORS; i++) 
            Serial.printf("E%d %d   ", i+1, (int32_t)encoders[i].getCount());        
        Serial.println();
    }
}