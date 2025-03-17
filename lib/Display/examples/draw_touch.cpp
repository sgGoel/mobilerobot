#include "display.h"
#include "util.h"
#include <Arduino.h>

#define PRINT_DELAY 50

#define TFT_CS_PIN 12
#define TFT_DC_PIN 13

// TODO: This code is not validated.

Display display(TFT_CS_PIN, TFT_DC_PIN);

void setup() {
    Serial.begin();
    display.setup();
    Serial.println("Setup complete.");
}

void loop() {    
    EVERY_N_MILLIS(PRINT_DELAY) {
        display.read(true, 3);
    }
}