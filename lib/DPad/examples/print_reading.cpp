#include "dpad.h"
#include "util.h"
#include <Arduino.h>

#define PRINT_DELAY 50

#define SEESAW_ADDR 0x49

// TODO: This code is not validated.

DPad dPad(SEESAW_ADDR);

void setup() {
    Serial.begin(115200);
    dPad.setup();
    Serial.println("Setup complete.");
}

void loop() {
    dPad.update();
    
    EVERY_N_MILLIS(PRINT_DELAY) {
        dPad.read(true);
    }
}
