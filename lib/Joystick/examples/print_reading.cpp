#include "joystick.h"
#include "util.h"
#include <Arduino.h>

#define PRINT_DELAY 100

#define X_PIN A2
#define Y_PIN A3

Joystick joystick(X_PIN, Y_PIN);

void setup() {
    Serial.begin(115200);
    joystick.setup();
    Serial.println("Setup complete.");
}

void loop() {
    EVERY_N_MILLIS(PRINT_DELAY) {
        joystick.read(true);
    }
}
