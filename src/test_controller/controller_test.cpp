#include <Bounce2.h>
#include "wireless.h"
#include "util.h"
#include "joystick.h"
#include "dpad.h"
#include "display.h"
#include "controller_pinout.h"

#define PRINT_DELAY 250

ControllerMessage controllerMessage;

Joystick joystick1(JOYSTICK1_X_PIN, JOYSTICK1_Y_PIN);

void setup() {
    Serial.begin(115200);
    joystick1.setup();
    Serial.println("Setup complete.");
}

void loop() {
    // Read and print controller sensors
    EVERY_N_MILLIS(PRINT_DELAY) {
        controllerMessage.millis = millis();
        controllerMessage.joystick1 = joystick1.read(true);
    }
}
