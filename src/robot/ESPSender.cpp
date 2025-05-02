#include <Arduino.h>

int incomingByte = 0;

void setup() {
    Serial.begin();
}

/*void loop() {
    // int input = analogRead(A3);
    Serial.println("sensor: ");
    // Serial.println("Sending: " + input);

    delay(1000);
}*/

// read from Serial port version of loop() function
// (not inherently incompatible with writing to Serial port, but the writing capability is just not needed in this version)
void loop() {
    if (Serial.available() > 0){
        incomingByte = Serial.read();

        Serial.print("I received");
        Serial.println(incomingByte, DEC);
    }

}