#include <Arduino.h>
HardwareSerial SerialTwo(2);  // Using UART2

void setup() {
  Serial.begin(115200);
  SerialTwo.begin(9600, SERIAL_8N1, 16, 17); // RX=16, TX=17
}

void loop() {
  //if (SerialTwo.available()) {
    String received = SerialTwo.readStringUntil('\n');
    Serial.println("ESP32-B received: " + received);
  //}
}