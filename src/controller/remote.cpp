#include <Arduino.h>
#include "remote.h"


void setup() {
  ums3.begin();

  initPeripherals();
  initRotary();  
  Serial.println("Starting!");
  delay(1000);
  initSender();

}

void loop() {
  readJoysticks();
  readSwitches();
  //readRotary();
  
  sendData(); // Sends data using ESP-NOW to reciever
  printData();  // Prints data via serial port
  delay(20);
}