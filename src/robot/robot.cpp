#include <Arduino.h>
#include "remote.h"

void setup() {
  ums3.begin();

  initPeripherals();
  initRotary();  
  Serial.println("Starting!");
  delay(1000);
  initReceiver();

  // ADDED
  Serial.begin(9600);
  while (!Serial); // wait for serial port to connect
  Serial.println("Ready for input:");
}


void loop() {
  //readJoysticks();
  //readSwitches();
  readRotary();
  
  //sendData(); // Sends data using ESP-NOW to reciever
  printData();  // Prints data via serial port
  delay(20);

  // ADDED
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    Serial.print("Input: ");
    Serial.println(input);
    if (input == "Forward" or input == "forward"){
      //Forward command
      
    }
    else if (input == "Back" or input == "back"){
      //Back command
    }
    else if (input == "Left" or input == "left"){
      //Left command
    }
    else if (input == "Right" or input == "right"){
      //Right command
    }
    else if (input == "Stop" or input == "stop"){
      //Stop command
    }
  }

}