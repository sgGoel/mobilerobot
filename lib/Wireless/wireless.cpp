#include <WiFi.h>
#include "util.h"
#include "wireless.h"

void ControllerMessage::print() {
    Serial.print("Controller Message\n");
    Serial.printf("millis: %d\n", millis);
    Serial.printf("joystick1:\n");  joystick1.print(1);
    Serial.printf("joystick2:\n");  joystick2.print(1);
    Serial.printf("dPad:\n");  dPad.print(1);
    Serial.printf("buttonL: %s\n", buttonL);
    Serial.printf("buttonR: %s\n", buttonR);
    Serial.printf("touchPoint:\n"); touchPoint.print(1);
} 

bool ControllerMessage::operator==(const ControllerMessage& other) {
    return joystick1 == other.joystick1 &&
           joystick2 == other.joystick2 &&
           dPad == other.dPad &&
           buttonL == other.buttonL &&
           buttonR == other.buttonR &&
           touchPoint == other.touchPoint;
}

void RobotMessage::print() {
    Serial.print("Robot Message\n");
    Serial.printf("millis: %d\n", millis);
    Serial.printf("x: %.2f\n", x);
    Serial.printf("y: %.2f\n", y);
    Serial.printf("theta: %.2f\n", theta);
} 

bool RobotMessage::operator==(const RobotMessage& other) {
    return x == other.x &&
           y == other.y &&
           theta == other.theta;
}

void setupWireless() {
    // ESP_NOW Setup
    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
    	if (Serial) Serial.println("Error initializing ESP-NOW.");
    	return;
    }

    // Tell the microcontroller which functions to call when
    // data is sent or received
	esp_now_register_send_cb(onSendData);
	esp_now_register_recv_cb(onRecvData);
    
    // Register peer
    memcpy(peerInfo.peer_addr, peerAddr, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
  
    // Add peer        
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
    	if (Serial) Serial.println("Failed to add peer");
    	return;
    }
    // ESP-NOW Setup Complete
}
