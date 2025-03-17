#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "wireless.h"

// #define PRINT_CONTROLLER
#define PRINT_ROBOT

const uint8_t * peerAddr = robotAddr;
esp_now_peer_info_t peerInfo;

bool freshWirelessData = false;
ControllerMessage controllerMessage;
RobotMessage robotMessage;

void onSendData(const uint8_t *mac_addr, esp_now_send_status_t status) {
    bool success = status == ESP_NOW_SEND_SUCCESS ;
    if (success && Serial) {
    	Serial.println("Sent");
		#ifdef PRINT_CONTROLLER
			controllerMessage.print();
		#endif
    } else {
      	Serial.println("Failed");
    }
}

void onRecvData(const uint8_t * mac, const uint8_t *incomingData, int len) {
	memcpy(&robotMessage, incomingData, sizeof(robotMessage));
	freshWirelessData = true;
	#ifdef PRINT_ROBOT
		if (Serial) robotMessage.print();
	#endif
}

bool sendControllerData(){
	esp_err_t result = esp_now_send(robotAddr, (uint8_t *) &controllerMessage, sizeof(controllerMessage));
	return result == ESP_OK;
}