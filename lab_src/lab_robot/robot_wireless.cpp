#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "wireless.h"

// #define PRINT_CONTROLLER
// #define PRINT_ROBOT

const uint8_t * peerAddr = controllerAddr;
esp_now_peer_info_t peerInfo;

bool freshWirelessData = false;
ControllerMessage controllerMessage;
RobotMessage robotMessage;

void onSendData(const uint8_t *mac_addr, esp_now_send_status_t status) {
    bool success = status == ESP_NOW_SEND_SUCCESS ;
    if (success && Serial) {
    	Serial.println("Sent");
		#ifdef PRINT_ROBOT
			robotMessage.print();
		#endif
    } else {
      	Serial.println("Failed");
    }
}

void onRecvData(const uint8_t * mac, const uint8_t *incomingData, int len) {
	memcpy(&controllerMessage, incomingData, sizeof(controllerMessage));
	freshWirelessData = true;
	#ifdef PRINT_CONTROLLER
		if (Serial) controllerMessage.print();
	#endif
}

bool sendRobotData(){
	esp_err_t result = esp_now_send(controllerAddr, (uint8_t *) &robotMessage, sizeof(robotMessage));
	return result == ESP_OK;
}