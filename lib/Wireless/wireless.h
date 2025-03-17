#ifndef WIRELESS_H
#define WIRELESS_H

#include <esp_now.h>
#include "joystick.h"
#include "dpad.h"
#include "display.h"

const uint8_t controllerAddr[] = {0xEC, 0xDA, 0x3B, 0x41, 0xB5, 0x74};
const uint8_t robotAddr[] = {0xEC, 0xDA, 0x3B, 0x41, 0xB5, 0x9C};

struct ControllerMessage {
    unsigned long millis;
    JoystickReading joystick1;
    JoystickReading joystick2;
    DPadReading dPad;
    bool buttonL;
    bool buttonR;
    TouchReading touchPoint;

    void print();
    bool operator==(const ControllerMessage& other);
} ;

struct RobotMessage {
    unsigned long millis;
    float x;
    float y;
    float theta;

    void print();
    bool operator==(const RobotMessage& other);
} ;

void onSendData(const uint8_t * mac, esp_now_send_status_t status);
void onRecvData(const uint8_t * mac, const uint8_t *data, int len);
void setupWireless();
bool sendControllerData();
bool sendRobotData();

extern const uint8_t * peerAddr;
extern esp_now_peer_info_t peerInfo;

extern bool freshWirelessData;
extern ControllerMessage controllerMessage;
extern RobotMessage robotMessage;

#endif // WIRELESS_H