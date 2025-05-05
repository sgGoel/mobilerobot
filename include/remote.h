#ifndef REMOTEH
#define REMOTEH

#include <Arduino.h>
#include <UMS3.h>
#include <pinout.h>
#include <esp_now.h>
#include <WiFi.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <Adafruit_seesaw.h>

extern UMS3 ums3;
extern Adafruit_seesaw ss;
extern esp_now_peer_info_t peerInfo;

// Example Mac Addresses
// EC:DA:3B:41:A3:FC
// F4:12:FA:40:9A:A4
// 70:04:1D:AD:D5:08

// one of these
//uint8_t broadcastAddress[] = {0xEC, 0xDA, 0x3B, 0x41, 0xA3, 0xFC};
//uint8_t broadcastAddress[] = {0xF4, 0x12, 0xFA, 0x40, 0x9A, 0xA4};
//uint8_t broadcastAddress[] = {0x70, 0x04, 0x1D, 0xAD, 0xD5, 0x08};
extern uint8_t broadcastAddress[6];
extern TFT_eSPI tft;       // Invoke custom library

struct RemoteData {
    double rightX;
    double rightY;
    double leftX;
    double leftY;
    bool swch1;
    bool swch2;
    bool rotaryUp;
    bool rotaryDown;
    bool rotaryLeft;
    bool rotaryRight;
    bool rotarySelect;
    int rotaryEncoder;
};
extern RemoteData data;


struct RobotMessage {
    unsigned long millis;
    float x;
    float y;
    float theta;

    void print();
    bool operator==(const RobotMessage& other);
} ;

// Battery Variables
extern float batteryVoltage, batteryPercent;
extern float batteryAlpha;

// Joystick variables
extern double leftX, leftY, rightX, rightY;
extern double joystickAlpha;

extern int32_t rotaryStart;

void initPeripherals();

void initRotary();

void readBattery();

void readJoysticks();

void readSwitches();

void readRotary();

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);

void initSender();

void initReceiver();

void sendData();

void initScreen();

void printScreen();

void printData();

void printRotary();

void manualDrive();
void gripperOpen();
void gripperClose();


#endif //REMOTEH