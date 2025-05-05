#include <Arduino.h>
#include "remote.h"
#include "robot_drive.h"
#include "robot_motion_control.h"
//#include "wireless.h"
#include "util.h"
#include "imu.h"
#include "EulerAngles.h"
//#include <Wire.h>
#include "Adafruit_VL6180X.h"
#include "esp_sender.h"
#include <iostream>
//#include "remote.h"


std::atomic<float> apriltagx{10};
std::atomic<float> apriltagy{10};
std::atomic<int> apriltagid{0};
std::atomic<int> colorid{0}; // color reading from Apriltag

void setup() {
    Serial.begin(115200);
    setupDrive();
    resetState();
    Serial.print("resetted");
    //setupWireless();

    // apriltag integration
    setupComm();

    // joystick
    ums3.begin();
    initReceiver();

    //gripper
    /*initPeripherals();
    initRotary();  
    Serial.println("Starting!");
    delay(1000);
    initSender();*/
    //manipulatorSetup();
}

int test = 0;

void loop() {
    // Update velocity setpoints based on trajectory at 50Hz
    if(test == 0){
        resetState();
    }
    test = test + 1;
    
    //TODO: bring back delay for serial connection

    //else
    EVERY_N_MILLIS(20) {
        followTrajectory();
        //manipulatorLoop();
    }

    // Update PID at 200Hz
    EVERY_N_MILLIS(5) {
        updatePIDs();
    }

    // Send and print robot values at 20Hz
    EVERY_N_MILLIS(50) {
        updateOdometry();
        //sendRobotData();

        //Serial.printf("x: %.2f, y: %.2f, theta: %.2f\n",
        //            robotMessage.x, robotMessage.y, robotMessage.theta);
    }

    EVERY_N_MILLIS(500) { //TODO: finetune this delay
        AprilTagData d = loopComm();
        if (d.id != -1) {
            if (d.x != -1) {
                apriltagid.store(d.id);
                apriltagx.store(d.x);
                apriltagy.store(d.y);
                colorid.store(d.col);
            } else {
                Serial.printf("done with gripper task");
            }
        }
        //d::cout << apriltagid<< ", " << apriltagx << ", " apriltagy;
        Serial.printf("%d, %d, %.2f, %.2f", apriltagid.load(), colorid.load(), apriltagx.load(), apriltagy.load()); //debug
    }
  
}