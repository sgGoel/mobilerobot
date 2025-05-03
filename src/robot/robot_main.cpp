#include <Arduino.h>
#include "robot_drive.h"
#include "wireless.h"
#include "util.h"
#include "robot_motion_control.h"
#include "imu.h"
#include "EulerAngles.h"
#include <Wire.h>
#include "Adafruit_VL6180X.h"
#include "esp_sender.h"

void setup() {
    Serial.begin(115200);
    setupDrive();
    resetState();
    Serial.print("resetted");
    setupWireless();

    // apriltag integration
    setupComm();
    std::atomic<float> apriltagx{10};
    std::atomic<float> apriltagy{10};
    std::atomic<int> apriltagid{0}; // color reading from Apriltag
    std::atomic<int> colorid{0};
}

int test = 0;

void loop() {
    // Update velocity setpoints based on trajectory at 50Hz
    if(test == 0){
        resetState();
    }
    test = test + 1;

    EVERY_N_MILLIS(20) {
        followTrajectory();
    }

    // Update PID at 200Hz
    EVERY_N_MILLIS(5) {
        updatePIDs();
    }

    // Send and print robot values at 20Hz
    EVERY_N_MILLIS(50) {
        updateOdometry();
        sendRobotData();

        //Serial.printf("x: %.2f, y: %.2f, theta: %.2f\n",
        //            robotMessage.x, robotMessage.y, robotMessage.theta);
    }

    EVERY_N_MILLIS(500) { //TODO: finetune this delay
        AprilTagData d = loopComm();
        apriltagid = d.id;
        apriltagx = d.x;
        apriltagy = d.y;
        //TODO: do color assignment
    }
  
}