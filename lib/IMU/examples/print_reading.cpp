#include <Arduino.h>
#include "imu.h"
#include "EulerAngles.h"
#include "util.h"

#define PRINT_DELAY 100

#define IMU_RST 9
#define IMU_CS 10
#define IMU_INT 11

IMU imu(IMU_RST, IMU_CS, IMU_INT);

void setup() {
    Serial.begin(115200);
    imu.setup();
    Serial.println("Setup complete.");
}

void loop() {
    EVERY_N_MILLIS(PRINT_DELAY) {
        imu.update();
        printEulerDeg(imu.getEulerAngles());
        // printGyroDeg(imu.getGyroReadings());
    }
}

