// IMU Inclusions
#include <Arduino.h>
#include "imu.h"
#include "EulerAngles.h"
#include "util.h"
#include <Wire.h>
#include "Adafruit_VL6180X.h"

// TOF Inclusions

// IMU Definitions
#define PRINT_DELAY 100

#define IMU_RST 14
#define IMU_CS 12
#define IMU_INT 13

// TOF Defnitions
Adafruit_VL6180X vl = Adafruit_VL6180X();


IMU imu(IMU_RST, IMU_CS, IMU_INT);

void setup(){
    Serial.begin();

    // Delay!
    while (!Serial) {
        delay(1);
      }

    // IMU
    imu.setup();
    Serial.println("IMU Activated");

    // TOF
    Serial.println("Adafruit VL6180x test!");
    if (! vl.begin()) {
        Serial.println("Failed to find sensor");
        while (1);
    }
    Serial.println("TOF Activated");
}
void loop(){
    
    // IMU Angles
    EVERY_N_MILLIS(PRINT_DELAY) {
        imu.update();
        printEulerDeg(imu.getEulerAngles());
        imu.getGyroReadings(); // include or the code breaks
    }

    // TOF Readings
    float lux = vl.readLux(VL6180X_ALS_GAIN_5);

    Serial.print("Lux: "); 
    Serial.print(lux);
    
    uint8_t range = vl.readRange();
    uint8_t status = vl.readRangeStatus();

    if (status == VL6180X_ERROR_NONE) {
        Serial.print(" Range: "); Serial.println(range);
    }

    // Some error occurred, print it out!
    
    if  ((status >= VL6180X_ERROR_SYSERR_1) && (status <= VL6180X_ERROR_SYSERR_5)) {
        Serial.println(" System error");
    }
    else if (status == VL6180X_ERROR_ECEFAIL) {
        Serial.println(" ECE failure");
    }
    else if (status == VL6180X_ERROR_NOCONVERGE) {
        Serial.println(" No convergence");
    }
    else if (status == VL6180X_ERROR_RANGEIGNORE) {
        Serial.println(" Ignoring range");
    }
    else if (status == VL6180X_ERROR_SNR) {
        Serial.println(" Signal/Noise error");
    }
    else if (status == VL6180X_ERROR_RAWUFLOW) {
        Serial.println(" Raw reading underflow");
    }
    else if (status == VL6180X_ERROR_RAWOFLOW) {
        Serial.println(" Raw reading overflow");
    }
    else if (status == VL6180X_ERROR_RANGEUFLOW) {
        Serial.println(" Range reading underflow");
    }
    else if (status == VL6180X_ERROR_RANGEOFLOW) {
        Serial.println(" Range reading overflow");
    }
}