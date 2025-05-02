#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include <Arduino.h>
#include "imu.h"

// Wheel encoder globals (defined in localization.cpp)
extern double currPhiL;
extern double currPhiR;
extern double prevPhiL;
extern double prevPhiR;

// Robot geometry parameters (defined in localization.cpp)
extern double r;
extern double b;

// IMU configuration constants
#define IMU_DELAY 50  // loop interval in ms
#define IMU_RST   14  // IMU reset pin
#define IMU_CS    12  // IMU chip-select pin
#define IMU_INT   13  // IMU interrupt pin

// IMU instance (constructed in localization.cpp)
extern IMU imu;

// Update IMU readings and print position/orientation
void readIMU();

void localizationSetup();

// Main localization loop: schedules IMU updates
void localizationLoop();

#endif // LOCALIZATION_H