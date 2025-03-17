#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include "EulerAngles.h"
#include "EveryNMillis.h"
//#include "TimerOne.h"

class IMU {
private:
    Adafruit_BNO08x  bno08x;
    sh2_SensorValue_t sensorValue;
    GyroReadings gyroReadings;
    EulerAngles eulerAngles;
    bool imuDataReady;
    int _intPin;
    int _csPin;
    int _resetPin;

public:
    IMU(int resetPin, int csPin, int intPin);
    static void imuISR();
    void setup();
    void setReports();
    void readIMU();
    void update();
    GyroReadings getGyroReadings();
    EulerAngles getEulerAngles();
};

extern IMU imu;
#endif