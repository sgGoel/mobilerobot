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

    // thanks chat for suggesting this modification
    Vector3    accelReadings{0,0,0};     // latest body-frame accel (m/s^2)
    Vector3    velocity{0,0,0};         // integrated velocity (m/s)
    Vector3    position{0,0,0};         // integrated position (m)
    uint32_t   lastTimestamp = 0;       // for dt computation (micosecs)

public:
    IMU(int resetPin, int csPin, int intPin);
    static void imuISR();
    void setup();
    void setReports();
    void readIMU();
    void update();
    GyroReadings getGyroReadings();
    EulerAngles getEulerAngles();

    Vector3 getPosition() const { return position; }
};

extern IMU imu;
#endif