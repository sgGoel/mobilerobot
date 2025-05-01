#include "imu.h"


IMU::IMU(int resetPin, int csPin, int intPin) 
: bno08x(resetPin), imuDataReady(false), _resetPin(resetPin), _csPin(csPin), _intPin(intPin){

}

void IMU::imuISR() {
    imu.imuDataReady = true;
}

void IMU::setup() {
    Serial.println("Setting up IMU");

    if (!bno08x.begin_SPI(_csPin, _intPin)) {
        Serial.println("Failed to find BNO08x chip");
        while (1) { delay(10); }
    } 

    Serial.println("BNO08x Found!");

    setReports();
    pinMode(_intPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(_intPin), imuISR, RISING);
}

void IMU::setReports() {
    Serial.println("Setting desired reports");
    if (! bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 1000)) {
        Serial.println("Could not enable game vector");
    }
    if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 1000)) {
        Serial.println("Could not enable gyroscope");
    }
    // adding acceleration report
    if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION, 100)) { //TODO: probably want to adjust this value, and the IMU_DELAY in localization.cpp
        Serial.println("Could not enable linear acceleration");
    }
}

void IMU::readIMU() {
    if (bno08x.wasReset()) {
        Serial.print("sensor was reset ");
        setReports();
        return;
    }

    if (! bno08x.getSensorEvent(&sensorValue)) {
        return;
    }

    switch (sensorValue.sensorId) {
        case SH2_GAME_ROTATION_VECTOR:
            Quaternion quatReading;
            quatReading.w = sensorValue.un.gameRotationVector.real;
            quatReading.x = sensorValue.un.gameRotationVector.i;
            quatReading.y = sensorValue.un.gameRotationVector.j;
            quatReading.z = sensorValue.un.gameRotationVector.k;

            eulerAngles =  ToEulerAngles(quatReading);
        break;

        case SH2_GYROSCOPE_CALIBRATED:
            gyroReadings.rollRate = sensorValue.un.gyroscope.x;
            gyroReadings.pitchRate = sensorValue.un.gyroscope.y;
            gyroReadings.yawRate= sensorValue.un.gyroscope.z;
        break;

        case SH2_LINEAR_ACCELERATION:
            accelReadings.x = sensorValue.un.linearAcceleration.x;
            accelReadings.y = sensorValue.un.linearAcceleration.y;
            accelReadings.z = sensorValue.un.linearAcceleration.z;
        break;
    }  
}

void IMU::update() {
    if (imuDataReady) {
        readIMU();
        // thanks chat for suggesting modification
        // compute dt (difference between lastTimestamp and curent time)
        uint32_t now = micros();
        if (lastTimestamp != 0) {
            double dt = (now - lastTimestamp) * 1e-6;   // unit = secs

            // rotate body acceleration into the world frame (assuming 2d yaw)
            double yaw = eulerAngles.yaw; // radians
            double ax = accelReadings.x;
            double ay = accelReadings.y;
            double ax_w = ax * cos(yaw) - ay * sin(yaw);
            double ay_w = ax * sin(yaw) + ay * cos(yaw);

            // integrate twice (to get velocity, then position)
            // NOTE: position will be quite noisy
            velocity.x += ax_w * dt;
            velocity.y += ay_w * dt;
            position.x += velocity.x * dt;
            position.y += velocity.y * dt;

            Serial.printf("Timestamp: %.3f, Yaw: %.3f, ax: %3.f, ay: %3.f, vx: %3.f, vy: %3.f: ", dt, yaw, ax, ay, velocity.x, velocity.y);
        }
        lastTimestamp = now;
        imuDataReady = false;
    }
}

GyroReadings IMU::getGyroReadings() {
    return gyroReadings;
}

EulerAngles IMU::getEulerAngles() {
    return eulerAngles;
}
