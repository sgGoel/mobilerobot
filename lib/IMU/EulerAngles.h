#ifndef EULER_ANGLES_H
#define EULER_ANGLES_H
#include <Arduino.h>
#define RAD_2_DEG 57.295779513082320876798154814105
struct Quaternion {
    double w, x, y, z;
};

struct EulerAngles {
    double roll, pitch, yaw;
};

struct GyroReadings {
    double rollRate, pitchRate, yawRate;
};

struct Vector3 { double x, y, z; };

EulerAngles ToEulerAngles(Quaternion q);
void printEuler(EulerAngles angles);
void printEulerDeg(EulerAngles angles);
void printGyro(GyroReadings gyroReadings);
void printGyroDeg(GyroReadings gyroReadings);

void printPosition(Vector3 pos);

#endif // EULER_ANGLES_H