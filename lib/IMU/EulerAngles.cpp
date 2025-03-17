#include "EulerAngles.h"

// this implementation assumes normalized quaternion
// converts to Euler angles in 3-2-1 sequence
// based on https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
EulerAngles ToEulerAngles(Quaternion q) {
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = std::sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
    double cosp = std::sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
    angles.pitch = 2 * std::atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

void printEuler(EulerAngles angles){
    Serial.printf("Roll (rad): %.2f Pitch (rad): %.2f Yaw (rad): %.2f\n", angles.roll, angles.pitch, angles.yaw);    
}

void printEulerDeg(EulerAngles angles){
    Serial.printf("Roll (deg): %.2f Pitch (deg): %.2f Yaw (deg): %.2f\n", angles.roll * RAD_2_DEG, angles.pitch * RAD_2_DEG, angles.yaw * RAD_2_DEG);    
}

void printGyro(GyroReadings gyroReadings){
    Serial.printf("Roll Rate (rad/s): %.2f Pitch Rate (rad/s): %.2f Yaw Rate (rad/s): %.2f\n", gyroReadings.rollRate, gyroReadings.pitchRate, gyroReadings.yawRate);
}

void printGyroDeg(GyroReadings gyroReadings){
    Serial.printf("Roll Rate (deg/s): %.2f Pitch Rate (deg/s): %.2f Yaw Rate (deg/s): %.2f\n", gyroReadings.rollRate * RAD_2_DEG, gyroReadings.pitchRate * RAD_2_DEG, gyroReadings.yawRate * RAD_2_DEG);
}