#ifndef ROBOT_MOTION_CONTROL_H
#define ROBOT_MOTION_CONTROL_H

#include <atomic>

// wheel radius in meters
extern float r;
// distance from back wheel to center in meters
extern float b;

extern std::atomic<float> apriltagx;
extern std::atomic<float> apriltagy;
extern std::atomic<int> apriltagid; // apriltag id
extern std::atomic<int> colorid; // color reading from Apriltag -- 1 = yellow, 2 = blue, 3 = red, 4 = clear

void resetState();
void followTrajectory();
void updateOdometry();

#endif