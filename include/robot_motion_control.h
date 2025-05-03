#ifndef ROBOT_MOTION_CONTROL_H
#define ROBOT_MOTION_CONTROL_H

// wheel radius in meters
extern float r;
// distance from back wheel to center in meters
extern float b;

void resetState();
void followTrajectory();
void updateOdometry();

#endif