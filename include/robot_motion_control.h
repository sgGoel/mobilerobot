#ifndef ROBOT_MOTION_CONTROL_H
#define ROBOT_MOTION_CONTROL_H

// wheel radius in meters
#define r 0.06
// distance from back wheel to center in meters
#define b 0.2

void followTrajectory();
void updateOdometry();

#endif