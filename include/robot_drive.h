#ifndef ROBOT_DRIVE_H
#define ROBOT_DRIVE_H

#define NUM_MOTORS 4

#define Kp 0.25
#define Ki 0.01
#define Kd 0
#define pidTau 0.1

#define MAX_FORWARD 6
#define MAX_TURN 3

void setupDrive();
void updateSetpoints(double left, double right);
void updatePIDs();
void AWDupdateSetpoints(double m0, double m1, double m2, double m3);

#endif // ROBOT_DRIVE_H