#include <Arduino.h>
#include "util.h"
#include "robot_drive.h"
#include "EncoderVelocity.h"
#include "wireless.h"
#include "robot_motion_control.h"

// #define UTURN
// #define CIRCLE
// #define JOYSTICK

#define FWD // Test run for forward motion
// #define REV // Test run for backward motion
// #define TLEFT // Test run for turning left
// #define TRIGHT // Test run for turning right
// #define SLEFT // Test run for strafing left
// #define SRIGHT // Test run for strafing right

// #define DPOS // Dropoff: Positioning for dropoff
// #define PUSHR // Dropoff: Push and return

extern RobotMessage robotMessage;
extern ControllerMessage controllerMessage;

int state = 0;
double robotVelocity = 0; // forwards-backwards velocity of robot, in m/s
double k = 0; // k is 1/radius from center of rotation circle

extern EncoderVelocity encoders[NUM_MOTORS];
double currPhiL = 0;
double currPhiR = 0;
double prevPhiL = 0;
double prevPhiR = 0;

// New variables (Layla)
double robotYVelocity = 0; // Left-right velocity of robot
double thetaSpeed = 0; // Rotation of robot

float d1 = 1;
float d2 = 1;
float dgripper = 1;
float dclear_offset = 1;
float dclear1 = 1;
float dclear2 = 1;

// Sets the desired wheel velocities based on desired robot velocity in m/s
// and k curvature in 1/m representing 1/(radius of curvature)
void setWheelVelocities(float robotVelocity, float k){
    double left = (robotVelocity - k*b*robotVelocity)/r;
    double right = 2*robotVelocity/r  - left;
    updateSetpoints(left, right);
}

// Layla function -- All Wheel Drive
void AWDsetWheelVelocities(float robotVelocity, float robotYVelocity, float thetaSpeed){ 
    // Default to following x-y motion, where fwd and right are positive
    double m0 = -robotVelocity + robotYVelocity;
    double m1 = -robotVelocity - robotYVelocity;
    double m2 = robotVelocity - robotYVelocity;
    double m3 = robotVelocity + robotYVelocity;
    // If x-y motion is zero, check turning, where clockwise motion is positive
    if (robotVelocity == 0 && robotYVelocity ==0){
        double m0 = thetaSpeed;
        double m1 = -thetaSpeed;
        double m2 = thetaSpeed;
        double m3 = -thetaSpeed;
    }
    AWDupdateSetpoints(m0, m1, m2, m3);
}

// Makes robot follow a trajectory
void followTrajectory() {

    #ifdef FWD
    switch (state) {
        case 0: 
            // Until robot has achieved a x translation of 1m
            if (robotMessage.x <= 1.0) {
                // Move in a straight line forward
                robotVelocity = 0.2;
                k = 0;
            } else {
                // Move on to next state
                state++;
            }
            break;
    }
    #endif

    #ifdef JOYSTICK
    if (freshWirelessData) {
        double forward = abs(controllerMessage.joystick1.y) < 0.1 ? 0 : mapDouble(controllerMessage.joystick1.y, -1, 1, -MAX_FORWARD, MAX_FORWARD);
        double turn = abs(controllerMessage.joystick1.x) < 0.1 ? 0 : mapDouble(controllerMessage.joystick1.x, -1, 1, -MAX_TURN, MAX_TURN);
        updateSetpoints(forward + turn, forward - turn);
    }
    #endif 

    #ifdef CIRCLE
    robotVelocity = 0.2;
    k = 1/0.5;
    setWheelVelocities(robotVelocity, k);
    #endif 

    #ifdef UTURN
    switch (state) {
        case 0: 
            // Until robot has achieved a x translation of 1m
            if (robotMessage.x <= 1.0) {
                // Move in a straight line forward
                robotVelocity = 0.2;
                k = 0;
            } else {
                // Move on to next state
                state++;
            }
            break;

        case 1:
            // Until robot has achieved a 180 deg turn in theta
            if (robotMessage.theta <= M_PI) {
                // Turn in a circle with radius 25cm 
                robotVelocity = 0.2;
                k = 1/0.25;
            } else {
                state++;
            }
            break;

        case 2:
            // Until robot has achieved a x translation of -1m
            if (robotMessage.x >= 0) {
                // Move in a straight line forward
                robotVelocity = 0.2;
                k = 0;
            } else {
                // Move on to next state
                state++;
            }
            break;

        default: 
            // If none of the states, robot should just stop
            robotVelocity = 0;
            k = 0;
            break;
    }
    setWheelVelocities(robotVelocity, k);
    #endif

    #ifdef DPOS
    switch (state) {
        case 0: 
            // Assuming starting at yellowpos
            // Until robot has achieved a x translation of -d meters
            if (robotMessage.x >= -d1) {
                // Move in a straight line forward
                robotVelocity = -0.2;
                k = 0;
            } else {
                // Move on to next state
                state++;
            }
            break;

        case 1:
            // Until robot has achieved a 180 deg turn in theta -- FIX
            if (robotMessage.theta <= M_PI) {
                // Turn in a circle with radius 25cm 
                robotVelocity = 0.2;
                k = 1/0.25;
            } else {
                state++;
            }
            break;

        case 2:
            // Until robot has achieved a y translation of d2 meters
            if (robotMessage.y >= d2) {
                // Move in a straight line forward
                robotVelocity = 0.2;
                k = 0;
            } else {
                // Move on to next state
                state++;
            }
            break;
        case 3:
            // Until robot has seen desired color
            if ([// Color NOT DETECTED !!]) {
                // Strafe left
                robotVelocity = 0.2;
                k = 0;
            } else {
                // Move on to next state
                state++;
            }
            break;

        default: 
            // If none of the states, robot should just stop
            robotVelocity = 0;
            k = 0;
            break;
    }

    setWheelVelocities(robotVelocity, k);
    #endif 




    #ifdef PUSHR
    // TODO: Create a state machine to define your custom trajectory!



    setWheelVelocities(robotVelocity, k);
    #endif

}

void updateOdometry() {
    // take angles from traction wheels only since they don't slip
    currPhiL = encoders[2].getPosition();
    currPhiR = -encoders[3].getPosition();
    
    double dPhiL = currPhiL - prevPhiL;
    double dPhiR = currPhiR - prevPhiR;
    prevPhiL = currPhiL;
    prevPhiR = currPhiR;

    float dtheta = r/(2*b)*(dPhiR-dPhiL);
    float dx = r/2.0 * (cos(robotMessage.theta)*dPhiR + cos(robotMessage.theta)*dPhiL);
    float dy = r/2.0 * (sin(robotMessage.theta)*dPhiR + sin(robotMessage.theta)*dPhiL);

    // Update robot message 
    robotMessage.millis = millis();
    robotMessage.x += dx;
    robotMessage.y += dy;
    robotMessage.theta += dtheta;
}

