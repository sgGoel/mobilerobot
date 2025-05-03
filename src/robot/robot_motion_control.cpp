#include <Arduino.h>
#include "util.h"
#include "robot_drive.h"
#include "EncoderVelocity.h"
#include "wireless.h"
#include "robot_motion_control.h"

// #define UTURN
// #define CIRCLE
// #define JOYSTICK

// Layla functions
#define FWD // Test run for forward motion
// #define PICKUP // Test run for positioning to pick up a box
// #define DROPOFF // Test run for positioning to drop off a box

// #define REV // Test run for backward motion
// #define TLEFT // Test run for turning left
// #define TRIGHT // Test run for turning right
// #define SLEFT // Test run for strafing left
// #define SRIGHT // Test run for strafing right

// end new functions

extern RobotMessage robotMessage; // old robotMessage
extern ControllerMessage controllerMessage;
// extern RobotMess robotMess; // Layla Added

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
float d3 = 1;
float dgripper = 1;
float dclear_offset = 0; // for motion based on opaque vs clear box
float dclear1 = 1;
float dclear2 = 1;
float ddirection = -4;
float dsetpoint = 10;
float dtest = 10;
int dropoffstate = 0;
int pickupstate = 0;
int cleardropstate = 0;
int yellowstate = 0;
int testerstate = 0;
float r = 5.0;
float b = 2.0;
bool shouldOpen = false;
bool shouldClose = false;

std::atomic<float> x{0.0f}; 

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
        m0 = thetaSpeed;
        m1 = -thetaSpeed;
        m2 = thetaSpeed;
        m3 = -thetaSpeed;
    }
    AWDupdateSetpoints(m0, m1, m2, m3);
}

// Layla -- track position and time
unsigned long prevTime = 0;
float Xrobot = 0.0;
float Yrobot = 0.0;
float Trobot = 0.0;
//

// Layla reset state function
void resetState() {
    state = 0;
    Xrobot = 0.0;
    Yrobot = 0.0;
    Trobot = 0.0;
    Serial.print("RESETSTATE");
    Serial.print("XRESET: ");
    Serial.print(Xrobot);
    Serial.print(", YRESET: ");
    Serial.print(Yrobot);
    Serial.print(", ThetaRESET: ");
    Serial.println(Trobot);
}
// end layla function


// layla distance tester

void distTester(){
    Serial.print("tester");
    float dgripper = 1; // temp
    float dclear_offset = 0; // for motion based on opaque vs clear box
    float dclear1 = 1; // temp
    float dclear2 = 1; // temp
    float ddriection = -4;
    float dsetpoint = 10;
    float dtest = 10;
    dtest = 10; // temp variable -- length of gripper
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - prevTime) / 1000.0;  // in seconds
    prevTime = currentTime;
    // Update robot position and orientation
    Xrobot += robotVelocity * deltaTime;  // Update x position
    Yrobot += robotYVelocity * deltaTime;  // Update y position
    Trobot += thetaSpeed * deltaTime;  // Update orientation
    // Print the updated values
    Serial.print("XP: ");
    Serial.print(Xrobot);
    Serial.print(", YP: ");
    Serial.print(Yrobot);
    Serial.print(", ThetaP: ");
    Serial.println(Trobot);
    switch (testerstate){
        // Position for search - 0.4.1
        case 0:
            // Until robot has achieved a translation of -dtest
            if (Xrobot >= -dtest) {
                // Move in a straight line backward
                Serial.print("STEP 1");
                robotVelocity = -4;
                robotYVelocity = 0;
            } else {
                // Move on to next state
                Serial.print("END STEP 1");
                Xrobot = 0;
                Yrobot = 0;
                Trobot = 0;
                testerstate++;
            }
            break;
        case 1:
            Xrobot = 0;
            Yrobot = 0;
            Trobot = 0;
            robotVelocity = 0;
            robotYVelocity = 0;
            break;
        }
    AWDsetWheelVelocities(robotVelocity, robotYVelocity, thetaSpeed);
}

// end distance tester


// Layla function -- pickup function based on color of box (for ALL boxes)
bool pickup(String COLOR) {
    Serial.print("pickup");
    Serial.print(COLOR);
    float dgripper = 1; // temp
    float dclear_offset = 0; // for motion based on opaque vs clear box
    float dclear1 = 1; // temp
    float dclear2 = 1; // temp
    float ddriection = -4;
    float dsetpoint = 10;
    dgripper = 2; // temp variable -- length of gripper
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - prevTime) / 1000.0;  // in seconds
    prevTime = currentTime;
    // Update robot position and orientation
    Xrobot += robotVelocity * deltaTime;  // Update x position
    Yrobot += robotYVelocity * deltaTime;  // Update y position
    Trobot += thetaSpeed * deltaTime;  // Update orientation
    // Print the updated values
    Serial.print("XP: ");
    Serial.print(Xrobot);
    Serial.print(", YP: ");
    Serial.print(Yrobot);
    Serial.print(", ThetaP: ");
    Serial.println(Trobot);
    switch (pickupstate){
        // Position for search - 0.4.1
        case 0:
            // Until robot has achieved a translation of -dgripper
            if (Xrobot >= -dgripper) {
                // Move in a straight line backward
                Serial.print("STEP 1");
                robotVelocity = -4;
                robotYVelocity = 0;
            } else {
                // Move on to next state
                Serial.print("END STEP 1");
                Xrobot = 0;
                Yrobot = 0;
                Trobot = 0;
                pickupstate++;
            }
            break;
        // Search for COLOR tag -- 0.4.2
        case 1:
            // Specify switch in direction for clear box 
            Serial.print("ddirection");
            Serial.print(ddirection);
            Serial.print("dsetpoint");
            Serial.print(dsetpoint);
            if (COLOR == "CLEAR" && ddirection == -4){
                ddirection = ddirection * -1;
                dsetpoint = 5;
            }
            // Strafe left (by default), right if for clear box
            if (COLOR == "CLEAR"){ // For clear box
                if (Yrobot <= dsetpoint) {
                    // Strafe left
                    Serial.print("STEP 2");
                    robotVelocity = 0;
                    robotYVelocity = ddirection;
                } else {
                    // Move on to next state
                    Serial.print("END STEP 2");
                    Xrobot = 0;
                    pickupstate++;
                }
            }
            else{ // For opaque boxes
                if (Yrobot >= -dsetpoint) {
                    // Strafe left
                    Serial.print("STEP 2");
                    robotVelocity = 0;
                    robotYVelocity = ddirection;
                } else {
                    // Move on to next state
                    Serial.print("END STEP 2");
                    Xrobot = 0;
                    pickupstate++;
                }
            }
            break;
        // Position for dropoff -- 0.4.3/0.4.4
        case 2:
            // Until robot has achieved a translation of dgripper
            if (Xrobot <= dgripper) {
                // Move in a straight line backward
                Serial.print("STEP 3");
                robotVelocity = 4;
                robotYVelocity = 0;
            } else {
                // Move on to next state
                Serial.print("END STEP 3");
                Xrobot = 0;
                Yrobot = 0;
                Trobot = 0;
                pickupstate++;
            }
            break;
        case 3:
            if (COLOR == "CLEAR"){
                if (Yrobot >= -dsetpoint) {
                    // Strafe right by default, left if going for clear box
                    Serial.print("STEP 4");
                    robotVelocity = 0;
                    robotYVelocity = -ddirection;
                } else {
                    // Move on to next state
                    Serial.print("END STEP 4");
                    Xrobot = 0;
                    Yrobot = 0;
                    Trobot = 0;
                    robotVelocity = 0;
                    robotYVelocity = 0;
                    thetaSpeed = 0;
                    pickupstate++;
                } 
            }
            else{
                if (Yrobot <= dsetpoint) {
                    // Strafe right by default, left if going for clear box
                    Serial.print("STEP 4");
                    robotVelocity = 0;
                    robotYVelocity = -ddirection;
                } else {
                    // Move on to next state
                    Serial.print("END STEP 4");
                    Xrobot = 0;
                    Yrobot = 0;
                    Trobot = 0;
                    robotVelocity = 0;
                    robotYVelocity = 0;
                    thetaSpeed = 0;
                    pickupstate++;
                    return true;
                }
            }
            break;
        case 4:
            return true;
        break;
    }
    AWDsetWheelVelocities(robotVelocity, robotYVelocity, thetaSpeed);
    return false;
}
// end Layla pickup function


// Layla function -- dropoff trajectory based on color of box (for non-clear boxes)
bool dropoff(String COLOR){
    Serial.print("dropoff");
    d1 = 5; // temp variable -- distance from home to yellow box
    d2 = 5; // temp variable -- distance from home to in front of parking spots
    d3 = 10; // temp variable -- strafe distance
    dgripper = 2; // temp variable -- length of gripper
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - prevTime) / 1000.0;  // in seconds
    prevTime = currentTime;
    // Update robot position and orientation
    Xrobot += robotVelocity * deltaTime;  // Update x position
    Yrobot += robotYVelocity * deltaTime;  // Update y position
    Trobot += thetaSpeed * deltaTime;  // Update orientation
    // Keep theta within the range [0, 2*PI)
    //if (Trobot >= 2 * PI) {
    //    Trobot -= 2 * PI;
    //} else if (Trobot < -2 * PI) {
    //    Trobot += 2 * PI;
    //}

    // Print the updated values
    Serial.print("X2: ");
    Serial.print(Xrobot);
    Serial.print(", Y2: ");
    Serial.print(Yrobot);
    Serial.print(", Theta2: ");
    Serial.println(Trobot);

    switch (dropoffstate){
        // Position for dropoff -- 0.3.1
        case 0:
            // Until robot has achieved a translation of -d1 m
            Serial.print("D1");
            Serial.print(d1);
            Serial.print("D2");
            Serial.print(d2);
            Serial.print("Dropoff state:");
            Serial.print(dropoffstate);
            if (Xrobot >= -d1) {
                Serial.print("STEP 1");
                // Move in a straight line backward
                robotVelocity = -4;
                robotYVelocity = 0;
                Serial.print("X0: ");
                Serial.print(Xrobot);
                Serial.print(", Y0: ");
                Serial.print(Yrobot);
                Serial.print(", Theta0: ");
                Serial.println(Trobot);
            } else {
                // Move on to next state
                Serial.print("END STEP 1");
                dropoffstate++;
            }
            break;
        case 1:
            // Until robot has turned -90 degrees
            if (Trobot >= -15) {
                robotVelocity = 0;
                robotYVelocity = 0;
                thetaSpeed = -4;
                Serial.print("STEP 2");
                Serial.print("X0: ");
                Serial.print(Xrobot);
                Serial.print(", Y0: ");
                Serial.print(Yrobot);
                Serial.print(", Theta0: ");
                Serial.println(Trobot);
            } else {
                Serial.print("END STEP 2");
                Xrobot = 0;
                dropoffstate++;
            }
            break;
        case 2:
            // Until robot as achieved a translation of d2 m
            if (Xrobot <= d2) {
                // Move in a straight line forward
                Serial.print("STEP 3");
                Serial.print("X0: ");
                Serial.print(Xrobot);
                Serial.print(", Y0: ");
                Serial.print(Yrobot);
                Serial.print(", Theta0: ");
                Serial.println(Trobot);
                thetaSpeed = 0;
                robotVelocity = 4;
                robotYVelocity = 0;
            } else {
                // Move on to next state
                Serial.print("END STEP 3");
                Xrobot = 0;
                Yrobot = 0;
                dropoffstate++;
            }
            break;
        // Look for COLOR tag -- 0.3.2
        case 3:
            // Until robot sees COLOR tag [SKIPPED]// if apriltagx < 0.05 --- say if apriltagid == color id...
            if (Yrobot >= -d3) {
                // Strafe left
                Serial.print("STEP 4");
                Serial.print("X0: ");
                Serial.print(Xrobot);
                Serial.print(", Y0: ");
                Serial.print(Yrobot);
                Serial.print(", Theta0: ");
                Serial.println(Trobot);
                robotVelocity = 0;
                robotYVelocity = -4;
            } else {
                // Move on to next state
                Serial.print("END STEP 4");
                Xrobot = 0;
                dropoffstate++;
            }
            break;

        // Release box-- 0.3.3 [SKIPPED]

        // Push box into parking spot -- 0.3.4
        case 4:
            // Until robot has achieved a translation of -dgripper m
            if (Xrobot >= -dgripper) {
                // Move in a straight line backward
                Serial.print("STEP 5");
                robotVelocity = -4;
                robotYVelocity = 0;
            } else {
                // Move on to next state
                Serial.print("END STEP 5");
                dropoffstate++;
            }
            break;
        
        // Close gripper [SKIPPED]

        case 5:
            // Until robot has achieved a translation of dgripper m
            if (Xrobot <= 0) {
                // Move in a straight line forward
                Serial.print("STEP 6");
                robotVelocity = 4;
                robotYVelocity = 0;
            } else {
                // Move on to next state
                Serial.print("END STEP 6");
                robotVelocity = 0;
                robotYVelocity = 0;
                thetaSpeed = 0;
                Yrobot = 0;
                dropoffstate++;
            }
            break;

        // Return to setpoint -- 0.3.5 - [SKIPPED]
        // Strafe back
        case 6:
        // Until robot sees COLOR tag [SKIPPED]
        if (Yrobot <= d3) {
            // Strafe right
            Serial.print("STEP 7");
            Serial.print("X0: ");
            Serial.print(Xrobot);
            Serial.print(", Y0: ");
            Serial.print(Yrobot);
            Serial.print(", Theta0: ");
            Serial.println(Trobot);
            robotVelocity = 0;
            robotYVelocity = 4;
        } else {
            // Move on to next state
            Serial.print("END STEP 7");
            Xrobot = 0;
            dropoffstate++;
        }
        break;

        case 7:
        // Until robot as achieved a translation of -d2 m
        if (Xrobot >= -d2) {
            // Move in a straight line backwards
            Serial.print("STEP 8");
            Serial.print("X0: ");
            Serial.print(Xrobot);
            Serial.print(", Y0: ");
            Serial.print(Yrobot);
            Serial.print(", Theta0: ");
            Serial.println(Trobot);
            thetaSpeed = 0;
            robotVelocity = -4;
            robotYVelocity = 0;
        } else {
            // Move on to next state
            Serial.print("END STEP 8");
            Xrobot = 0;
            Yrobot = 0;
            Trobot = 0;
            dropoffstate++;
        }
        break;

        case 8:
        // Until robot has turned 90 degrees
        if (Trobot <= 15) {
            robotVelocity = 0;
            robotYVelocity = 0;
            thetaSpeed = 4;
            Serial.print("STEP 9");
            Serial.print("X0: ");
            Serial.print(Xrobot);
            Serial.print(", Y0: ");
            Serial.print(Yrobot);
            Serial.print(", Theta0: ");
            Serial.println(Trobot);
        } else {
            Serial.print("END STEP 9");
            Xrobot = 0;
            dropoffstate++;
        }
        break;

        case 9:
            // Until robot has achieved a translation of d1 m
            Serial.print("D1");
            Serial.print(d1);
            Serial.print("D2");
            Serial.print(d2);
            if (Xrobot <= d1) {
                Serial.print("STEP 10");
                // Move in a straight line forward
                robotVelocity = 4;
                robotYVelocity = 0;
                Serial.print("X0: ");
                Serial.print(Xrobot);
                Serial.print(", Y0: ");
                Serial.print(Yrobot);
                Serial.print(", Theta0: ");
                Serial.println(Trobot);
            } else {
                // Move on to next state
                Serial.print("END STEP 10");
                robotVelocity = 0;
                robotYVelocity = 0;
                thetaSpeed = 0;
                Xrobot = 0;
                Yrobot = 0;
                Trobot = 0;
                dropoffstate++;
                return true;
            }
            break;
        case 10:
            return true;
            break;
    }
    AWDsetWheelVelocities(robotVelocity, robotYVelocity, thetaSpeed);
    return false;
}
// end Layla dropoff function


// Layla clearDropoff function
bool clearDropoff() {
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - prevTime) / 1000.0;  // in seconds
    prevTime = currentTime;
    // Update robot position and orientation
    Xrobot += robotVelocity * deltaTime;  // Update x position
    Yrobot += robotYVelocity * deltaTime;  // Update y position
    Trobot += thetaSpeed * deltaTime;  // Update orientation
    Serial.print("X2: ");
    Serial.print(Xrobot);
    Serial.print(", Y2: ");
    Serial.print(Yrobot);
    Serial.print(", Theta2: ");
    Serial.println(Trobot);
    switch (cleardropstate){
        // Position for dropoff -- 8.1
        case 0:
            // Until robot has achieved a translation of -dclear1 m
            if (Xrobot >= -dclear1) {
                // Move in a straight line backward
                Serial.print("CLEARDROP 1");
                robotVelocity = -4;
                robotYVelocity = 0;
            } else {
                // Move on to next state
                Serial.print("END CLEARDOP 1");
                Xrobot = 0;
                Yrobot = 0;
                Trobot = 0;
                cleardropstate++;
            }
            break;
        case 1:
            // Until robot has turned 90 degrees
            if (Trobot <= 15) {
                robotVelocity = 0;
                robotYVelocity = 0;
                thetaSpeed = 4;
                Serial.print("CLEARDROP 2");
            } else {
                Serial.print("END CLEARDROP 2");
                Xrobot = 0;
                Yrobot = 0;
                Trobot = 0;
                cleardropstate++;
            }
            break;
        case 2:
            // Until robot has achieved a translation of dclear2 m
            if (Xrobot <= d2) {
                Serial.print("CLEARDROP 3");
                // Move in a straight line forward
                robotVelocity = 4;
                robotYVelocity = 0;
            } else {
                // Move on to next state
                Serial.print("END CLEARDROP 3");
                robotVelocity = 0;
                robotYVelocity = 0;
                thetaSpeed = 0;
                Xrobot = 0;
                Yrobot = 0;
                Trobot = 0;
                cleardropstate++;
            }
            break;
        
        //[SKIPPED]--- open gripper and release box

        case 3:
            // Until robot has achieved a translation of - dclear2 m
            if (Xrobot >= -dclear2) {
                // Move in a straight line backward
                Serial.print("CLEARDROP 4");
                robotVelocity = -4;
                robotYVelocity = 0;
            } else {
                // Move on to next state
                Serial.print("END CLEARDOP 4");
                Xrobot = 0;
                Yrobot = 0;
                Trobot = 0;
                robotVelocity = 0;
                robotYVelocity = 0;
                thetaSpeed = 0;
                cleardropstate++;
                return true;
            }
            break;
        case 4:
            Xrobot = 0;
            Yrobot = 0;
            Trobot = 0;
            robotVelocity = 0;
            robotYVelocity = 0;
            thetaSpeed = 0;
            return true;
            break;

    }
    AWDsetWheelVelocities(robotVelocity, robotYVelocity, thetaSpeed);
    return false;
}
// end Layla clearDropoff function

int started = 0;

// start Layla reachYellow function
bool reachYellow(){
    d1 = 5; // temp variable -- distance from home to yellow box
    d2 = 5; // temp variable -- distance from home to in front of parking spots
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - prevTime) / 1000.0;  // in seconds
    prevTime = currentTime;
    // Update robot position and orientation
    Xrobot += robotVelocity * deltaTime;  // Update x position
    Yrobot += robotYVelocity * deltaTime;  // Update y position
    Trobot += thetaSpeed * deltaTime;  // Update orientation
    Serial.print("X2: ");
    Serial.print(Xrobot);
    Serial.print(", Y2: ");
    Serial.print(Yrobot);
    Serial.print(", Theta2: ");
    Serial.println(Trobot);
    switch(yellowstate){
        case 0:
            if (started == 0) {
                Xrobot = 0;
                Yrobot = 0;
                Trobot = 0;
                started++;
            }
            if (Xrobot <= d1) {
                // Move in a straight line forward
                Serial.print("TOYELLOW");
                robotVelocity = 4;
                robotYVelocity = 0;
                started++;
            } else {
                // Move on to next state
                Serial.print("END RESET");
                Xrobot = 0;
                Yrobot = 0;
                Trobot = 0;
                robotVelocity = 0;
                robotYVelocity = 0;
                thetaSpeed = 0;
                yellowstate++;
                return true;
            }
        break;
        case 1:
            return true;
        break;
    }
    AWDsetWheelVelocities(robotVelocity, robotYVelocity, thetaSpeed);
    return false;
}
// end Layla reachYellow function


// Makes robot follow a trajectory
void followTrajectory() {
    #ifdef FWD
    switch (state){
        case 0: // STEP 0: get to yellowpos setpoint
            if(reachYellow()){
                yellowstate = 0;
                Serial.print("YELLOWREACHED");
                state++;

            }
            else{
                Serial.print("TOYELLOW");
                reachYellow();
                Serial.print("yellowstate");
                Serial.print(yellowstate);
            }
            break;

        case 1: // STEP 1: run pickup function for yellow box
            if (pickup("YELLOW")) {
                pickupstate = 0;
                Serial.print("END PICKUP 1");
                state++;
            }
            else {
                Serial.print("PICKUP 1");
                colorid = 1;
                pickup("YELLOW");
                Serial.print("pickupstate");
                Serial.print(pickupstate);
            }
            break;

        case 2: // STEP 2: run dropoff function for yellow box
            if (dropoff("YELLOW")) {
                dropoffstate = 0;
                Serial.print("END DROPOFF 1");
                state++;
            } else {
                Serial.print("DROPOFF 1");
                colorid = 1;
                dropoff("YELLOW");
                Serial.print("dropoffstate");
                Serial.print(dropoffstate);
            }
            break;

        case 3: // STEP 3: run pickup function for blue box
            if (pickup("BLUE")) {
                pickupstate = 0;
                Serial.print("END PICKUP 2");
                state++;
            }
            else {
                Serial.print("PICKUP 2");
                colorid = 2;
                pickup("BLUE");
                Serial.print("pickupstate");
                Serial.print(pickupstate);
            }
            break;
        
        case 4: // STEP 4: run dropoff function for blue box
            if (dropoff("BLUE")) {
                dropoffstate = 0;
                Serial.print("END DROPOFF 2");
                state++;
            } else {
                Serial.print("DROPOFF 2");
                colorid = 2;
                dropoff("BLUE");
                Serial.print("dropoffstate");
                Serial.print(dropoffstate);
            }
            break;

        case 5: // STEP 5: run pickup function for red box
            if (pickup("RED")) {
                pickupstate = 0;
                Serial.print("END PICKUP 3");
                state++;
            }
            else {
                Serial.print("PICKUP 3");
                colorid = 3;
                pickup("RED");
                Serial.print("pickupstate");
                Serial.print(pickupstate);
            }
            break;
        
        case 6: // STEP 6: run dropoff function for red box
            if (dropoff("RED")) {
                dropoffstate = 0;
                Serial.print("END DROPOFF 3");
                state++;
            } else {
                Serial.print("DROPOFF 3");
                colorid = 3;
                dropoff("RED");
                Serial.print("dropoffstate");
                Serial.print(dropoffstate);
            }
            break;
        
        case 7: // STEP 7: run pickup function for clear box
            if (pickup("CLEAR")) {
                pickupstate = 0;
                Serial.print("END PICKUP Clear");
                state++;
            }
            else {
                Serial.print("PICKUP Clear");
                colorid = 4;
                pickup("CLEAR");
                Serial.print("pickupstate");
                Serial.print(pickupstate);
            }
            break;

        case 8: // STEP 8: run clearDropoff function
        if (clearDropoff()) {
            cleardropstate = 0;
            Serial.print("END DROPOFF Clear");
            state++;
        } else {
            Serial.print("DROPOFF Clear");
            colorid = 4;
            clearDropoff();
            Serial.print("cleardropstate");
            Serial.print(cleardropstate);
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
