#include <Arduino.h>
#include "util.h"
#include "EncoderVelocity.h"
//#include "wireless.h"
#include "remote.h"
#include "robot_motion_control.h"
#include "robot_drive.h"
#include <cmath>
#include <cstdlib>

// #define UTURN
// #define CIRCLE
// #define JOYSTICK

// Layla functions
#define FWD // Test run for forward motion
// #define PICKUP // Test run for positioning to pick up a box
// #define DROPOFF // Test run for positioning to drop off a box

 // #define TEST // Test run
// #define TLEFT // Test run for turning left
// #define TRIGHT // Test run for turning right
// #define SLEFT // Test run for strafing left
// #define SRIGHT // Test run for strafing right

// end new functions

RobotMessage robotMessage; // old robotMessage
//extern ControllerMessage controllerMessage;
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
float dgripper = 5.8;
float dclear_offset = 0; // for motion based on opaque vs clear box
float dclear1 = 1;
float dclear2 = 1;
float ddirection = -4;
float dsetpoint = 10;
float dtest = 2;
int dropoffstate = 0;
int pickupstate = 0;
int cleardropstate = 0;
int yellowstate = 0;
int testerstate = 0;
float r = 5.0;
float b = 2.0;
bool shouldOpen = false;
bool shouldClose = false;
float dropoffstrafe = 0;
float pickupstrafe = 0;


std::atomic<float> x{0.0f}; 


float Xapril = apriltagx.load(); // assume distance from april tag (x)
float Yapril = apriltagy.load(); // assume distance from april tag (y)
float IDapril = apriltagid.load(); // assume 0, 1, 2, or 3
float Colorapril = colorid.load(); // color reading from camera
float error = 0.25; // about 1cm tolerance on each side
float desiredColor = 0; // color we want

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

/// joystick attempt

// motor PWMs
double pwm_0; // backwards
double pwm_1;
double pwm_2;
double pwm_3; //backwards

bool pickupButton = false;
bool dropoffButton = false;

void manualDrive(){
    delay(1000);
    while(!data.swch2){
        printData();
        delay(20);
        if(data.leftY > 0.2 or data.leftY < -0.2){
            Serial.println("Forward/Backward");
            pwm_0 += data.leftY/3;
            pwm_1 += data.leftY/3;
            pwm_2 += data.leftY/3;
            pwm_3 += data.leftY/3;
          }
          // strafe, left X joystick
          if(data.leftX > 0.2 or data.leftX < -0.2){
            Serial.println("Strafing");
            pwm_0 += -data.leftX/3;
            pwm_1 += data.leftX/3;
            pwm_2 += -data.leftX/3;
            pwm_3 += data.leftX/3;
      
          }
          // turn, right X joystick
          if(data.rightX > 0.2 or data.rightX < -0.2){
            Serial.println("Turning");
            pwm_0 += -data.rightX/3;
            pwm_1 += data.rightX/3;
            pwm_2 += data.rightX/3;
            pwm_3 += -data.rightX/3;
          }

          if(data.rightX < 0.2 and data.leftX < 0.2 and data.leftY < 0.2 and data.rightX > -0.2 and data.leftX > -0.2 and data.leftY > -0.2){
            pwm_0=0;
            pwm_1=0;
            pwm_2=0;
            pwm_3=0;
          }
          Serial.print(pwm_0);
          Serial.print(" ");
          Serial.print(pwm_1);
          Serial.print(" ");
          Serial.print(pwm_2);
          Serial.print(" "); 
          Serial.println(pwm_3);

          double m0 = -pwm_0;
          double m1 = -pwm_1;
          double m2 = pwm_2;
          double m3 = pwm_3;

          if(m0 > 1){
            m0 = 1;
          }
          if(m1 > 1){
            m1 = 1;
          }
          if(m2 > 1){
            m2 = 1;
          }
          if(m3 > 1){
            m3 = 1;
          }

        //   AWDupdateSetpoints(m0, m1, m2, m3);
        //   updatePIDs();

          updateSpeeds(m0,m1,m2,m3);

        // make sure the robot is at zero for certain
        if(data.swch1){
            updateSpeeds(0,0,0,0);
            if(pickupButton){
              delay(1000);
              gripperClose();
              pickupButton = false;
              dropoffButton = true;
        
            } else if(dropoffButton){
              delay(1000);
              gripperOpen();
              pickupButton = true;
              dropoffButton = false;
            }
        }

    }
    updateSpeeds(0,0,0,0);
    printData();
    Serial.println("WARNING: FINGER OFF THE TRIGGER!");
    delay(1000);
    printData();
}

/// end joystick attempt



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


// gripper functions

void gripperOpen(){
    activate = true;
    delay(20);
}

void gripperClose(){
    activate = true;
    delay(20);
}

//end gripper functions


// layla distance tester

void distTester(){
    Serial.print("tester");
    float dgripper = 5.8;
    float dclear_offset = 0; // for motion based on opaque vs clear box
    float dclear1 = 1; // temp
    float dclear2 = 1; // temp
    float ddriection = -4;
    float dsetpoint = 2;
    float dtest = 2;
    dtest = 2; // temp variable -- length of gripper
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
    float dgripper = 5.8; // temp
    float dclear_offset = 0; // for motion based on opaque vs clear box
    float dclear1 = 1; // temp
    float dclear2 = 1; // temp
    float ddriection = -4;
    float dsetpoint = 10;
    dgripper = 5.8; // temp variable -- length of gripper
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

        case 1: // GRIPPER - fake!
            Serial.print("STEP 4");
            gripperClose();
            Xrobot = 0;
            Yrobot = 0;
            Trobot = 0;
            pickupstate++;
            break;

        // Search for COLOR tag -- 0.4.2
        case 2:
            Serial.print("Desired color ID");
            Serial.print(desiredColor);
            Serial.print("ID");
            Serial.print(colorid);
            Serial.print("April x");
            Serial.print(Xapril);
            Serial.print("April y");
            Serial.print(Yapril);
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
                if (!((colorid == desiredColor) && abs(Xapril) < error)) {
                    // Strafe right
                    Serial.print("STEP 2");
                    robotVelocity = 0;
                    robotYVelocity = ddirection;
                } else {
                    // Move on to next state
                    Serial.print("END STEP 2");
                    pickupstrafe = Yrobot;
                    Xrobot = 0;
                    Yrobot = 0;
                    Trobot = 0;
                    robotVelocity = 0;
                    robotYVelocity = 0;
                    thetaSpeed = 0;
                    pickupstate++;
                }
            }
            else{ // For opaque boxes
                if (!((colorid == desiredColor) && abs(Xapril) < error)) {
                    // Strafe left
                    Serial.print("STEP 2");
                    robotVelocity = 0;
                    robotYVelocity = ddirection;
                } else {
                    // Move on to next state
                    Serial.print("END STEP 2");
                    pickupstrafe = Yrobot;
                    Xrobot = 0;
                    Yrobot = 0;
                    Trobot = 0;
                    robotVelocity = 0;
                    robotYVelocity = 0;
                    thetaSpeed = 0;
                    pickupstate++;
                }
            }
            break;
        // Position for dropoff -- 0.4.3/0.4.4
        case 3:
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

        case 4: // GRIPPER !
            Serial.print("STEP 4");
            gripperClose();
            Xrobot = 0;
            Yrobot = 0;
            Trobot = 0;
            pickupstate++;
            break;

        case 5:
            if (COLOR == "CLEAR"){
                if (Yrobot >= -pickupstrafe) {
                    // Strafe right by default, left if going for clear box
                    Serial.print("STEP 5");
                    robotVelocity = 0;
                    robotYVelocity = -ddirection;
                } else {
                    // Move on to next state
                    Serial.print("END STEP 5");
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
                if (Yrobot <= pickupstrafe) {
                    // Strafe right by default, left if going for clear box
                    Serial.print("STEP 5");
                    robotVelocity = 0;
                    robotYVelocity = -ddirection;
                } else {
                    // Move on to next state
                    Serial.print("END STEP 5");
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
        case 6:
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
    d1 = 7.23; // temp variable -- distance from home to yellow box
    d2 = 5; // temp variable -- distance from home to in front of parking spots
    d3 = 10; // temp variable -- strafe distance
    dgripper = 5.8; // temp variable -- length of gripper
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
            // Until robot sees COLOR tag [SKIPPED]// if id color matches and |apriltagx| < ~1cm --- say if apriltagid == color id...
            if (!((colorid == desiredColor) && abs(Xapril) < error)){
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
                dropoffstrafe = Yrobot;
                Xrobot = 0;
                dropoffstate++;
            }
            break;

        // Release box-- 0.3.3 
        case 4: // GRIPPER !
            Serial.print("STEP 5");
            gripperOpen();
            Xrobot = 0;
            Yrobot = 0;
            Trobot = 0;
            pickupstate++;
            break;

        // Push box into parking spot -- 0.3.4
        case 5:
            // Until robot has achieved a translation of -dgripper m
            if (Xrobot >= -dgripper) {
                // Move in a straight line backward
                Serial.print("STEP 6");
                robotVelocity = -4;
                robotYVelocity = 0;
            } else {
                // Move on to next state
                Serial.print("END STEP 6");
                dropoffstate++;
            }
            break;
        
        case 6:
            gripperClose();
            // Until robot has achieved a translation of dgripper m
            if (Xrobot <= 0) {
                // Move in a straight line forward
                Serial.print("STEP 7");
                robotVelocity = 4;
                robotYVelocity = 0;
            } else {
                // Move on to next state
                Serial.print("END STEP 7");
                robotVelocity = 0;
                robotYVelocity = 0;
                thetaSpeed = 0;
                Yrobot = 0;
                dropoffstate++;
            }
            break;

        // Return to setpoint -- 0.3.5 
        // Strafe back
        case 7:
        // Until robot sees COLOR tag
        if (Yrobot <= dropoffstrafe) {
            // Strafe right
            Serial.print("STEP 8");
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
            Serial.print("END STEP 8");
            Xrobot = 0;
            dropoffstate++;
        }
        break;

        case 8:
        // Until robot as achieved a translation of -d2 m
        if (Xrobot >= -d2) {
            // Move in a straight line backwards
            Serial.print("STEP 9");
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
            Serial.print("END STEP 9");
            Xrobot = 0;
            Yrobot = 0;
            Trobot = 0;
            dropoffstate++;
        }
        break;

        case 9:
        // Until robot has turned 90 degrees
        if (Trobot <= 15) {
            robotVelocity = 0;
            robotYVelocity = 0;
            thetaSpeed = 4;
            Serial.print("STEP 10");
            Serial.print("X0: ");
            Serial.print(Xrobot);
            Serial.print(", Y0: ");
            Serial.print(Yrobot);
            Serial.print(", Theta0: ");
            Serial.println(Trobot);
        } else {
            Serial.print("END STEP 10");
            Xrobot = 0;
            dropoffstate++;
        }
        break;

        case 10:
            // Until robot has achieved a translation of d1 m
            Serial.print("D1");
            Serial.print(d1);
            Serial.print("D2");
            Serial.print(d2);
            if (Xrobot <= d1) {
                Serial.print("STEP 11");
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
                Serial.print("END STEP 11");
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
        case 11:
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
        
        // strafe right to get to box
        case 3:
            // Until robot sees COLOR tag [SKIPPED]// if id color matches and |apriltagx| < ~1cm --- say if apriltagid == color id...
            if (!((colorid == desiredColor) && abs(Xapril) < error)){
                // Strafe left
                Serial.print("CLEARDROP 4");
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
                Serial.print("END CLEARDROP 4");
                Xrobot = 0;
                dropoffstate++;
            }
            break;
        
        //[SKIPPED]--- open gripper and release box

        case 4:
            // Until robot has achieved a translation of - dclear2 m
            if (Xrobot >= -dclear2) {
                // Move in a straight line backward
                Serial.print("CLEARDROP 5");
                robotVelocity = -4;
                robotYVelocity = 0;
            } else {
                // Move on to next state
                Serial.print("END CLEARDOP 5");
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
        case 5:
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
    d1 = 7.23; // temp variable -- distance from home to yellow box
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
    #ifdef TEST
    Serial.print("In tester");
    distTester();
    #endif
    #ifdef FWD
    Serial.print("april tag");
    Serial.print(Xapril);
    switch (state){
        case 0: // STEP 0: get to yellowpos setpoint
            if(reachYellow()){
                yellowstate = 0;
                Serial.print("YELLOWREACHED");
                state++;

            }
            else{
                Serial.print("TOYELLOW");
                if(data.swch2){
                    Serial.println("Switching to manual");
                    AWDsetWheelVelocities(0, 0, 0);
                    pickupButton = true;
                    dropoffButton = false;
                    manualDrive();
                    Serial.println("READY TO EXIT");
                    state++;
                }
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
                desiredColor = 0;
                if(data.swch2){
                    Serial.println("Switching to manual");
                    Xrobot = 0;
                    Yrobot = 0;
                    Trobot = 0;
                    robotVelocity = 0;
                    robotYVelocity = 0;
                    thetaSpeed = 0;
                    AWDsetWheelVelocities(0, 0, 0);
                    pickupButton = true;
                    dropoffButton = false;
                    manualDrive();
                    Serial.println("READY TO EXIT");
                    state++;
                }
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
                desiredColor = 0;
                if(data.swch2){
                    Serial.println("Switching to manual");
                    Xrobot = 0;
                    Yrobot = 0;
                    Trobot = 0;
                    robotVelocity = 0;
                    robotYVelocity = 0;
                    thetaSpeed = 0;
                    AWDsetWheelVelocities(0, 0, 0);
                    pickupButton = false;
                    dropoffButton = true;
                    manualDrive();
                    Serial.println("READY TO EXIT");
                    state++;
                }
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
                desiredColor = 1;
                if(data.swch2){
                    Serial.println("Switching to manual");
                    Xrobot = 0;
                    Yrobot = 0;
                    Trobot = 0;
                    robotVelocity = 0;
                    robotYVelocity = 0;
                    thetaSpeed = 0;
                    AWDsetWheelVelocities(0, 0, 0);
                    pickupButton = true;
                    dropoffButton = false;
                    manualDrive();
                    Serial.println("READY TO EXIT");
                    state++;
                }
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
                desiredColor = 1;
                if(data.swch2){
                    Serial.println("Switching to manual");
                    Xrobot = 0;
                    Yrobot = 0;
                    Trobot = 0;
                    robotVelocity = 0;
                    robotYVelocity = 0;
                    thetaSpeed = 0;
                    AWDsetWheelVelocities(0, 0, 0);
                    pickupButton = false;
                    dropoffButton = true;
                    manualDrive();
                    Serial.println("READY TO EXIT");
                    state++;
                }
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
                desiredColor = 2;
                if(data.swch2){
                    Serial.println("Switching to manual");
                    Xrobot = 0;
                    Yrobot = 0;
                    Trobot = 0;
                    robotVelocity = 0;
                    robotYVelocity = 0;
                    thetaSpeed = 0;
                    AWDsetWheelVelocities(0, 0, 0);
                    pickupButton = true;
                    dropoffButton = false;
                    manualDrive();
                    Serial.println("READY TO EXIT");
                    state++;
                }
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
                desiredColor = 2;
                if(data.swch2){
                    Serial.println("Switching to manual");
                    Xrobot = 0;
                    Yrobot = 0;
                    Trobot = 0;
                    robotVelocity = 0;
                    robotYVelocity = 0;
                    thetaSpeed = 0;
                    AWDsetWheelVelocities(0, 0, 0);
                    pickupButton = false;
                    dropoffButton = true;
                    manualDrive();
                    Serial.println("READY TO EXIT");
                    state++;
                }
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
                desiredColor = 3;
                if(data.swch2){
                    Serial.println("Switching to manual");
                    Xrobot = 0;
                    Yrobot = 0;
                    Trobot = 0;
                    robotVelocity = 0;
                    robotYVelocity = 0;
                    thetaSpeed = 0;
                    AWDsetWheelVelocities(0, 0, 0);
                    pickupButton = true;
                    dropoffButton = false;
                    manualDrive();
                    Serial.println("READY TO EXIT");
                    state++;
                }
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
            desiredColor = 3;
            if(data.swch2){
                Serial.println("Switching to manual");
                Xrobot = 0;
                Yrobot = 0;
                Trobot = 0;
                robotVelocity = 0;
                robotYVelocity = 0;
                thetaSpeed = 0;
                AWDsetWheelVelocities(0, 0, 0);
                pickupButton = false;
                dropoffButton = true;
                manualDrive();
                Serial.println("READY TO EXIT");
                state++;
            }
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
