// TODOS:
// read pe from serial (ie jetson)
// integrate with pe from imu

//#include <Arduino.h>
//#include "robot.cpp"
//#include "imu.h"

#include "localization.h"

IMU imu(IMU_RST, IMU_CS, IMU_INT);

//TODO: integrate the encoder variables and imu variables into one global position
//TODO: make atomic so can be read by other files? (and move into a header file for cleanliness)
/*double currPhiL = 0;
double currPhiR = 0;
double prevPhiL = 0;
double prevPhiR = 0;

// keep track of time & state across calls, for imu
static uint32_t lastUs = 0;
static float    velX = 0, velY = 0;   // m/s in world frame
static float    posX = 0, posY = 0;   // m    in world frame

double r = 1; //TODO: replace with actual values
double b = 1;

#define IMU_DELAY 50 //TODO: adjust value!

#define IMU_RST 14 //TODO: replace with actual values
#define IMU_CS 12
#define IMU_INT 13*/

/*
void readEncoders(float theta) {
    // take angles from traction wheels only since they don't slip
    currPhiL = encoders[2].getPosition();
    currPhiR = -encoders[3].getPosition();
    
    double dPhiL = currPhiL - prevPhiL;
    double dPhiR = currPhiR - prevPhiR;
    prevPhiL = currPhiL;
    prevPhiR = currPhiR;

    float dtheta = r/(2*b)*(dPhiR-dPhiL);
    float dx = r/2.0 * (cos(theta)*dPhiR + cos(theta)*dPhiL);
    float dy = r/2.0 * (sin(theta)*dPhiR + sin(theta)*dPhiL);

    // Update robot message 
    //float millis = millis();
    float x = dx;
    float y = dy;
    float theta = dtheta;
    
    //TODO: return x,y
}
*/

void readIMU() {
    imu.update();
    //printEulerDeg(imu.getEulerAngles());
    printPosition(imu.getPosition());
    // printGyroDeg(imu.getGyroReadings());
}

void localizationSetup(){
    imu.setup();
}

void localizationLoop(){
    // NOTE: commented out for testing purposees (want to use serial monitor differently for testing vs production)
    /*if (Serial.available() > 0) {
        String pe = Serial.readStringUntil('\n');
        //readEncoders(); //TODO: where to get input?
    }*/
    EVERY_N_MILLIS(IMU_DELAY) {
        readIMU();
    }
}