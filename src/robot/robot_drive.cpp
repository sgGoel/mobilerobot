#include <Arduino.h>
#include "robot_pinout.h"
#include "MotorDriver.h"
#include "PID.h"
#include "EncoderVelocity.h"
#include "robot_drive.h"
//#include "remote.h"
//#include "robot_motion_control.h"

#define DIR_PIN_FM1 34
#define PWM_PIN_FM1 7
#define LEDC_CHANNEL_FM1 0
#define DIR_PIN_FM2 3
#define PWM_PIN_FM2 6
#define LEDC_CHANNEL_FM2 0
#define DIR_PIN_RM1 39
#define PWM_PIN_RM1 41
#define LEDC_CHANNEL_RM1 0
#define DIR_PIN_RM2 40
#define PWM_PIN_RM2 42
#define LEDC_CHANNEL_RM2 0

MotorDriver motors[NUM_MOTORS] = { {DIR_PIN_RM1, PWM_PIN_RM1, 0}, {DIR_PIN_RM2, PWM_PIN_RM2, 1},
{DIR_PIN_FM1, PWM_PIN_FM1, 2}, {DIR_PIN_FM2, PWM_PIN_FM2, 3} };

EncoderVelocity encoders[NUM_MOTORS] = { {ENCODER1_A_PIN, ENCODER1_B_PIN, CPR_312_RPM, 0.2},
                                         {ENCODER2_A_PIN, ENCODER2_B_PIN, CPR_312_RPM, 0.2},
                                         {ENCODER3_A_PIN, ENCODER3_B_PIN, CPR_312_RPM, 0.2}, 
                                         {ENCODER4_A_PIN, ENCODER4_B_PIN, CPR_312_RPM, 0.2} };

PID pids[NUM_MOTORS] = { {Kp, Ki, Kd, 0, pidTau, false}, {Kp, Ki, Kd, 0, pidTau, false}, 
                         {Kp, Ki, Kd, 0, pidTau, false}, {Kp, Ki, Kd, 0, pidTau, false} };

double setpoints[NUM_MOTORS] = {0, 0, 0, 0};
double velocities[NUM_MOTORS] = {0, 0, 0, 0};
double controlEfforts[NUM_MOTORS] = {0, 0, 0, 0};


void setupDrive(){
    for (uint8_t i = 0; i < NUM_MOTORS; i++)
        motors[i].setup();
}


void updateSetpoints(double left, double right) {
    setpoints[0] = left;
    setpoints[1] = right;
    setpoints[2] = left;
    setpoints[3] = right;
}

//Layla Added -- update setpoints with all-wheel drive
void AWDupdateSetpoints(double m0, double m1, double m2, double m3) {
    setpoints[0] = m0;
    setpoints[1] = m1;
    setpoints[2] = m2;
    setpoints[3] = m3;
}

// START LAYLA KINEMATICS

float robot_x = 0.0;
float robot_y = 0.0;
float robot_theta = 0.0;
unsigned long lastTime = 0;  // Store time for integration

struct RobotMess {
    float x;
    float y;
    float theta;
};

RobotMess robotMess;

// Update kinematics based on wheel velocities
void updateKinematics() {
    // Time step
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastTime) / 1000.0;  // in seconds
    lastTime = currentTime;

    // Get individual wheel velocities
    float m0 = setpoints[0];  // Front-left wheel velocity
    float m1 = setpoints[1];  // Front-right wheel velocity
    float m2 = setpoints[2];  // Rear-left wheel velocity
    float m3 = setpoints[3];  // Rear-right wheel velocity

    // Robot dimensions (distance from center to wheel)
    float d = 0.35;  // Assuing 35 cm wheelbase

    // Calculate robot velocities (linear and angular)
    float v_x = (m0 + m1 + m2 + m3) / 4.0;
    float v_y = (m0 - m1 + m2 - m3) / 4.0;
    float omega = (-m0 + m1 - m2 + m3) / (4.0 * d);

    // Update robot position and orientation
    robot_x += v_x * deltaTime;  // Update x position
    robot_y += v_y * deltaTime;  // Update y position
    robot_theta += omega * deltaTime;  // Update orientation

    // Keep theta within the range [0, 2*PI)
    if (robot_theta >= 2 * PI) {
        robot_theta -= 2 * PI;
    } else if (robot_theta < 0) {
        robot_theta += 2 * PI;
    }

    robotMess.x = robot_x;
    robotMess.y = robot_y;
    robotMess.theta = robot_theta;

    // Print the updated values
 //   Serial.print("X1: ");
 //   Serial.print(robot_x);
 //   Serial.print(", Y1: ");
 //   Serial.print(robot_y);
 //   Serial.print(", Theta1: ");
 //   Serial.println(robot_theta);
}

// END LAYLA KINEMATICS

void updatePIDs() {
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        velocities[i] = pow(-1, i) * encoders[i].getVelocity();
        controlEfforts[i] = pids[i].calculateParallel(velocities[i], setpoints[i]);
        motors[i].drive(controlEfforts[i]);
    }
    // Layla Added
    updateKinematics();
    //end Layla added
}

void updateSpeeds(double a, double b, double c, double d){

    motors[0].drive(a);
    motors[1].drive(b);
    motors[2].drive(c);
    motors[3].drive(d);
}