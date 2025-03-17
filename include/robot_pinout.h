#ifndef ROBOT_PINOUT_H
#define ROBOT_PINOUT_H

//motor pins
#define NUM_MOTORS 4
#define A_DIR1 39
#define A_PWM1 41
#define A_DIR2 40
#define A_PWM2 42
#define B_DIR1 34
#define B_PWM1 7
#define B_DIR2 3
#define B_PWM2 6

//encoder pins
#define ENCODER1_A_PIN 1
#define ENCODER1_B_PIN 2
#define ENCODER2_A_PIN 4
#define ENCODER2_B_PIN 5
#define ENCODER3_A_PIN 21
#define ENCODER3_B_PIN 38
#define ENCODER4_A_PIN 16
#define ENCODER4_B_PIN 15

//IMU pins
#define BNO08X_CS 12
#define BNO08X_INT 13
#define BNO08X_RESET 14

#endif // ROBOT_PINOUT_H