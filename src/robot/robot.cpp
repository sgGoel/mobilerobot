#include <Arduino.h>
#include "remote.h"
#include "MotorDriver.h"

// get these values right!!!
#define DIR_PIN_FM1 1034
#define PWM_PIN_FM1 107
#define LEDC_CHANNEL_FM1 0
#define DIR_PIN_FM2 103
#define PWM_PIN_FM2 106
#define LEDC_CHANNEL_FM2 0
#define DIR_PIN_RM1 1039
#define PWM_PIN_RM1 1041
#define LEDC_CHANNEL_RM1 0
#define DIR_PIN_RM2 1040
#define PWM_PIN_RM2 1042
#define LEDC_CHANNEL_RM2 0

// motor PWMs
double pwm_FM1; // backwards
double pwm_FM2;
double pwm_RM1;
double pwm_RM2; //backwards

// switch mode booleans
bool nav_mode = false;
bool man_mode = false;


// nav mode booleans
bool forward = false;
bool backward = false;
bool turning_CCW = false;
bool turning_CW = false;
bool strafe_L = false;
bool strafe_R = false;
double min_val = 0.2;

// Initialize the motor driver object with DIR and PWM pin numbers and LEDC channel
// F is front, R is rear
MotorDriver FM1(DIR_PIN_FM1, PWM_PIN_FM1, LEDC_CHANNEL_FM1);
MotorDriver FM2(DIR_PIN_FM2, PWM_PIN_FM2, LEDC_CHANNEL_FM2);
MotorDriver RM1(DIR_PIN_RM1, PWM_PIN_RM1, LEDC_CHANNEL_RM1);
MotorDriver RM2(DIR_PIN_RM2, PWM_PIN_RM2, LEDC_CHANNEL_RM2);

void setup() {
  ums3.begin();

  initPeripherals();
  initRotary();  
  Serial.println("Starting!");
  delay(1000);
  initReceiver();

  // ADDED
  Serial.begin(9600);
  while (!Serial); // wait for serial port to connect
  Serial.println("Ready for input:");
  //setup the motor objects
  FM1.setup();
  FM2.setup();
  RM1.setup();
  RM2.setup();
}


void loop() {
  //readJoysticks();
  //readSwitches();
  readRotary();
  
  //sendData(); // Sends data using ESP-NOW to reciever
  printData();  // Prints data via serial port
  delay(20);

  // ADDED
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    Serial.print("Input: ");
    Serial.println(input);
    if (input == "Forward" or input == "forward"){
      //Forward command
      
    }
    else if (input == "Back" or input == "back"){
      //Back command
    }
    else if (input == "Left" or input == "left"){
      //Left command
    }
    else if (input == "Right" or input == "right"){
      //Right command
    }
    else if (input == "Stop" or input == "stop"){
      //Stop command
    }
  }

  // test, all forward, 50% pwm
  FM1.drive(-0.5);
  FM2.drive(0.5);
  RM1.drive(0.5);
  RM2.drive(-0.5);

  // // might be right or left, unsure
  // if(data.swch1){
  //   Serial.println("Switching to Nav Mode");
  //   man_mode = false;
  //   nav_mode = true;
  // }
  // if(data.swch2){
  //   Serial.print("Switching to Man Mode");
  //   man_mode = true;
  //   nav_mode = false;
  // }

  // // when in man_mode
  // if(man_mode){
  //   Serial.println("you're in man mode!");
  // }

  // // when in nav_mode
  // if(nav_mode){
  //   Serial.println("you're in nav mode!");

  //   // forward, Y joystick is positive
  //   if(data.leftY > 0.2){
  //     pwm_FM1 = data.leftY;
  //     pwm_FM2 = data.leftY;
  //   } else if(data.leftY < 0.2){ // backward, Y joystick is negative
  //     pwm_FM1 = data.leftY;
  //     pwm_FM2 = data.leftY;
  //   }
  //   // strafe right, X joystick is positive
  //   if(data.leftX > 0.2){

  //   }
  //   // strafe left, X joystick is negative
  //   if(data.leftX < 0.2){

  //   }

  //   FM1.drive(-pwm_FM1);
  //   FM2.drive(pwm_FM2);
  //   RM1.drive(pwm_RM1);
  //   RM2.drive(-pwm_RM2);

  // }
  
}