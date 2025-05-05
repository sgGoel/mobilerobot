// Force Inclusions
#include <Arduino.h>

// Servo Inclusions
#include <Wire.h>
// #include <Adafruit_PWMServoDriver.h>
#include <ESP32Servo.h>

// Stepper Inclusions
#include <UMS3.h>

// Force Definitions
#define FORCE_SENSOR_PIN A3

// Serial inclusion
#include "esp_sender2.h"
#include <util.h>
#include <atomic>

// Servo Definitions

int angle = 0;
long Servo_Time = 0;
long dt_Servo = 2000;

Servo myservo;  // create servo object to control a servo
// 16 servo objects can be created on the ESP32

int pos = 0;    // variable to store the servo position
// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33 
// Possible PWM GPIO pins on the ESP32-S2: 0(used by on-board button),1-17,18(used by on-board LED),19-21,26,33-42
// Possible PWM GPIO pins on the ESP32-S3: 0(used by on-board button),1-21,35-45,47,48(used by on-board LED)
// Possible PWM GPIO pins on the ESP32-C3: 0(used by on-board button),1-7,8(used by on-board LED),9-10,18-21
int servoPin = 38;

// Stepper Definitions
int Step_Time = 0;
long dt_Step = 2;
bool wasHigh = true;
int Step_Up = 0;
UMS3 ums3;

bool navigating = false;
bool pickup = true;
bool dropoff = false;

std::atomic<int> task{-1}; //1 = pickup, 2 = dropoff

void open_gripper(){
    Serial.println("Opening...");
    myservo.write(150); 
    delay(2000);
}

void close_gripper(){
    Serial.println("Closing...");
    myservo.write(50);
    delay(2000);
}

void lower_stepper(){
    digitalWrite(16,LOW);
    int time = 0;
    while(time <= 2000){
        digitalWrite(15,HIGH);
        delay(2);
        digitalWrite(15,LOW);
        delay(2);
        time++;
    }
}

void raise_stepper(){
    digitalWrite(16,HIGH);
    int time = 0;
    while(time <= 2000){
        digitalWrite(15,HIGH);
        delay(2);
        digitalWrite(15,LOW);
        delay(2);
        time++;
    }
}

void setup() {
  //Serial.begin();
  setupComm();

  // Force Sensor
  // set the ADC attenuation to 11 dB (up to ~3.3V input)
  analogSetAttenuation(ADC_11db);
  Serial.println("Force Sensor Activated");

  // Servo
  Serial.println("Starting Servo Test");

  myservo.setPeriodHertz(60);    // standard 50 hz servo
  myservo.attach(servoPin, 500, 2500); // attaches the servo on pin 18 to the servo object

  Serial.println("Servo Activated");

  // Stepper
  ums3.begin();
  pinMode(15, OUTPUT); //step pin
  pinMode(16, OUTPUT); // directinon pinMatrixInAttach
  
  Serial.println("Stepper Activated");
  delay(10);
}

void loop() {
  EVERY_N_MILLIS(200) { //TODO: finetune this delay
    SensorData d = loopComm();


    if (d.task != -1) { 
      task.store(d.task); //1 = pickup, 2 = dropoff
      
      if(d.task == 1) {
        // make sure the stepper is as low as possible and the servo is open as much as possible 
        // hope();
        // start the servo closing until the force sensor reads a nonzero value
        close_gripper();
        // when the force sensor begins to read, set a setpoint (prob the max??) and control it open loop? 
        // force_feedback();
        // once this is done, lift 
        raise_stepper();
        delay(2000);
        
        sendToJetson(); //let jetson know we're ending task
      } else if(d.task == 2){
        // move the stepper motor down as low as possible
        lower_stepper();
        // open the servo as far as possible
        open_gripper();
        delay(2000); //TODO: Alessandro -- do we need this?


      sendToJetson(); //let jetson know we're ending task
    }
  }
}
}