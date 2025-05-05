//Alessandro -- please transfer any code for ManipulatorPickup.cpp into the sensors version of this file


// Force Inclusions
#include <Arduino.h>

// Servo Inclusions
#include <Wire.h>
// #include <Adafruit_PWMServoDriver.h>
#include <ESP32Servo.h>

// Stepper Inclusions
#include <UMS3.h>

// #include "remote.h" //NEW ONE?

// Force Definitions
#define FORCE_SENSOR_PIN A3

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
// UMS3 ums3;

bool navigating = false;
bool pickup = true;
bool dropoff = false;
bool activate = false;

// PySerial Definitions
int incomingByte = 0;

void open_gripper(){
    //Serial.println("Opening...");
    myservo.write(150); 
    delay(2000);
}

void close_gripper(){
    //Serial.println("Closing...");
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
  //wireless
  // Serial.begin();

  // ums3.begin();

  // //initPeripherals();
  // //initRotary();  
  // Serial.println("Starting!");
  // delay(1000);
  // initReceiver();

  // Force Sensor
  // set the ADC attenuation to 11 dB (up to ~3.3V input)
  analogSetAttenuation(ADC_11db);
  //Serial.println("Force Sensor Activated");

  // Servo
  //Serial.println("Starting Servo Test");

  myservo.setPeriodHertz(60);    // standard 50 hz servo
  myservo.attach(servoPin, 500, 2500); // attaches the servo on pin 18 to the servo object

  //Serial.println("Servo Activated");

  // Stepper
  // ums3.begin();
  pinMode(15, OUTPUT); //step pin
  pinMode(16, OUTPUT); // directinon pinMatrixInAttach
  
  //Serial.println("Stepper Activated");
  delay(10);
}

void loop() {
  // printData();
  // if(data.swch1){
  //   activate = true;
  //   delay(1000);
  // }
  // testing 
   /*
  {  
  // Force Sensor reading
  int analogReading = analogRead(FORCE_SENSOR_PIN);

  //Move servo
  if(millis() - Servo_Time > dt_Servo){
    Servo_Time = millis();
    uint16_t pulselen = map(angle, 0, 180, SERVOMIN, SERVOMAX);
    pwm.setPWM(SERVO_CHANNEL, 0, pulselen);
    angle = (angle+90)%180;
    // delay(100);
  }

  //Move Stepper
  if(millis() - Step_Time > dt_Step){
    Step_Time = millis();
    digitalWrite(15,HIGH);
    if(wasHigh){
      digitalWrite(15,LOW);
      wasHigh = false;
    } else{
      digitalWrite(15,HIGH);
      wasHigh = true;
    }
    Step_Up+=1;
    if(Step_Up <= 2000){
      digitalWrite(16,LOW);
    } else if(Step_Up <= 4000){
      digitalWrite(16,HIGH);
    } else{
      Step_Up = 0;
    }


  }


  // // Move servo according to the force sensor

  //   Servo_Time = millis();
  //   uint16_t pulselen = map(int(180*long(analogReading)/4096.0), 0, 180, SERVOMIN, SERVOMAX);
  //   pwm.setPWM(SERVO_CHANNEL, 0, pulselen);
  //   delay(100);



  // Printing
  Serial.print("Force = ");
  Serial.print(analogReading); // print the raw analog reading
  Serial.print(" ");


  Serial.print("Current Servo Target = ");

  // Servo with pwm cycle
  Serial.print(angle);

  // Servo with force sensor control
  // Serial.print(int(180*long(analogReading)/4096.0));

  // Stepper motor
  Serial.print(" Stepper Direction = ");

  if(Step_Up <= 2000){
    Serial.print("DOWN");
  } else if(Step_Up <= 4000){
    Serial.print("UP");
  }

  Serial.println();
}
  */
    // use pyserial here in the loop to see when it's time to go into man mode (the loop automatically takes us out of man mode)
    // if (Serial.available() > 0){
    //   incomingByte = Serial.read();

    //   Serial.println("I received his good word: ");
    //   Serial.println(incomingByte, DEC);
    // }

    Serial.println("Manipulato rPrint Test");


    // glory to his kingdom
    if(activate){

        if(pickup) {
        // make sure the stepper is as low as possible and the servo is open as much as possible 
        // hope();
        // start the servo closing until the force sensor reads a nonzero value
        close_gripper();
        // when the force sensor begins to read, set a setpoint (prob the max??) and control it open loop? 
        // force_feedback();
        // once this is done, lift 
        raise_stepper();
        pickup = false;
        dropoff = true; 
        } else if(dropoff){
        // move the stepper motor down as low as possible
        lower_stepper();
        // open the servo as far as possible
        open_gripper();
        dropoff = false;
        pickup = true;

        delay(2000);

}
activate = false;
delay(1000);
}
}