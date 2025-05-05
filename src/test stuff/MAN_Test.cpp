
// Force Inclusions
#include <Arduino.h>

// Servo Inclusions
#include <Wire.h>
//#include <Adafruit_PWMServoDriver.h>
#include <ESP32Servo.h>

// Stepper Inclusions
#include <UMS3.h>

// Force Definitions
#define FORCE_SENSOR_PIN A3

// Servo Definitions
// Create the PWM driver object
// Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// # define SERVO_FREQ = 60

// These values depend on your servo, usually around these
#define SERVOMIN  500 // Minimum pulse length count (0 degrees)
#define SERVOMAX  2500 // Maximum pulse length count (180 degrees)

#define SERVO_CHANNEL 0 // which output 0-15 on the PCA9685
int angle = 0;
long Servo_Time = 0;
long dt_Servo = 500;

// Servo Library Version
Servo myservo;  // create servo object to control a servo
// 16 servo objects can be created on the ESP32
int pos = 0;    // variable to store the servo position
int servoPin = 38;


// Stepper Definitions
int Step_Time = 0;
long dt_Step = 2;
bool wasHigh = false;
int Step_Up = 0;
UMS3 ums3;



void setup() {
  Serial.begin();

  // Force Sensor
  // set the ADC attenuation to 11 dB (up to ~3.3V input)
  analogSetAttenuation(ADC_11db);
  Serial.println("Force Sensor Activated");

  // Servo
  // Serial.println("Starting PCA9685 Servo Test");
  Serial.println("Attaching Servo");
  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	myservo.setPeriodHertz(60);    // nonstandard 60 hz servo
	myservo.attach(servoPin, 500, 2500); // attaches the servo on pin 38 to the servo object

  Serial.println("Servo Activated");

  // Stepper
  ums3.begin();
  pinMode(15, OUTPUT); //step pin
  pinMode(16, OUTPUT); // directinon pinMatrixInAttach
  
  Serial.println("Stepper Activated");
  delay(10);
}

void loop() {
  // Force Sensor reading
  int analogReading = analogRead(FORCE_SENSOR_PIN);

  //Move servo
  if(millis() - Servo_Time > dt_Servo){

    Servo_Time = millis();
    //old
    // uint16_t pulselen = map(angle, 0, 180, SERVOMIN, SERVOMAX);
    // uint16_t new_pulselen = (long(angle)/180.0)*(SERVOMAX - SERVOMIN) + SERVOMIN;
    // pwm.setPWM(SERVO_CHANNEL, 0, new_pulselen);
    
    //new
    angle = (angle+10) % 190;
    // // delay(100);
  }

  //Move Stepper
  if(false){ // just commented out so it does not go(millis() - Step_Time > dt_Step){ 
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