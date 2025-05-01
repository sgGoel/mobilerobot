#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Create the PWM driver object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// These values depend on your servo, usually around these
#define SERVOMIN  150 // Minimum pulse length count (0 degrees)
#define SERVOMAX  600 // Maximum pulse length count (180 degrees)

#define SERVO_CHANNEL 0 // which output 0-15 on the PCA9685

int angle = 0;
long Servo_Time = 0;
long dt_Servo = 2000;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting PCA9685 Servo Test");

  pwm.begin();
  pwm.setPWMFreq(50); // Analog servos run at ~50 Hz

  delay(10);
}

void loop() {

  if(millis() - Servo_Time > dt_Servo){
    Servo_Time = millis();
    uint16_t pulselen = map(angle, 0, 180, SERVOMIN, SERVOMAX);
    pwm.setPWM(SERVO_CHANNEL, 0, pulselen);
    angle = (angle+90)%270;
    delay(100);
  }
  Serial.print("Current Servo Target = ");
  Serial.print(angle);

  // Serial.print(" Current Time = ");
  // Serial.print(millis());
  
  // Serial.print(" Timer Time = ");
  // Serial.print(Servo_Time);

  Serial.println();

  delay(100);

  // // Move servo to 0 degrees
  // uint16_t pulselen = map(0, 0, 180, SERVOMIN, SERVOMAX);
  // pwm.setPWM(SERVO_CHANNEL, 0, pulselen);
  // Serial.println("Moved to 0 degrees");
  // delay(2000);
  // Serial.println("End of Delay");
  // delay(2000);
  // // Move servo to 90 degrees
  // pulselen = map(90, 0, 180, SERVOMIN, SERVOMAX);
  // pwm.setPWM(SERVO_CHANNEL, 0, pulselen);
  // Serial.println("Moved to 90 degrees");
  // delay(2000);
  // Serial.println("End of Delay");

  // // Move servo to 180 degrees
  // pulselen = map(180, 0, 180, SERVOMIN, SERVOMAX);
  // pwm.setPWM(SERVO_CHANNEL, 0, pulselen);
  // Serial.println("Moved to 180 degrees");
  // delay(2000);
  // Serial.println("End of Delay");
  // delay(2000);
}