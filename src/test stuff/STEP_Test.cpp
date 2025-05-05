#include <Arduino.h>
#include <UMS3.h>


UMS3 ums3;

// put these in main setup
void setup() {
  ums3.begin();
  pinMode(15, OUTPUT); //step pin
  pinMode(16, OUTPUT); // directinon pinMatrixInAttach
  Serial.begin();
}

int color = 0;

void drive(){
  digitalWrite(15,HIGH);
  delay(10);
  digitalWrite(15,LOW);
  delayMicroseconds(200);
}


void down(int a){
  // a is cm 
  digitalWrite(16,LOW);// need to check Low is cw or ccw
  
  int stepNum = 250*a;  
  for(int i = 1; i < stepNum; i++){
  digitalWrite(15,HIGH);
  delay(1); // speed up, then decrease the delay
  digitalWrite(15,LOW);
  delay(1);  // speed up, then decrease the delay
  }
  digitalWrite(15,LOW);
}

void lift(int a){
  // a is cm 
  digitalWrite(16,HIGH);

  int stepNum = 250*a;  
  for(int i = 1; i < stepNum; i++){
  digitalWrite(15,HIGH);
  delay(1); // speed up, then decrease the delay
  digitalWrite(15,LOW);
  delay(1);// speed up, then decrease the delay
  }
  digitalWrite(15,LOW);
}

// get rid!
// to use the function
bool hasRun = false;  // 


void loop() {
  int distance = 2;

  if (!hasRun) {
    Serial.println("Lifting");
    lift(distance);  
    Serial.println("Lifted");
    Serial.println("Lowering");
	down(distance);   // 
    Serial.println("Lowered");
    
    hasRun = true;      // 
  }



  
}