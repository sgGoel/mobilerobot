#include "remote.h"

void setup() {
  ums3.begin();

  initPeripherals();
}

void loop() {
  readJoysticks();
  Serial.print("LX: ");
  Serial.print(data.leftX);
  Serial.print(" LY: ");
  Serial.print(data.leftY);
  Serial.print(" RX: ");
  Serial.print(data.rightX);
  Serial.print(" RY: ");
  Serial.println(data.rightY);
  delay(200);
}