#include <Arduino.h>
#include <UMS3.h>

UMS3 ums3;

void setup() {
  // Initialize all board peripherals, call this first
  ums3.begin();

  // Brightness is 0-255.
  ums3.setPixelBrightness(255);

  Serial.begin();
}

int color = 0;

void loop() {
  // colorWheel cycles red, orange, ..., back to red at 256
  ums3.setPixelColor(UMS3::colorWheel(color));
  Serial.println(String(color));
  color++;
  delay(15);
}
