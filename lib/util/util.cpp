#include "util.h"
#include <Arduino.h>

//map from an input to output range linearly
double mapDouble(double x, double in_min, double in_max, double out_min, double out_max) {
  //if (x <= in_min) return out_min;
  //if (x >= in_max) return out_max;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void printTabs(uint8_t nTabs) {
    for (uint8_t i = 0; i < nTabs; ++i) {
      if (Serial)
        Serial.print("\t");
    }
} 