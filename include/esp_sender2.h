#include <Arduino.h>
#include <string>

struct SensorData { int task; };

void setupComm();
SensorData loopComm();