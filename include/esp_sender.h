#include <Arduino.h>
#include <string>

struct AprilTagData { int id; float x, y, z; };

void setupComm();
AprilTagData loopComm();