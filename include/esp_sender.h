#ifndef ESPSH
#define ESPSH
#include <Arduino.h>
#include <string>

struct AprilTagData { int id; float x, y, z; int col; };

void setupComm();
AprilTagData loopComm();

void sendToJetson(int task);

#endif //ESPSH