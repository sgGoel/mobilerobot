#ifndef ESPSH
#define ESPSH
#include <Arduino.h>
#include <string>
#include <atomic>
#include <condition_variable>

struct AprilTagData { int id; float x, y, z; int col; };

extern std::atomic<bool> taskComp;
extern std::condition_variable cv;

void setupComm();
AprilTagData loopComm();

void sendToJetson(int task);

#endif //ESPSH