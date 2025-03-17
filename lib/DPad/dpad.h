#ifndef DPAD_H
#define DPAD_H

#include <stdint.h>
#include "Adafruit_seesaw.h"
#include <Bounce2.h>

#define UP_PIN 1
#define DOWN_PIN 2
#define LEFT_PIN 3
#define RIGHT_PIN 4
#define SELECT_PIN 5

struct DPadReading {
    bool up;
    bool down;
    bool left;
    bool right;
    bool select;
    int32_t encoderPosition;

    void print(uint8_t nTabs = 0);
    bool operator==(const DPadReading& other);
} ;

class SeesawBounce : public Debouncer {
private:
    Adafruit_seesaw _ss;
    int _pin;

protected:
    virtual bool readCurrentState();

public:
    SeesawBounce();
    void attach(Adafruit_seesaw ss, int pin, int mode);
};

class DPad {
private:
    int _seesawAddr;

    Adafruit_seesaw _ss;
    int32_t _encoderPosition;

    SeesawBounce _upButton;
    SeesawBounce _downButton;
    SeesawBounce _leftButton;
    SeesawBounce _rightButton;
    SeesawBounce _selectButton;

    DPadReading _currReading;

public:
    DPad(int seesawAddr = 0x49);
    void setup();
    void update();
    DPadReading read(bool debugPrint = false);
} ;

#endif // DPAD_H
