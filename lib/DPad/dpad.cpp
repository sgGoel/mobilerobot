# include "dpad.h"
# include "util.h"
# include <Arduino.h>

void DPadReading::print(uint8_t nTabs) {
    printTabs(nTabs); Serial.printf("up: %s\n", up);
    printTabs(nTabs); Serial.printf("down: %s\n", down);
    printTabs(nTabs); Serial.printf("left: %s\n", left);
    printTabs(nTabs); Serial.printf("right: %s\n", right);
    printTabs(nTabs); Serial.printf("select: %s\n", select);
    printTabs(nTabs); Serial.printf("encoder position: %.2f\n"); encoderPosition;
}

bool DPadReading::operator==(const DPadReading& other) {
    return up == other.up &&
           down == other.down &&
           left == other.left &&
           right == other.right &&
           select == other.select &&
           encoderPosition == other.encoderPosition;
}

SeesawBounce::SeesawBounce() { }

void SeesawBounce::attach(Adafruit_seesaw ss, int pin, int mode) {
    _ss = ss;
    _pin = pin;
    _ss.pinMode(_pin, mode);
    begin();
}

bool SeesawBounce::readCurrentState() { 
    return _ss.digitalRead(_pin); 
}

DPad::DPad(int seesawAddr) : _seesawAddr(seesawAddr), _encoderPosition(0),
                             _currReading({0, 0, 0, 0, 0, 0}) { }

void DPad::setup() {
    if (! _ss.begin(_seesawAddr)) {
        if (Serial) Serial.println("Couldn't find rotary encoder seesaw on default address.");
        while(1) delay(10);
    }

    uint32_t version = ((_ss.getVersion() >> 16) & 0xFFFF);
    if (version  != 5740){
        Serial.printf("Wrong firmware loaded. Found %d\n", version);
        while(1) delay(10);
    }

    _upButton.attach(_ss, UP_PIN, INPUT_PULLUP);
    _downButton.attach(_ss, DOWN_PIN, INPUT_PULLUP);
    _leftButton.attach(_ss, LEFT_PIN, INPUT_PULLUP);
    _rightButton.attach(_ss, RIGHT_PIN, INPUT_PULLUP);
    _selectButton.attach(_ss, SELECT_PIN, INPUT_PULLUP);

    _ss.setEncoderPosition(0);
    _ss.enableEncoderInterrupt();
}

void DPad::update() {
    _upButton.update();
    _downButton.update();
    _leftButton.update();
    _rightButton.update();
    _selectButton.update();
}

DPadReading DPad::read(bool debugPrint) {
    _currReading.up = _upButton.read();
    _currReading.down = _downButton.read();
    _currReading.left = _leftButton.read();
    _currReading.right = _rightButton.read();
    _currReading.select = _selectButton.read();
    _currReading.encoderPosition = _ss.getEncoderPosition();

    if (debugPrint && Serial) {
        Serial.printf("Read dpad at I2C address 0x%x\n", 
                       _seesawAddr);
        _currReading.print();
    }     

    return _currReading;
}