# include "joystick.h"
# include "util.h"
# include <Arduino.h>

void JoystickReading::print(uint8_t nTabs) {
    printTabs(nTabs); Serial.printf("x: %.3f\n", x);
    printTabs(nTabs); Serial.printf("y: %.3f\n", y);
} 

bool JoystickReading::operator==(const JoystickReading& other) {
    return x == other.x &&
           y == other.y;
}

Joystick::Joystick(int xPin, int yPin, float alpha) 
    : _xPin(xPin), _yPin(yPin), _alpha(alpha), _rawReading({0, 0}), _filtReading({0, 0}) { }

void Joystick::setup() {
    pinMode(_xPin, INPUT);
    pinMode(_yPin, INPUT);
}

JoystickReading Joystick::read(bool debugPrint) {
    _rawReading.x = mapDouble(analogRead(_xPin), JOYSTICK_ANALOG_MIN, JOYSTICK_ANALOG_MAX,
                                                  JOYSTICK_READING_MIN, JOYSTICK_READING_MAX);
    _rawReading.y = mapDouble(analogRead(_yPin), JOYSTICK_ANALOG_MIN, JOYSTICK_ANALOG_MAX,
                                                  JOYSTICK_READING_MIN, JOYSTICK_READING_MAX);
    
    _filtReading.x = _alpha * _rawReading.x + (1 - _alpha) * _filtReading.x;
    _filtReading.y = _alpha * _rawReading.y + (1 - _alpha) * _filtReading.y;

    if (debugPrint && Serial) {
        Serial.printf("Read joystick with pins %d %d\n", _xPin, _yPin);
        _filtReading.print();
    }     

    return _filtReading;
}