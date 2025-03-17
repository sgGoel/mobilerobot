#include "SimpleFilters.h"
#include <Arduino.h>

LagFilter::LagFilter(double Ti) : _Ti(Ti), _lastOutput(0.0), _lastTime(micros()) {}

void LagFilter::setParameters(double Ti) {
    _Ti = Ti;
}

double LagFilter::calculate(double input) {
    unsigned long currentTime = micros();
    double T = (currentTime - _lastTime) / 1000000.0; // Convert to seconds
    double alpha = T / _Ti;

    double output = _lastOutput + (alpha * (input - _lastOutput));

    _lastOutput = output;
    _lastTime = currentTime;

    return output;
}
