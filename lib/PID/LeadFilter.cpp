#include "SimpleFilters.h"
#include <Arduino.h>

LeadFilter::LeadFilter(double alpha, double Td) : _alpha(alpha), _Td(Td), _lastInput(0.0), _lastOutput(0.0), _lastTime(micros()) {}

void LeadFilter::setParameters(double alpha, double Td) {
    _alpha = alpha;
    _Td = Td;
}

double LeadFilter::calculate(double input) {
    unsigned long currentTime = micros();
    double T = (currentTime - _lastTime) / 1000000.0; // Convert to seconds
    double K = T / (_Td + T);

    double output = (_alpha * K * input + (1 - K) * _lastOutput + (_alpha - 1) * K * _lastInput) / (1 + (_alpha - 1) * K);

    _lastInput = input;
    _lastOutput = output;
    _lastTime = currentTime;

    return output;
}
