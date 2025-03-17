#include "SimpleFilters.h"
#include <Arduino.h>

LeadLagFilter::LeadLagFilter(double alpha, double Td, double Ti) : _alpha(alpha), _Td(Td), _Ti(Ti), _leadFilter(alpha, Td), _lagFilter(Ti) {}

void LeadLagFilter::setParameters(double alpha, double Td, double Ti) {
    _alpha = alpha;
    _Td = Td;
    _Ti = Ti;
    _leadFilter.setParameters(alpha, Td);
    _lagFilter.setParameters(Ti);
}

double LeadLagFilter::calculate(double input) {

    //skip the lag if Ti is 0 and lead is nonzero
    if (_Ti == 0) {
        //skip the lead if alpha is 0 and Td is nonzero
        if (_alpha == 0 || _Td == 0) {
            return input;
        }
        return _leadFilter.calculate(input);
    }
    //skip the lead if alpha is 0 or Td is zero
    if (_Td == 0 || _alpha == 0) {
        //skip the lag if Ti is 0 and lead is nonzero
        if (_Ti == 0) {
            return input;
        }
        return _lagFilter.calculate(input);
    }
    //calculate the lag and lead if both are nonzero
    double lagOutput = _lagFilter.calculate(input);
    return _leadFilter.calculate(lagOutput);
}