#ifndef EVERY_N_MILLIS_H
#define EVERY_N_MILLIS_H

#include <Arduino.h>

class EveryNMillis {
public:
    EveryNMillis(unsigned long interval) : intervalMillis(interval), previousMillis(0) {}

    bool shouldRun(unsigned long currentMillis) {
        if (currentMillis - previousMillis >= intervalMillis) {
            previousMillis = currentMillis;
            return true;
        }
        return false;
    }
    
private:
    unsigned long previousMillis;
    unsigned long intervalMillis;
};

#define CONCAT_INTERNAL(x, y) x##y
#define CONCAT(x, y) CONCAT_INTERNAL(x, y)
#define EVERY_N_MILLIS(N) \
    static EveryNMillis CONCAT(everyNMillis_, __LINE__)(N); \
    if (CONCAT(everyNMillis_, __LINE__).shouldRun(millis()))

#endif