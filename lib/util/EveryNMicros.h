#ifndef EVERY_N_MICROS_H
#define EVERY_N_MICROS_H

#include <Arduino.h>

class EveryNMicros {
public:
    EveryNMicros(unsigned long interval) : interval(interval), prevMicros(0) {}

    bool update() {
        unsigned long currentMicros = micros();
        if ((unsigned long)(currentMicros - prevMicros) >= interval) {
            prevMicros = currentMicros;
            return true;
        }
        return false;
    }

private:
    unsigned long interval;
    unsigned long prevMicros;
};

#define EVERY_N_MICROS(n) for (static EveryNMicros _timer(n); _timer.update(); )

#endif // EVERY_N_MICROS_H
