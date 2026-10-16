#pragma once
#include "Arduino.h"
class Servo {
public:
    Servo() {}
    void attach(uint8_t) {}
    void attach(uint8_t, int, int) {}
    void detach() {}
    void write(int) {}
    int read() { return 0; }
};
