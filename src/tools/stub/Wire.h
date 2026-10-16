#pragma once
#include "Arduino.h"
#define Wire _WireStub()
struct _WireStub {
    void begin() {}
    void begin(int) {}
    void onReceive(void (*)(int)) {}
    void onRequest(void (*)(void)) {}
    uint8_t requestFrom(uint8_t, uint8_t) { return 0; }
    int available() { return 0; }
    int read() { return -1; }
    void readBytes(uint8_t *, size_t) {}
    void beginTransmission(uint8_t) {}
    void endTransmission() {}
    size_t write(uint8_t) { return 1; }
    size_t write(const uint8_t *, size_t) { return 0; }
};
