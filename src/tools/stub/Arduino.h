// Arduino 环境桩头文件 —— 仅用于 g++ 语法检查，不参与烧录
#pragma once
// 用 C 标准头(而非 <cstdint> 等)，保证 x86 g++ 与 avr-g++ 7.3 都能编译
#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define PROGMEM
#define PI 3.1415926535897932384626433832795
#define F(x) x
typedef bool boolean;
typedef uint8_t byte;

inline unsigned long micros() { return 0; }
inline unsigned long millis() { return 0; }
inline void delay(unsigned long) {}
inline int analogRead(uint8_t) { return 513; }
inline void pinMode(uint8_t, uint8_t) {}
inline void digitalWrite(uint8_t, uint8_t) {}
inline int digitalRead(uint8_t) { return 0; }
inline long random(long a, long b) { return a; }
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define HIGH 1
#define LOW 0
#define A0 14
#define A1 15
#define A2 16
#define A3 17

#define Serial _SerialStub()
struct _SerialStub {
    void begin(long) {}
    void print(const char *) {}
    void print(int) {}
    void print(unsigned int) {}
    void print(long) {}
    void print(unsigned long) {}
    void print(float) {}
    void print(char) {}
    template <typename T> void print(T) {}
    void println(const char *) {}
    void println(int) {}
    void println(unsigned int) {}
    void println(long) {}
    void println(unsigned long) {}
    void println(float) {}
    void println(char) {}
    template <typename T> void println(T) {}
    void println() {}
    int available() { return 0; }
    int read() { return -1; }
    void write(uint8_t) {}
    void writeBytes(const uint8_t *, size_t) {}
};
