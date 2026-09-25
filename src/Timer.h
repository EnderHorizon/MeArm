#pragma once
#include <Arduino.h>

class Timer
{
public:
    float lastTs; // 上次调用的时间
    float Ts;     // 时间差
    float nowTs;  // 本次调用的时间
    Timer()
        : lastTs(0), Ts(0), nowTs(0)
    {
    }
    ~Timer() {}
    float getTimeInterval(); // 刷新，并返回每次loop的时间间隔

private:
};