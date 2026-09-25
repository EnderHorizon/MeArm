#include "Timer.h"

float Timer::getTimeInterval()
{
    nowTs = micros();
    Ts = (nowTs - lastTs) * 1e-6f;
    lastTs = nowTs;
    return Ts;
}