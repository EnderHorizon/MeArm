#include "Timer.h"

float Timer::getTimeInterval()
{
    if (lastTs == 0)
    {
        nowTs = micros();
        lastTs = nowTs;
        return 0.0f;
    }
    nowTs = micros();
    Ts = (nowTs - lastTs) * 1e-6f;
    lastTs = nowTs;
    return Ts;
}