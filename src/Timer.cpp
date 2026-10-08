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

bool Timer::isIntervalEnough(float interval)
{
    if (lastTs == 0)
    {
        nowTs = micros();
        lastTs = nowTs;
        return false;
    }
    nowTs = micros();
    Ts = (nowTs - lastTs) * 1e-6f;
    if (Ts >= interval)
    {
        lastTs = nowTs;
        return true;
    }
    else
        return false;
}
