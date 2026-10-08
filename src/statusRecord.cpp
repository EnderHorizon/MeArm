#include "statusRecord.h"

void statusRecord::clear()
{
    for (int i = 0; i < len; ++i) // 遍历清除
    {
        array[i].x = 0;
        array[i].y = 0;
        array[i].z = 0;
        array[i].claw_angle = 0;
    }
    len = 0;
}

uint8_t statusRecord::push_back(int16_t x_r, int16_t y_r, int16_t z_r, uint8_t claw_angle_r)
{
    // 边界检测,录制时间过长默认丢弃数据
    if (len < 180)
    {
        // 存储录制坐标点&钳子角度
        array[len].x = x_r;
        array[len].y = y_r;
        array[len].z = z_r;
        array[len].claw_angle = claw_angle_r;
        ++len;
        /*
        Serial.print("录制数据点数：");
        Serial.println(len);
        Serial.print("录制数据点：");
        Serial.print(x_r);
        Serial.print(",");
        Serial.print(y_r);
        Serial.print(",");
        Serial.print(z_r);
        Serial.print(",");
        Serial.println(claw_angle_r);
        */
        return 1; // 返回1表示录制成功
    }
    else
    {
        Serial.println("录制数据点数已满,结束录制");
        return 0; // 返回0表示录制满了
    }
}

status statusRecord::read()
{
    if(len == readindex)
    {
        Serial.println("结束播放");
        readindex = 0;
        return status(0,0,0,0);
    }
    status sta{array[readindex].x, array[readindex].y, array[readindex].z, array[readindex].claw_angle};
    ++readindex;
    return sta;
}
