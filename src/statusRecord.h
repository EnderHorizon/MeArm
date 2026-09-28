#pragma once
#include <Arduino.h>

struct status // 因为 AVR 是 8 位单片机，avr-gcc 默认对 int16_t 的对齐要求是 1 字节，所以成员之间不会插入填充字节,总大小为7字节
{
    int16_t x, y, z;
    uint8_t claw_angle;
    //默认构造函数
    status()
        : x(0), y(0), z(0), claw_angle(0)
    {
    }
    // 带参构造函数
    status(int16_t x, int16_t y, int16_t z, uint8_t claw_angle)
        : x(x), y(y), z(z), claw_angle(claw_angle)
    {
    }
    //析构
    ~status(){}
};

class statusRecord // 存储状态的数组
{
private:
    uint8_t len; //uint8_t为1字节，可存储0~255，数组大小为240，故不会溢出
    status array[240];
public:
    statusRecord()
        : len(0), array{} // 初始化，array会自动调用status的默认构造函数
    {
    }
    ~statusRecord() {}
    void clear();
    void push_back(int16_t x_r, int16_t y_r, int16_t z_r, uint8_t claw_angle_r);
};
