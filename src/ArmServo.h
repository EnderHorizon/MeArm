#pragma once
#include <Arduino.h>
#include <Servo.h>

class ArmServo
{
public:
    uint8_t pin;
    float target_angle;
    float current_angle;
    uint8_t speed; // 角度每秒（角度制）：每次更新转动小角度 = current_angle ± speed;
    Servo servo{}; // 内封装官方servo库实例

    // 限位
    uint8_t max_angle, min_angle;
    //初始角度
    uint8_t init_angle;

public:
    ArmServo()
        : pin(0), target_angle(40), current_angle(40), speed(40), max_angle(180), min_angle(40), init_angle(40)
    {
    }
    ArmServo(uint8_t pin)
        : pin(pin), target_angle(40), current_angle(40), speed(40), max_angle(180), min_angle(40), init_angle(40) // 👈speed初始化值修改处
    {
    }
    //主要采用这个构造函数，同时设置舵机引脚和限位角度
    ArmServo(uint8_t pin, uint8_t min_angle, uint8_t max_angle, uint8_t init_angle)
        : pin(pin), target_angle(init_angle), current_angle(init_angle), speed(40), max_angle(max_angle), min_angle(min_angle), init_angle(init_angle) // 👈speed初始化值修改处
    {
    }
    ~ArmServo() {}

    // 转动函数
    void init(); // 初始化引脚，并使机械臂回到待机位置
    void setLimitation(uint8_t min, uint8_t max);
    void fastturn(uint8_t target); // 用于测试类的可行性，仅封装write(angle)
    void MoveTo(float target);     // 设置目标角度，拆分大角度为小角度
    void update(float Ts);         // 舵机每次Loop中的刷新函数，实现平滑转动，且多个电机同步运行

    void addSpeed(); // 加速
    void lowSpeed(); // 减速
    uint8_t getSpeed();
    float getCurrentAngle() { return current_angle; } // 获取当前角度
    float getMaxAngle() { return max_angle; }         // 获取最大角度
    float getMinAngle() { return min_angle; }         // 获取最小角度
    void setInitAngle(uint8_t angle) { init_angle = angle; } // 设置初始角度
};

class Claw : public ArmServo // 钳子：继承舵机大类，添加钳子的功能
{
public:
    Claw()
        : ArmServo(0)
    {
    }
    Claw(uint8_t pin)
        : ArmServo(pin)
    {
    }
    Claw(uint8_t pin, uint8_t min_angle, uint8_t max_angle, uint8_t init_angle)
        : ArmServo(pin, min_angle, max_angle, init_angle)
    {
    }
    ~Claw() {}
    void catchEntity(uint8_t Entity_need_angle); // 钳子夹持，具体角度由常量来
    void release();                              // 张开钳子
};