#pragma once
#include <Arduino.h>
#include <Servo.h>

class ArmServo
{
public:
    int pin;
    float target_angle;
    float current_angle;
    int speed; // 角度每秒（角度制）：每次更新转动小角度 = current_angle ± speed;
    Servo servo;//内封装官方servo库实例
    ArmServo() : pin(0), target_angle(0), current_angle(0), speed(40) {}
    ArmServo(int pin)
        : pin(pin), target_angle(0), current_angle(0), speed(40) // 👈speed初始化值修改处
    {}
    ~ArmServo() {}

    // 转动函数
    void init();//初始化引脚，并使机械臂回到待机位置
    void fastturn(int target);//用于测试类的可行性，仅封装write
    void MoveTo(float target); // 实现平滑转动，拆分大角度为小角度
    void update(float Ts);//舵机每次Loop中的刷新函数

    void addSpeed();
    void lowSpeed();

private:
};