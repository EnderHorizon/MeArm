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
    Servo servo;
    ArmServo(int pin)
        : pin(pin), target_angle(0), current_angle(0), speed(40) // 👈speed初始化值修改处
    {
        servo.attach(pin);
    }
    ~ArmServo() {}

    // 转动函数
    void MoveTo(float target); // 实现平滑转动，拆分大角度为小角度
    void update(float Ts);

    void addSpeed();
    void lowSpeed();

private:
};