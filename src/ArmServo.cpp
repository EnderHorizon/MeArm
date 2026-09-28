#include "ArmServo.h"

void ArmServo::init()
{
    servo.attach(pin);
    // 初始化舵机至待机位置
    servo.write(0);
    Serial.print("舵机初始化引脚：");
    Serial.println(pin);
}

void ArmServo::fastturn(uint8_t target)
{
    servo.write(target);
}

void ArmServo::MoveTo(float target)
{
    target_angle = target;
    Serial.println(current_angle);
    Serial.println(target_angle);
}

void ArmServo::update(float Ts)
{
    if (((int)target_angle - (int)current_angle) == 0) // 判断是否到达目标位置
        return;
    if ((target_angle - current_angle) > 0) // speed为正
    {
        if ((current_angle += speed * Ts) > target_angle) // 限位，防止设置的speed过大，超过目标角度
            servo.write(180);
        else
            servo.write((int)(current_angle += speed * Ts)); // 原理：角度=角速度×时间
    }
    else // speed为负
    {
        if ((current_angle -= speed * Ts) < target_angle) // 限位，防止设置的speed过大，超过目标角度
            servo.write(0);
        else
            servo.write((int)(current_angle -= speed * Ts));
    }
}

void ArmServo::addSpeed()
{
    if (speed < 180)
        speed += 10; // 速度加量
}

void ArmServo::lowSpeed()
{
    if (speed > 10)
        speed -= 10; // 速度减量
}
