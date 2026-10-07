#include "ArmServo.h"

void ArmServo::init()
{
    servo.attach(pin, 500, 2500); // 舵机初始化，设置脉宽范围，单位微秒
    // 初始化舵机至待机位置
    servo.write(init_angle);
    Serial.print("舵机初始化引脚：");
    Serial.println(pin);
}

void ArmServo::setLimitation(uint8_t min, uint8_t max)
{
    this->min_angle = min;
    this->max_angle = max;
}

void ArmServo::fastturn(uint8_t target)
{
    servo.write(target);
}

void ArmServo::MoveTo(float target)
{
    if (target > max_angle)
        target_angle = max_angle;
    else if (target < min_angle)
        target_angle = min_angle;
    else
        target_angle = target;
}

void ArmServo::update(float Ts)
{
    if (((int)target_angle - (int)current_angle) == 0) // 判断是否到达目标位置
        return;
    if ((target_angle - current_angle) > 0) // speed为正
    {
        if ((current_angle += speed * Ts) > target_angle) // 限位，防止设置的speed过大，超过目标角度
            servo.write(target_angle);
        else
            servo.write((int)(current_angle += speed * Ts)); // 原理：角度=角速度×时间
    }
    else // speed为负
    {
        if ((current_angle -= speed * Ts) < target_angle) // 限位，防止设置的speed过大，超过目标角度
            servo.write(target_angle);
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

uint8_t ArmServo::getSpeed()
{
    return speed;
}

void Claw::catchEntity(uint8_t Entity_need_angle) // Entity由宏定义，来控制对应物体的不同大小
{
    MoveTo(Entity_need_angle);
}

void Claw::release()
{
    MoveTo(min_angle);
}
