#include "arm_servo.h"

void ArmServo::MoveTo(float target)
{
    target_angle = target;
}

void ArmServo::update(float Ts)
{
    if (((int)target_angle - (int)current_angle) == 0)
        return;
    if ((target_angle - current_angle) > 0) // speed为正
    {
        servo.write((int)(current_angle += speed * Ts));
    }
    else // speed为负
    {
        servo.write((int)(current_angle -= speed * Ts));
    }
}