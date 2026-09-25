#include <Arduino.h>
#include <Servo.h>

#include "arm_servo.h"
#include "Timer.h"
/*
封装类：
1.arm_servo里的ArmServo可以直接调用MoveTo()实现目标角度设置
2.Timer用于舵机实时角度更新
*/

// 舵机初始化:(引脚)
ArmServo servo[4];
servo[0] = base(1);     // 底座舵机
servo[1] = shoulder(2); // 大臂舵机
servo[2] = elbow(3);    // 小臂舵机
servo[3] = claw(4);     // 钳子舵机

#define BASE 0
#define SHOULDER 1
#define ELBOW 2
#define CLAW 3

// 从串口读取命令
char cmd[64] = {0};
size_t index = 0;
void SerialReadCommand(); // 函数声明
// 计时器初始化
Timer timer;

void setup()
{
  Serial.begin(115200);
}

void loop()
{
  SerialReadCommand();
  // 刷新舵机角度
  float Ts = timer.getTimeInterval(); // Ts单位为秒
  for (int i = 0; i < 4; ++i)
  {
    servo[i].update(Ts);
  }

  // test
  base.MoveTo(90);
}

// 函数实现
void SerialReadCommand()
{
  while (Serial.available())
  {
    char c = Serial.read();

    if (c == '\n')
    {
      cmd[index] = '\n';
      /*
      print("接收命令：");
      print(cmd);
      */
      index = 0;
      break;
    }
    else if (c != '\r')
    {
      if (index < sizeof(cmd) - 1) // 防止溢出
      {
        cmd[index] = c;
        ++index;
      }
    }
  }

  // 命令判断：
  //  上位机发送 “O”：机械臂爪子张开；
  //  上位机发送 “S”：机械臂爪子关闭；
  //  上位机发送 “H”：提升机械臂整体运行速度；
  //  上位机发送 “L”：降低机械臂整体运行速度。
  if (cmd[0] == '0' && cmd[1] == '0')
    return;
  switch (cmd[0])
  {
  case 'O':
  case 'S':
  case 'H':
    for (int i = 0; i < 4; ++i)
    {
      servo[i].addSpeed();
    }
  case 'L':
    for (int i = 0; i < 4; ++i)
    {
      servo[i].lowSpeed();
    }
  case 'A':
  case 'B':
  case 'C':

  case 'x':
    char *end;
    int x = strtol(cmd + 1, &end, 10);
    int y = strtol(end + 2, &end, 10);
    int z = strtol(end + 2, &end, 10);

    servo[BASE].MoveTo(x);
    servo[SHOULDER].MoveTo(y);
    servo[ELBOW].MoveTo(z);
  }

  memset(cmd, '0', sizeof(cmd));
}