#include <Arduino.h>
#include <Servo.h>

#include "arm_servo.h"
#include "Timer.h"
/*
封装类：
1.arm_servo里的ArmServo可以直接调用MoveTo()实现目标角度设置
2.Timer用于舵机实时角度更新

函数：
1.传感器读取
2.机械臂运动控制
3.串口命令
*/
// 弧度制转角度制
inline float ToDegree(float radian)
{
  return radian / 180 * (float)PI;
}
// 平方
inline float square(float x)
{
  return x * x;
}
// 舵机初始化:(引脚)
const int BASE = 0;
const int SHOULDER = 1;
const int ELBOW = 2;
const int CLAW = 3;
// 舵机限位

ArmServo servo[4] = {
    ArmServo(9), // 底座舵机
    ArmServo(8), // 大臂舵机
    ArmServo(7), // 小臂舵机
    ArmServo(6)  // 钳子舵机
};

// 计时器初始化
Timer timer;
// 从串口读取命令
char cmd[64] = {0};
size_t index = 0;
void SerialReadCommand();
// 传感器读取
void sensorRead(); // 未实现
// 机械臂运动控制
const float FORARM = 10;               // 小臂/cm
const float UPPERARM = 10;             // 大臂
const float BASEHEIGHT = 4;            // 底座高度
const float CLAWLEN = 4;               // 钳子长度
void catchEntity(int x, int y, int z); // 输入目标坐标->舵机角度（机械臂逆运动解）
void record();                         // 记录运动路线
void executeRecord();                  // 播放记录

//  —————————————————
//  ———— SetUp ——————
//  —————————————————
void setup()
{
  Serial.begin(9600);
  for (int i = 0; i < 4; ++i)
  {
    servo[i].init();
  }
  delay(2000);
}
//  —————————————————
//  ————— Loop ——————
//  —————————————————
// 每次循环刷新舵机角度，实现多舵机同时运动
void loop()
{
  SerialReadCommand();
  //  刷新舵机角度
  float Ts = timer.getTimeInterval(); // Ts单位为秒
  for (int i = 0; i < 4; ++i)
  {
    servo[i].update(Ts);
  }
  delay(15);
  // 测试代码 ↓↓↓
}
//  —————————————————
//  ———— 函数实现 ————
//  —————————————————

void SerialReadCommand()
{
  while (Serial.available())
  {
    char c = Serial.read();

    if (c == '\n')
    {
      cmd[index] = '\n';

      Serial.println("接收命令：");
      Serial.println(cmd);

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
  if (cmd[0] == '0' && cmd[1] == '0')
    return;
  switch (cmd[0])
  {
  case 'O':
    //  上位机发送 “O”：机械臂爪子张开
  case 'S':
    //  上位机发送 “S”：机械臂爪子关闭
  case 'H':
    //  上位机发送 “H”：提升机械臂整体运行速度
    for (int i = 0; i < 4; ++i)
    {
      servo[i].addSpeed();
    }
  case 'L':
    //  上位机发送 “L”：降低机械臂整体运行速度
    for (int i = 0; i < 4; ++i)
    {
      servo[i].lowSpeed();
    }

  case 'A':
  case 'B':
  case 'C':

  case 'x':
    // 检测到第一位是x，就分别取出xyz后的数字，并赋给舵机目标
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

void catchEntity(int x, int y, int z)
{
  // 具体数学推到公式见手写纸
  float shoulder_angle = 0;
  float elbow_angle = 0;
  float base_angle = 0;

  float h = 0;
  float L = sqrtf(square(x) + square(y)) - CLAWLEN;

  base_angle = atanf(y / x);
  if (z > BASEHEIGHT) // z与底座高度比较
  {
    h = z - BASEHEIGHT;
    shoulder_angle = acosf(square(UPPERARM) + square(L) + square(h) - square(FORARM) / 2 * UPPERARM * sqrtf(square(L) + square(h))) + atanf(h / L);
    elbow_angle = shoulder_angle + acosf(square(UPPERARM) + square(FORARM) - (square(L) + square(h)) / 2 * UPPERARM * FORARM);
  }
  else if (z < BASEHEIGHT)
  {
    h = BASEHEIGHT - z;
    shoulder_angle = acosf(square(UPPERARM) + square(L) + square(h) - square(FORARM) / 2 * UPPERARM * sqrtf(square(L) + square(h))) + atanf(L / h);
    elbow_angle = shoulder_angle + acosf(square(UPPERARM) + square(FORARM) - (square(L) + square(h)) / 2 * UPPERARM * FORARM);
  }
  else // z == BASEHEIGHT默认为0
  {
    shoulder_angle = acosf(square(UPPERARM) + square(L) - square(FORARM) / 2 * UPPERARM * L);
    elbow_angle = shoulder_angle + acosf(square(UPPERARM) + square(FORARM) - square(L) / 2 * UPPERARM * FORARM);
  }

  // 输出给舵机
  servo[BASE].MoveTo(ToDegree(base_angle));
  servo[SHOULDER].MoveTo(ToDegree(shoulder_angle));
  servo[ELBOW].MoveTo(ToDegree(elbow_angle));
}