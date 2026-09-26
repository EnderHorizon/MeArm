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

// 舵机初始化:(引脚)
const int BASE = 0;
const int SHOULDER = 1;
const int ELBOW = 2;
const int CLAW = 3;
// 舵机限位

ArmServo servo[4] = {
    ArmServo(1), // 底座舵机
    ArmServo(2), // 大臂舵机
    ArmServo(3), // 小臂舵机
    ArmServo(4)  // 钳子舵机
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
void catchEntity(int x, int y, int z); // 输入坐标->舵机角度

//  —————————————————
//  ———— SetUp ——————
//  —————————————————
void setup()
{
  Serial.begin(115200);
}
//  —————————————————
//  ————— Loop ——————
//  —————————————————
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

void catchEntity(int x, int y, int z)
{
  float shoulder_angle = 0;
  float elbow_angle = 0;
  float base_angle = 0;

  float h = 0;
  float L = sqrtf(square(x) + square(y)) - CLAWLEN;

  base_angle = atanf(y / x);
  if (z > BASEHEIGHT)
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
}

inline float square(float x)
{
  return x * x;
}