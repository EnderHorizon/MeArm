#include <Arduino.h>
#include <Servo.h>

#include "ArmServo.h"
#include "Timer.h"
#include "statusRecord.h"
/*————————————————————————————————————————————————————————————————————————————————
封装类：
1.ArmServo可以直接调用MoveTo()实现目标角度设置
2.Timer用于舵机实时角度更新
3.statusRecord用于录制时存储路径坐标点

函数：
1.传感器读取
2.机械臂运动控制
3.串口命令
4.记录&播放

!!!注意：因为要涉及记录，所以数组占用空间较大，需要对内存严格管理（arduino uno只有2kb闪存）
———————————————————————————————————————————————————————————————————————————————————*/

// 弧度制转角度制
inline float ToDegree(float radian)
{
  return radian / 180 * (float)PI;
}
// 平方：两个重载
inline int square(int16_t x)
{
  return x * x;
}
inline float square(float x)
{
  return x * x;
}
// 模式：摇杆/命令
const bool joystickMode = 0;
// 舵机初始化:(引脚)
const uint8_t BASE = 0;
const uint8_t SHOULDER = 1;
const uint8_t ELBOW = 2;
const uint8_t CLAW = 3;
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
uint8_t index = 0;
void SerialReadCommand();
// 传感器读取
void sensorRead(); // 未实现
// 机械臂运动控制
const float FORARM = 10;                           // 小臂/cm
const float UPPERARM = 10;                         // 大臂
const float BASEHEIGHT = 4;                        // 底座高度
const float CLAWLEN = 4;                           // 钳子长度
void catchEntity(int16_t x, int16_t y, int16_t z); // 输入目标坐标->舵机角度（机械臂逆运动解）

// 录制&播放
statusRecord strd; // 可录制60s，每秒取样4次，60 * 4 * 7字节 = 1680字节（具体采样频率根据具体测试再定）
void record();     // 记录运动路线
void execute();    // 播放记录

//  —————————————————
//  ———— SetUp ——————
//  —————————————————
void setup()
{
  Serial.println("正在进行初始化......");
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
  if (joystickMode == true) // 摇杆模式
  {
    sensorRead();
  }
  else // 指令模式
  {
    SerialReadCommand();
  }

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
    int x = strtol(cmd + 1, &end, 10); // strtol（初始位指针，返回数字末位指针，进制）
    int y = strtol(end + 2, &end, 10);
    int z = strtol(end + 2, &end, 10);

    servo[BASE].MoveTo(x);
    servo[SHOULDER].MoveTo(y);
    servo[ELBOW].MoveTo(z);
  }

  memset(cmd, '0', sizeof(cmd));
}

void sensorRead()
{
  // 摇杆模拟信号读取x，y
  // 传输给舵机
}

void catchEntity(int16_t x, int16_t y, int16_t z)
{
  // 具体数学推导公式见手写纸
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

void record()
{
  // 清零上次录制
  // 每1/4秒记录一次坐标点&钳子状态
  // 暂停键
}

void execute()
{
}
