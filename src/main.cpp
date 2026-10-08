#include <Arduino.h>
#include <Servo.h>
#include <Wire.h> //用于arduino之间的I2C通信

#include "ArmServo.h"
#include "Timer.h"
#include "statusRecord.h"

/*————————————————————————————————————————————————————————————————————————————————
封装类：
1.ArmServo可以直接调用MoveTo()实现目标角度设置
2.Claw类继承ArmServo
3.Timer用于舵机实时角度更新
4.statusRecord用于录制时存储路径坐标点

函数：
1.传感器读取
2.机械臂运动控制
3.串口命令
4.记录&播放
5.arduino I2C通信

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
const bool joystickMode = true; // true:摇杆模式，false:命令模式
// 舵机初始化:(引脚)
// 参数：pin, min_angle, max_angle, init_angle
ArmServo base{9, 0, 180, 90};      // 底座舵机
ArmServo shoulder{8, 20, 115, 20}; // 大臂舵机
ArmServo elbow{7, 50, 130, 70};    // 小臂舵机
ArmServo claw{6, 56, 146, 56};     // 钳子舵机,此处Claw类继承ArmServo类

// 计时器初始化
Timer looptimer;   // loop计时器
Timer recordtimer; // 录制计时器，0.5s一次
Timer playtimer;   // 播放计时器，0.5s一次
Timer serialtimer; // 串口计时器，0.5s一次
// 从串口读取命令
char cmd[16] = {0};
uint8_t index = 0;
void SerialReadCommand();
// 传感器读取
void sensorRead(); // 未实现
// 机械臂运动控制
const float FORARM = 10;                              // 小臂/cm
const float UPPERARM = 10;                            // 大臂
const float BASEHEIGHT = 4;                           // 底座高度
const float CLAWLEN = 4;                              // 钳子长度
void MoveToPosition(int16_t x, int16_t y, int16_t z); // 输入目标坐标->舵机角度（机械臂逆运动解）
void returnToInitPosition();                          // 归中

// 夹取，常量定义不同物体的不用钳子需求角度
const uint8_t EMPTY = 150;
const uint8_t CUBE = 60;
const uint8_t BALL = 45;
const uint8_t COIN = 40;
// 夹取，三种物体
// 录制&播放
uint8_t temp_x = 0, temp_y = 0, temp_z = 0, temp_clawAngle = 0;      // 用于
bool isrecord = false;                                               // 由摇杆模块上的按钮控制,是否录制
bool isplay = false;                                                 // 由摇杆模块上的按钮控制,是否播放
statusRecord strd{};                                                 // 可录制60s，每秒取样3次，60 * 3 * 7字节 = 1260字节（具体采样频率根据具体测试再定）
void startrecord();                                                  // 由摇杆按钮控制调用,进行录制前的初始化
void record(uint16_t x, uint16_t y, uint16_t z, uint8_t claw_angle); // loop中定时调用
void play();                                                         // 播放记录，由摇杆按钮控制调用
                                                                     // arduino I2C通信

//  —————————————————
//  ———— SetUp ——————
//  —————————————————
void setup()
{
  // 传感器引脚初始化
  pinMode(A0, INPUT); // 左摇杆左右
  pinMode(A1, INPUT); // 左摇杆上下
  pinMode(A2, INPUT); // 右摇杆左右
  pinMode(A3, INPUT); // 右摇杆上下
  // pinMode(2, INPUT);  // 按钮1
  // pinMode(3, INPUT);  // 按钮2

  Serial.println("正在进行初始化......");
  Serial.begin(9600);
  // 舵机初始化
  base.init();
  shoulder.init();
  elbow.init();
  claw.init();

  delay(2000);
}
//  —————————————————
//  ————— Loop ——————
//  —————————————————
// 每次循环刷新舵机角度，实现多舵机同时运动
void loop()
{
  // 限位：小臂角>大臂角-80
  int8_t limit = 90 - shoulder.getCurrentAngle();
  limit = limit > 0 ? limit : 0; // 限位角度不能小于0,否则取0
  elbow.setLimitation(limit, 130);
  //  刷新舵机角度
  float Ts = looptimer.getTimeInterval(); // Ts单位为秒
  base.update(Ts);
  shoulder.update(Ts);
  elbow.update(Ts);
  claw.update(Ts);

  delay(15); // 延时15ms，防止舵机刷新过快，导致舵机抖动

  // 判断控制模式
  if (joystickMode == true) // 摇杆模式
  {
    sensorRead();
    // 判断是否进入录制模式，前提要进入摇杆模式
    if (isrecord == true)
    {
      if (recordtimer.isIntervalEnough(0.3333f)) // 单位:秒，间隔0.33s进行一次坐标记录
      {
        record(base.getCurrentAngle(), shoulder.getCurrentAngle(), elbow.getCurrentAngle(), claw.getCurrentAngle());
      }
    }
    if (isplay == true)
    {
      if (playtimer.isIntervalEnough(0.3333f)) // 单位:秒，间隔0.33s进行一次坐标记录
      {
        Serial.print(".");
        play();
      }
    }
  }
  // 每0.5秒读取一次串口命令，防止主循环过快，未接收完
  if (serialtimer.isIntervalEnough(0.5f))
  {
    SerialReadCommand();
  }
  // 测试代码 ↓↓↓
  /*
  Serial.print("base:");
  Serial.print(base.getCurrentAngle());
  Serial.print("shoulder:");
  Serial.print(shoulder.getCurrentAngle());
  Serial.print("elbow:");
  Serial.print(elbow.getCurrentAngle());
  Serial.print("claw:");
  Serial.println(claw.getCurrentAngle());
  */
}
//  —————————————————
//  ———— 函数实现 ———
//  —————————————————

void SerialReadCommand()
{
  static bool cmdReady = false; // 设置一个标志位，表示命令是否接收完整，防止主循环快，未接收完
  while (Serial.available())
  {
    char c = Serial.read();

    if (c == '\n')
    {
      cmd[index] = '\0';
      Serial.print("接收命令：");
      Serial.println(cmd);
      index = 0;
      cmdReady = true;
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
  if (!cmdReady) // 是否接收完整
    return;
  cmdReady = false;
  // 命令判断：
  switch (cmd[0])
  {
  case 'O':
    //  上位机发送 “O”：机械臂爪子张开
    Serial.println("机械臂爪子张开");
    claw.MoveTo(40);
    break;
  case 'S':
    //  上位机发送 “S”：机械臂爪子关闭
    Serial.println("机械臂爪子关闭");
    claw.MoveTo(180);
    break;
  case 'H':
    //  上位机发送 “H”：提升机械臂整体运行速度
    base.addSpeed();
    shoulder.addSpeed();
    elbow.addSpeed();
    claw.addSpeed();
    break;
  case 'L':
    //  上位机发送 “L”：降低机械臂整体运行速度
    base.lowSpeed();
    shoulder.lowSpeed();
    elbow.lowSpeed();
    claw.lowSpeed();
    break;
  case 'A':
    // 硬币
    MoveToPosition(10, 10, 10);
    // claw.catchEntity(EMPTY);
    MoveToPosition(100, 100, 100);
    // claw.release();
    break;
  case 'B':
    break;
  case 'C':
    break;

  case 'R':
    // record
    startrecord();
    isrecord = true;
    Serial.println("开始录制");
    break;
  case 'E':
    // end record
    isrecord = false;
    Serial.println("结束录制");
    break;
  case 'P':
    // play
    isplay = true;
    Serial.print("开始播放");
    break;

  case 'x':
    // 检测到第一位是x，就分别取出x10,y120,z180后的数字，并赋给舵机目标
    {
      int x = 0, y = 0, z = 0;
      if (sscanf(cmd, "x%d,y%d,z%d", &x, &y, &z) == 3)
      {
        /*
        Serial.print("接收坐标：");
        Serial.print(x);
        Serial.print(",");
        Serial.print(y);
        Serial.print(",");
        Serial.println(z);
        */
        base.MoveTo(x);
        shoulder.MoveTo(y);
        elbow.MoveTo(z);
        break;
      }
    }
  }
  memset(cmd, '\0', sizeof(cmd)); // 清空命令数组
  index = 0;                      // 重置索引
}

void sensorRead()
{
  if(!isplay) // 只有在非播放模式下，才读取摇杆信号，防止播放时被摇杆信号覆盖
  {
    // 摇杆模拟信号读取x，y, z, claw_angle
    float dx = -(analogRead(A1) - 513) / 1023.0f * shoulder.getSpeed() / 5;     // deltax
    float dy = -(analogRead(A0) - 513) / 1023.0f * base.getSpeed() / 5;         // deltay
    float dz = (analogRead(A3) - 513) / 1023.0f * elbow.getSpeed() / 5;         // deltaz
    float dclaw_angle = (analogRead(A2) - 513) / 1023.0f * claw.getSpeed() / 5; // 钳子角度
    // 传输给舵机，用MoveTo设置目标角度
    shoulder.MoveTo(shoulder.getCurrentAngle() + dx);
    base.MoveTo(base.getCurrentAngle() + dy);
    elbow.MoveTo(elbow.getCurrentAngle() + dz);
    claw.MoveTo(claw.getCurrentAngle() + dclaw_angle);
  }
  // 按钮，调控 recordMode & isplay
  // uint8_t button1 = digitalRead(2); // 按钮1
  // uint8_t button2 = digitalRead(3); // 按钮2
}

void MoveToPosition(int16_t x, int16_t y, int16_t z)
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
  base.MoveTo(ToDegree(base_angle));
  shoulder.MoveTo(ToDegree(shoulder_angle));
  elbow.MoveTo(ToDegree(elbow_angle));
}

void returnToInitPosition()
{
  base.MoveTo(base.getInitAngle());
  shoulder.MoveTo(shoulder.getInitAngle());
  elbow.MoveTo(elbow.getInitAngle());
  claw.MoveTo(claw.getInitAngle());
}

void startrecord()
{
  // 清零上次录制
  strd.clear();
}

inline void record(uint16_t x, uint16_t y, uint16_t z, uint8_t claw_angle)
{
  // 每1/2秒记录三舵机角度&钳子状态
  uint8_t res = strd.push_back(x, y, z, claw_angle);
  if (res == 0) // 录制满了，结束录制
  {
    isrecord = false;
  }
}

inline void play()
{
  // 从statusRecord中取出一位数据
  status sta = strd.read();
  /*
  Serial.print("播放数据点：");
  Serial.print(sta.x);
  Serial.print(",");
  Serial.print(sta.y);
  Serial.print(",");
  Serial.print(sta.z);
  Serial.print(",");
  Serial.println(sta.claw_angle);
  */
  if (sta.x == 0 && sta.y == 0 && sta.z == 0 && sta.claw_angle == 0)
  {
    isplay = false;
    Serial.println("播放结束");
    return;
  }
  base.MoveTo(sta.x);          // 控制底座舵机
  shoulder.MoveTo(sta.y);      // 控制大臂舵机
  elbow.MoveTo(sta.z);         // 控制小臂舵机
  claw.MoveTo(sta.claw_angle); // 控制钳子开合
}