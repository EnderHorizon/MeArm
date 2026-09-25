#include <Arduino.h>
#include <Servo.h>

#include "arm_servo.h"
#include "Timer.h"

// 舵机初始化:(引脚)
ArmServo base(1);     // 底座舵机
ArmServo shoulder(2); // 大臂舵机
ArmServo elbow(3);    // 小臂舵机
ArmServo gripper(4);  // 钳子舵机
// 从串口读取命令
char cmd[64] = {0};
size_t index = 0;
void SerialReadCommand(); // 函数声明
// 计时器初始化
Timer timer;

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop()
{
  // put your main code here, to run repeatedly:
  SerialReadCommand();
  // 刷新舵机角度
  float Ts = timer.getTimeInterval(); // Ts单位为秒
  base.update(Ts);
  shoulder.update(Ts);
  elbow.update(Ts);
  gripper.update(Ts);

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
  if (strcmp(cmd, "O"))
  {
  }
  else if (strcmp(cmd, "S"))
  {
  }
  else if (strcmp(cmd, "H"))
  {
  }
  else if (strcmp(cmd, "L"))
  {
  }
  memset(cmd, '0', sizeof(cmd));
}