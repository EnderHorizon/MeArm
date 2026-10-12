#include <Arduino.h>
#include <Servo.h> //舵机库
#include <Wire.h>  //用于arduino之间的I2C通信

/*————————————————————————————————————————————————————————————————————————————————
封装类：

函数：

!!!注意：因为要涉及记录，所以数组占用空间较大，需要对内存严格管理（arduino uno只有2kb闪存）
———————————————————————————————————————————————————————————————————————————————————*/

// 弧度制转角度制
inline float ToDegree(float radian)
{
  return radian * 180.0f / (float)PI;
}
// 角度制转弧度制
inline float ToRadian(float degree)
{
  return degree * (float)PI / 180.0f;
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

//  ——————————————————————————————————————
//  ——————————————— 封装类 ———————————————
//  ——————————————————————————————————————
class Timer
{
public:
  float lastTs; // 上次调用的时间
  float Ts;     // 时间差
  float nowTs;  // 本次调用的时间
  Timer()
      : lastTs(0), Ts(0), nowTs(0)
  {
  }
  ~Timer() {}
  float getTimeInterval();               // 刷新，并返回每次loop的时间间隔
  bool isIntervalEnough(float interval); // 判断是否达到指定时间间隔，若达到则返回true，并刷新计时器

private:
};

float Timer::getTimeInterval()
{
  if (lastTs == 0)
  {
    nowTs = micros();
    lastTs = nowTs;
    return 0.0f;
  }
  nowTs = micros();
  Ts = (nowTs - lastTs) * 1e-6f;
  lastTs = nowTs;
  return Ts;
}

bool Timer::isIntervalEnough(float interval)
{
  if (lastTs == 0)
  {
    nowTs = micros();
    lastTs = nowTs;
    return false;
  }
  nowTs = micros();
  Ts = (nowTs - lastTs) * 1e-6f;
  if (Ts >= interval)
  {
    lastTs = nowTs;
    return true;
  }
  else
    return false;
}

class ArmServo
{
protected:
  uint8_t pin;
  float target_angle;
  float current_angle;
  uint8_t speed; // 角度每秒（角度制）：每次更新转动小角度 = current_angle ± speed;
  Servo servo{}; // 内封装官方servo库实例

  // 限位
  uint8_t max_angle, min_angle;
  // 初始角度
  uint8_t init_angle;

public:
  ArmServo()
      : pin(0), target_angle(40), current_angle(40), speed(40), max_angle(180), min_angle(40), init_angle(40)
  {
  }
  ArmServo(uint8_t pin)
      : pin(pin), target_angle(40), current_angle(40), speed(40), max_angle(180), min_angle(40), init_angle(40) // 👈speed初始化值修改处
  {
  }
  // 主要采用这个构造函数，同时设置舵机引脚和限位角度
  ArmServo(uint8_t pin, uint8_t min_angle, uint8_t max_angle, uint8_t init_angle)
      : pin(pin), target_angle(init_angle), current_angle(init_angle), speed(40), max_angle(max_angle), min_angle(min_angle), init_angle(init_angle) // 👈speed初始化值修改处
  {
  }
  ~ArmServo() {}

  // 转动函数
  void init(); // 初始化引脚，并使机械臂回到待机位置
  void setLimitation(uint8_t min, uint8_t max);
  void fastturn(uint8_t target); // 用于测试类的可行性，仅封装write(angle)
  void MoveTo(float target);     // 设置目标角度，拆分大角度为小角度
  void update(float Ts);         // 舵机每次Loop中的刷新函数，实现平滑转动，且多个电机同步运行

  void addSpeed(); // 加速
  void lowSpeed(); // 减速
  uint8_t getSpeed();
  float getCurrentAngle() { return current_angle; }        // 获取当前角度
  float getMaxAngle() { return max_angle; }                // 获取最大角度
  float getMinAngle() { return min_angle; }                // 获取最小角度
  void setInitAngle(uint8_t angle) { init_angle = angle; } // 设置初始角度
  uint8_t getInitAngle() { return init_angle; }            // 获取初始角度
};

class Claw : public ArmServo // 钳子：继承舵机大类，添加钳子的功能
{
public:
  Claw()
      : ArmServo(0)
  {
  }
  Claw(uint8_t pin)
      : ArmServo(pin)
  {
  }
  Claw(uint8_t pin, uint8_t min_angle, uint8_t max_angle, uint8_t init_angle)
      : ArmServo(pin, min_angle, max_angle, init_angle)
  {
  }
  ~Claw() {}
  void catchEntity(uint8_t Entity_need_angle); // 钳子夹持，具体角度由常量来
  void release();                              // 张开钳子
};

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

struct status // 因为 AVR 是 8 位单片机，avr-gcc 默认对 int16_t 的对齐要求是 1 字节，所以成员之间不会插入填充字节,总大小为7字节
{
  uint8_t x, y, z;
  uint8_t claw_angle;
  // 默认构造函数
  status()
      : x(0), y(0), z(0), claw_angle(0)
  {
  }
  // 带参构造函数
  status(uint8_t x, uint8_t y, uint8_t z, uint8_t claw_angle)
      : x(x), y(y), z(z), claw_angle(claw_angle)
  {
  }
  // 析构
  ~status() {}
};

class statusRecord // 存储状态的数组
{
private:
  uint8_t len;       // uint8_t为1字节，可存储0~255，数组大小为240，故不会溢出
  uint8_t readindex; // 用于读取的时候记录读取位置
  status array[241];

public:
  statusRecord()
      : len(0), readindex(0), array{} // 初始化，array会自动调用status的默认构造函数
  {
  }
  ~statusRecord() {}
  void clear();  // 清除数据点数组
  void reRead(); // 重新开始播放
  uint8_t push_back(uint8_t x_r, uint8_t y_r, uint8_t z_r, uint8_t claw_angle_r);
  status read();
};

void statusRecord::clear()
{
  for (int i = 0; i < len; ++i) // 遍历清除
  {
    array[i].x = 0;
    array[i].y = 0;
    array[i].z = 0;
    array[i].claw_angle = 0;
  }
  len = 0;
}

void statusRecord::reRead()
{
  readindex = 0;
}

uint8_t statusRecord::push_back(uint8_t x_r, uint8_t y_r, uint8_t z_r, uint8_t claw_angle_r)
{
  // 边界检测,录制时间过长默认丢弃数据
  if (len < 241)
  {
    // 存储录制坐标点&钳子角度
    array[len].x = x_r;
    array[len].y = y_r;
    array[len].z = z_r;
    array[len].claw_angle = claw_angle_r;
    ++len;
    /*
    Serial.print("录制数据点数：");
    Serial.println(len);
    Serial.print("录制数据点：");
    Serial.print(x_r);
    Serial.print(",");
    Serial.print(y_r);
    Serial.print(",");
    Serial.print(z_r);
    Serial.print(",");
    Serial.println(claw_angle_r);
    */
    return 1; // 返回1表示录制成功
  }
  else
  {
    Serial.println("录制数据点数已满,结束录制");
    return 0; // 返回0表示录制满了
  }
}

status statusRecord::read()
{
  if (len == readindex)
  {
    readindex = 0;
    return status(0, 0, 0, 0);
  }
  status sta{array[readindex].x, array[readindex].y, array[readindex].z, array[readindex].claw_angle};
  ++readindex;
  return sta;
}

class Vector
{
public:
  float x, y, z;
  Vector()
      : x(0), y(0), z(0)
  {
  }
  Vector(float x, float y, float z)
      : x(x), y(y), z(z)
  {
  }
  Vector operator+(const Vector &other) const
  {
    return Vector{
        (float)(x + other.x),
        (float)(y + other.y),
        (float)(z + other.z)};
  }
  Vector operator-(const Vector &other) const
  {
    return Vector{
        (float)(x - other.x),
        (float)(y - other.y),
        (float)(z - other.z)};
  }
  Vector operator*(const float num) const
  {
    return Vector{
        (float)(x * num),
        (float)(y * num),
        (float)(z * num)};
  }
  Vector operator/(const float num) const
  {
    return Vector{
        (float)(x / num),
        (float)(y / num),
        (float)(z / num)};
  }
  float modulus()
  {
    return sqrt(square(x) + square(y) + square(z));
  }
};

//  ——————————————————————————————————————
//  ———————————— 函数&全局变量 ————————————
//  ——————————————————————————————————————
// 模式：摇杆/命令
bool joystickMode = true; // 摇杆模式
bool drawMode = false;    // 绘图模式
// 舵机初始化:(引脚)
// 参数：pin, min_angle, max_angle, init_angle
ArmServo base{9, 0, 180, 90};      // 底座舵机
ArmServo shoulder{8, 20, 115, 20}; // 大臂舵机
ArmServo elbow{7, 50, 130, 70};    // 小臂舵机
ArmServo claw{6, 56, 146, 56};     // 钳子舵机,此处Claw类继承ArmServo类

// 计时器初始化
Timer looptimer;   // loop计时器
Timer recordtimer; // 录制计时器，0.25s一次
Timer playtimer;   // 播放计时器，0.25s一次
Timer serialtimer; // 串口计时器，0.5s一次
// 从串口读取命令
char cmd[16] = {0};
uint8_t index = 0;
void SerialReadCommand();
// 传感器读取
void sensorRead(); // 未实现
// 机械臂运动控制
const float FORARM = 8;      // 小臂/cm
const float UPPERARM = 8;    // 大臂
const float BASEHEIGHT = 6;  // 底座高度
const float CLAWLEN = 6.3f;  // 钳子长度
void returnToInitPosition(); // 归中

// 夹取，常量定义不同物体的不用钳子需求角度
const uint8_t EMPTY = claw.getMaxAngle(); // 空物体，钳子张开
const uint8_t CUBE = 60;
const uint8_t BALL = 132;
const uint8_t COIN = 146;
// 夹取，三种物体
void catchA();
void catchB();
void catchC();
// 录制&播放
uint8_t temp_x = 0, temp_y = 0, temp_z = 0, temp_clawAngle = 0;   // 用于
bool isrecord = false;                                            // 由摇杆模块上的按钮控制,是否录制
bool isSampling = false;                                          // 五点取样
bool isplay = false;                                              // 由摇杆模块上的按钮控制,是否播放
statusRecord strd{};                                              // 可录制60s，每秒取样4次，60 * 4 * 4字节 = 960字节（具体采样频率根据具体测试再定）
void startrecord();                                               // 由摇杆按钮控制调用,进行录制前的初始化
void record(uint8_t x, uint8_t y, uint8_t z, uint8_t claw_angle); // loop中定时调用
void play();                                                      // 播放记录，由摇杆按钮控制调用
void interruptPlay();                                             // 由摇杆按钮控制调用,中断播放
void continuePlay();                                              // 由摇杆按钮控制调用,继续播放
void cancelPlay();                                                // 由摇杆按钮控制调用,取消播放
// arduino I2C通信
bool button[4] = {0};               // 用于接收从机发送的四个字节数据,并控制按钮事件
const uint8_t SLAVE_ADDRESS = 0x08; // 从机地址
void receiveData();                 // 接收数据事件: 接收四个字节数据，分别为k1,k2,k3,k4

// 绘图相关函数
Vector samples[7];                            // 5个示教点，2个虚拟点
void sampling();                              // 按键触发采样事件
void createStraightPath();                    // 生成折线
void createSmoothPath();                      // 生成平滑曲线 ，并生成采样点
Vector PosToAngle(float x, float y, float z); // 输入目标坐标->舵机角度（机械臂逆运动解）
Vector AngleToPos(float a, float b, float c); // 舵机角度->目标坐标（机械臂运动正解）

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

  Serial.println("正在进行初始化......");
  Serial.begin(9600);
  // 舵机初始化
  base.init();
  shoulder.init();
  elbow.init();
  claw.init();
  // I2C通信初始化
  Wire.begin();

  delay(2000);
}
//  —————————————————
//  ————— Loop ——————
//  —————————————————
// 每次循环刷新舵机角度，实现多舵机同时运动
void loop()
{
  // 限位：小臂角>大臂角-80
  int8_t limit = 95 - shoulder.getCurrentAngle();
  limit = limit > 0 ? limit : 0; // 限位角度不能小于0,否则取0
  elbow.setLimitation(limit, 130);
  //  刷新舵机角度
  float Ts = looptimer.getTimeInterval(); // Ts单位为秒
  base.update(Ts);
  shoulder.update(Ts);
  elbow.update(Ts);
  claw.update(Ts);

  delay(15); // 延时15ms，防止舵机刷新过快，导致舵机抖动

  // 摇杆模式
  if (joystickMode == true)
  {
    sensorRead();
    // 判断是否进入录制模式，前提要进入摇杆模式
    if (isrecord == true)
    {
      if (recordtimer.isIntervalEnough(0.25f)) // 单位:秒，间隔0.25s进行一次坐标记录
      {
        record(base.getCurrentAngle(), shoulder.getCurrentAngle(), elbow.getCurrentAngle(), claw.getCurrentAngle());
      }
    }
    if (isplay == true)
    {
      if (playtimer.isIntervalEnough(0.25f)) // 单位:秒，间隔0.25s进行一次坐标记录
      {
        Serial.print(".");
        play();
      }
    }
  }
  // 绘图模式
  if (drawMode)
  {
    sensorRead();
    if (isSampling == true)
    {
      sampling();
    }
    if (isplay == true)
    {
      // 定时器
      Serial.print(".");
      play();
    }
  }
  // 每0.5秒读取一次串口命令，防止主循环过快，未接收完
  if (serialtimer.isIntervalEnough(0.5f))
  {
    SerialReadCommand();
  }
  // 测试代码 ↓↓↓

  // Serial.print("base:");
  // Serial.print(base.getCurrentAngle());
  // Serial.print("shoulder:");
  // Serial.print(shoulder.getCurrentAngle());
  // Serial.print("elbow:");
  // Serial.print(elbow.getCurrentAngle());
  // Serial.print("claw:");
  // Serial.println(claw.getCurrentAngle());
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
    break;
  case 'B':
    break;
  case 'C':
    break;

  case 'r':
    // record
    startrecord();
    isrecord = true;
    Serial.println("开始录制");
    break;
  case 'e':
    // end record
    isrecord = false;
    Serial.println("结束录制");
    break;
  case 'p':
    // play
    isplay = true;
    Serial.print("开始播放");
    break;
  case 'i':
    // interrupt play
    interruptPlay();
    break;
  case 'c':
    // continue play
    continuePlay();
    break;
  case 'd':
    if (drawMode)
    {
      drawMode = false;
      Serial.println("推出绘图模式");
    }
    else
    {
      drawMode = true;
      Serial.println("进入绘图模式");
    }
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
  if (!isplay) // 只有在非播放模式下，才读取摇杆信号，防止播放时被摇杆信号覆盖
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
}

Vector PosToAngle(float x, float y, float z) // x∈(-50, 50), y∈(0, 50), z∈(0,25)
{
  // 限制，防止机械臂超出范围
  if (x < -50 || x > 50)
    return Vector(base.getInitAngle(), shoulder.getInitAngle(), elbow.getInitAngle());
  if (y < 0 || y > 50)
    return Vector(base.getInitAngle(), shoulder.getInitAngle(), elbow.getInitAngle());
  if (z < 0 || z > 25)
    return Vector(base.getInitAngle(), shoulder.getInitAngle(), elbow.getInitAngle());
  // 具体数学推导公式见手写纸
  float shoulder_angle = 0;
  float elbow_angle = 0;
  float base_angle = 0;

  float h = 0;
  float L = sqrtf(square(x) + square(y)) - CLAWLEN;

  // 工作空间判断

  base_angle = atan2f(y, x);
  if (z > BASEHEIGHT) // z与底座高度比较
  {
    h = z - BASEHEIGHT;
    shoulder_angle = acosf((square(UPPERARM) + square(L) + square(h) - square(FORARM)) /
                           (2.0f * UPPERARM * sqrtf(square(L) + square(h)))) +
                     atan2f(h, L);
    elbow_angle = shoulder_angle + acosf((square(UPPERARM) + square(FORARM) - (square(L) + square(h))) /
                                         (2.0f * UPPERARM * FORARM));
  }
  else if (z < BASEHEIGHT)
  {
    h = BASEHEIGHT - z;
    shoulder_angle = acosf((square(UPPERARM) + square(L) + square(h) - square(FORARM)) /
                           (2.0f * UPPERARM * sqrtf(square(L) + square(h)))) +
                     atan2f(L, h);
    elbow_angle = shoulder_angle + acosf((square(UPPERARM) + square(FORARM) - (square(L) + square(h))) /
                                         (2.0f * UPPERARM * FORARM));
  }
  else // z == BASEHEIGHT默认为0
  {
    shoulder_angle = acosf((square(UPPERARM) + square(L) - square(FORARM)) /
                           (2.0f * UPPERARM * L));
    elbow_angle = shoulder_angle + acosf((square(UPPERARM) + square(FORARM) - square(L)) /
                                         (2.0f * UPPERARM * FORARM));
  }
  if (sqrt(square(h) + square(L)) < UPPERARM + FORARM && sqrt(square(h) + square(L)) > 8.0f)
  {
    return Vector{ToDegree(base_angle), 135.0f - ToDegree(shoulder_angle), ToDegree(elbow_angle) - 65.0f};
  }
  else
  {
    return Vector(base.getInitAngle(), shoulder.getInitAngle(), elbow.getInitAngle());
  }
}

Vector AngleToPos(float a, float b, float c)
{
  b = 135.0f - b;
  c = c + 65.0f;
  // 角度制转弧度制
  a = ToRadian(a);
  b = ToRadian(b);
  c = ToRadian(c);

  float L = sqrtf(square(UPPERARM) + square(FORARM) - 2.0f * UPPERARM * FORARM * cosf(c - b));
  float angle1 = acosf((square(UPPERARM) + square(L) - square(FORARM)) /
                       (2.0f * UPPERARM * L));
  float angle2 = 0.0f; // 初始化0.0f
  // 分类讨论∠2大小,当∠1 == b时，∠2=0默认值
  if (angle1 < b)
    angle2 = b - angle1;
  else if (angle1 > b)
    angle2 = angle1 - b;
  // 投影
  float D = L * cosf(angle2);
  float h = L * sinf(angle2);

  float x = (D + CLAWLEN) * cosf(a);
  float y = (D + CLAWLEN) * sinf(a);
  float z = BASEHEIGHT + h;

  return Vector(x, y, z);
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

inline void record(uint8_t x, uint8_t y, uint8_t z, uint8_t claw_angle)
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

inline void interruptPlay()
{
  isplay = false;
  Serial.println("播放中断");
}

inline void continuePlay()
{
  isplay = true;
  Serial.println("继续播放");
}

void cancelPlay()
{
  isplay = false;
  strd.reRead(); // 重新开始播放
  Serial.println("取消播放");
}

void receiveData()
{
  // 请求数据
  uint8_t count = Wire.requestFrom(SLAVE_ADDRESS, sizeof(button)); // 请求从机发送数据
  if (count == 4)
  {
    Wire.readBytes((uint8_t *)button, sizeof(button)); // 读取四个字节
    for (auto &b : button)
    {
      Serial.print("接收从机数据：");
      Serial.print(b);
    }
    Serial.println("\n");
  }
}

void sampling()
{
}

void createSmoothPath()
{
  // 由5点生成曲线取样点:  sample1 - 47 - sample2 - 47 - sample3 - 47 - sample4 - 47 - endVec
  // samples首尾生成虚拟点
  samples[0] = samples[1] * 2 - samples[2];
  samples[6] = samples[5] * 2 - samples[4];
  // 计算节点
  float t[7] = {0.0f};
  for (uint8_t i = 0; i < 6; ++i)
  {
    t[i + 1] = t[i] + sqrt((samples[i + 1] - samples[i]).modulus());
  }
  // Hermite插值
  // 计算每点切线（向量）
  Vector tangentline[7];
  tangentline[0] = (samples[1] - samples[0]) / (t[1] - t[0]);
  tangentline[6] = (samples[6] - samples[5]) / (t[6] - t[5]);
  for (uint8_t i = 1; i < 6; ++i)
  {
    // 加权切线估算
    tangentline[i] = (samples[i] - samples[i - 1]) / (t[i] - t[i - 1]) * (t[i + 1] - t[i]) / (t[i + 1] - t[i - 1]) +
                     (samples[i + 1] - samples[i]) / (t[i + 1] - t[i]) * (t[i] - t[i - 1]) / (t[i + 1] - t[i - 1]);
  }
  // t->u映射使不等距的t变为等距的u∈[0,1]

  // 三次Hermite基函数
  const uint8_t sampling_density = 48;
  for (uint8_t i = 0; i < 4; ++i) // 分别计算每一段曲线解析式，并生成采样点存入strd中
  {
    Vector sampleVec = PosToAngle(samples[i + 1].x, samples[i + 1].y, samples[i + 1].z);
    strd.push_back(sampleVec.x, sampleVec.y, sampleVec.z, 0);
    float dt = t[i + 2] - t[i + 1]; // 当前段的节点间隔
    for (uint8_t k = 1; k < sampling_density; ++k)
    {
      float u = (float)k / (float)sampling_density;
      float u2 = u * u;
      float u3 = u2 * u;

      float h00 = 2 * u3 - 3 * u2 + 1;
      float h10 = u3 - 2 * u2 + u;
      float h01 = (-2) * u3 + 3 * u2;
      float h11 = u3 - u2;

      Vector res_loc = samples[i + 1] * h00 +
                       tangentline[i + 1] * h10 * dt +
                       samples[i + 2] * h01 +
                       tangentline[i + 2] * h11 * dt;
      Vector res_angle = PosToAngle(res_loc.x, res_loc.y, res_loc.z);
      strd.push_back(res_angle.x, res_angle.y, res_angle.z, 0);
    }
  }
  Vector endVec = PosToAngle(samples[5].x, samples[5].y, samples[5].z);
  strd.push_back(endVec.x, endVec.y, endVec.z, 0);
}
