#include <Arduino.h>
#include <Servo.h> //舵机库
#include <Wire.h>  //用于arduino之间的I2C通信

/*————————————————————————————————————————————————————————————————————————————————
封装类：
Timer计时器
ArmServo对Servo进行了封装，内部记录舵机的状态，实现4舵机同步刷新运动的基本功能（同时运动），速度控制
status结构体用于存储4舵机某时刻的角度
statusRecord是status的数组类，实现类似于队列的功能push_back（），然后read（）
Vector类用于数学计算的向量类，*已重载运算符

函数：

!!!注意：因为要涉及记录，所以数组占用空间较大，需要对内存严格管理（arduino uno只有2kbSRAM）
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
  unsigned long lastTs; // 上次调用的时间（微秒）。必须用 unsigned long 存 micros()，float 精度不够会在 >16.7s 后溢出
  float Ts;             // 时间差（秒）
  unsigned long nowTs;  // 本次调用的时间（微秒）
  Timer()
      : lastTs(0), Ts(0), nowTs(0)
  {
  }
  ~Timer() {}
  float getTimeInterval();               // 刷新，并返回每次loop的时间间隔
  bool isIntervalEnough(float interval); // 判断是否达到指定时间间隔，若达到则返回true，并刷新计时器
  void update() { lastTs = micros(); }

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
  Ts = (float)(nowTs - lastTs) * 1e-6f; // 无符号减法，micros() 回绕时也自然正确
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
  Ts = (float)(nowTs - lastTs) * 1e-6f; // 无符号减法，micros() 回绕时也自然正确
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
      : pin(pin), target_angle(init_angle), current_angle(init_angle), speed(60), max_angle(max_angle), min_angle(min_angle), init_angle(init_angle) // 👈speed初始化值修改处
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
  target_angle = target;
  current_angle = target;
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
    if ((current_angle + speed * Ts) > target_angle) // 限位，防止设置的speed过大，超过目标角度
    {
      servo.write(target_angle);
      current_angle = target_angle;
    }
    else
      servo.write((int)(current_angle += speed * Ts)); // 原理：角度=角速度×时间
  }
  else // speed为负
  {
    if ((current_angle - speed * Ts) < target_angle) // 限位，防止设置的speed过大，超过目标角度
    {
      servo.write(target_angle);
      current_angle = target_angle;
    }
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
  uint8_t len;       // uint8_t为1字节，可存储0~255，数组大小为120，故不会溢出
  uint8_t readindex; // 用于读取的时候记录读取位置
  bool fullNotified; // "已满"只提示一次，clear()时复位
  status array[121];

public:
  statusRecord()
      : len(0), readindex(0), fullNotified(false), array{} // 初始化，array会自动调用status的默认构造函数
  {
  }
  ~statusRecord() {}
  void clear();                                                                // 清除数据点数组
  void reRead();                                                               // 重新开始播放
  void interrupt(uint8_t x_r, uint8_t y_r, uint8_t z_r, uint8_t claw_angle_r); // 中断播放,保存中断位置
  uint8_t push_back(uint8_t x_r, uint8_t y_r, uint8_t z_r, uint8_t claw_angle_r);
  status read();
  uint8_t getNum() { return readindex; } // 获取读取位数，eg第一位
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
  readindex = 0;
  fullNotified = false;
}

void statusRecord::reRead()
{
  readindex = 0;
}

void statusRecord::interrupt(uint8_t x_r, uint8_t y_r, uint8_t z_r, uint8_t claw_angle_r)
{
  if (readindex < 121)
  {
    array[readindex].x = x_r;
    array[readindex].y = y_r;
    array[readindex].z = z_r;
    array[readindex].claw_angle = claw_angle_r;
  }
}

uint8_t statusRecord::push_back(uint8_t x_r, uint8_t y_r, uint8_t z_r, uint8_t claw_angle_r)
{
  // 边界检测,录制时间过长默认丢弃数据
  if (len < 121)
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
    if (!fullNotified) // 只在第一次满时提示，避免刷屏
    {
      Serial.println("录制数据点数已满,结束录制");
      fullNotified = true;
    }
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

// 向量方便公式计算
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
// 模式: 由上位机命令来控制具体模式，模式内部小事件由从机按钮信号控制
bool recordMode = 0; // 录制模式 / 绘图模式

// 舵机初始化:(引脚)
// 参数：pin, min_angle, max_angle, init_angle
ArmServo base{9, 0, 180, 90};      // 底座舵机
ArmServo shoulder{8, 20, 120, 20}; // 大臂舵机
ArmServo elbow{7, 50, 145, 95};    // 小臂舵机（下限95会钳死IK解，实测可到56°，放宽到50）
ArmServo claw{6, 81, 178, 81};     // 钳子舵机,此处Claw类继承ArmServo类

// 计时器初始化
Timer looptimer;   // loop计时器
Timer recordtimer; // 录制计时器，0.25s一次
Timer playtimer;   // 播放计时器，0.25s一次
Timer serialtimer; // 串口计时器，0.5s一次
Timer I2Ctimer;    // I2C计时器，0.1s一次
Timer catchingtimer;

// 从串口读取命令
char cmd[12] = {0};
uint8_t index = 0;
void SerialReadCommand();
// 传感器读取
void sensorRead();

// 机械臂运动控制
const float FORARM = 8;        // 小臂/cm
const float UPPERARM = 8;      // 大臂
const float BASEHEIGHT = 6;    // 底座高度
const float CLAWLEN = 6.3f;    // 钳子长度
uint8_t PENHEIGHT = 5;         // 落笔高度
const float CLAWOFFSET = 0.4f; // 爪子与中轴线的偏移量（俯视：摆平面与底座转轴的侧向距离 w，实测）
const float BASE_ZERO = 4.64f; // 底座舵机 0° 零位补偿：底座角 0° 时臂指向与 x 轴的夹角（两点法标定，单位度）

void returnToInitPosition(); // 归中

// 夹取，常量定义不同物体的不用钳子需求角度
const uint8_t PEN = 178;
const uint8_t COIN = 178;

// 事件定义 ： 用于按钮触发
bool SAMPLING = false; // 只有在isSampling才能触发sampling事件

// 夹取，三种物体
// 基本思路也是，用数组控制夹取路线
void catchA();
void catchB();
void catchC();
void takePen();

// 录制&播放
// 录制与播放步骤；1.录制 2.play由摇杆模块上的按钮控制
bool isrecord = false;                                            // ,是否录制
bool isSampling = false;                                          // 五点取样
bool isplay = false;                                              // 由摇杆模块上的按钮控制,是否播放
bool iscatching = false;                                          // 用于play的时候判断，是夹取物品还是播放
statusRecord strd{};                                              // 120 点 × 4 字节 = 480 字节（0.25s 取样一次时约 30s；采样密度决定单条路径点数）
void startrecord();                                               // 由摇杆按钮控制调用,进行录制前的初始化
void record(uint8_t x, uint8_t y, uint8_t z, uint8_t claw_angle); // loop中定时调用
void play();                                                      // 播放记录，由摇杆按钮控制调用
void interruptPlay();                                             // 由摇杆按钮控制调用,中断播放
void continuePlay();                                              // 由摇杆按钮控制调用,继续播放
void cancelPlay();                                                // 由摇杆按钮控制调用,取消播放

// 绘图
// 绘图步骤：1.取笔 2.示教点's' 3.选择生成路径'1'/'2' 4.play'p' 5.可选择是否重置
Vector samples[7];                            // 5个示教点，2个虚拟点
void sampling();                              // 按键触发采样事件
void createLine();                            // 画直线
void createLetter();                          // 画字母
void createStraightPath();                    // 生成折线
void createSmoothPath();                      // 生成平滑曲线 ，并生成采样点
Vector PosToAngle(float x, float y, float z); // 输入目标坐标->舵机角度（机械臂逆运动解）
Vector AngleToPos(float a, float b, float c); // 舵机角度->目标坐标（机械臂运动正解）

// arduino I2C通信
// drawMode: k1->夹取笔 k2->取样 k3->暂停&继续 k4->取消
// recordMode: k1->夹取物体 k2->录制&停止录制 k3->播放 k4->回中
bool button[2][4] = {0};            // 用于接收从机发送的四个字节数据,并控制按钮事件
const uint8_t SLAVE_ADDRESS = 0x08; // 从机地址
void receiveData();                 // 接收数据事件: 接收四个字节数据，分别为k1,k2,k3,k4

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

  Serial.begin(9600);
  Serial.println("正在进行初始化......");
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
void loop()
{
  // 限位：小臂角>大臂角-115
  if (isplay)
  {
    elbow.setLimitation(40, 145);
  }
  else
  {
    int limit = 115 - shoulder.getCurrentAngle();
    limit = limit > 0 ? limit : 0; // 限位角度不能小于0,否则取0
    elbow.setLimitation(limit, 145);
  }
  // 每次循环刷新舵机角度，实现多舵机同时运动
  float Ts = looptimer.getTimeInterval(); // Ts单位为秒
  base.update(Ts);
  shoulder.update(Ts);
  elbow.update(Ts);
  claw.update(Ts);

  delay(10);
  // 录制模式
  sensorRead();
  if (recordMode == true)
  {
    if (isrecord == true)
    {
      if (recordtimer.isIntervalEnough(0.25f)) // 单位:秒，间隔0.25s进行一次坐标记录
      {
        record(base.getCurrentAngle(), shoulder.getCurrentAngle(), elbow.getCurrentAngle(), claw.getCurrentAngle());
      }
    }
    if (isplay == true)
    {
      if (iscatching == true)
      {
        if (catchingtimer.isIntervalEnough(1.5f))
        {
          play();
        }
      }
      else if (playtimer.isIntervalEnough(0.25f)) // 单位:秒，间隔0.25s播放一个记录点
      {
        Serial.print(".");
        play();
      }
    }
  }
  // 绘图模式
  else
  {
    if (SAMPLING == true) // 事件触发
    {
      // 拿笔，摇杆操控移动到示教点，按钮按下执行记录
      sampling();
      SAMPLING = false;
    }
    if (isplay == true)
    {
      if (iscatching == true)
      {
        if (catchingtimer.isIntervalEnough(1.5f))
        {
          play();
        }
      }
      else if (strd.getNum() == 0)
      {
        play();
      }
      else if (strd.getNum() == 1)
      {
        if (playtimer.isIntervalEnough(2.0f))
        {
          play();
        }
      }
      else if (playtimer.isIntervalEnough(0.005f)) // 单位:秒，间隔0.25s播放一个记录点
      {
        Serial.print(".");
        play();
      }
    }
  }
  // 每0.5秒读取一次串口命令，防止主循环过快，未接收完
  if (serialtimer.isIntervalEnough(0.5f))
  {
    // 测试代码 ↓↓↓
    SerialReadCommand();
  }
  // Serial.print("angle:");
  // Serial.print(base.getCurrentAngle());
  // Serial.print("|");
  // Serial.print(shoulder.getCurrentAngle());
  // Serial.print("|");
  // Serial.print(elbow.getCurrentAngle());
  // Serial.print("|");
  // Serial.println(claw.getCurrentAngle());
  // Serial.print("||");
  // Serial.print("position:");
  //  Vector testvec = AngleToPos(base.getCurrentAngle(), shoulder.getCurrentAngle(), elbow.getCurrentAngle());
  //  Serial.print(testvec.x);
  //  Serial.print(",");
  //  Serial.print(testvec.y);
  //  Serial.print(",");
  //  Serial.println(testvec.z);
  //  每100ms向从机获取按钮数据:防止读取多读频繁中断阻塞
  if (I2Ctimer.isIntervalEnough(0.2f))
  {
    receiveData();
  }
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
    for (uint8_t i = claw.getCurrentAngle(); i > 81; --i)
    {
      claw.fastturn(i);
      delay(10);
    }
    break;
  case 'S':
    //  上位机发送 “S”：机械臂爪子关闭
    Serial.println("机械臂爪子关闭");
    for (uint8_t i = claw.getCurrentAngle(); i < 178; ++i)
    {
      claw.fastturn(i);
      delay(10);
    }
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
    catchA();
    break;
  case 'B':
    catchB();
    break;
  case 'C':
    catchC();
    break;
  case 'P':
    takePen();
    break;
  case 'r':
    // record
    if (recordMode)
    {
      startrecord();
      isrecord = true;
      Serial.println("开始录制");
    }
    break;
  case 'e':
    // end record
    if (recordMode)
    {
      isrecord = false;
      Serial.println("结束录制");
    }
    break;
  case 'p':
    // play
    if (!isrecord)
    {
      isplay = true;
      Serial.print("开始播放");
    }
    else
    {
      Serial.println("请先结束录制");
    }
    break;
  case 'i':
    // interrupt play
    if (recordMode)
      break;
    interruptPlay();
    break;
  case 'c':
    // continue play
    if (recordMode)
      break;
    continuePlay();
    break;
  case 'm':
    if (!isplay && !isrecord)
    {
      if (recordMode)
      {
        recordMode = false;
        Serial.println("绘图模式");
      }
      else
      {
        recordMode = true;
        Serial.println("录制模式");
      }
    }
    else
    {
      Serial.println("请先结束播放或录制");
    }
    break;
  case 's':
    SAMPLING = true;
    break;
  case '1': // 折线
    createStraightPath();
    break;
  case '2': // 曲线
    createSmoothPath();
    break;
  case '3': // 直线
    createLine();
    break;
  case '4': // 字母
    createLetter();
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
        strd.clear();
        for (uint8_t i = 0; i < 40; ++i)
        {
          strd.push_back(x, y, z, claw.getCurrentAngle());
        }
        isplay = true;
        break;
      }
    }
  }
  memset(cmd, '\0', sizeof(cmd)); // 清空命令数组
  index = 0;                      // 重置索引
}

void sensorRead()
{
  if (!isplay) // 只有在非播放情况下，才读取摇杆信号，防止播放时被摇杆信号覆盖
  {
    // 摇杆模拟信号读取x，y, z, claw_angle
    int rawA1 = analogRead(A1);
    int rawA0 = analogRead(A0);
    int rawA3 = analogRead(A3);
    int rawA2 = analogRead(A2);
    float dx = (abs(rawA1 - 513) < 20) ? 0.0f : -(rawA1 - 513) / 1023.0f * shoulder.getSpeed() / 5;     // deltax
    float dy = (abs(rawA0 - 513) < 20) ? 0.0f : -(rawA0 - 513) / 1023.0f * base.getSpeed() / 5;         // deltay
    float dz = (abs(rawA3 - 513) < 20) ? 0.0f : (rawA3 - 513) / 1023.0f * elbow.getSpeed() / 5;         // deltaz
    float dclaw_angle = (abs(rawA2 - 513) < 20) ? 0.0f : (rawA2 - 513) / 1023.0f * claw.getSpeed() / 5; // 钳子角度
    // 传输给舵机，用MoveTo设置目标角度
    shoulder.MoveTo(shoulder.getCurrentAngle() + dx);
    base.MoveTo(base.getCurrentAngle() + dy);
    elbow.MoveTo(elbow.getCurrentAngle() + dz);
    claw.MoveTo(claw.getCurrentAngle() + dclaw_angle);
  }
}

const float minR = 11;
const float maxR = 22.5;

Vector PosToAngle(float x, float y, float z)
{
  // 工作空间判断：形状为半圆环
  float distance = sqrtf(square(x) + square(y));
  if (distance < minR || distance > maxR)
  {
    Serial.println("PosToAngle角度超出限制!!!");
    return Vector(base.getInitAngle(), shoulder.getInitAngle(), elbow.getInitAngle());
  }
  // 具体数学推导公式见手写纸
  float shoulder_angle = 0;
  float elbow_angle = 0;
  float base_angle = 0;

  float h = 0;
  // 消除肩宽：直角三角形
  float L = sqrtf(square(distance) - square(CLAWOFFSET)) - CLAWLEN;

  base_angle = atan2f(y, x) + asinf(CLAWOFFSET / distance) + ToRadian(BASE_ZERO);
  h = z - BASEHEIGHT;
  shoulder_angle = acosf((square(UPPERARM) + square(L) + square(h) - square(FORARM)) /
                         (2.0f * UPPERARM * sqrtf(square(L) + square(h)))) +
                   atan2f(h, L);
  elbow_angle = shoulder_angle + acosf((square(UPPERARM) + square(FORARM) - (square(L) + square(h))) /
                                       (2.0f * UPPERARM * FORARM));

  return Vector{ToDegree(base_angle), 122.0f - ToDegree(shoulder_angle), ToDegree(elbow_angle) - 50.0f};
}
Vector AngleToPos(float a, float b, float c)
{
  b = 122.0f - b;
  c = c + 50.0f;
  // 角度制转弧度制
  b = ToRadian(b);
  c = ToRadian(c);

  float L = sqrtf(square(UPPERARM) + square(FORARM) - 2.0f * UPPERARM * FORARM * cosf(c - b));
  float angle1 = acosf((square(UPPERARM) + square(L) - square(FORARM)) /
                       (2.0f * UPPERARM * L));
  float angle2 = b - angle1;
  // 投影
  float D = L * cosf(angle2);
  float h = L * sinf(angle2);

  a = ToRadian(a - BASE_ZERO) - atan2f(CLAWOFFSET, D + CLAWLEN);

  float Length = sqrtf(square(CLAWOFFSET) + square(D + CLAWLEN));
  float x = Length * cosf(a);
  float y = Length * sinf(a);
  float z = BASEHEIGHT + h;

  return Vector(x, y, z);
}

void returnToInitPosition()
{
  strd.clear();
  strd.push_back(base.getInitAngle(), shoulder.getInitAngle(), elbow.getInitAngle(), claw.getInitAngle());
  isplay = true;
}

void catchB()
{
  strd.clear();
  strd.push_back(160, 40, 90, 81); // 预设夹取路径
  strd.push_back(160, 106, 85, 81);
  strd.push_back(160, 106, 85, PEN);
  strd.push_back(159, 40, 90, PEN);
  strd.push_back(78, 40, 90, PEN);
  strd.push_back(78, 106, 85, PEN);
  strd.push_back(78, 106, 85, 81);
  strd.push_back(base.getInitAngle(), shoulder.getInitAngle(), elbow.getInitAngle(), 81);

  catchingtimer.update();
  iscatching = true;
  isplay = true;
}

void catchA()
{
  strd.clear();
  strd.push_back(129, 40, 90, 81); // 预设夹取路径
  strd.push_back(129, 112, 81, 81);
  strd.push_back(129, 112, 81, COIN);
  strd.push_back(129, 40, 90, COIN);
  strd.push_back(20, 40, 90, COIN);
  strd.push_back(20, 112, 81, COIN);
  strd.push_back(20, 115, 85, 81);
  strd.push_back(base.getInitAngle(), shoulder.getInitAngle(), elbow.getInitAngle(), 81);

  catchingtimer.update();
  iscatching = true;
  isplay = true;
}

void catchC()
{
  strd.clear();
  strd.push_back(112, 40, 90, 81); // 预设夹取路径
  strd.push_back(112, 111, 85, 81);
  strd.push_back(112, 111, 85, COIN);
  strd.push_back(112, 40, 90, COIN);
  strd.push_back(50, 40, 90, COIN);
  strd.push_back(50, 111, 85, COIN);
  strd.push_back(50, 111, 85, 81);
  strd.push_back(base.getInitAngle(), shoulder.getInitAngle(), elbow.getInitAngle(), 81);

  catchingtimer.update();
  iscatching = true;
  isplay = true;
}

void takePen()
{
  strd.clear();
  strd.push_back(159, 40, 90, 81); // 预设夹取路径
  strd.push_back(159, 90, 85, 81);
  strd.push_back(159, 90, 85, PEN);
  strd.push_back(159, 40, 90, PEN);
  strd.push_back(90, 40, 90, PEN);
  Vector getHeight = AngleToPos(160, 106, 85);
  PENHEIGHT = getHeight.z;

  catchingtimer.update();
  iscatching = true;
  isplay = true;
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
  if (sta.x == 0 && sta.y == 0 && sta.z == 0 && sta.claw_angle == 0)
  {
    isplay = false;
    iscatching = false;
    Serial.println("播放结束");
    return;
  }
  base.MoveTo(sta.x);          // 控制底座舵机
  shoulder.MoveTo(sta.y);      // 控制大臂舵机
  elbow.MoveTo(sta.z);         // 控制小臂舵机
  claw.MoveTo(sta.claw_angle); // 控制钳子开合
  playtimer.update();
}

inline void interruptPlay()
{
  isplay = false;
  strd.interrupt(base.getCurrentAngle(), shoulder.getCurrentAngle(), elbow.getCurrentAngle(), claw.getCurrentAngle());
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
  returnToInitPosition();
  Serial.println("取消播放");
}

void receiveData()
{
  // 请求数据
  uint8_t count = Wire.requestFrom(SLAVE_ADDRESS, 4); // 请求从机发送数据
  if (count == 4)
  {
    // 前一按钮状态右移4字节
    for (uint8_t i = 0; i < 4; ++i)
    {
      button[1][i] = button[0][i];
    }
    Wire.readBytes((uint8_t *)button, 4); // 读取四个字节
    Serial.println("\n");
  }
  else
  {
    return;
  }

  // recordMode: k1->夹取物体abc k2->录制&停止录制 k3->播放 k4->回中
  // drawMode: k1->夹取笔 k2->取样 k3->暂停&继续 k4->取消
  if (recordMode == true)
  {
    if (!isplay) // 非播放中
    {
      if (button[1][0] == false && button[0][0] == true) // 按钮1按下:夹取物体abc
      {
        static uint8_t abcindex = 0;
        if (abcindex == 0)
        {
          catchA();
          abcindex++;
        }
        else if (abcindex == 1)
        {
          catchB();
          abcindex++;
        }
        else
        {
          catchC();
          abcindex = 0;
        }
      }
      if (button[1][1] == false && button[0][1] == true) // 按钮2按下:录制&停止录制
      {
        if (isrecord)
        {
          isrecord = false;
          Serial.println("结束录制");
        }
        else
        {
          startrecord();
          isrecord = true;
          Serial.println("开始录制");
        }
      }
      if (button[1][2] == false && button[0][2] == true) // 按钮3按下:播放
      {
        if (isrecord)
        {
          Serial.print("请先退出录制");
        }
        else
        {
          isplay = true;
          Serial.print("开始播放");
        }
      }
    }
    if (button[1][3] == false && button[0][3] == true) // 按钮4按下:回中
    {
      returnToInitPosition();
      Serial.println("归中");
    }
  }
  else
  {
    if (button[1][3] == false && button[0][3] == true) // 按钮4按下:取消
    {
      cancelPlay();
      Serial.println("取消播放");
    }
    if (isplay)
    {
      if (button[1][2] == false && button[0][2] == true) // 按钮3按下:暂停
      {
        interruptPlay();
        Serial.println("暂停播放");
      }
    }
    else
    {
      if (button[1][0] == false && button[0][0] == true) // 按钮1按下:夹取笔
      {
        takePen();
        Serial.println("拿笔");
      }
      if (button[1][1] == false && button[0][1] == true) // 按钮2按下:取样
      {
        SAMPLING = true;
        Serial.println("取样");
      }
      if (button[1][2] == false && button[0][2] == true) // 按钮3按下:继续播放
      {
        continuePlay();
        Serial.println("继续播放");
      }
    }
  }
}

void sampling()
{
  static uint8_t index = 0; // 函数静态变量
  if (index >= 5)
  {
    Serial.println("示教点已满,再按一次重新开始示教");
    index = 0;
    return;
  }
  // 获取舵机当前角度，并调用逆运算
  Vector pos = AngleToPos(base.getCurrentAngle(), shoulder.getCurrentAngle(), elbow.getCurrentAngle());
  pos.z = PENHEIGHT;        // 固定落笔高度
  samples[index + 1] = pos; // 从1开始填充至5
  ++index;
}

void createLine()
{
  // 设置samples，调用生成折线路径
  samples[5] = Vector(0, 13, PENHEIGHT);
  samples[4] = Vector(0, 14, PENHEIGHT);
  samples[3] = Vector(0, 16, PENHEIGHT);
  samples[2] = Vector(0, 18, PENHEIGHT);
  samples[1] = Vector(0, 19, PENHEIGHT);
  createStraightPath();
}

void createLetter() // 字母“Z”
{
  samples[1] = Vector(-3, 15, PENHEIGHT);
  samples[2] = Vector(-3, 19, PENHEIGHT);
  samples[3] = Vector(0, 17, PENHEIGHT);
  samples[4] = Vector(3, 15, PENHEIGHT);
  samples[5] = Vector(3, 19, PENHEIGHT);
  createStraightPath();
}

void createStraightPath()
{
  strd.clear();
  const uint8_t sampling_density = 30; // 每条线段分成 48 段

  // 数据点结构：sample1 - 47 - sample2 - 47 - sample3 - 47 - sample4 - 47 - sample5
  for (uint8_t i = 1; i < 5; ++i) // 5 个采样点 -> 4 条线段
  {
    const float x0 = samples[i].x, y0 = samples[i].y;
    const float x1 = samples[i + 1].x, y1 = samples[i + 1].y;

    // k = 0 .. 47，每段只推到终点前一个点，终点留给下一段起点（最后单独补）
    for (uint8_t k = 0; k < sampling_density; ++k)
    {
      // 参数化插值（t 从 0 到 1）
      float t = (float)k / sampling_density; // 0, 1/48, 2/48, ... 47/48
      float x = x0 + (x1 - x0) * t;
      float y = y0 + (y1 - y0) * t;

      Vector angle = PosToAngle(x, y, PENHEIGHT);
      strd.push_back(angle.x, angle.y, angle.z, PEN);
    }
  }

  // 补上最后一个采样点（第 5 个点）
  Vector endVec = PosToAngle(samples[5].x, samples[5].y, PENHEIGHT);
  strd.push_back(endVec.x, endVec.y, endVec.z, PEN);
  Serial.println("折线生成完成，可以播放！");
}

void createSmoothPath()
{
  strd.clear();
  // 由5点生成曲线取样点:  sample1 - 47 - sample2 - 47 - sample3 - 47 - sample4 - 47 - endVec
  // samples首尾生成虚拟点
  // 旧版本：
  // samples[0] = samples[1] * 2 - samples[2];
  // samples[6] = samples[5] * 2 - samples[4];
  // 新版本：分量式
  samples[0].x = 2 * samples[1].x - samples[2].x;
  samples[0].y = 2 * samples[1].y - samples[2].y;
  samples[0].z = 2 * samples[1].z - samples[2].z;

  samples[6].x = 2 * samples[5].x - samples[4].x;
  samples[6].y = 2 * samples[5].y - samples[4].y;
  samples[6].z = 2 * samples[5].z - samples[4].z;
  // 计算节点
  float t[7] = {0.0f};
  for (uint8_t i = 0; i < 6; ++i)
  {
    // 旧版：
    // t[i + 1] = t[i] + sqrt((samples[i + 1] - samples[i]).modulus());
    float dx = samples[i + 1].x - samples[i].x;
    float dy = samples[i + 1].y - samples[i].y;
    float dz = samples[i + 1].z - samples[i].z;
    t[i + 1] = t[i] + sqrtf(dx * dx + dy * dy + dz * dz);
  }
  // Hermite插值
  // 计算每点切线（向量）
  Vector tangentline[7];
  tangentline[0].x = (samples[1].x - samples[0].x) / (t[1] - t[0]);
  tangentline[0].y = (samples[1].y - samples[0].y) / (t[1] - t[0]);
  tangentline[0].z = (samples[1].z - samples[0].z) / (t[1] - t[0]);
  tangentline[6].x = (samples[6].x - samples[5].x) / (t[6] - t[5]);
  tangentline[6].y = (samples[6].y - samples[5].y) / (t[6] - t[5]);
  tangentline[6].z = (samples[6].z - samples[5].z) / (t[6] - t[5]);
  for (uint8_t i = 1; i < 6; ++i)
  {
    // 加权切线估算
    // 旧版：
    // tangentline[i] = (samples[i] - samples[i - 1]) / (t[i] - t[i - 1]) * (t[i + 1] - t[i]) / (t[i + 1] - t[i - 1]) +
    //                  (samples[i + 1] - samples[i]) / (t[i + 1] - t[i]) * (t[i] - t[i - 1]) / (t[i + 1] - t[i - 1]);
    // 新版：
    float dx = (samples[i].x - samples[i - 1].x) / (t[i] - t[i - 1]) * (t[i + 1] - t[i]) / (t[i + 1] - t[i - 1]) +
               (samples[i + 1].x - samples[i].x) / (t[i + 1] - t[i]) * (t[i] - t[i - 1]) / (t[i + 1] - t[i - 1]);
    float dy = (samples[i].y - samples[i - 1].y) / (t[i] - t[i - 1]) * (t[i + 1] - t[i]) / (t[i + 1] - t[i - 1]) +
               (samples[i + 1].y - samples[i].y) / (t[i + 1] - t[i]) * (t[i] - t[i - 1]) / (t[i + 1] - t[i - 1]);
    float dz = (samples[i].z - samples[i - 1].z) / (t[i] - t[i - 1]) * (t[i + 1] - t[i]) / (t[i + 1] - t[i - 1]) +
               (samples[i + 1].z - samples[i].z) / (t[i + 1] - t[i]) * (t[i] - t[i - 1]) / (t[i + 1] - t[i - 1]);
    tangentline[i].x = dx;
    tangentline[i].y = dy;
    tangentline[i].z = dz;
  }
  // 三次Hermite基函数
  const uint8_t sampling_density = 30;
  for (uint8_t i = 0; i < 4; ++i) // 分别计算每一段曲线解析式，并生成采样点存入strd中
  {
    Vector sampleVec = PosToAngle(samples[i + 1].x, samples[i + 1].y, samples[i + 1].z);
    strd.push_back(sampleVec.x, sampleVec.y, sampleVec.z, PEN);
    float dt = t[i + 2] - t[i + 1]; // 当前段的节点间隔
    for (uint8_t k = 1; k < sampling_density; ++k)
    {
      // 参数化插值: t->u映射使不等距的t变为等距的u∈[0,1],
      float u = (float)k / (float)sampling_density;
      float u2 = u * u;
      float u3 = u2 * u;

      float h00 = 2 * u3 - 3 * u2 + 1;
      float h10 = u3 - 2 * u2 + u;
      float h01 = (-2) * u3 + 3 * u2;
      float h11 = u3 - u2;
      // 旧写法：过程量压栈会导致爆内存
      //  Vector res_loc = samples[i + 1] * h00 +
      //                   tangentline[i + 1] * h10 * dt +
      //                   samples[i + 2] * h01 +
      //                   tangentline[i + 2] * h11 * dt;
      // 新写法：拆开表达式
      Vector res_loc;
      res_loc.x = samples[i + 1].x * h00 + tangentline[i + 1].x * h10 * dt + samples[i + 2].x * h01 + tangentline[i + 2].x * h11 * dt;
      res_loc.y = samples[i + 1].y * h00 + tangentline[i + 1].y * h10 * dt + samples[i + 2].y * h01 + tangentline[i + 2].y * h11 * dt;
      res_loc.z = samples[i + 1].z * h00 + tangentline[i + 1].z * h10 * dt + samples[i + 2].z * h01 + tangentline[i + 2].z * h11 * dt;
      Vector res_angle = PosToAngle(res_loc.x, res_loc.y, res_loc.z);
      strd.push_back(res_angle.x, res_angle.y, res_angle.z, PEN);
    }
  }
  Vector endVec = PosToAngle(samples[5].x, samples[5].y, PENHEIGHT);
  strd.push_back(endVec.x, endVec.y, endVec.z, PEN);
  Serial.println("曲线生成完成，可以播放！");
}