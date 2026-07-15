/****************************************************************************
  平衡小车代码 - 适配版
  基于：Minibalance For Arduino

  适配修改：
  1. 移除ESP32 I2C通信模块（纯平衡控制）
  2. 启用双编码器模式（PinChangeInt支持右编码器Pin 4）
  3. 精简串口调试输出
  4. 添加详细中文注释

  硬件连接：
  参见下方接线说明，或打开串口监视器(9600波特率)查看启动信息

  控制算法：
  - 直立环（PD）：5ms周期，维持小车直立
  - 速度环（PI）：40ms周期，控制前进速度
  - 转向环：20ms周期（当前禁用）

  硬件要求：
  - Arduino Uno
  - MPU6050 陀螺仪加速度计模块
  - TB6612FNG 电机驱动模块
  - 2个带编码器的直流减速电机（11线双通道编码器）
  - 按键开关（Pin 3，内部上拉）
  - 电池（推荐7.4V-12V锂电池）
 ****************************************************************************/

// ==================== 接线说明 ====================
//
// 【MPU6050 - I2C接口】(A4/A5)
//   VCC  -> Arduino 5V
//   GND  -> Arduino GND
//   SCL  -> Arduino A5
//   SDA  -> Arduino A4
//
// 【TB6612FNG 电机驱动】
//   IN1  -> Pin 12  (电机1方向)
//   IN2  -> Pin 13  (电机1方向)
//   IN3  -> Pin 7   (电机2方向)
//   IN4  -> Pin 6   (电机2方向)
//   PWMA -> Pin 10  (电机1 PWM调速)
//   PWMB -> Pin 9   (电机2 PWM调速)
//   STBY -> 接5V     (TB6612使能，或者接Pin 11控制)
//   VM   -> 电池正极 (电机电源 2.5V-13.5V)
//   VCC  -> Arduino 5V (逻辑电源)
//   GND  -> 电池负极 + Arduino GND (共地!!!)
//
//   AO1/AO2 -> 电机1的两根电源线
//   BO1/BO2 -> 电机2的两根电源线
//
// 【左编码器】
//   VCC  -> Arduino 5V (或3.3V，取决于编码器规格)
//   GND  -> Arduino GND
//   A相  -> Pin 2   (Arduino外部中断0)
//   B相  -> Pin 5   (方向检测)
//
// 【右编码器】  (轮询模式，无中断冲突)
//   VCC  -> Arduino 5V (或3.3V)
//   GND  -> Arduino GND
//   A相  -> Pin 4   (5ms轮询，原理图接口1 Pin3)
//   B相  -> Pin 8   (方向检测，原理图接口1 Pin4)
//
//   ⚠️ 为什么右编码器A相在Pin 4却不用中断？
//      原理图确实把右编码器A相接在Pin 4，但PinChangeInt在Arduino UNO
//      上与串口TX/RX同PORTD端口，启用后会因为串口通信而反复崩溃。
//      因此右编码器改用轮询方式：在5ms(200Hz)定时中断里检测A相边沿。
//   ⚠️ 如果编码器是开漏输出，需要外接上拉电阻(10KΩ)到VCC
//   ⚠️ 11线电机通常：6线(电机1)+6线(电机2)-1共用线=11线
//
// 【按键】
//   一脚 -> Pin 3 (内置上拉，按下为LOW)
//   一脚 -> GND
//
// 【电池电压检测（可选）】
//   分压电路 -> A0
//   推荐：电池正极 -> 10K -> A0 -> 3.3K -> GND
//   (12V电池分压后约3.1V，在Arduino 5V ADC范围内)
//
// ==================== 接线说明结束 ====================

#include <avr/wdt.h>
#include <Wire.h>
#include <MsTimer2.h>
#include <KalmanFilter.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

// ========== 硬件引脚定义 ==========

// 按键
#define KEY 3

// TB6612电机驱动
#define IN1 12
#define IN2 13
#define IN3 7
#define IN4 6
#define PWMA 10
#define PWMB 9

// 左编码器 (使用外部中断0，Pin 2)
#define ENCODER_L 2
#define DIRECTION_L 5

// 右编码器 (轮询模式，Pin 4 - 原理图电机编码器接口1的A相，无中断冲突)
#define ENCODER_R 4
#define DIRECTION_R 8

// ========== 控制参数（最终调参版本）==========
#define TARGET_ANGLE -1.0   // 目标平衡角度/机械中值
                            // 如果小车向前倒，减小这个值（往负方向调）
                            // 如果小车向后倒，增大这个值（往正方向调）
                            // 范围通常在 -5 到 5 之间

#define BALANCE_KP 11.0      // 直立环P参数（比例）
                            // 越大响应越快，但过大会震荡
                            // 典型范围：5-25

#define BALANCE_KD 0.8     // 直立环D参数（微分/阻尼）
                            // 越大阻尼越强，减少震荡
                            // 典型范围：0.1-0.8

#define VELOCITY_KP 2.5     // 速度环P参数
                            // 控制速度响应，典型范围：0.5-3.0

#define VELOCITY_KI 0.011   // 速度环I参数（积分）
                            // 消除稳态误差，典型范围：0.002-0.02

#define PWM_MAX 180         // PWM最大幅值
                            // 范围：150-255

// ========== 卡尔曼滤波参数（一般不需修改）==========
#define K1 0.05
#define Q_ANGLE 0.001
#define Q_GYRO 0.005
#define R_ANGLE 0.5
#define C_0 1.0
#define DT 0.005            // 采样周期 5ms

// ========== 全局对象 ==========
MPU6050 Mpu6050;
KalmanFilter KalFilter;

// ========== 全局变量 ==========
int16_t ax, ay, az, gx, gy, gz;
int Balance_Pwm, Velocity_Pwm, Turn_Pwm;
int Motor1, Motor2;
float Battery_Voltage;

// 编码器变量
volatile long Velocity_L = 0, Velocity_R = 0;
int Velocity_Left = 0, Velocity_Right = 0;
volatile long Encoder_Left_Total = 0;
volatile long Encoder_Right_Total = 0;

int Angle;
unsigned char Flag_Stop = 1;  // 1=停止, 0=运行

// ========== 调试输出控制 ==========
// 设置为false可关闭串口数据输出，减少串口开销
bool enable_serial_output = true;


/**************************************************************************
 函数功能：直立PD控制
 输入：Angle - 当前角度, Gyro - 当前角速度
 输出：平衡控制PWM值
 控制周期：5ms
**************************************************************************/
int balance(float Angle, float Gyro) {
  float Bias;
  int balance_pwm;

  Bias = Angle - 0;
  balance_pwm = BALANCE_KP * Bias + Gyro * BALANCE_KD;

  return balance_pwm;
}

/**************************************************************************
 函数功能：速度PI控制
 输入：encoder_left, encoder_right - 左右编码器增量
 输出：速度控制PWM值
 控制周期：40ms（内部每8次调用执行一次）
**************************************************************************/
int velocity(int encoder_left, int encoder_right) {
  static float Velocity, Encoder_Least, Encoder, Movement;
  static float Encoder_Integral, Target_Velocity;

  Movement = 0;

  // 积分限幅，防止积分饱和
  if (Encoder_Integral > 300)   Encoder_Integral -= 200;
  if (Encoder_Integral < -300)  Encoder_Integral += 200;

  Encoder_Least = (encoder_left + encoder_right) - 0;
  Encoder *= 0.7;
  Encoder += Encoder_Least * 0.3;
  Encoder_Integral += Encoder;
  Encoder_Integral = Encoder_Integral - Movement;

  if (Encoder_Integral > 21000)   Encoder_Integral = 21000;
  if (Encoder_Integral < -21000)  Encoder_Integral = -21000;

  Velocity = Encoder * VELOCITY_KP + Encoder_Integral * VELOCITY_KI;

  // 停止时清除积分
  if (Turn_Off(KalFilter.angle, Battery_Voltage) == 1 || Flag_Stop == 1)
    Encoder_Integral = 0;

  return Velocity;
}

/**************************************************************************
 函数功能：转向控制（当前版本禁用）
**************************************************************************/
int turn(float gyro) {
  // 如果将来需要遥控转向，在这里实现
  return 0;
}

/**************************************************************************
 函数功能：检测小车是否被拿起（自动停止）
**************************************************************************/
int Pick_Up(float Acceleration, float Angle, int encoder_left, int encoder_right) {
  static unsigned int flag, count0, count1, count2;

  if (flag == 0) {
    if (abs(encoder_left) + abs(encoder_right) < 15) count0++;
    else count0 = 0;
    if (count0 > 10) { flag = 1; count0 = 0; }
  }

  if (flag == 1) {
    if (++count1 > 400) { count1 = 0; flag = 0; }
    if (Acceleration > 27000 &&
        (Angle > (-14 + TARGET_ANGLE)) &&
        (Angle < (14 + TARGET_ANGLE)))
      flag = 2;
  }

  if (flag == 2) {
    if (++count2 > 200) { count2 = 0; flag = 0; }
    if (abs(encoder_left + encoder_right) > 300) {
      flag = 0;
      return 1;
    }
  }
  return 0;
}

/**************************************************************************
 函数功能：检测小车是否被放下（自动启动）
**************************************************************************/
int Put_Down(float Angle, int encoder_left, int encoder_right) {
  static unsigned int flag, count;

  if (Flag_Stop == 0) return 0;

  if (flag == 0) {
    if (Angle > (-10 + TARGET_ANGLE) &&
        Angle < (10 + TARGET_ANGLE) &&
        encoder_left == 0 && encoder_right == 0)
      flag = 1;
  }

  if (flag == 1) {
    if (++count > 100) { count = 0; flag = 0; }
    if (encoder_left > 12 && encoder_right > 12 &&
        encoder_left < 80 && encoder_right < 80) {
      flag = 0;
      return 1;
    }
  }
  return 0;
}

/**************************************************************************
 函数功能：异常保护 - 角度过大或电池电压过低时关闭电机
**************************************************************************/
unsigned char Turn_Off(float angle, float voltage) {
  unsigned char temp;

  if (angle < -40 || angle > 40 || Flag_Stop == 1 || voltage < 10) {
    temp = 1;
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);
  } else {
    temp = 0;
  }

  return temp;
}

/**************************************************************************
 函数功能：按键检测（按下返回1）
**************************************************************************/
unsigned char My_click(void) {
  static unsigned char flag_key = 1;
  unsigned char Key;

  Key = digitalRead(KEY);
  if (flag_key && Key == 0) {
    flag_key = 0;
    return 1;
  } else if (Key == 1) {
    flag_key = 1;
  }

  return 0;
}

/**************************************************************************
 函数功能：设置电机PWM
**************************************************************************/
void Set_Pwm(int moto1, int moto2) {
  if (moto1 > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  analogWrite(PWMA, abs(moto1));

  if (moto2 < 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  analogWrite(PWMB, abs(moto2));
}

/**************************************************************************
 函数功能：PWM限幅
**************************************************************************/
void Xianfu_Pwm(void) {
  if (Motor1 < -PWM_MAX) Motor1 = -PWM_MAX;
  if (Motor1 >  PWM_MAX) Motor1 =  PWM_MAX;
  if (Motor2 < -PWM_MAX) Motor2 = -PWM_MAX;
  if (Motor2 >  PWM_MAX) Motor2 =  PWM_MAX;
}

/**************************************************************************
 函数功能：5ms核心控制函数（由MsTimer2定时器调用）
**************************************************************************/
void control() {
  static int Velocity_Count, Turn_Count;
  static float Voltage_All, Voltage_Count;
  static byte prevRencState = 0;   // 右编码器A相上一次状态（轮询用）
  int Temp;

  sei();  // 开中断

  // 0. 右编码器轮询（替代PinChangeInt，5ms周期足够低速平衡车使用）
  {
    byte currRenc = digitalRead(ENCODER_R);
    if (currRenc != prevRencState) {           // A相发生边沿变化
      byte dirR = digitalRead(DIRECTION_R);
      if (currRenc == LOW) {
        if (dirR == LOW) { Velocity_R--; Encoder_Right_Total--; }
        else            { Velocity_R++; Encoder_Right_Total++; }
      } else {
        if (dirR == LOW) { Velocity_R++; Encoder_Right_Total++; }
        else            { Velocity_R--; Encoder_Right_Total--; }
      }
    }
    prevRencState = currRenc;
  }

  // 1. 读取MPU6050数据
  Mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // 2. 卡尔曼滤波，计算角度
  KalFilter.Angletest(ax, ay, az, gx, gy, gz, DT, Q_ANGLE, Q_GYRO, R_ANGLE, C_0, K1);
  Angle = KalFilter.angle;

  // 3. 直立PD控制（5ms周期）
  Balance_Pwm = balance(KalFilter.angle + TARGET_ANGLE, KalFilter.Gyro_x);

  // 4. 速度PI控制（40ms周期，每8次5ms执行一次）
  if (++Velocity_Count >= 8) {
    Velocity_Left = Velocity_L;
    Velocity_L = 0;
    Velocity_Right = Velocity_R;
    Velocity_R = 0;
    Velocity_Pwm = velocity(Velocity_Left, Velocity_Right);
    Velocity_Count = 0;
  }

  // 5. 转向控制（20ms周期，当前禁用）
  if (++Turn_Count >= 4) {
    Turn_Pwm = turn(gz);
    Turn_Count = 0;
  }

  // 6. 三环叠加
  Motor1 = Balance_Pwm - Velocity_Pwm + Turn_Pwm;
  Motor2 = Balance_Pwm - Velocity_Pwm - Turn_Pwm;
  Xianfu_Pwm();

  // 7. 拿起/放下检测
  if (Pick_Up(az, KalFilter.angle, Velocity_Left, Velocity_Right)) Flag_Stop = 1;
  if (Put_Down(KalFilter.angle, Velocity_Left, Velocity_Right)) Flag_Stop = 0;

  // 8. 电机输出
  if (Turn_Off(KalFilter.angle, Battery_Voltage) == 0)
    Set_Pwm(Motor1, Motor2);

  // 9. 按键控制启停
  if (My_click()) Flag_Stop = !Flag_Stop;

  // 10. 电池电压采样
  Temp = analogRead(0);
  Voltage_Count++;
  Voltage_All += Temp;
  if (Voltage_Count == 200) {
    Battery_Voltage = Voltage_All * 0.05371 / 200;
    Voltage_All = 0;
    Voltage_Count = 0;
  }
}

/**************************************************************************
 函数功能：I2C总线恢复（解除被锁死的SDA线）
 当从设备异常时将SDA拉低，主设备无法通信，需要发送时钟脉冲恢复
**************************************************************************/
void I2C_Recover() {
  // 将SCL和SDA设为输出，模拟9个时钟脉冲来释放SDA
  pinMode(A5, OUTPUT);  // SCL
  pinMode(A4, OUTPUT);  // SDA

  // 先确保两条线都是高电平
  digitalWrite(A5, HIGH);
  digitalWrite(A4, HIGH);
  delayMicroseconds(10);

  // 发送最多9个时钟脉冲来释放SDA
  for (int i = 0; i < 9; i++) {
    digitalWrite(A5, LOW);
    delayMicroseconds(10);
    digitalWrite(A5, HIGH);
    delayMicroseconds(10);

    // 检查SDA是否已经被释放
    if (digitalRead(A4) == HIGH) break;
  }

  // 发送停止条件：SCL高时SDA从低变高
  digitalWrite(A4, LOW);
  delayMicroseconds(10);
  digitalWrite(A5, HIGH);
  delayMicroseconds(10);
  digitalWrite(A4, HIGH);
  delayMicroseconds(10);

  // 恢复为输入模式，让Wire库接管
  pinMode(A5, INPUT);
  pinMode(A4, INPUT);
}

/**************************************************************************
 函数功能：I2C设备扫描
 返回值：true=找到设备, false=未找到
**************************************************************************/
bool I2C_Scan(byte target_addr) {
  Wire.beginTransmission(target_addr);
  byte error = Wire.endTransmission();
  return (error == 0);
}

/**************************************************************************
 函数功能：安全的I2C写寄存器（带重试）
**************************************************************************/
byte I2C_WriteReg(byte dev_addr, byte reg, byte value, byte retries) {
  byte err;
  for (byte i = 0; i < retries; i++) {
    Wire.beginTransmission(dev_addr);
    Wire.write(reg);
    Wire.write(value);
    err = Wire.endTransmission();
    if (err == 0) return 0;
    delay(5);
  }
  return err;
}

/**************************************************************************
 函数功能：初始化
**************************************************************************/
void setup() {
  // --- 【第0步】立即禁用看门狗（必须在所有操作之前）---
  wdt_disable();
  // 清除看门狗复位标志
  MCUSR = 0;
  WDTCSR |= _BV(WDCE) | _BV(WDE);
  WDTCSR = 0;

  // --- TB6612 引脚初始化 ---
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);

  // --- 编码器引脚初始化 ---
  pinMode(ENCODER_L, INPUT);
  pinMode(DIRECTION_L, INPUT);
  pinMode(ENCODER_R, INPUT);
  pinMode(DIRECTION_R, INPUT);

  // --- 按键引脚初始化 ---
  pinMode(KEY, INPUT);

  // --- 串口初始化 ---
  Serial.begin(9600);
  delay(300);
  Serial.println(F("\n========================================"));
  Serial.println(F("    平衡小车 - 适配版 v1.1"));
  Serial.println(F("    硬件：Arduino UNO + MPU6050"));
  Serial.println(F("    驱动：TB6612 双编码器模式"));
  Serial.println(F("========================================"));
  Serial.flush();
  delay(200);

  // ========== I2C / MPU6050 初始化 ==========
  Serial.print(F("[1/5] 恢复I2C总线... "));
  Serial.flush();
  I2C_Recover();   // 先恢复可能被锁死的I2C总线
  delay(50);
  Serial.println(F("完成"));
  Serial.flush();

  Serial.print(F("[2/5] 启动I2C (100kHz)... "));
  Serial.flush();
  Wire.begin();
  Wire.setClock(100000L);
  delay(100);
  Serial.println(F("完成"));
  Serial.flush();

  // 扫描I2C总线
  Serial.print(F("[3/5] 扫描I2C设备... "));
  Serial.flush();

  if (I2C_Scan(0x68)) {
    Serial.println(F("找到 MPU6050 (0x68)"));
  } else {
    Serial.println(F("未找到设备!"));
    Serial.println(F(">> 请检查:"));
    Serial.println(F("   1. MPU6050 VCC -> Arduino 5V"));
    Serial.println(F("   2. MPU6050 GND -> Arduino GND"));
    Serial.println(F("   3. SDA -> A4, SCL -> A5"));
    Serial.println(F("   4. 是否需要外接4.7K上拉电阻?"));
    Serial.println(F(">> 系统将在此等待，修复后按Reset重试"));
    Serial.flush();
    // 死循环闪烁板载LED提示错误
    while (1) {
      digitalWrite(13, HIGH); delay(200);
      digitalWrite(13, LOW);  delay(200);
    }
  }
  Serial.flush();

  // 配置MPU6050寄存器
  Serial.print(F("[4/5] 配置MPU6050寄存器... "));
  Serial.flush();

  byte err;

  // 唤醒MPU6050 (PWR_MGMT_1 = 0x00)
  err = I2C_WriteReg(0x68, 0x6B, 0x00, 3);
  if (err != 0) {
    Serial.print(F("失败! 唤醒错误码="));
    Serial.println(err);
    Serial.flush();
    while (1) { digitalWrite(13, HIGH); delay(100); digitalWrite(13, LOW); delay(100); }
  }
  delay(50);

  // 设置陀螺仪量程 ±250°/s (GYRO_CONFIG = 0x00)
  err = I2C_WriteReg(0x68, 0x1B, 0x00, 3);
  if (err != 0) {
    Serial.print(F("失败! 陀螺仪配置错误码="));
    Serial.println(err);
    Serial.flush();
    while (1) { digitalWrite(13, HIGH); delay(100); digitalWrite(13, LOW); delay(100); }
  }
  delay(10);

  // 设置加速度计量程 ±2g (ACCEL_CONFIG = 0x00)
  err = I2C_WriteReg(0x68, 0x1C, 0x00, 3);
  if (err != 0) {
    Serial.print(F("失败! 加速度计配置错误码="));
    Serial.println(err);
    Serial.flush();
    while (1) { digitalWrite(13, HIGH); delay(100); digitalWrite(13, LOW); delay(100); }
  }
  delay(10);

  // 设置DLPF (CONFIG = 0x06, 5Hz低通滤波)
  I2C_WriteReg(0x68, 0x1A, 0x06, 3);
  delay(10);

  Serial.println(F("完成"));
  Serial.flush();

  // ========== 编码器中断初始化 ==========
  Serial.print(F("[5a] 初始化左编码器中断(Pin2)... "));
  Serial.flush();
  delay(30);

  // 先确保编码器引脚状态稳定
  pinMode(ENCODER_L, INPUT_PULLUP);      // 使用上拉防止浮空
  pinMode(DIRECTION_L, INPUT_PULLUP);    // 使用上拉防止浮空
  delay(20);

  attachInterrupt(0, READ_ENCODER_L, CHANGE);
  Serial.println(F("完成"));
  Serial.flush();
  delay(50);

  // ========== 右编码器初始化（轮询模式）==========
  Serial.print(F("[5b] 初始化右编码器(Pin4,轮询模式)... "));
  Serial.flush();
  delay(30);

  pinMode(ENCODER_R, INPUT_PULLUP);      // Pin 4, 原理图编码器接口1 A相, 5ms轮询
  pinMode(DIRECTION_R, INPUT_PULLUP);      // Pin 8, 原理图编码器接口1 B相
  delay(20);

  // 右编码器采用轮询方式读取（在5ms定时中断中检测边沿）
  // 不再使用PinChangeInt，避免Pin4所在PORTD与串口冲突导致崩溃

  Serial.println(F("完成"));
  Serial.flush();
  delay(50);

  // ========== 定时器初始化 ==========
  Serial.print(F("[5c] 启动5ms控制定时器... "));
  Serial.flush();
  delay(30);

  MsTimer2::set(5, control);
  delay(10);
  MsTimer2::start();

  Serial.println(F("完成"));
  Serial.flush();
  delay(100);

  // ========== 启动完成 ==========
  Serial.println(F("========================================"));
  Serial.println(F("  系统就绪!"));
  Serial.println(F("  按下按键(Pin3)启动平衡控制"));
  Serial.println(F("========================================"));
  Serial.println();
  Serial.println(F("--- 引脚分配 ---"));
  Serial.println(F("MPU6050: SDA=A4 SCL=A5"));
  Serial.println(F("电机: IN1=12 IN2=13 IN3=7 IN4=6 PWMA=10 PWMB=9"));
  Serial.println(F("编码器L: A=2 B=5  编码器R: A=4 B=8"));
  Serial.println(F("按键: 3  电压: A0"));
  Serial.println(F("=================\n"));
  Serial.flush();
}

/**************************************************************************
 函数功能：主循环（200ms输出一次调试信息）
**************************************************************************/
void loop() {
  static unsigned long lastTime = 0;
  unsigned long currentTime = millis();

  // 每200ms输出一次状态
  if (enable_serial_output && (currentTime - lastTime >= 200)) {
    Serial.print(F("Angle:"));
    Serial.print(Angle);
    Serial.print(F("  L:"));
    Serial.print(Velocity_Left);
    Serial.print(F("  R:"));
    Serial.print(Velocity_Right);
    Serial.print(F("  PWM:"));
    Serial.print(Motor1);
    Serial.print(F(","));
    Serial.print(Motor2);
    Serial.print(F("  Bat:"));
    Serial.print(Battery_Voltage);
    Serial.print(F("V  Stop:"));
    Serial.print(Flag_Stop ? F("YES") : F("NO"));
    Serial.println();

    lastTime = currentTime;
  }
}

/**************************************************************************
 函数功能：左编码器中断（外部中断0，Pin 2）
 11线编码器通常为霍尔传感器，AB两相正交输出
 根据AB相电平判断旋转方向
**************************************************************************/
void READ_ENCODER_L() {
  if (digitalRead(ENCODER_L) == LOW) {
    if (digitalRead(DIRECTION_L) == LOW) {
      Velocity_L--;
      Encoder_Left_Total--;
    } else {
      Velocity_L++;
      Encoder_Left_Total++;
    }
  } else {
    if (digitalRead(DIRECTION_L) == LOW) {
      Velocity_L++;
      Encoder_Left_Total++;
    } else {
      Velocity_L--;
      Encoder_Left_Total--;
    }
  }
}

// 右编码器已改为轮询模式在control()中读取，不再使用中断
// 详见 control() 函数开头
