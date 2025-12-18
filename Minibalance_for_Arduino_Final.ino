/****************************************************************************
   简化版平衡小车代码 - 最终工作版本
   基于：Minibalance For Arduino
   
   主要修改：
   1. 修复MPU6050初始化问题（手动配置寄存器代替initialize()）
   2. 移除上位机和蓝牙交互
   3. 使用单编码器模式（左编码器）
   4. 串口调试输出（9600波特率）
   5. 添加I2C主机功能，向ESP32发送编码器和MPU6050数据
   
   硬件要求：
   - Arduino Uno (I2C主机)
   - ESP32 (I2C从机，地址0x55)
   - MPU6050 (I2C: A4/SDA, A5/SCL)
   - TB6612FNG电机驱动
   - 左编码器 (Pin 2 + Pin 5)
   - 按键 (Pin 3)
   
   I2C连接：
   - Arduino SDA (A4) <--> ESP32 SDA
   - Arduino SCL (A5) <--> ESP32 SCL
   - 共地 GND <--> GND
****************************************************************************/
#include <avr/wdt.h>  // 看门狗
#include <Wire.h>
#include <PinChangeInt.h>
#include <MsTimer2.h>
#include <KalmanFilter.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

// ========== 硬件引脚定义 ==========
#define KEY 3
#define IN1 12
#define IN2 13
#define IN3 7
#define IN4 6
#define PWMA 10
#define PWMB 9
#define ENCODER_L 2
#define DIRECTION_L 5
#define ENCODER_R 4
#define DIRECTION_R 8

// ========== 控制参数 ==========
#define TARGET_ANGLE -2.3   // 目标平衡角度（机械中值）
#define DIFFERENCE 2
#define BALANCE_KP 15.0
#define BALANCE_KD 0.4
#define VELOCITY_KP 2.0
#define VELOCITY_KI 0.01

// ========== 卡尔曼滤波参数 ==========
#define K1 0.05
#define Q_ANGLE 0.001
#define Q_GYRO 0.005
#define R_ANGLE 0.5
#define C_0 1.0
#define DT 0.005

// ========== I2C通信参数 ==========
#define ESP32_I2C_ADDR 0x55  // ESP32从机地址

// I2C数据包结构（30字节，紧凑排列）
struct I2C_Data_Packet {
  // 帧头和校验
  uint8_t header;           // 帧头：0xAA
  uint8_t data_type;        // 数据类型：1=编码器+IMU
  
  // 编码器数据（8字节）
  int32_t encoder_left;     // 左轮编码器计数
  int32_t encoder_right;    // 右轮编码器计数
  
  // IMU数据（18字节）
  int16_t accel_x;          // 加速度X
  int16_t accel_y;          // 加速度Y
  int16_t accel_z;          // 加速度Z
  int16_t gyro_x;           // 角速度X
  int16_t gyro_y;           // 角速度Y
  int16_t gyro_z;           // 角速度Z
  float angle;              // 卡尔曼滤波后的角度
  
  uint8_t checksum;         // 校验和
} __attribute__((packed));  // 总计30字节，紧凑排列无填充

// ========== 全局对象 ==========
MPU6050 Mpu6050;
KalmanFilter KalFilter;

// ========== 全局变量 ==========
int16_t ax, ay, az, gx, gy, gz;
int Balance_Pwm, Velocity_Pwm, Turn_Pwm;
int Motor1, Motor2;
float Battery_Voltage;
volatile long Velocity_L = 0, Velocity_R = 0;
int Velocity_Left = 0, Velocity_Right = 0;
int Angle;
unsigned char Flag_Stop = 1;

// I2C通信变量
I2C_Data_Packet i2c_packet;
volatile long Encoder_Left_Total = 0;   // 累计编码器计数
volatile long Encoder_Right_Total = 0;  // 累计编码器计数

/**************************************************************************
函数功能：计算校验和
**************************************************************************/
uint8_t Calculate_Checksum(uint8_t* data, uint8_t len) {
  uint8_t sum = 0;
  for (uint8_t i = 0; i < len; i++) {
    sum += data[i];
  }
  return sum;
}

/**************************************************************************
函数功能：向ESP32发送I2C数据
**************************************************************************/
void Send_Data_To_ESP32() {
  static unsigned int send_count = 0;
  
  // 准备数据包
  i2c_packet.header = 0xAA;
  i2c_packet.data_type = 0x01;  // 同时发送编码器和IMU数据
  
  // 编码器数据（累计值）
  i2c_packet.encoder_left = Encoder_Left_Total;
  i2c_packet.encoder_right = Encoder_Right_Total;
  
  // IMU原始数据
  i2c_packet.accel_x = ax;
  i2c_packet.accel_y = ay;
  i2c_packet.accel_z = az;
  i2c_packet.gyro_x = gx;
  i2c_packet.gyro_y = gy;
  i2c_packet.gyro_z = gz;
  
  // 卡尔曼滤波后的角度
  i2c_packet.angle = KalFilter.angle;
  
  // 计算校验和（不包括checksum字段本身，即前29字节）
  i2c_packet.checksum = Calculate_Checksum((uint8_t*)&i2c_packet, sizeof(I2C_Data_Packet) - 1);
  
  // 通过I2C发送数据到ESP32
  Wire.beginTransmission(ESP32_I2C_ADDR);
  
  // 逐字节发送数据包
  uint8_t* data_ptr = (uint8_t*)&i2c_packet;
  for (int i = 0; i < sizeof(I2C_Data_Packet); i++) {
    Wire.write(data_ptr[i]);
  }
  
  byte error = Wire.endTransmission();
  
  // 每100次发送输出一次调试信息（50ms*100=5秒）
  if (++send_count >= 100) {
    Serial.print("[I2C] Sent ");
    Serial.print(sizeof(I2C_Data_Packet));
    Serial.print(" bytes to 0x");
    Serial.print(ESP32_I2C_ADDR, HEX);
    Serial.print(" result=");
    Serial.print(error);
    Serial.print(" (0=OK) EncL=");
    Serial.print(Encoder_Left_Total);
    Serial.print(" EncR=");
    Serial.println(Encoder_Right_Total);
    send_count = 0;
  }
}

/**************************************************************************
函数功能：检测小车是否被拿起
**************************************************************************/
int Pick_Up(float Acceleration, float Angle, int encoder_left, int encoder_right) {
  static unsigned int flag, count0, count1, count2;
  
  if (flag == 0) {
    if (abs(encoder_left) + abs(encoder_right) < 15) count0++;
    else count0 = 0;
    if (count0 > 10) flag = 1, count0 = 0;
  }
  
  if (flag == 1) {
    if (++count1 > 400) count1 = 0, flag = 0;
    if (Acceleration > 27000 && (Angle > (-14 + TARGET_ANGLE)) && (Angle < (14 + TARGET_ANGLE))) flag = 2;
  }
  
  if (flag == 2) {
    if (++count2 > 200) count2 = 0, flag = 0;
    if (abs(encoder_left + encoder_right) > 300) {
      flag = 0;
      return 1;
    }
  }
  return 0;
}

/**************************************************************************
函数功能：检测小车是否被放下
**************************************************************************/
int Put_Down(float Angle, int encoder_left, int encoder_right) {
  static unsigned int flag, count;
  
  if (Flag_Stop == 0) return 0;
  
  if (flag == 0) {
    if (Angle > (-10 + TARGET_ANGLE) && Angle < (10 + TARGET_ANGLE) && 
        encoder_left == 0 && encoder_right == 0) flag = 1;
  }
  
  if (flag == 1) {
    if (++count > 100) count = 0, flag = 0;
    if (encoder_left > 12 && encoder_right > 12 && 
        encoder_left < 80 && encoder_right < 80) {
      flag = 0;
      return 1;
    }
  }
  return 0;
}

/**************************************************************************
函数功能：异常关闭电机
**************************************************************************/
unsigned char Turn_Off(float angle, float voltage) {
  unsigned char temp;
  
  if (angle < -40 || angle > 40 || 1 == Flag_Stop || voltage < 10) {
    temp = 1;
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);
  }
  else temp = 0;
  
  return temp;
}

/**************************************************************************
函数功能：按键扫描
**************************************************************************/
unsigned char My_click(void) {
  static unsigned char flag_key = 1;
  unsigned char Key;
  
  Key = digitalRead(KEY);
  if (flag_key && Key == 0) {
    flag_key = 0;
    return 1;
  }
  else if (1 == Key) flag_key = 1;
  
  return 0;
}

/**************************************************************************
函数功能：直立PD控制
**************************************************************************/
int balance(float Angle, float Gyro) {
  float Bias;
  int balance;
  
  Bias = Angle - 0;
  balance = BALANCE_KP * Bias + Gyro * BALANCE_KD;
  
  return balance;
}

/**************************************************************************
函数功能：速度PI控制
**************************************************************************/
int velocity(int encoder_left, int encoder_right) {
  static float Velocity, Encoder_Least, Encoder, Movement;
  static float Encoder_Integral, Target_Velocity;
  
  Movement = 0;
  if (Encoder_Integral > 300) Encoder_Integral -= 200;
  if (Encoder_Integral < -300) Encoder_Integral += 200;
  
  Encoder_Least = (encoder_left + encoder_right) - 0;
  Encoder *= 0.7;
  Encoder += Encoder_Least * 0.3;
  Encoder_Integral += Encoder;
  Encoder_Integral = Encoder_Integral - Movement;
  
  if (Encoder_Integral > 21000) Encoder_Integral = 21000;
  if (Encoder_Integral < -21000) Encoder_Integral = -21000;
  
  Velocity = Encoder * VELOCITY_KP + Encoder_Integral * VELOCITY_KI;
  
  if (Turn_Off(KalFilter.angle, Battery_Voltage) == 1 || Flag_Stop == 1) 
    Encoder_Integral = 0;
  
  return Velocity;
}

/**************************************************************************
函数功能：转向控制
**************************************************************************/
int turn(float gyro) {
  return 0;
}

/**************************************************************************
函数功能：赋值给PWM寄存器
**************************************************************************/
void Set_Pwm(int moto1, int moto2) {
  if (moto1 > 0) digitalWrite(IN1, HIGH), digitalWrite(IN2, LOW);
  else digitalWrite(IN1, LOW), digitalWrite(IN2, HIGH);
  analogWrite(PWMA, abs(moto1));
  
  if (moto2 < 0) digitalWrite(IN3, HIGH), digitalWrite(IN4, LOW);
  else digitalWrite(IN3, LOW), digitalWrite(IN4, HIGH);
  analogWrite(PWMB, abs(moto2));
}

/**************************************************************************
函数功能：限制PWM赋值
**************************************************************************/
void Xianfu_Pwm(void) {
  int Amplitude = 250;
  
  if (Motor1 < -Amplitude) Motor1 = -Amplitude;
  if (Motor1 > Amplitude) Motor1 = Amplitude;
  if (Motor2 < -Amplitude) Motor2 = -Amplitude;
  if (Motor2 > Amplitude) Motor2 = Amplitude;
}

/**************************************************************************
函数功能：5ms控制函数 - 核心代码
**************************************************************************/
void control() {
  static int Velocity_Count, Turn_Count, I2C_Count;
  static float Voltage_All, Voltage_Count;
  int Temp;
  
  sei();
  
  // 获取MPU6050数据
  Mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // 卡尔曼滤波获取角度
  KalFilter.Angletest(ax, ay, az, gx, gy, gz, DT, Q_ANGLE, Q_GYRO, R_ANGLE, C_0, K1);
  Angle = KalFilter.angle;
  
  // 直立PD控制，控制周期5ms
  Balance_Pwm = balance(KalFilter.angle + TARGET_ANGLE, KalFilter.Gyro_x);
  
  // 速度控制，控制周期40ms
  if (++Velocity_Count >= 8) {
    Velocity_Left = Velocity_L;
    Velocity_L = 0;
    Velocity_Right = Velocity_Left;  // 暂时使用左编码器数据（右编码器禁用）
    Velocity_R = 0;
    Velocity_Pwm = velocity(Velocity_Left, Velocity_Right);
    Velocity_Count = 0;
  }
  
  // 转向控制，控制周期20ms
  if (++Turn_Count >= 4) {
    Turn_Pwm = turn(gz);
    Turn_Count = 0;
  }
  
  // 三环叠加
  Motor1 = Balance_Pwm - Velocity_Pwm + Turn_Pwm;
  Motor2 = Balance_Pwm - Velocity_Pwm - Turn_Pwm;
  Xianfu_Pwm();
  
  // 检测拿起和放下
  if (Pick_Up(az, KalFilter.angle, Velocity_Left, Velocity_Right)) Flag_Stop = 1;
  if (Put_Down(KalFilter.angle, Velocity_Left, Velocity_Right)) Flag_Stop = 0;
  
  // 电机控制
  if (Turn_Off(KalFilter.angle, Battery_Voltage) == 0) Set_Pwm(Motor1, Motor2);
  
  // 按键控制
  if (My_click()) Flag_Stop = !Flag_Stop;
  
  // 电池电压采样
  Temp = analogRead(0);
  Voltage_Count++;
  Voltage_All += Temp;
  if (Voltage_Count == 200) {
    Battery_Voltage = Voltage_All * 0.05371 / 200;
    Voltage_All = 0;
    Voltage_Count = 0;
  }
  
  // I2C数据发送，控制周期50ms（每10个5ms周期）
  if (++I2C_Count >= 10) {
    Send_Data_To_ESP32();
    I2C_Count = 0;
  }
}

/**************************************************************************
函数功能：初始化
**************************************************************************/
void setup() {
  // TB6612引脚初始化
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 0);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 0);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  
  // 编码器引脚初始化
  pinMode(2, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  pinMode(8, INPUT);
  
  // 按键引脚初始化
  pinMode(3, INPUT);
  
  // 禁用看门狗（防止复位）
  wdt_disable();
  
  // 串口初始化
  Serial.begin(9600);
  delay(500);
  Serial.println("\n=== MiniBalance Starting ===");
  delay(500);
  
  // I2C初始化
  Serial.print("Init I2C...");
  Wire.begin();
  Wire.setClock(100000L);
  delay(100);
  Serial.println("OK");
  Serial.print("I2C packet size: ");
  Serial.print(sizeof(I2C_Data_Packet));
  Serial.println(" bytes");
  
  // MPU6050手动配置（使用纯Wire库）
  Serial.println("Config MPU6050:");
  Serial.flush();
  
  Serial.print("  Wake up...");
  Serial.flush();
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  byte err = Wire.endTransmission();
  Serial.print("err=");
  Serial.println(err);
  delay(50);
  
  Serial.print("  Set gyro...");
  Serial.flush();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x00);
  err = Wire.endTransmission();
  Serial.print("err=");
  Serial.println(err);
  delay(10);
  
  Serial.print("  Set accel...");
  Serial.flush();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x00);
  err = Wire.endTransmission();
  Serial.print("err=");
  Serial.println(err);
  Serial.flush();
  delay(10);
  
  Serial.println("MPU OK");
  Serial.flush();
  delay(100);
  
  // 编码器外部中断初始化（先初始化中断）
  Serial.println("Init Encoder");
  Serial.flush();
  
  attachInterrupt(0, READ_ENCODER_L, CHANGE);
  Serial.println("Encoder L OK");
  Serial.flush();
  delay(50);
  
  // 注意：右编码器已禁用（单编码器模式）
  // Arduino Uno Pin 4 不支持PinChangeInt库
  // 单编码器模式足以实现平衡功能
  Serial.println("Single encoder mode");
  Serial.flush();
  delay(50);
  
  // 定时中断初始化（最后启动定时器）
  Serial.println("Init Timer");
  Serial.flush();
  delay(50);
  
  MsTimer2::set(5, control);
  Serial.println("Timer set");
  Serial.flush();
  delay(50);
  
  MsTimer2::start();
  Serial.println("Timer started");
  Serial.flush();
  delay(100);
  
  Serial.println("\n=== SYSTEM READY ===");
  Serial.println("Mode: Single encoder (left only)");
  Serial.println("Press KEY (Pin 3) to start/stop");
  Serial.println("Balance control active\n");
  Serial.flush();
  delay(100);
}

/**************************************************************************
函数功能：主循环
**************************************************************************/
void loop() {
  static unsigned long lastTime = 0;
  unsigned long currentTime = millis();
  
  // 每200ms输出一次状态
  if (currentTime - lastTime >= 200) {
    Serial.print("Ang:");
    Serial.print(Angle);
    Serial.print(" VL:");
    Serial.print(Velocity_Left);
    Serial.print(" VR:");
    Serial.print(Velocity_Right);
    Serial.print(" Bat:");
    Serial.print(Battery_Voltage);
    Serial.print("V PWM:");
    Serial.print(Motor1);
    Serial.print(",");
    Serial.print(Motor2);
    Serial.print(" Stop:");
    Serial.println(Flag_Stop);
    
    lastTime = currentTime;
  }
}

/**************************************************************************
函数功能：左编码器中断
**************************************************************************/
void READ_ENCODER_L() {
  if (digitalRead(ENCODER_L) == LOW) {
    if (digitalRead(DIRECTION_L) == LOW) {
      Velocity_L--;
      Encoder_Left_Total--;
    }
    else {
      Velocity_L++;
      Encoder_Left_Total++;
    }
  }
  else {
    if (digitalRead(DIRECTION_L) == LOW) {
      Velocity_L++;
      Encoder_Left_Total++;
    }
    else {
      Velocity_L--;
      Encoder_Left_Total--;
    }
  }
}

/**************************************************************************
函数功能：右编码器中断
**************************************************************************/
void READ_ENCODER_R() {
  if (digitalRead(ENCODER_R) == LOW) {
    if (digitalRead(DIRECTION_R) == LOW) {
     
      Encoder_Right_Total++;
    }
    else {
      Velocity_R--;
      Encoder_Right_Total--;
    }
  }
}
