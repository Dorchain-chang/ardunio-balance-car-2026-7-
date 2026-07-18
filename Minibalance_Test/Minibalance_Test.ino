 /****************************************************************************
  平衡小车 - Arduino 独立测试版
  基于 Minibalance_Nav.ino，移除 X5 通信，添加简单串口命令

  用法：打开 Arduino IDE 串口监视器（115200 波特率），输入以下命令：

    cal       校准平衡角度（握住车到平衡位置，输入此命令自动记录零点）
    ta-1.5    微调平衡角度（校准后可用此命令微调）
    go        启动平衡
    stop      停止（倒地）
    s50       前进速度 50（编码器脉冲/40ms）
    s-30      后退速度 30
    t20       右转 20
    t-15      左转 15
    h         原地停下（保持平衡，速度归零）
    kp12      设置 Balance_Kp = 12
    kd0.8     设置 Balance_Kd = 0.8
    vkp2.5    设置 Velocity_Kp = 2.5
    vki0.011  设置 Velocity_Ki = 0.011
    ta-1.5    设置 Target_Angle = -1.5
    info      显示所有当前参数
    reset     恢复默认参数
    test      运行自动测试序列：前进→右转→左转→后退→停下
    raw       切换调试输出格式（简洁/原始上行协议）

  硬件连接：同 Minibalance_Nav.ino
****************************************************************************/

#include <avr/wdt.h>
#include <Wire.h>
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

// ========== 控制参数（与原版 Minibalance_for_Arduino_Final 一致）==========
float Calibration_Angle = 0;   // 校准角度：MPU6050在车子平衡时的读数（用 cal 命令自动获取）
float Target_Angle  = -2.3;    // 微调角度（在校准基础上的偏移，原版值）
float Balance_Kp    = 15.0;
float Balance_Kd    = 0.4;
float Velocity_Kp   = 2.0;
float Velocity_Ki   = 0.01;

// ========== 运动控制目标 ==========
float Target_Speed    = 0;
float Target_Steering = 0;

// ========== 安全限制 ==========
#define PWM_MAX 250

// ========== 卡尔曼滤波参数 ==========
#define K1 0.05
#define Q_ANGLE 0.001
#define Q_GYRO 0.005
#define R_ANGLE 0.5
#define C_0 1.0
#define DT 0.005

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
volatile long Encoder_Left_Total = 0;
volatile long Encoder_Right_Total = 0;
int Angle;
unsigned char Flag_Stop = 1;

// ========== 测试模式 ==========
bool debugRaw = false;          // true=原始上行协议格式, false=人类可读格式
bool testMode = false;          // 自动测试序列运行中
int testStep = 0;
unsigned long testStepStart = 0;

// ========== 直立PD控制 (5ms) ==========
int balance(float Angle, float Gyro) {
  float Bias = Angle - 0;
  return (int)(Balance_Kp * Bias + Gyro * Balance_Kd);
}

// ========== 速度PI控制 (40ms) ==========
int velocity(int encoder_left, int encoder_right) {
  static float Velocity, Encoder_Least, Encoder, Movement;
  static float Encoder_Integral;
  Movement = 0;

  static float lastTarget = 0;
  if (Target_Speed != lastTarget) {
    Encoder_Integral = 0;
    lastTarget = Target_Speed;
  }

  if (Encoder_Integral > 300)   Encoder_Integral -= 200;
  if (Encoder_Integral < -300)  Encoder_Integral += 200;

  Encoder_Least = (encoder_left + encoder_right) - Target_Speed;
  Encoder *= 0.7;
  Encoder += Encoder_Least * 0.3;
  Encoder_Integral += Encoder;
  Encoder_Integral = Encoder_Integral - Movement;

  if (Encoder_Integral > 21000)   Encoder_Integral = 21000;
  if (Encoder_Integral < -21000)  Encoder_Integral = -21000;

  Velocity = Encoder * Velocity_Kp + Encoder_Integral * Velocity_Ki;

  if (Turn_Off(KalFilter.angle, Battery_Voltage) == 1 || Flag_Stop == 1)
    Encoder_Integral = 0;

  return Velocity;
}

// ========== 转向控制 ==========
bool turnEnabled = false;  // 默认关闭转向控制（与原版一致），需要时用 turnon 开启
int turn(float gyro) {
  if (!turnEnabled) return 0;
  float Turn_Kp = 1.0;
  float Turn_Kd = 0.02;
  return (int)(Target_Steering * Turn_Kp - gyro * Turn_Kd);
}

// ========== 拿起检测 ==========
int Pick_Up(float Acceleration, float Angle, int encoder_left, int encoder_right) {
  static unsigned int flag, count0, count1, count2;
  if (flag == 0) {
    if (abs(encoder_left) + abs(encoder_right) < 15) count0++;
    else count0 = 0;
    if (count0 > 10) { flag = 1; count0 = 0; }
  }
  if (flag == 1) {
    if (++count1 > 400) { count1 = 0; flag = 0; }
    if (Acceleration > 27000 && Angle > (-14 + Target_Angle) && Angle < (14 + Target_Angle))
      flag = 2;
  }
  if (flag == 2) {
    if (++count2 > 200) { count2 = 0; flag = 0; }
    if (abs(encoder_left + encoder_right) > 300) { flag = 0; return 1; }
  }
  return 0;
}

// ========== 放下检测 ==========
int Put_Down(float Angle, int encoder_left, int encoder_right) {
  static unsigned int flag, count;
  if (Flag_Stop == 0) return 0;
  if (flag == 0) {
    if (Angle > (-10 + Target_Angle) && Angle < (10 + Target_Angle) &&
        encoder_left == 0 && encoder_right == 0)
      flag = 1;
  }
  if (flag == 1) {
    if (++count > 100) { count = 0; flag = 0; }
    if (encoder_left > 12 && encoder_right > 12 &&
        encoder_left < 80 && encoder_right < 80) {
      flag = 0; return 1;
    }
  }
  return 0;
}

// ========== 异常保护 ==========
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

// ========== 按键检测 ==========
unsigned char My_click(void) {
  static unsigned char flag_key = 1;
  unsigned char Key = digitalRead(KEY);
  if (flag_key && Key == 0) { flag_key = 0; return 1; }
  else if (Key == 1) flag_key = 1;
  return 0;
}

// ========== 设置电机PWM ==========
void Set_Pwm(int moto1, int moto2) {
  if (moto1 > 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
  else           { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); }
  analogWrite(PWMA, abs(moto1));

  if (moto2 < 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
  else           { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); }
  analogWrite(PWMB, abs(moto2));
}

// ========== PWM限幅 ==========
void Xianfu_Pwm(void) {
  if (Motor1 < -PWM_MAX) Motor1 = -PWM_MAX;
  if (Motor1 >  PWM_MAX) Motor1 =  PWM_MAX;
  if (Motor2 < -PWM_MAX) Motor2 = -PWM_MAX;
  if (Motor2 >  PWM_MAX) Motor2 =  PWM_MAX;
}

// ========== 5ms核心控制 ==========
void control() {
  static int Velocity_Count, Turn_Count;
  static float Voltage_All, Voltage_Count;
  static byte prevRencState = 0;
  int Temp;

  sei();

  // 0. 右编码器轮询
  {
    byte currRenc = digitalRead(ENCODER_R);
    if (currRenc != prevRencState) {
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

  // 1. 读取MPU6050
  Mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // 2. 卡尔曼滤波 + 校准
  KalFilter.Angletest(ax, ay, az, gx, gy, gz, DT, Q_ANGLE, Q_GYRO, R_ANGLE, C_0, K1);
  Angle = KalFilter.angle - Calibration_Angle;  // 校准后的角度（平衡点≈0°）

  // 3. 直立PD
  Balance_Pwm = balance(Angle + Target_Angle, KalFilter.Gyro_x);

  // 4. 速度PI (40ms)
  if (++Velocity_Count >= 8) {
    Velocity_Left = Velocity_L;  Velocity_L = 0;
    Velocity_Right = Velocity_R; Velocity_R = 0;
    Velocity_Pwm = velocity(Velocity_Left, Velocity_Right);
    Velocity_Count = 0;
  }

  // 5. 转向 (20ms)
  if (++Turn_Count >= 4) {
    Turn_Pwm = turn(gz);
    Turn_Count = 0;
  }

  // 6. 三环叠加
  Motor1 = Balance_Pwm - Velocity_Pwm + Turn_Pwm;
  Motor2 = Balance_Pwm - Velocity_Pwm - Turn_Pwm;
  Xianfu_Pwm();

  // 7. 拿起/放下检测（使用校准后的 Angle）
  if (Pick_Up(az, Angle, Velocity_Left, Velocity_Right)) Flag_Stop = 1;
  if (Put_Down(Angle, Velocity_Left, Velocity_Right)) Flag_Stop = 0;

  // 8. 电机输出（使用校准后的 Angle）
  if (Turn_Off(Angle, Battery_Voltage) == 0)
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

// ========== I2C恢复 ==========
void I2C_Recover() {
  pinMode(A5, OUTPUT); pinMode(A4, OUTPUT);
  digitalWrite(A5, HIGH); digitalWrite(A4, HIGH);
  delayMicroseconds(10);
  for (int i = 0; i < 9; i++) {
    digitalWrite(A5, LOW); delayMicroseconds(10);
    digitalWrite(A5, HIGH); delayMicroseconds(10);
    if (digitalRead(A4) == HIGH) break;
  }
  digitalWrite(A4, LOW); delayMicroseconds(10);
  digitalWrite(A5, HIGH); delayMicroseconds(10);
  digitalWrite(A4, HIGH); delayMicroseconds(10);
  pinMode(A5, INPUT); pinMode(A4, INPUT);
}

// ========== I2C设备扫描 ==========
bool I2C_Scan(byte addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

// ========== 安全I2C写 ==========
byte I2C_WriteReg(byte dev, byte reg, byte val, byte retries) {
  byte err;
  for (byte i = 0; i < retries; i++) {
    Wire.beginTransmission(dev);
    Wire.write(reg); Wire.write(val);
    err = Wire.endTransmission();
    if (err == 0) return 0;
    delay(5);
  }
  return err;
}

// ========== 左编码器中断 ==========
void READ_ENCODER_L() {
  if (digitalRead(ENCODER_L) == LOW) {
    if (digitalRead(DIRECTION_L) == LOW) { Velocity_L--; Encoder_Left_Total--; }
    else                                 { Velocity_L++; Encoder_Left_Total++; }
  } else {
    if (digitalRead(DIRECTION_L) == LOW) { Velocity_L++; Encoder_Left_Total++; }
    else                                 { Velocity_L--; Encoder_Left_Total--; }
  }
}

// ========== 显示帮助 ==========
void printHelp() {
  Serial.println(F("\n========== 命令列表 =========="));
  Serial.println(F("  go        启动平衡"));
  Serial.println(F("  stop      停止（倒地）"));
  Serial.println(F("  s<N>      设定速度： s50=前进  s-30=后退  s0=原地"));
  Serial.println(F("  t<N>      设定转向： t20=右转  t-15=左转"));
  Serial.println(F("  h         原地停下（保持平衡，速度清零）"));
  Serial.println(F("  cal       校准平衡角度（握住车在平衡位置后输入）"));
  Serial.println(F("  ta<N>     微调平衡角度  例: ta-1.5"));
  Serial.println(F("  kp<N>     设置 Balance_Kp  例: kp12"));
  Serial.println(F("  kd<N>     设置 Balance_Kd  例: kd0.8"));
  Serial.println(F("  vkp<N>    设置 Velocity_Kp 例: vkp2.5"));
  Serial.println(F("  vki<N>    设置 Velocity_Ki 例: vki0.011"));
  Serial.println(F("  info      显示当前参数"));
  Serial.println(F("  reset     恢复默认参数"));
  Serial.println(F("  test      运行自动测试序列"));
  Serial.println(F("  raw       切换调试格式（人类可读 / 原始协议）"));
  Serial.println(F("  help      显示本帮助"));
  Serial.println(F("==============================="));
}

// ========== 显示参数 ==========
void printInfo() {
  Serial.println(F("\n========== 当前参数 =========="));
  Serial.print(F("  Balance_Kp    = ")); Serial.println(Balance_Kp);
  Serial.print(F("  Balance_Kd    = ")); Serial.println(Balance_Kd);
  Serial.print(F("  Velocity_Kp   = ")); Serial.println(Velocity_Kp);
  Serial.print(F("  Velocity_Ki   = ")); Serial.println(Velocity_Ki);
  Serial.print(F("  Calibration_Angle= ")); Serial.println(Calibration_Angle);
  Serial.print(F("  Target_Angle  = ")); Serial.println(Target_Angle);
  Serial.print(F("  Target_Speed  = ")); Serial.println(Target_Speed);
  Serial.print(F("  Target_Steering= ")); Serial.println(Target_Steering);
  Serial.print(F("  Battery       = ")); Serial.print(Battery_Voltage); Serial.println(F("V"));
  Serial.print(F("  Stop          = ")); Serial.println(Flag_Stop ? F("YES") : F("NO"));
  Serial.println(F("==============================="));
}

// ========== 自动测试序列 ==========
void testSequence() {
  unsigned long now = millis();

  switch (testStep) {
    case 0:  // 启动平衡
      Flag_Stop = 0;
      Target_Speed = 0;
      Target_Steering = 0;
      Serial.println(F("\n[测试] 步骤1: 启动平衡，原地站立 3 秒..."));
      testStepStart = now;
      testStep = 1;
      break;
    case 1:
      if (now - testStepStart > 3000) {
        Target_Speed = 50;
        Serial.println(F("[测试] 步骤2: 前进(s50)，行驶 3 秒..."));
        testStepStart = now;
        testStep = 2;
      }
      break;
    case 2:
      if (now - testStepStart > 3000) {
        Target_Steering = 30;
        Serial.println(F("[测试] 步骤3: 右转(t30)，行驶 2 秒..."));
        testStepStart = now;
        testStep = 3;
      }
      break;
    case 3:
      if (now - testStepStart > 2000) {
        Target_Steering = -30;
        Serial.println(F("[测试] 步骤4: 左转(t-30)，行驶 2 秒..."));
        testStepStart = now;
        testStep = 4;
      }
      break;
    case 4:
      if (now - testStepStart > 2000) {
        Target_Speed = -30;
        Target_Steering = 0;
        Serial.println(F("[测试] 步骤5: 后退(s-30)，行驶 3 秒..."));
        testStepStart = now;
        testStep = 5;
      }
      break;
    case 5:
      if (now - testStepStart > 3000) {
        Target_Speed = 0;
        Serial.println(F("[测试] 步骤6: 原地停下(h)，站立 2 秒后结束..."));
        testStepStart = now;
        testStep = 6;
      }
      break;
    case 6:
      if (now - testStepStart > 2000) {
        Serial.println(F("[测试] 测试序列完成!"));
        testMode = false;
        testStep = 0;
      }
      break;
  }
}

// ========== 串口命令解析 ==========
void parseCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  char c = cmd.charAt(0);
  String rest = cmd.substring(1);

  // go / stop / halt
  if (cmd == F("go")) {
    Flag_Stop = 0;
    Serial.println(F("[OK] 启动平衡控制"));
  }
  else if (cmd == F("stop")) {
    Flag_Stop = 1;
    Target_Speed = 0;
    Target_Steering = 0;
    Serial.println(F("[OK] 停止（电机断电）"));
  }
  else if (cmd == F("h")) {
    Target_Speed = 0;
    Target_Steering = 0;
    Serial.println(F("[OK] 原地停下（保持平衡）"));
  }

  // speed: s50, s-30, s0
  else if (c == 's' || c == 'S') {
    float v = rest.toFloat();
    if (v >= -300 && v <= 300) {
      Target_Speed = v;
      Serial.print(F("[OK] 目标速度 = ")); Serial.println(v);
    } else Serial.println(F("[ERR] 速度范围: -300 ~ 300"));
  }

  // steer: t20, t-15
  else if (c == 't' && cmd.charAt(1) != 'a' && cmd.charAt(1) != 'u') {  // 避免与 "ta" "turn" 冲突
    float v = rest.toFloat();
    if (v >= -200 && v <= 200) {
      Target_Steering = v;
      Serial.print(F("[OK] 目标转向 = ")); Serial.println(v);
    } else Serial.println(F("[ERR] 转向范围: -200 ~ 200"));
  }

  // turnon / turnoff: 开启/关闭转向控制（默认关闭，与原版一致）
  else if (cmd == F("turnon")) {
    turnEnabled = true;
    Serial.println(F("[OK] 转向控制已开启"));
  }
  else if (cmd == F("turnoff")) {
    turnEnabled = false;
    Serial.println(F("[OK] 转向控制已关闭（纯平衡模式）"));
  }

  // ta: target angle（微调，校准基础上的偏移）
  else if (cmd.startsWith(F("ta"))) {
    float v = rest.toFloat();
    if (v >= -5 && v <= 5) {
      Target_Angle = v;
      Serial.print(F("[OK] 目标角度微调 = ")); Serial.println(v);
    } else Serial.println(F("[ERR] 微调范围: -5 ~ 5（大偏移请用 cal 校准）"));
  }

  // cal: 校准平衡角度（握住车在平衡位置，然后输入 cal）
  else if (cmd == F("cal")) {
    Calibration_Angle = KalFilter.angle;  // 记录当前MPU原始读数作为平衡零点
    Target_Angle = 0;                     // 清除微调
    Serial.print(F("[OK] 已校准! MPU原始读数 = ")); Serial.print(KalFilter.angle);
    Serial.print(F("  校准后角度 = ")); Serial.println(Angle);
    Serial.println(F("  提示：可用 ta<值> 微调平衡角度"));
  }

  // kp
  else if (cmd.startsWith(F("kp"))) {
    float v = rest.toFloat();
    if (v >= 1 && v <= 30) { Balance_Kp = v; Serial.print(F("[OK] KP = ")); Serial.println(v); }
    else Serial.println(F("[ERR] KP 范围: 1 ~ 30"));
  }

  // kd
  else if (cmd.startsWith(F("kd"))) {
    float v = rest.toFloat();
    if (v >= 0.05 && v <= 2) { Balance_Kd = v; Serial.print(F("[OK] KD = ")); Serial.println(v); }
    else Serial.println(F("[ERR] KD 范围: 0.05 ~ 2"));
  }

  // vkp
  else if (cmd.startsWith(F("vkp"))) {
    float v = rest.toFloat();
    if (v >= 0.1 && v <= 8) { Velocity_Kp = v; Serial.print(F("[OK] VKP = ")); Serial.println(v); }
    else Serial.println(F("[ERR] VKP 范围: 0.1 ~ 8"));
  }

  // vki
  else if (cmd.startsWith(F("vki"))) {
    float v = rest.toFloat();
    if (v >= 0.001 && v <= 0.1) { Velocity_Ki = v; Serial.print(F("[OK] VKI = ")); Serial.println(v); }
    else Serial.println(F("[ERR] VKI 范围: 0.001 ~ 0.1"));
  }

  // info
  else if (cmd == F("info")) {
    printInfo();
  }

  // reset
  else if (cmd == F("reset")) {
    Calibration_Angle = 0;
    Balance_Kp    = 15.0;
    Balance_Kd    = 0.4;
    Velocity_Kp   = 2.0;
    Velocity_Ki   = 0.01;
    Target_Angle  = -2.3;
    Target_Speed  = 0;
    Target_Steering = 0;
    Serial.println(F("[OK] 参数已恢复默认（含校准清零）"));
  }

  // test
  else if (cmd == F("test")) {
    if (!testMode) {
      testMode = true;
      testStep = 0;
      Serial.println(F("[测试] 开始自动测试序列..."));
    } else {
      testMode = false;
      testStep = 0;
      Target_Speed = 0;
      Target_Steering = 0;
      Serial.println(F("[测试] 测试序列已手动中止"));
    }
  }

  // raw
  else if (cmd == F("raw")) {
    debugRaw = !debugRaw;
    Serial.print(F("[OK] 调试格式: "));
    Serial.println(debugRaw ? F("原始协议") : F("人类可读"));
  }

  // help
  else if (cmd == F("help")) {
    printHelp();
  }

  // 未知命令
  else {
    Serial.print(F("[ERR] 未知命令: ")); Serial.println(cmd);
    Serial.println(F("  输入 help 查看命令列表"));
  }
}

// ========== 调试输出（人类可读格式）==========
void printDebug() {
  // 第一行：角度和速度
  Serial.print(F("A:")); Serial.print(Angle);
  Serial.print(F(" VL:")); Serial.print(Velocity_Left);
  Serial.print(F(" VR:")); Serial.print(Velocity_Right);
  Serial.print(F(" | "));
  // 第二行：PWM和电池
  Serial.print(F("M1:")); Serial.print(Motor1);
  Serial.print(F(" M2:")); Serial.print(Motor2);
  Serial.print(F(" BAT:")); Serial.print(Battery_Voltage, 1);
  Serial.print(F("V | "));
  // 第三行：状态
  Serial.print(Flag_Stop ? F("STOP") : F("RUN"));
  Serial.print(F(" SPD:")); Serial.print(Target_Speed, 0);
  Serial.print(F(" STR:")); Serial.println(Target_Steering, 0);
}

// ========== 调试输出（原始协议格式，兼容X5解析器）==========
void printDebugRaw() {
  Serial.print(F("$IMU,"));
  Serial.print(KalFilter.angle, 2);
  Serial.print(F(","));
  Serial.print(KalFilter.Gyro_x, 1);
  Serial.print(F(","));
  Serial.print(KalFilter.Gyro_z, 1);
  Serial.print(F(";ODO,"));
  Serial.print(Encoder_Left_Total);
  Serial.print(F(","));
  Serial.print(Encoder_Right_Total);
  Serial.print(F(","));
  Serial.print(Velocity_Left);
  Serial.print(F(","));
  Serial.print(Velocity_Right);
  Serial.print(F(";STS,"));
  Serial.print(Battery_Voltage, 2);
  Serial.print(F(","));
  Serial.print(Flag_Stop);
  Serial.print(F(";TGT,"));
  Serial.print(Target_Speed, 0);
  Serial.print(F(","));
  Serial.println(Target_Steering, 0);
}

// ========== 初始化 ==========
void setup() {
  wdt_disable();
  MCUSR = 0;
  WDTCSR |= _BV(WDCE) | _BV(WDE);
  WDTCSR = 0;

  // 引脚初始化
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(PWMA, 0); analogWrite(PWMB, 0);

  pinMode(ENCODER_L, INPUT); pinMode(DIRECTION_L, INPUT);
  pinMode(ENCODER_R, INPUT); pinMode(DIRECTION_R, INPUT);
  pinMode(KEY, INPUT);

  // 串口
  Serial.begin(115200);
  delay(300);

  Serial.println(F("\n========================================"));
  Serial.println(F("  平衡小车 - Arduino 独立测试版 v1.0"));
  Serial.println(F("  硬件：Arduino UNO + MPU6050 + TB6612"));
  Serial.println(F("========================================"));
  Serial.flush();

  // I2C初始化
  I2C_Recover(); delay(50);
  Wire.begin(); Wire.setClock(100000L); delay(100);

  if (!I2C_Scan(0x68)) {
    Serial.println(F("[错误] 未找到MPU6050! 请检查接线"));
    while (1) { digitalWrite(13, HIGH); delay(200); digitalWrite(13, LOW); delay(200); }
  }
  Serial.println(F("[OK] 找到 MPU6050 (0x68)"));

  // MPU6050配置
  byte err = I2C_WriteReg(0x68, 0x6B, 0x00, 3);  // 唤醒
  if (err) { Serial.println(F("[错误] MPU6050唤醒失败!")); while (1); }
  I2C_WriteReg(0x68, 0x1B, 0x00, 3);  // 陀螺仪 ±250°/s
  I2C_WriteReg(0x68, 0x1C, 0x00, 3);  // 加速度计 ±2g
  I2C_WriteReg(0x68, 0x1A, 0x06, 3);  // DLPF 5Hz
  Serial.println(F("[OK] MPU6050 配置完成"));

  // 左编码器中断
  pinMode(ENCODER_L, INPUT_PULLUP);
  pinMode(DIRECTION_L, INPUT_PULLUP);
  delay(20);
  attachInterrupt(0, READ_ENCODER_L, CHANGE);
  Serial.println(F("[OK] 左编码器中断(Pin2)就绪"));

  // 右编码器轮询
  pinMode(ENCODER_R, INPUT_PULLUP);
  pinMode(DIRECTION_R, INPUT_PULLUP);
  Serial.println(F("[OK] 右编码器轮询(Pin4)就绪"));

  // 定时器
  MsTimer2::set(5, control);
  MsTimer2::start();
  Serial.println(F("[OK] 5ms控制定时器已启动"));

  Serial.println(F("----------------------------------------"));
  Serial.println(F("  系统就绪!"));
  Serial.println(F("  输入 help 查看命令，输入 go 启动平衡"));
  Serial.println(F("========================================\n"));
  Serial.flush();
}

// ========== 主循环 ==========
void loop() {
  static unsigned long lastDebug = 0;
  unsigned long now = millis();

  // 自动测试序列
  if (testMode) {
    testSequence();
  }

  // 调试输出 (每200ms)
  if (now - lastDebug >= 200) {
    if (debugRaw)
      printDebugRaw();
    else
      printDebug();
    lastDebug = now;
  }

  // 串口命令
  while (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0)
      parseCommand(cmd);
  }
}
