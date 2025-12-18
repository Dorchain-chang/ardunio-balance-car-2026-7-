# 平衡小车

## 主要特性

- ✅ **自平衡控制**：直立PD控制 + 速度PI控制
- ✅ **卡尔曼滤波**：融合MPU6050加速度和陀螺仪数据
- ✅ **串口调试**：9600波特率实时输出状态
- ✅ **按键控制**：启动/停止功能
- ✅ **单编码器模式**：左编码器足以实现平衡

## 硬件要求

### 必需组件
- Arduino Uno (或兼容板)
- MPU6050 陀螺仪加速度计模块
- TB6612FNG 电机驱动模块
- 2个直流减速电机
- 至少1个编码器（左侧）
- 按键开关
- 电池（推荐11.1V锂电池）

### 接线说明

#### MPU6050 (I2C)
```
VCC  -> Arduino 5V
GND  -> Arduino GND
SCL  -> Arduino A5
SDA  -> Arduino A4
```

#### TB6612FNG 电机驱动
```
IN1  -> Pin 12
IN2  -> Pin 13
IN3  -> Pin 7
IN4  -> Pin 6
PWMA -> Pin 10
PWMB -> Pin 9
```

#### 编码器（左侧）
```
Encoder A -> Pin 2 (中断0)
Encoder B -> Pin 5 (方向检测)
```

#### 按键
```
KEY -> Pin 3 (内部上拉)
```

#### 电池电压检测
```
分压电路 -> A0
```

## 控制参数

代码中的关键参数（可根据实际调整）：

```cpp
#define ZHONGZHI 2          // 机械中值
#define BALANCE_KP 15.0     // 直立环P参数
#define BALANCE_KD 0.4      // 直立环D参数
#define VELOCITY_KP 2.0     // 速度环P参数
#define VELOCITY_KI 0.01    // 速度环I参数
```

## 使用说明

1. **上传代码**：使用Arduino IDE上传 `MiniBalance.ino`
2. **打开串口监视器**：波特率设置为 9600
3. **放置小车**：将小车竖立放置
4. **按下按键**：启动平衡控制
5. **观察状态**：通过串口监视器查看实时数据

### 串口输出示例

```
=== MiniBalance Starting ===
Init I2C...OK
Config MPU6050:
  Wake up...err=0
  Set gyro...err=0
  Set accel...err=0
MPU OK
Init Encoder
Encoder L OK
Single encoder mode
Init Timer
Timer set
Timer started

=== SYSTEM READY ===
Mode: Single encoder (left only)
Press KEY (Pin 3) to start/stop
Balance control active

Ang:2 VL:15 VR:15 Bat:11.8V PWM:120,120 Stop:0
```

### 状态指示

- **Ang**: 当前倾角（度）
- **VL/VR**: 左/右轮速度
- **Bat**: 电池电压
- **PWM**: 左/右电机PWM值
- **Stop**: 停止标志 (1=停止, 0=运行)

## 技术细节

### 关键问题修复

1. **MPU6050初始化问题**
   - 原因：`MPU6050::initialize()` 在某些配置下导致系统重启
   - 解决：使用Wire库直接配置寄存器
   ```cpp
   Wire.beginTransmission(0x68);
   Wire.write(0x6B);  // PWR_MGMT_1
   Wire.write(0x00);  // 唤醒
   Wire.endTransmission();
   ```

2. **单编码器模式**
   - 原因：Arduino Uno Pin 4 不支持PinChangeInt库
   - 解决：只使用左编码器，右轮速度复制左轮
   ```cpp
   Velocity_Right = Velocity_Left;
   ```

### 控制算法

#### 三环控制
1. **直立环（PD）**：5ms周期，维持小车直立
2. **速度环（PI）**：40ms周期，控制前进速度
3. **转向环**：20ms周期（当前禁用）

#### 卡尔曼滤波
- 采样时间：5ms
- 融合加速度计和陀螺仪数据
- 输出平滑的倾角值

## 调试与优化

### 参数调整建议

如果小车不稳定，可以尝试：

1. **增加直立环P参数**：提高响应速度
   ```cpp
   #define BALANCE_KP 20.0  // 从15增加到20
   ```

2. **调整直立环D参数**：减少震荡
   ```cpp
   #define BALANCE_KD 0.6   // 从0.4增加到0.6
   ```

3. **修改机械中值**：根据小车结构调整
   ```cpp
   #define ZHONGZHI 0       // 尝试不同值 (-5 到 5)
   ```

### 常见问题

**Q: 小车一直重启？**
A: 检查电池电压，确保供电充足（> 11V）

**Q: 小车无法平衡？**
A: 
1. 检查MPU6050方向和安装位置
2. 调整机械中值 ZHONGZHI
3. 检查电机接线是否正确

**Q: 编码器不工作？**
A: 检查编码器接线和中断引脚

## 库依赖

需要安装以下库（位于 `libs/` 目录）：

- Wire（Arduino内置）
- I2Cdev
- MPU6050
- KalmanFilter
- MsTimer2
- PinChangeInt


