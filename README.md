# 平衡小车 - 实习适配版

> 本项目基于开源项目 [Minibalance For Arduino](https://github.com/lxbme/arduino-balance-car) 进行适配和调试，作为暑期实习的嵌入式控制实践项目。

---

## 目录

- [实习工作概述](#实习工作概述)
- [与原项目的差异](#与原项目的差异)
- [实习中遇到的问题与解决](#实习中遇到的问题与解决)
- [最终调参记录](#最终调参记录)
- [适配版硬件接线](#适配版硬件接线)
- [使用说明](#使用说明)
- [原项目引用](#原项目引用)

---

## 实习工作概述

本项目为暑期实习期间基于 Arduino UNO 的平衡小车调试与适配工作。原项目为单编码器模式，本实习将其扩展为双编码器模式，并对控制参数进行了实验性调优。

**主要工作：**
- 将原单编码器方案扩展为双编码器方案（右编码器使用轮询模式）
- 修复了 PinChangeInt 库与串口冲突导致系统反复崩溃的问题
- 系统性地调整了 PID 控制参数（直立环 KP/KD、速度环 KP/KI）
- 添加了详细的中文注释和硬件接线说明

## 与原项目的差异

| 项目 | 原项目 | 实习适配版 |
|------|--------|-----------|
| 编码器模式 | 单编码器（仅左轮） | 双编码器（左右独立） |
| 右编码器方案 | 不使用 | Pin 4 轮询模式（5ms 周期） |
| 中断 | 仅外部中断 0 | 外部中断 0 + 定时器中轮询 |
| 参数调优 | 默认参数 | 实验性调优参数 |
| 代码注释 | 英文 | 详细中文注释 |
| 启动诊断 | 无 | I2C 扫描 + 分步初始化 + 错误提示 |

## 实习中遇到的问题与解决

### 问题 1：Pin 4 使用 PinChangeInt 导致系统反复崩溃

**现象：**
右编码器 A 相连接在 Pin 4，尝试使用 PinChangeInt 库启用引脚变化中断后，系统不断重启，串口输出乱码。

**原因分析：**
Arduino UNO 上 Pin 4 属于 PORTD 端口，该端口与串口（TX=Pin1, RX=Pin0）共用。PinChangeInt 在 PORTD 上的中断处理会与硬件串口的中断产生冲突，导致程序反复崩溃。

**解决方案：**
放弃 PinChangeInt 方案，改为在 5ms 定时中断（MsTimer2）中轮询检测 Pin 4 的电平变化。5ms 周期（200Hz）对于平衡车的低速编码器来说采样率足够。

```cpp
// control() 函数中的右编码器轮询代码
byte currRenc = digitalRead(ENCODER_R);
if (currRenc != prevRencState) {
    // 检测到边沿变化，根据 B 相判断方向
    ...
}
prevRencState = currRenc;
```

### 问题 2：I2C 总线锁死导致 MPU6050 初始化失败

**现象：**
上电后 I2C 扫描找不到 MPU6050（0x68），程序卡在初始化阶段。但断开电源重新上电后偶尔能正常工作。

**原因分析：**
MPU6050 在上次通信异常时可能将 SDA 线拉低，导致 I2C 总线处于"锁死"状态。Arduino 的 Wire 库无法自动恢复这种状态。

**解决方案：**
在初始化 I2C 之前增加总线恢复函数，手动模拟 9 个时钟脉冲 + 停止条件来释放 SDA 线。

```cpp
void I2C_Recover() {
    // 模拟时钟脉冲释放被锁死的 SDA
    pinMode(A5, OUTPUT); pinMode(A4, OUTPUT);
    for (int i = 0; i < 9; i++) {
        digitalWrite(A5, LOW); delayMicroseconds(10);
        digitalWrite(A5, HIGH); delayMicroseconds(10);
        if (digitalRead(A4) == HIGH) break;
    }
    // 发送停止条件 ...
}
```

### 问题 3：编码器浮空引脚导致误计数

**现象：**
编码器读取频繁出现随机计数跳变，速度环积分快速累积导致小车抖动。

**原因分析：**
编码器信号线在上电但编码器未输出时处于浮空状态，引脚电平不稳定，被误判为边沿变化。同时中断中的 `digitalRead()` 速度较慢，在 CHANGE 模式下会读到不确定的电平。

**解决方案：**
使用 `INPUT_PULLUP` 模式初始化编码器引脚，利用 Arduino 内部上拉电阻（约 20KΩ-50KΩ）确保空闲时电平稳定在高电平。

```cpp
pinMode(ENCODER_L, INPUT_PULLUP);
pinMode(DIRECTION_L, INPUT_PULLUP);
pinMode(ENCODER_R, INPUT_PULLUP);
pinMode(DIRECTION_R, INPUT_PULLUP);
```

### 问题 4：PID 参数调试经验

**调试过程：**

1. **初始状态：** 使用原项目参数（KP=15, KD=0.4），小车在倾斜后摆动幅度过大，有发散趋势。

2. **第一轮尝试：** 增大 KD 到 1.5 增加阻尼 → 小车高频抖动严重，电机发热。

3. **第二轮尝试：** 减小 KP 到 8，KD 保持 0.8 → 小车响应太慢，无法维持站立。

4. **最终方案：**
   - KP=11.0：适中响应速度，起步温和
   - KD=0.8：足够阻尼，减少高频抖动
   - VKP=2.5：速度环降低加速力度，避免振荡
   - VKI=0.011：积分项减半，减少积分累积导致的超调
   - PWM_MAX=180：限制最大输出，保护电机

**调试心得：**
- KP 决定"多用力"纠正，KD 决定"阻尼多强"
- 先调直立环（速度环关掉），等能站稳再加速度环
- 积分项（KI）最后调，加太多会引入低频振荡

## 最终调参记录

| 参数 | 原值 | 最终值 | 说明 |
|------|------|--------|------|
| BALANCE_KP | 15.0 | 11.0 | 直立环比例，调小使起步温和 |
| BALANCE_KD | 0.4 | 0.8 | 直立环微分，调大增加阻尼减少抖动 |
| VELOCITY_KP | 2.0 | 2.5 | 速度环比例 |
| VELOCITY_KI | 0.01 | 0.011 | 速度环积分 |
| TARGET_ANGLE | 2 | -1.0 | 机械中值/目标角度 |
| PWM_MAX | 255 | 180 | 最大输出限制 |

## 适配版硬件接线

### MPU6050（I2C，A4/A5）
```
VCC  -> Arduino 5V
GND  -> Arduino GND
SCL  -> Arduino A5
SDA  -> Arduino A4
```

### TB6612FNG 电机驱动
```
IN1  -> Pin 12    IN2  -> Pin 13
IN3  -> Pin 7     IN4  -> Pin 6
PWMA -> Pin 10    PWMB -> Pin 9
STBY -> 5V
```

### 编码器（双编码器模式）
```
左编码器 A相 -> Pin 2（外部中断0）  B相 -> Pin 5
右编码器 A相 -> Pin 4（轮询模式）  B相 -> Pin 8
```

### 其他
```
按键 -> Pin 3（内部上拉）
电池电压检测 -> A0（分压电路）
```

## 使用说明

1. 使用 Arduino IDE 打开 `Minibalance_Adapted.ino`
2. 安装依赖库（见 `libs/` 目录）
3. 上传到 Arduino UNO
4. 打开串口监视器（9600 波特率）查看初始化过程
5. 将小车竖立放置，按下按键（Pin 3）启动平衡控制

### 串口输出说明

启动时会逐步输出初始化诊断信息，运行后每 200ms 输出一行状态：
- **Angle**: 当前倾角
- **L/R**: 左右轮编码器计数
- **PWM**: 左右电机 PWM 输出值
- **Bat**: 电池电压
- **Stop**: 停止标志（YES=停止，NO=运行）

## 文件说明

| 文件 | 说明 |
|------|------|
| `Minibalance_Adapted.ino` | 实习适配版代码（双编码器 + 调优参数） |
| `Minibalance_for_Arduino_Final.ino` | 原项目最终版代码 |
| `libs/` | 依赖库文件 |
| `平衡小车原理图.pdf` | 硬件原理图 |

---

## 原项目引用

> 以下内容引自原项目 [lxbme/arduino-balance-car](https://github.com/lxbme/arduino-balance-car) 的 README，保留以尊重原作者知识产权。

<details>
<summary>展开查看原项目 README</summary>

### 主要特性

- ✅ **自平衡控制**：直立PD控制 + 速度PI控制
- ✅ **卡尔曼滤波**：融合MPU6050加速度和陀螺仪数据
- ✅ **串口调试**：9600波特率实时输出状态
- ✅ **按键控制**：启动/停止功能
- ✅ **单编码器模式**：左编码器足以实现平衡

### 原项目硬件要求

#### 必需组件
- Arduino Uno (或兼容板)
- MPU6050 陀螺仪加速度计模块
- TB6612FNG 电机驱动模块
- 2个直流减速电机
- 至少1个编码器（左侧）
- 按键开关
- 电池（推荐11.1V锂电池）

#### 原项目接线说明

**TB6612FNG 电机驱动**
```
IN1  -> Pin 12
IN2  -> Pin 13
IN3  -> Pin 7
IN4  -> Pin 6
PWMA -> Pin 10
PWMB -> Pin 9
```

**编码器（左侧）**
```
Encoder A -> Pin 2 (中断0)
Encoder B -> Pin 5 (方向检测)
```

**按键**
```
KEY -> Pin 3 (内部上拉)
```

### 原项目控制算法

#### 三环控制
1. **直立环（PD）**：5ms周期，维持小车直立
2. **速度环（PI）**：40ms周期，控制前进速度
3. **转向环**：20ms周期（当前禁用）

#### 卡尔曼滤波
- 采样时间：5ms
- 融合加速度计和陀螺仪数据
- 输出平滑的倾角值

### 原项目已知问题与修复

1. **MPU6050初始化问题**：`MPU6050::initialize()` 在某些配置下导致系统重启，改用Wire库直接配置寄存器。

2. **单编码器模式**：Arduino Uno Pin 4 不支持PinChangeInt库，只使用左编码器，右轮速度复制左轮。

### 原项目库依赖

- Wire（Arduino内置）
- I2Cdev
- MPU6050
- KalmanFilter
- MsTimer2
- PinChangeInt

</details>

---

**License & 致谢：** 本项目代码基于 [lxbme/arduino-balance-car](https://github.com/lxbme/arduino-balance-car) 修改，保留原项目的开源协议。感谢原作者的贡献。
