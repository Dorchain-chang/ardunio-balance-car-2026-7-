# 平衡车蓝牙串口调试工具

一个功能完整、模块化设计的平衡车蓝牙/串口通信调试工具。支持实时数据监控、PID参数调试、运动控制等功能。

## ✨ 特性

- 🔵 **蓝牙支持** - 使用Web Bluetooth API，支持常见蓝牙模块（HC-05/06, HM-10, MLT-BT05等）
- 🟢 **串口支持** - 使用Web Serial API，支持USB串口连接
- 📊 **实时图表** - 实时显示角度、速度、电压等传感器数据
- ⚙️ **PID调试** - 支持获取、设置、保存PID参数到EEPROM
- 🕹️ **运动控制** - 远程控制小车前进、后退、转向
- 📝 **通信日志** - 详细的十六进制数据日志，便于调试
- 🎨 **现代UI** - 美观的渐变设计，响应式布局
- 🧩 **模块化** - 代码分模块编写，易于维护和扩展

## 📁 文件结构

```
serials/
├── index.html              # 主页面
├── README.md              # 说明文档
├── css/
│   └── style.css          # 样式文件
└── js/
    ├── bluetooth.js       # 蓝牙连接管理模块
    ├── protocol.js        # 协议解析模块
    ├── chart-manager.js   # 图表管理模块
    ├── ui.js              # UI管理模块
    └── app.js             # 主应用逻辑模块
```

## 🚀 使用方法

### 1. 浏览器要求

使用最新版本的以下浏览器之一：
- Google Chrome (88+)
- Microsoft Edge (88+)
- Opera (74+)

**注意**: Firefox和Safari目前不支持Web Bluetooth API。

### 2. 启动方法

#### 方式一：直接打开（推荐用于测试）

```bash
# 在serials目录下
open index.html
```

#### 方式二：使用本地服务器（推荐用于开发）

```bash
# 使用Python
cd serials
python -m http.server 8000

# 或使用Node.js
npx http-server -p 8000
```

然后在浏览器中访问：`http://localhost:8000`

### 3. 连接设备

#### 蓝牙连接
1. 确保蓝牙模块已上电
2. 点击"连接蓝牙"按钮
3. 在弹出的设备列表中选择您的设备（通常以HC-、HM-或MLT-BT开头）
4. 等待连接成功提示

#### 串口连接
1. 使用USB线连接Arduino
2. 选择合适的波特率（默认9600）
3. 点击"连接串口"按钮
4. 在弹出的端口列表中选择对应的COM口
5. 等待连接成功提示

### 4. 功能使用

#### PID参数调试
- **获取PID**: 点击"获取PID"从Arduino读取当前PID参数
- **发送PID**: 修改参数后点击"发送PID"更新Arduino的PID参数
- **保存到EEPROM**: 点击"保存到EEPROM"将当前参数永久保存

#### 运动控制
使用方向按钮控制小车运动：
- ⬆️ 前进
- ⬇️ 后退
- ⬅️ 左转
- ➡️ 右转
- ⏹️ 停止

#### 数据监控
- 实时图表自动更新，显示最近100个数据点
- 传感器数据面板显示当前值
- 电压低于11V时会显示警告颜色

## 🔧 Arduino通信协议

### 接收数据格式

#### 传感器数据（格式A）
```
{A<左轮速度>:<右轮速度>:<电压>:<角度>}$
示例: {A25:28:65:2}$
```

#### 传感器数据（格式B）
```
{B<角度>:<电压>:<左轮速度>:<右轮速度>}$
示例: {B2:65:25:28}$
```

#### PID参数数据（格式C）
```
{C<平衡Kp*100>:<平衡Kd*100>:<速度Kp*100>:<速度Ki*100>}$
示例: {C1500:40:200:1}$
```

### 发送命令格式

#### 获取PID参数
```
0x7B 0x00 0x00 0x50 0x7D
```

#### 设置PID参数
```
0x7B <参数ID> 0x00 <数字1> <数字2> <数字3> <数字4> 0x7D

参数ID:
- 0x30: Balance_Kp
- 0x31: Balance_Kd
- 0x32: Velocity_Kp
- 0x33: Velocity_Ki

数字为ASCII码，例如设置Balance_Kp=15.00:
0x7B 0x30 0x00 0x31 0x35 0x30 0x30 0x7D
      (Kp) (0)  (1)  (5)  (0)  (0)
```

#### 保存到EEPROM
```
0x7B 0x00 0x00 0x57 0x7D
```

#### 运动控制
```
0x01 - 前进
0x05 - 后退
0x02 - 右转
0x06 - 左转
0x00 - 停止
```

## 🎯 代码模块说明

### bluetooth.js - 蓝牙连接管理
负责蓝牙和串口的连接、断开、数据收发。

主要方法：
- `connectBluetooth()` - 连接蓝牙设备
- `connectSerial(baudRate)` - 连接串口设备
- `send(data)` - 发送数据
- `disconnect()` - 断开连接
- `onData(callback)` - 设置数据接收回调

### protocol.js - 协议解析
负责解析Arduino通信协议，构建发送命令。

主要方法：
- `addData(data)` - 添加接收数据到缓冲区
- `parsePacket(packet)` - 解析单个数据包
- `createPIDCommand(type, value)` - 创建PID设置命令
- `createGetPIDCommand()` - 创建获取PID命令
- `createSavePIDCommand()` - 创建保存命令
- `createMotionCommand(direction)` - 创建运动控制命令

### chart-manager.js - 图表管理
负责实时图表的创建和更新。

主要方法：
- `initCharts()` - 初始化所有图表
- `updateCharts(sensorData)` - 更新图表数据
- `clearCharts()` - 清空图表
- `setMaxDataPoints(max)` - 设置最大数据点数

### ui.js - UI管理
负责界面交互和显示。

主要方法：
- `log(message, type)` - 记录日志
- `logSuccess/Error/Warning()` - 记录不同类型的日志
- `updateConnectionStatus(connected, type)` - 更新连接状态
- `updateSensorDisplay(data)` - 更新传感器显示
- `updatePIDDisplay(pidData)` - 更新PID参数显示
- `showNotification(message, type)` - 显示通知

### app.js - 主应用
协调各个模块，处理用户操作。

主要方法：
- `init()` - 初始化应用
- `connectBluetooth/Serial()` - 连接设备
- `getPID/sendPID/savePID()` - PID操作
- `sendMotionCommand(direction)` - 发送运动命令

## 🛠️ 自定义和扩展

### 添加新的数据格式
在 `protocol.js` 的 `parsePacket()` 方法中添加新的正则表达式匹配。

### 修改图表样式
在 `chart-manager.js` 中修改Chart.js配置选项。

### 添加新的控制命令
在 `protocol.js` 中添加新的命令生成方法，在 `app.js` 中添加对应的处理逻辑。

### 调整UI样式
修改 `css/style.css` 中的样式定义。

## 📝 常见问题

### 1. 找不到蓝牙设备
- 确保蓝牙模块已上电且未被其他设备连接
- 检查蓝牙模块是否支持BLE（低功耗蓝牙）
- 某些经典蓝牙模块（如HC-05）可能不被Web Bluetooth API支持，建议使用串口连接

### 2. 无法连接串口
- 确保没有其他程序（如Arduino IDE）占用串口
- 检查波特率设置是否与Arduino代码一致
- 某些系统可能需要额外的驱动程序

### 3. 数据显示不正常
- 检查Arduino发送的数据格式是否符合协议
- 查看通信日志中的原始数据
- 确认波特率设置正确

### 4. PID参数无法保存
- 确保小车处于运行状态（Flag_Stop == 0）
- 检查通信日志确认命令是否发送成功
- Arduino可能需要一定时间处理EEPROM写入

## 📄 许可证

MIT License

## 👥 作者

Created for Arduino Balance Car Project

## 🤝 贡献

欢迎提交Issue和Pull Request！

