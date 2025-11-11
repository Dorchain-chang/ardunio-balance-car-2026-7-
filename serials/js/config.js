/**
 * 配置文件
 * 用户可以在这里修改默认设置
 */

const CONFIG = {
    // 图表配置
    chart: {
        maxDataPoints: 100,          // 最大显示数据点数
        updateInterval: 50,          // 图表更新间隔（毫秒）
        colors: {
            angle: 'rgb(255, 99, 132)',
            velocityLeft: 'rgb(75, 192, 192)',
            velocityRight: 'rgb(54, 162, 235)',
            voltage: 'rgb(255, 205, 86)'
        }
    },

    // 串口配置
    serial: {
        defaultBaudRate: 9600,       // 默认波特率
        availableBaudRates: [        // 可选波特率列表
            9600, 19200, 38400, 57600, 115200, 128000
        ]
    },

    // 蓝牙配置
    bluetooth: {
        // 蓝牙服务UUID（HM-10/HC-08标准）
        serviceUUID: '0000ffe0-0000-1000-8000-00805f9b34fb',
        characteristicUUID: '0000ffe1-0000-1000-8000-00805f9b34fb',

        // 设备名称前缀过滤
        devicePrefixes: ['HC-', 'HM-', 'MLT-BT'],

        // 重连设置
        autoReconnect: false,        // 是否自动重连
        reconnectDelay: 3000,        // 重连延迟（毫秒）
        maxReconnectAttempts: 3      // 最大重连次数
    },

    // PID参数配置
    pid: {
        defaults: {
            balanceKp: 15,
            balanceKd: 0.4,
            velocityKp: 2,
            velocityKi: 0.01
        },
        ranges: {
            balanceKp: { min: 0, max: 100, step: 0.1 },
            balanceKd: { min: 0, max: 10, step: 0.01 },
            velocityKp: { min: 0, max: 20, step: 0.1 },
            velocityKi: { min: 0, max: 1, step: 0.001 }
        }
    },

    // UI配置
    ui: {
        logMaxLines: 100,            // 日志最大行数
        notificationDuration: 3000,  // 通知显示时长（毫秒）
        theme: 'default',            // 主题：default, dark, light

        // 电压警告阈值
        voltageWarning: {
            critical: 10.5,          // 严重警告（红色）
            warning: 11.0            // 警告（黄色）
        }
    },

    // 协议配置
    protocol: {
        // 数据包分隔符
        startMarker: '{',
        endMarker: '}$',

        // 命令字节
        commands: {
            getPID: 0x50,
            savePID: 0x57,
            motionForward: 0x01,
            motionBackward: 0x05,
            motionLeft: 0x06,
            motionRight: 0x02,
            motionStop: 0x00
        },

        // 参数ID
        paramIds: {
            balanceKp: 0x30,
            balanceKd: 0x31,
            velocityKp: 0x32,
            velocityKi: 0x33
        }
    },

    // 调试配置
    debug: {
        enabled: true,               // 是否启用调试模式
        showRawData: true,           // 是否显示原始数据
        showHexData: true,           // 是否显示十六进制数据
        logLevel: 'debug'            // 日志级别：debug, info, warning, error
    },

    // 统计配置
    statistics: {
        updateInterval: 1000,        // 统计更新间隔（毫秒）
        enabled: true                // 是否启用统计
    }
};

// 导出配置
window.CONFIG = CONFIG;

// 工具函数：合并配置
window.mergeConfig = function(userConfig) {
    return Object.assign({}, CONFIG, userConfig);
};

// 工具函数：获取配置值
window.getConfig = function(path) {
    const keys = path.split('.');
    let value = CONFIG;
    for (const key of keys) {
        value = value[key];
        if (value === undefined) return null;
    }
    return value;
};

// 工具函数：设置配置值
window.setConfig = function(path, newValue) {
    const keys = path.split('.');
    let obj = CONFIG;
    for (let i = 0; i < keys.length - 1; i++) {
        obj = obj[keys[i]];
        if (obj === undefined) return false;
    }
    obj[keys[keys.length - 1]] = newValue;
    return true;
};

console.log('✓ 配置文件已加载', CONFIG);

