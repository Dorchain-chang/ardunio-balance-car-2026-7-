/**
 * 主应用模块
 * 协调各个模块的工作
 */

class BalanceCarApp {
    constructor() {
        this.bluetooth = new BluetoothManager();
        this.protocol = new ProtocolParser();
        this.charts = new ChartManager();
        this.ui = new UIManager();

        this.statistics = {
            packetCount: 0,
            errorCount: 0,
            startTime: Date.now(),
            lastUpdateTime: Date.now()
        };

        this.statsUpdateInterval = null;
    }

    /**
     * 初始化应用
     */
    init() {
        this.ui.init();
        this.checkBrowserSupport();
        this.setupEventListeners();
        this.charts.initCharts();
        this.setupProtocolCallbacks();
        this.setupBluetoothCallbacks();
        this.startStatisticsUpdater();

        this.ui.logSuccess('🚗 平衡车蓝牙调试工具已启动');
    }

    /**
     * 检查浏览器支持
     */
    checkBrowserSupport() {
        const support = this.bluetooth.checkSupport();

        if (!support.bluetooth && !support.serial) {
            this.ui.logError('您的浏览器不支持蓝牙或串口API！请使用最新版Chrome、Edge或Opera');
            document.getElementById('connect-bluetooth-btn').disabled = true;
            document.getElementById('connect-serial-btn').disabled = true;
            return;
        }

        if (!support.bluetooth) {
            this.ui.logWarning('浏览器不支持Web Bluetooth API，蓝牙连接不可用');
            document.getElementById('connect-bluetooth-btn').disabled = true;
        }

        if (!support.serial) {
            this.ui.logWarning('浏览器不支持Web Serial API，串口连接不可用');
            document.getElementById('connect-serial-btn').disabled = true;
        }

        if (support.bluetooth && support.serial) {
            this.ui.log('✓ 浏览器支持蓝牙和串口通信');
        }
    }

    /**
     * 设置事件监听器
     */
    setupEventListeners() {
        // 连接按钮
        document.getElementById('connect-bluetooth-btn')?.addEventListener('click', () => {
            this.connectBluetooth();
        });

        document.getElementById('connect-serial-btn')?.addEventListener('click', () => {
            this.connectSerial();
        });

        document.getElementById('disconnect-btn')?.addEventListener('click', () => {
            this.disconnect();
        });

        // PID控制按钮
        document.getElementById('get-pid-btn')?.addEventListener('click', () => {
            this.getPID();
        });

        document.getElementById('send-pid-btn')?.addEventListener('click', () => {
            this.sendPID();
        });

        document.getElementById('save-pid-btn')?.addEventListener('click', () => {
            this.savePID();
        });

        // 运动控制按钮
        document.getElementById('forward-btn')?.addEventListener('click', () => {
            this.sendMotionCommand('forward');
        });

        document.getElementById('backward-btn')?.addEventListener('click', () => {
            this.sendMotionCommand('backward');
        });

        document.getElementById('left-btn')?.addEventListener('click', () => {
            this.sendMotionCommand('left');
        });

        document.getElementById('right-btn')?.addEventListener('click', () => {
            this.sendMotionCommand('right');
        });

        document.getElementById('stop-btn')?.addEventListener('click', () => {
            this.sendMotionCommand('stop');
        });

        // 工具按钮
        document.getElementById('clear-log-btn')?.addEventListener('click', () => {
            this.ui.clearLog();
        });

        document.getElementById('clear-charts-btn')?.addEventListener('click', () => {
            this.charts.clearCharts();
            this.ui.logSuccess('图表已清空');
        });

        // 波特率选择
        document.getElementById('baudrate-select')?.addEventListener('change', (e) => {
            this.selectedBaudRate = parseInt(e.target.value);
        });
    }

    /**
     * 设置协议解析回调
     */
    setupProtocolCallbacks() {
        this.protocol.onParsedData((data) => {
            this.handleParsedData(data);
        });
    }

    /**
     * 设置蓝牙回调
     */
    setupBluetoothCallbacks() {
        this.bluetooth.onData((rawData) => {
            this.protocol.addData(rawData);
        });

        this.bluetooth.onStatus((message) => {
            this.ui.log(message);
        });
    }

    /**
     * 处理解析后的数据
     */
    handleParsedData(data) {
        this.statistics.packetCount++;
        if (data.type != 'sensor_data' && data.type != 'pid_data') {
            this.ui.logDebug(`收到了类型是 ${data.type} 的数据`);
        }
        if (data.type === 'sensor_data') {
            // 更新图表
            this.charts.updateCharts(data);
            // 更新传感器显示
            this.ui.updateSensorDisplay(data);

        } else if (data.type === 'pid_data') {
            // 更新PID参数显示
            // this.ui.updatePIDDisplay(data);
            // 显示接收到的数据（过滤掉A/B类型）
            this.ui.displayReceivedData(data, true);
            // this.ui.logSuccess(`✓ PID参数已更新 - Kp: ${data.balanceKp}, Kd: ${data.balanceKd}, VKp: ${data.velocityKp}, VKi: ${data.velocityKi}`);

        } else if (data.type === 'unknown') {
            this.statistics.errorCount++;
            // 显示未知但已接收的数据（非A/B类型）
            this.ui.displayReceivedData(data, false);
            this.ui.logWarning(`收到未知格式数据: ${data.raw}`);
        }
    }

    /**
     * 连接蓝牙
     */
    async connectBluetooth() {
        try {
            await this.bluetooth.connectBluetooth();
            this.ui.updateConnectionStatus(true, 'bluetooth');
            this.ui.updateButtons(true);
            this.ui.showNotification('蓝牙连接成功！', 'success');
        } catch (error) {
            this.ui.logError(`蓝牙连接失败: ${error.message}`);
            this.ui.showNotification('蓝牙连接失败', 'error');
        }
    }

    /**
     * 连接串口
     */
    async connectSerial() {
        try {
            const baudRate = this.selectedBaudRate || 9600;
            await this.bluetooth.connectSerial(baudRate);
            this.ui.updateConnectionStatus(true, 'serial');
            this.ui.updateButtons(true);
            this.ui.showNotification('串口连接成功！', 'success');
        } catch (error) {
            this.ui.logError(`串口连接失败: ${error.message}`);
            this.ui.showNotification('串口连接失败', 'error');
        }
    }

    /**
     * 断开连接
     */
    async disconnect() {
        try {
            await this.bluetooth.disconnect();
            this.ui.updateConnectionStatus(false);
            this.ui.updateButtons(false);
            this.protocol.clearBuffer();
            this.ui.showNotification('已断开连接', 'info');
        } catch (error) {
            this.ui.logError(`断开连接失败: ${error.message}`);
        }
    }

    /**
     * 获取PID参数
     */
    async getPID() {
        try {
            const command = ProtocolParser.createGetPIDCommand();
            this.ui.displaySentData(command, '发送获取PID命令');

            const success = await this.bluetooth.send(command);
            if (success) {
                this.ui.log('✓ 已发送获取PID请求，等待响应...');
            } else {
                this.ui.logError('✗ 发送获取PID请求失败');
            }
        } catch (error) {
            this.ui.logError(`获取PID失败: ${error.message}`);
        }
    }

    /**
     * 发送PID参数
     */
    async sendPID() {
        try {
            const pid = this.ui.getPIDValues();

            // 验证参数
            if (Object.values(pid).some(v => isNaN(v))) {
                this.ui.logError('PID参数无效，请检查输入');
                this.ui.showNotification('PID参数无效', 'error');
                return;
            }

            // 依次发送各个参数
            const params = [
                { type: 'balance_kp', value: pid.balanceKp, name: '平衡Kp' },
                { type: 'balance_kd', value: pid.balanceKd, name: '平衡Kd' },
                { type: 'velocity_kp', value: pid.velocityKp, name: '速度Kp' },
                { type: 'velocity_ki', value: pid.velocityKi, name: '速度Ki' }
            ];

            for (const param of params) {
                const command = ProtocolParser.createPIDCommand(param.type, param.value);
                this.ui.displaySentData(command, `发送${param.name}: ${param.value}`);

                const success = await this.bluetooth.send(command);
                if (!success) {
                    this.ui.logError(`✗ 发送${param.name}失败`);
                    return;
                }

                // 短暂延迟确保Arduino处理
                await new Promise(resolve => setTimeout(resolve, 50));
            }

            this.ui.logSuccess('✓ 所有PID参数发送完成');
            this.ui.showNotification('PID参数已发送', 'success');

        } catch (error) {
            this.ui.logError(`发送PID失败: ${error.message}`);
            this.ui.showNotification('发送PID失败', 'error');
        }
    }

    /**
     * 保存PID到EEPROM
     */
    async savePID() {
        try {
            const command = ProtocolParser.createSavePIDCommand();
            this.ui.displaySentData(command, '发送保存EEPROM命令');

            const success = await this.bluetooth.send(command);
            if (success) {
                this.ui.logSuccess('✓ 已发送保存PID到EEPROM命令');
                this.ui.showNotification('PID参数已保存到EEPROM', 'success');
            } else {
                this.ui.logError('✗ 发送保存命令失败');
            }
        } catch (error) {
            this.ui.logError(`保存PID失败: ${error.message}`);
        }
    }

    /**
     * 发送运动控制命令
     */
    async sendMotionCommand(direction) {
        try {
            const command = ProtocolParser.createMotionCommand(direction);
            const success = await this.bluetooth.send(command);

            if (success) {
                this.ui.log(`发送运动命令: ${direction}`);
            } else {
                this.ui.logError(`发送运动命令失败: ${direction}`);
            }
        } catch (error) {
            this.ui.logError(`发送运动命令失败: ${error.message}`);
        }
    }

    /**
     * 启动统计信息更新器
     */
    startStatisticsUpdater() {
        this.statsUpdateInterval = setInterval(() => {
            const now = Date.now();
            const elapsed = (now - this.statistics.lastUpdateTime) / 1000;

            if (elapsed > 0) {
                const dataRate = this.statistics.packetCount / elapsed;
                this.ui.updateStatistics({
                    packetCount: this.statistics.packetCount,
                    errorCount: this.statistics.errorCount,
                    dataRate: dataRate
                });
            }

            this.statistics.lastUpdateTime = now;
        }, 1000);
    }

    /**
     * 停止统计信息更新器
     */
    stopStatisticsUpdater() {
        if (this.statsUpdateInterval) {
            clearInterval(this.statsUpdateInterval);
            this.statsUpdateInterval = null;
        }
    }
}

// 页面加载完成后初始化应用
document.addEventListener('DOMContentLoaded', () => {
    window.app = new BalanceCarApp();
    window.app.init();
});

