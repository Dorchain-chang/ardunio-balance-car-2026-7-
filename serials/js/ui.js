/**
 * UI管理模块
 * 负责界面交互和显示
 */

class UIManager {
    constructor() {
        this.logMaxLines = 100;
        this.logContainer = null;
    }

    /**
     * 初始化UI
     */
    init() {
        this.logContainer = document.getElementById('log-display');
    }

    /**
     * 记录日志消息
     */
    log(message, type = 'info') {
        if (!this.logContainer) return;

        const timestamp = new Date().toLocaleTimeString('zh-CN', {
            hour12: false,
            hour: '2-digit',
            minute: '2-digit',
            second: '2-digit',
            fractionalSecondDigits: 3
        });

        const logEntry = document.createElement('div');
        logEntry.className = `log-entry log-${type}`;

        const timeSpan = document.createElement('span');
        timeSpan.className = 'log-time';
        timeSpan.textContent = `[${timestamp}]`;

        const msgSpan = document.createElement('span');
        msgSpan.className = 'log-message';
        msgSpan.textContent = message;

        logEntry.appendChild(timeSpan);
        logEntry.appendChild(msgSpan);
        this.logContainer.appendChild(logEntry);

        // 滚动到底部
        this.logContainer.scrollTop = this.logContainer.scrollHeight;

        // 限制日志行数
        while (this.logContainer.children.length > this.logMaxLines) {
            this.logContainer.removeChild(this.logContainer.firstChild);
        }
    }

    /**
     * 记录成功消息
     */
    logSuccess(message) {
        this.log(message, 'success');
    }

    /**
     * 记录错误消息
     */
    logError(message) {
        this.log(message, 'error');
    }

    /**
     * 记录警告消息
     */
    logWarning(message) {
        this.log(message, 'warning');
    }

    /**
     * 记录调试消息
     */
    logDebug(message) {
        this.log(message, 'debug');
    }

    /**
     * 清空日志
     */
    clearLog() {
        if (this.logContainer) {
            this.logContainer.innerHTML = '';
        }
    }

    /**
     * 更新连接状态
     */
    updateConnectionStatus(connected, type = null) {
        const statusElement = document.getElementById('connection-status');
        if (!statusElement) return;

        if (connected) {
            statusElement.className = 'status-connected';
            statusElement.textContent = type === 'bluetooth' ? '🔵 蓝牙已连接' : '🟢 串口已连接';
        } else {
            statusElement.className = 'status-disconnected';
            statusElement.textContent = '⚫ 未连接';
        }
    }

    /**
     * 更新按钮状态
     */
    updateButtons(connected) {
        const connectBtBtn = document.getElementById('connect-bluetooth-btn');
        const connectSerialBtn = document.getElementById('connect-serial-btn');
        const disconnectBtn = document.getElementById('disconnect-btn');
        const controlButtons = document.querySelectorAll('.control-btn');
        const pidButtons = document.querySelectorAll('.pid-btn');

        if (connectBtBtn) connectBtBtn.disabled = connected;
        if (connectSerialBtn) connectSerialBtn.disabled = connected;
        if (disconnectBtn) disconnectBtn.disabled = !connected;

        controlButtons.forEach(btn => btn.disabled = !connected);
        pidButtons.forEach(btn => btn.disabled = !connected);
    }

    /**
     * 更新传感器数据显示
     */
    updateSensorDisplay(data) {
        const elements = {
            'angle-value': data.angle?.toFixed(1) + '°',
            'voltage-value': data.voltage?.toFixed(2) + 'V',
            'left-velocity-value': data.leftVelocity,
            'right-velocity-value': data.rightVelocity
        };

        Object.entries(elements).forEach(([id, value]) => {
            const element = document.getElementById(id);
            if (element && value !== undefined) {
                element.textContent = value;
            }
        });

        // 更新电压指示器颜色
        const voltageElement = document.getElementById('voltage-value');
        if (voltageElement && data.voltage) {
            if (data.voltage < 10.5) {
                voltageElement.className = 'value-warning';
            } else if (data.voltage < 11.0) {
                voltageElement.className = 'value-caution';
            } else {
                voltageElement.className = '';
            }
        }
    }

    /**
     * 更新PID参数显示
     */
    updatePIDDisplay(pidData) {
        document.getElementById('balance-kp').value = pidData.balanceKp;
        document.getElementById('balance-kd').value = pidData.balanceKd;
        document.getElementById('velocity-kp').value = pidData.velocityKp;
        document.getElementById('velocity-ki').value = pidData.velocityKi;
    }

    /**
     * 获取PID参数
     */
    getPIDValues() {
        return {
            balanceKp: parseFloat(document.getElementById('balance-kp').value),
            balanceKd: parseFloat(document.getElementById('balance-kd').value),
            velocityKp: parseFloat(document.getElementById('velocity-kp').value),
            velocityKi: parseFloat(document.getElementById('velocity-ki').value)
        };
    }

    /**
     * 显示发送的数据（十六进制）
     */
    displaySentData(data, description) {
        const hexString = Array.from(data)
            .map(b => b.toString(16).padStart(2, '0'))
            .join(' ');
        this.logDebug(`${description}: ${hexString}`);
    }

    /**
     * 显示接收的原始数据
     */
    displayReceivedData(data, parsed = false) {
        if (parsed) {
            // 仅显示非 A/B 类型（即非 sensor_data）的数据
            if (data.type === 'pid_data') {
                this.logDebug(`接收PID数据: ${data.raw}`);
            } else if (data.type && data.type !== 'sensor_data') {
                this.logDebug(`接收其他数据: ${data.raw}`);
            }
        } else {
            // 原始未解析数据（一般不使用，以免混入 A/B 类型）
            this.logDebug(`接收: ${data}`);
        }
    }

    /**
     * 显示统计信息
     */
    updateStatistics(stats) {
        const elements = {
            'packet-count': stats.packetCount,
            'error-count': stats.errorCount,
            'data-rate': stats.dataRate?.toFixed(1) + ' pkt/s'
        };

        Object.entries(elements).forEach(([id, value]) => {
            const element = document.getElementById(id);
            if (element && value !== undefined) {
                element.textContent = value;
            }
        });
    }

    /**
     * 显示通知
     */
    showNotification(message, type = 'info', duration = 3000) {
        const notification = document.createElement('div');
        notification.className = `notification notification-${type}`;
        notification.textContent = message;

        document.body.appendChild(notification);

        setTimeout(() => {
            notification.classList.add('notification-show');
        }, 10);

        setTimeout(() => {
            notification.classList.remove('notification-show');
            setTimeout(() => {
                document.body.removeChild(notification);
            }, 300);
        }, duration);
    }
}

// 导出
window.UIManager = UIManager;

