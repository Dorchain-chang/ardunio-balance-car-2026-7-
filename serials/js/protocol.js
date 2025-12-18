/**
 * 协议解析模块
 * 负责解析Arduino平衡车的通信协议
 */

class ProtocolParser {
    constructor() {
        this.buffer = ''; // 数据缓冲区
        this.onParsedDataCallback = null;
    }

    /**
     * 添加接收到的数据到缓冲区并解析
     */
    addData(data) {
        this.buffer += data;
        this.parseBuffer();
    }

    /**
     * 解析缓冲区中的数据
     */
    parseBuffer() {
        // 持续解析直到没有完整的数据包
        while (true) {
            const packet = this.extractPacket();
            if (!packet) break;

            const parsed = this.parsePacket(packet);
            if (parsed && this.onParsedDataCallback) {
                this.onParsedDataCallback(parsed);
            }
        }
    }

    /**
     * 从缓冲区提取一个完整的数据包
     */
    extractPacket() {
        // 查找数据包 {X...}$
        const startIndex = this.buffer.indexOf('{');
        if (startIndex === -1) {
            // 没有起始标记，清空无用数据
            this.buffer = '';
            return null;
        }

        const endIndex = this.buffer.indexOf('}$', startIndex);
        if (endIndex === -1) {
            // 没有结束标记，保留从起始标记开始的数据
            this.buffer = this.buffer.substring(startIndex);
            return null;
        }

        // 提取数据包
        const packet = this.buffer.substring(startIndex, endIndex + 2);
        // 移除已处理的数据
        this.buffer = this.buffer.substring(endIndex + 2);

        return packet;
    }

    /**
     * 解析单个数据包
     */
    parsePacket(packet) {
        try {
            // 匹配不同类型的数据包（速度值支持负数表示反向）
            const regexA = /\{A(-?\d+):(-?\d+):(\d+):(-?\d+)\}\$/;
            const regexB = /\{B(-?\d+):(\d+):(-?\d+):(-?\d+)\}\$/;
            const regexC = /\{C(\d+):(\d+):(\d+):(\d+)\}\$/;

            let match;

            // 格式A: {A:leftVel:rightVel:voltage:angle}$
            if (match = packet.match(regexA)) {
                return {
                    type: 'sensor_data',
                    format: 'A',
                    leftVelocity: parseInt(match[1]),
                    rightVelocity: parseInt(match[2]),
                    voltageRaw: parseInt(match[3]),
                    voltage: 11.1 + (parseInt(match[3]) / 60),
                    angle: parseInt(match[4]),
                    raw: packet
                };
            }

            // 格式B: {B:angle:voltage:leftVel:rightVel}$
            if (match = packet.match(regexB)) {
                return {
                    type: 'sensor_data',
                    format: 'B',
                    angle: parseInt(match[1]),
                    voltageRaw: parseInt(match[2]),
                    voltage: 11.1 + (parseInt(match[2]) / 60),
                    leftVelocity: parseInt(match[3]),
                    rightVelocity: parseInt(match[4]),
                    raw: packet
                };
            }

            // 格式C: {C:balanceKp:balanceKd:velocityKp:velocityKi}$
            if (match = packet.match(regexC)) {
                return {
                    type: 'pid_data',
                    format: 'C',
                    balanceKp: parseInt(match[1]) / 100,
                    balanceKd: parseInt(match[2]) / 100,
                    velocityKp: parseInt(match[3]) / 100,
                    velocityKi: parseInt(match[4]) / 100,
                    raw: packet
                };
            }

            // 未识别的格式
            return {
                type: 'unknown',
                raw: packet
            };

        } catch (error) {
            console.error('解析错误:', error);
            return null;
        }
    }

    /**
     * 创建PID参数命令
     */
    static createPIDCommand(paramType, value) {
        // paramType: 'balance_kp' (0x30), 'balance_kd' (0x31),
        //           'velocity_kp' (0x32), 'velocity_ki' (0x33)
        const paramIds = {
            'balance_kp': 0x30,
            'balance_kd': 0x31,
            'velocity_kp': 0x32,
            'velocity_ki': 0x33
        };

        const paramId = paramIds[paramType];
        if (!paramId) {
            throw new Error('无效的参数类型');
        }

        // 将值转换为4位ASCII数字
        const intValue = Math.abs(Math.round(value * 100));
        const valueStr = intValue.toString().padStart(4, '0');
        const digits = [];
        for (let i = 0; i < 4; i++) {
            digits.push(valueStr.charCodeAt(i));
        }

        // 创建命令: 0x7B, paramId, 0x00, d0, d1, d2, d3, 0x7D
        return new Uint8Array([0x7B, paramId, 0x00,
                               digits[0], digits[1], digits[2], digits[3], 0x7D]);
    }

    /**
     * 创建获取PID参数命令
     */
    static createGetPIDCommand() {
        return new Uint8Array([0x7B, 0x23, 0x00, 0x50, 0x7D]);
    }

    /**
     * 创建保存PID到EEPROM命令
     */
    static createSavePIDCommand() {
        return new Uint8Array([0x7B, 0x00, 0x00, 0x57, 0x7D]);
    }

    /**
     * 创建运动控制命令
     */
    static createMotionCommand(direction) {
        // direction: 'forward', 'backward', 'left', 'right', 'stop'
        const commands = {
            'forward': 0x01,   // 或 0x41
            'backward': 0x05,  // 或 0x45
            'right': 0x02,     // 或 0x42
            'left': 0x06,      // 或 0x46
            'stop': 0x00
        };

        const cmd = commands[direction];
        if (cmd === undefined) {
            throw new Error('无效的运动方向');
        }

        return new Uint8Array([cmd]);
    }

    /**
     * 设置数据解析回调
     */
    onParsedData(callback) {
        this.onParsedDataCallback = callback;
    }

    /**
     * 清空缓冲区
     */
    clearBuffer() {
        this.buffer = '';
    }
}

// 导出
window.ProtocolParser = ProtocolParser;

