/**
 * 蓝牙连接管理模块
 * 支持 Web Bluetooth API 和 Web Serial API
 */

class BluetoothManager {
    constructor() {
        this.device = null;
        this.characteristic = null;
        this.onDataCallback = null;
        this.onStatusCallback = null;
        this.connectionType = null; // 'bluetooth' or 'serial'
        this.port = null;
        this.reader = null;
        this.writer = null;
        this.reading = false;
    }

    /**
     * 检查浏览器支持情况
     */
    checkSupport() {
        return {
            bluetooth: 'bluetooth' in navigator,
            serial: 'serial' in navigator
        };
    }

    /**
     * 连接蓝牙设备 (Web Bluetooth API)
     */
    async connectBluetooth() {
        try {
            this.updateStatus('正在扫描蓝牙设备...');

            // 请求蓝牙设备
            this.device = await navigator.bluetooth.requestDevice({
                filters: [
                    { services: ['0000ffe0-0000-1000-8000-00805f9b34fb'] }, // HM-10/HC-08
                    { namePrefix: 'HC-' }, // HC-05/06
                    { namePrefix: 'HM-' }, // HM-10
                    { namePrefix: 'MLT-BT' }, // MLT-BT05
                ],
                optionalServices: [
                    '0000ffe0-0000-1000-8000-00805f9b34fb', // HM-10/HC-08
                    '00001101-0000-1000-8000-00805f9b34fb'  // SPP
                ]
            });

            this.updateStatus(`已选择设备: ${this.device.name}`);

            // 连接到GATT服务器
            const server = await this.device.gatt.connect();
            this.updateStatus('已连接到GATT服务器');

            // 获取服务
            const service = await server.getPrimaryService(0xFFE0);

            // 获取特征值（用于读写）
            this.characteristic = await service.getCharacteristic("0000ffe1-0000-1000-8000-00805f9b34fb");

            // 启动通知
            await this.characteristic.startNotifications();
            this.characteristic.addEventListener('characteristicvaluechanged', (event) => {
                this.handleBluetoothData(event.target.value);
            });

            this.connectionType = 'bluetooth';
            this.updateStatus('✓ 蓝牙连接成功！');
            return true;

        } catch (error) {
            this.updateStatus(`蓝牙连接失败: ${error.message}`);
            throw error;
        }
    }
    async ensureWriter() {
        if (!this.writer && this.port?.writable) {
            this.writer = this.port.writable.getWriter();
        }
        return this.writer;
    }
    /**
     * 连接串口设备 (Web Serial API)
     */
    async connectSerial(baudRate = 9600) {
        try {
            this.updateStatus('正在请求串口访问...');

            // 请求串口
            this.port = await navigator.serial.requestPort();

            // 打开串口
            await this.port.open({ baudRate });

            // 获取writer
            this.writer = this.port.writable.getWriter();
            await this.ensureWriter();
            // 开始读取
            this.reading = true;
            this.startSerialReading();

            this.connectionType = 'serial';
            this.updateStatus(`✓ 串口连接成功！波特率: ${baudRate}`);
            return true;

        } catch (error) {
            this.updateStatus(`串口连接失败: ${error.message}`);
            throw error;
        }
    }

    /**
     * 开始读取串口数据
     */
    async startSerialReading() {
        while (this.reading && this.port.readable) {
            try {
                this.reader = this.port.readable.getReader();

                while (this.reading) {
                    const { value, done } = await this.reader.read();
                    if (done) break;

                    // 处理接收到的数据
                    const decoder = new TextDecoder();
                    const text = decoder.decode(value);
                    if (this.onDataCallback) {
                        this.onDataCallback(text);
                    }
                }
            } catch (error) {
                if (this.reading) {
                    this.updateStatus(`读取错误: ${error.message}`);
                }
            } finally {
                if (this.reader) {
                    this.reader.releaseLock();
                    this.reader = null;
                }
            }
        }
    }

    /**
     * 处理蓝牙接收的数据
     */
    handleBluetoothData(value) {
        const decoder = new TextDecoder();
        const text = decoder.decode(value);
        if (this.onDataCallback) {
            this.onDataCallback(text);
        }
    }

    /**
     * 发送数据
     */
    async send(data) {
        try {
            if (this.connectionType === 'bluetooth' && this.characteristic) {
                // 蓝牙发送
                let buffer;
                if (typeof data === 'string') {
                    const encoder = new TextEncoder();
                    buffer = encoder.encode(data);
                } else {
                    buffer = data;
                }

                console.log('Sending data via bluetooth:', buffer);
                // 连接到GATT服务器
                const server = await this.device.gatt.connect();
                this.updateStatus('已连接到GATT服务器');

                // 获取服务
                const service = await server.getPrimaryService(0xFFE0);

                const characteristics = await service.getCharacteristics();
                characteristics.forEach(char => {
                    console.log(`UUID: ${char.uuid}`);
                    console.log(`  - canRead: ${char.properties.read}`);
                    console.log(`  - canWrite: ${char.properties.write}`);
                    console.log(`  - canWriteWithoutResponse: ${char.properties.writeWithoutResponse}`);
                    console.log(`  - canNotify: ${char.properties.notify}`);
                    });
                this.characteristic.startNotifications();
                await this.characteristic.writeValueWithResponse(buffer);
                return true;

            } else if (this.connectionType === 'serial' && this.writer) {
                // 串口发送
                let buffer;
                if (typeof data === 'string') {
                    const encoder = new TextEncoder();
                    buffer = encoder.encode(data);
                } else {
                    buffer = data;
                }
                console.log('Sending data via serial:', buffer);    
                await this.writer.write(buffer);
                return true;

            } else {
                throw new Error('未连接设备');
            }
        } catch (error) {
            this.updateStatus(`发送失败: ${error.message}`);
            return false;
        }
    }

    /**
     * 断开连接
     */
    async disconnect() {
        try {
            if (this.connectionType === 'bluetooth' && this.device) {
                if (this.device.gatt.connected) {
                    this.device.gatt.disconnect();
                }
                this.device = null;
                this.characteristic = null;

            } else if (this.connectionType === 'serial' && this.port) {
                this.reading = false;

                if (this.reader) {
                    await this.reader.cancel();
                    this.reader.releaseLock();
                    this.reader = null;
                }

                if (this.writer) {
                    this.writer.releaseLock();
                    this.writer = null;
                }

                await this.port.close();
                this.port = null;
            }

            this.connectionType = null;
            this.updateStatus('已断开连接');
            return true;

        } catch (error) {
            this.updateStatus(`断开连接失败: ${error.message}`);
            return false;
        }
    }

    /**
     * 设置数据接收回调
     */
    onData(callback) {
        this.onDataCallback = callback;
    }

    /**
     * 设置状态更新回调
     */
    onStatus(callback) {
        this.onStatusCallback = callback;
    }

    /**
     * 更新状态
     */
    updateStatus(message) {
        if (this.onStatusCallback) {
            this.onStatusCallback(message);
        }
    }

    /**
     * 检查是否已连接
     */
    isConnected() {
        if (this.connectionType === 'bluetooth') {
            return this.device && this.device.gatt.connected;
        } else if (this.connectionType === 'serial') {
            return this.port && this.port.readable;
        }
        return false;
    }
}

// 导出
window.BluetoothManager = BluetoothManager;

