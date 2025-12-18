/**
 * 图表管理模块
 * 负责管理实时数据图表
 */

class ChartManager {
    constructor() {
        this.charts = {};
        this.maxDataPoints = 100;
        this.data = {
            time: [],
            angle: [],
            velocityLeft: [],
            velocityRight: [],
            voltage: []
        };
    }

    /**
     * 初始化所有图表
     */
    initCharts() {
        // 角度图表
        this.charts.angle = new Chart(document.getElementById('angle-chart'), {
            type: 'line',
            data: {
                labels: this.data.time,
                datasets: [{
                    label: '平衡角度 (°)',
                    data: this.data.angle,
                    borderColor: 'rgb(255, 99, 132)',
                    backgroundColor: 'rgba(255, 99, 132, 0.1)',
                    tension: 0.2,
                    fill: true,
                    pointRadius: 0
                }]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                animation: false,
                scales: {
                    y: {
                        title: {
                            display: true,
                            text: '角度 (度)'
                        }
                    },
                    x: {
                        display: false
                    }
                },
                plugins: {
                    legend: {
                        display: true
                    }
                }
            }
        });

        // 速度图表
        this.charts.velocity = new Chart(document.getElementById('velocity-chart'), {
            type: 'line',
            data: {
                labels: this.data.time,
                datasets: [{
                    label: '左轮速度',
                    data: this.data.velocityLeft,
                    borderColor: 'rgb(75, 192, 192)',
                    backgroundColor: 'rgba(75, 192, 192, 0.1)',
                    tension: 0.2,
                    fill: true,
                    pointRadius: 0
                }, {
                    label: '右轮速度',
                    data: this.data.velocityRight,
                    borderColor: 'rgb(54, 162, 235)',
                    backgroundColor: 'rgba(54, 162, 235, 0.1)',
                    tension: 0.2,
                    fill: true,
                    pointRadius: 0
                }]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                animation: false,
                scales: {
                    y: {
                        title: {
                            display: true,
                            text: '速度'
                        }
                    },
                    x: {
                        display: false
                    }
                },
                plugins: {
                    legend: {
                        display: true
                    }
                }
            }
        });

        // 电压图表
        this.charts.voltage = new Chart(document.getElementById('voltage-chart'), {
            type: 'line',
            data: {
                labels: this.data.time,
                datasets: [{
                    label: '电池电压 (V)',
                    data: this.data.voltage,
                    borderColor: 'rgb(255, 205, 86)',
                    backgroundColor: 'rgba(255, 205, 86, 0.1)',
                    tension: 0.2,
                    fill: true,
                    pointRadius: 0
                }]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                animation: false,
                scales: {
                    y: {
                        title: {
                            display: true,
                            text: '电压 (V)'
                        },
                        min: 10,
                        max: 13
                    },
                    x: {
                        display: false
                    }
                },
                plugins: {
                    legend: {
                        display: true
                    }
                }
            }
        });
    }

    /**
     * 更新图表数据
     */
    updateCharts(sensorData) {
        // 添加时间标签
        const now = new Date();
        const timeLabel = `${now.getMinutes()}:${now.getSeconds().toString().padStart(2, '0')}`;

        this.data.time.push(timeLabel);
        this.data.angle.push(sensorData.angle);
        this.data.velocityLeft.push(sensorData.leftVelocity);
        this.data.velocityRight.push(sensorData.rightVelocity);
        this.data.voltage.push(sensorData.voltage);

        // 限制数据点数量
        if (this.data.time.length > this.maxDataPoints) {
            this.data.time.shift();
            this.data.angle.shift();
            this.data.velocityLeft.shift();
            this.data.velocityRight.shift();
            this.data.voltage.shift();
        }

        // 更新所有图表
        Object.values(this.charts).forEach(chart => {
            if (chart) chart.update();
        });
    }

    /**
     * 清空图表数据
     */
    clearCharts() {
        this.data.time = [];
        this.data.angle = [];
        this.data.velocityLeft = [];
        this.data.velocityRight = [];
        this.data.voltage = [];

        Object.values(this.charts).forEach(chart => {
            if (chart) chart.update();
        });
    }

    /**
     * 设置最大数据点数
     */
    setMaxDataPoints(max) {
        this.maxDataPoints = max;
    }

    /**
     * 销毁所有图表
     */
    destroy() {
        Object.values(this.charts).forEach(chart => {
            if (chart) chart.destroy();
        });
        this.charts = {};
    }
}

// 导出
window.ChartManager = ChartManager;

