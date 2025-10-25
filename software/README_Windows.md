# Windows 串口通信优化指南

## 概述

本项目已针对Windows系统进行了优化，提供了完整的串口通信解决方案，支持自动端口检测和跨平台兼容性。

## 主要特性

### 1. 自动端口检测
- 智能扫描可用的串口设备
- 自动识别机器人通信协议
- 支持多种USB转串口芯片

### 2. Windows优化
- 优化的串口参数设置
- 改进的线程管理
- 增强的错误处理和重连机制

### 3. 跨平台兼容
- 支持Windows COM端口格式
- 兼容macOS和Linux设备路径
- 统一的API接口

## 文件结构

```
software/
├── modules/communication/
│   ├── port_detector.py              # 跨平台端口检测
│   ├── windows_robot_interface.py   # Windows优化接口
│   ├── robot_interface.py           # 原始接口
│   ├── ldr3_protocol.py             # LDR3协议解析
│   └── command_protocol.py          # 命令协议
├── windows_main.py                   # Windows主程序
├── windows_config.json              # Windows配置文件
└── README_Windows.md                # 本文档
```

## 快速开始

### 1. 安装依赖

```bash
pip install pyserial matplotlib numpy
```

### 2. 运行Windows版本

```bash
# 使用自动端口检测
python windows_main.py --auto-detect

# 指定配置文件
python windows_main.py --config windows_config.json

# 添加目标点
python windows_main.py --goals 1.0,1.0 2.0,2.0 3.0,1.0
```

### 3. 端口检测测试

```python
from modules.communication.port_detector import create_port_detector

# 创建检测器
detector = create_port_detector()

# 扫描可用端口
ports = detector.get_available_ports()
for port in ports:
    print(f"端口: {port['port']}")
    print(f"描述: {port['description']}")

# 自动检测机器人端口
robot_port = detector.find_robot_port()
if robot_port:
    print(f"找到机器人端口: {robot_port}")
```

## 配置说明

### Windows配置文件 (windows_config.json)

```json
{
  "communication": {
    "port": "auto",                    // 自动检测端口
    "baudrate": 921600,               // LDR3协议推荐波特率
    "auto_detect_port": true,         // 启用自动检测
    "timeout": 1.0,
    "retry_count": 3
  },
  "windows_optimizations": {
    "matplotlib_backend": "TkAgg",     // Windows友好的后端
    "thread_priority": "high",         // 高优先级线程
    "buffer_size": 8192,              // 缓冲区大小
    "connection_health_check": true,   // 连接健康检查
    "auto_reconnect": true            // 自动重连
  }
}
```

## 串口通信协议

### LDR3二进制协议

**帧格式：**
- 帧头：20字节
  - 魔数：4字节 (0x3352444C)
  - 节点数：2字节
  - IMU航向角：2字节 (厘度)
  - 左轮里程：4字节 (毫米)
  - 右轮里程：4字节 (毫米)
  - 左轮速度：2字节 (毫米/秒)
  - 右轮速度：2字节 (毫米/秒)

- 节点：N×6字节
  - 距离：2字节 (毫米)
  - 角度：2字节 (厘度)
  - 质量：1字节
  - 标志：1字节

**速度控制命令：**
```
格式: "A&B\r\n"
其中 A = int(vL_mps * 100), B = int(vR_mps * 100)
```

## 常见问题解决

### 1. 端口检测失败

**问题：** 无法自动检测到机器人端口

**解决方案：**
```python
# 手动指定端口
robot = WindowsRobotInterface(port="COM3", auto_detect_port=False)

# 或者扫描所有端口
detector = create_port_detector()
ports = detector.get_available_ports()
print("可用端口:", [p['port'] for p in ports])
```

### 2. 连接不稳定

**问题：** 串口连接经常断开

**解决方案：**
```python
# 启用自动重连
robot = WindowsRobotInterface(auto_detect_port=True)

# 检查连接状态
status = robot.get_connection_status()
print("连接状态:", status)

# 手动重连
robot.reconnect()
```

### 3. 性能优化

**问题：** 系统运行缓慢

**解决方案：**
```json
{
  "windows_optimizations": {
    "matplotlib_backend": "TkAgg",
    "thread_priority": "high",
    "buffer_size": 8192,
    "connection_health_check": true
  }
}
```

## 开发指南

### 1. 自定义端口检测

```python
from modules.communication.port_detector import PortDetector

class CustomPortDetector(PortDetector):
    def find_robot_port(self):
        # 自定义检测逻辑
        preferred_ports = ["COM3", "COM4", "COM5"]
        return self.find_robot_port(preferred_ports=preferred_ports)
```

### 2. 扩展通信协议

```python
from modules.communication.robot_interface import RobotInterface

class CustomRobotInterface(RobotInterface):
    def send_custom_command(self, command):
        # 自定义命令发送
        self.ser.write(command.encode())
```

### 3. 添加新的传感器支持

```python
class ExtendedRobotInterface(WindowsRobotInterface):
    def get_imu_data(self):
        # 获取IMU数据
        pass
    
    def get_encoder_data(self):
        # 获取编码器数据
        pass
```

## 调试技巧

### 1. 启用详细日志

```python
import logging
logging.basicConfig(level=logging.DEBUG)
```

### 2. 测试连接

```python
# 测试串口连接
if robot.test_connection():
    print("连接正常")
else:
    print("连接异常")
```

### 3. 监控数据流

```python
def data_callback(event_type, data):
    print(f"事件: {event_type}, 数据: {data}")

robot = WindowsRobotInterface(data_callback=data_callback)
```

## 性能监控

### 1. 连接状态监控

```python
status = robot.get_connection_status()
print(f"端口: {status['port']}")
print(f"波特率: {status['baudrate']}")
print(f"连接状态: {status['is_open']}")
print(f"接收线程: {status['rx_thread_alive']}")
print(f"发送线程: {status['tx_thread_alive']}")
```

### 2. 数据流量监控

```python
# 监控接收数据
def monitor_data():
    while True:
        pose = robot.get_pose()
        scan = robot.get_lidar_scan()
        print(f"位姿: {pose}, 扫描点数: {len(scan[0])}")
        time.sleep(0.1)
```

## 故障排除

### 1. 权限问题

**Windows：** 确保有串口访问权限
**Linux：** 将用户添加到dialout组
```bash
sudo usermod -a -G dialout $USER
```

### 2. 驱动问题

确保安装了正确的USB转串口驱动：
- FTDI: FT232/FT2232
- Silicon Labs: CP210x
- Prolific: PL2303
- CH340

### 3. 端口占用

```python
# 检查端口是否被占用
import serial.tools.list_ports
ports = serial.tools.list_ports.comports()
for port in ports:
    print(f"端口: {port.device}, 状态: {port.description}")
```

## 联系支持

如果遇到问题，请提供以下信息：
1. 操作系统版本
2. Python版本
3. 错误日志
4. 串口设备信息
5. 配置文件内容

---

**注意：** 本指南基于Windows 10/11系统编写，其他版本可能需要调整。
