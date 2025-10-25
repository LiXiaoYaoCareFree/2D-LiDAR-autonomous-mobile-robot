# 完全匹配硬件代码的蓝牙控制程序

## 🎯 程序特点

这个程序**完全匹配**你的`hardware.c`代码，同时借鉴了`bluetooth2.py`的优秀显示方式。

### 📋 数据格式完全匹配

#### 1. **控制命令格式** - 完全匹配hardware.c
```c
// hardware.c中的命令处理
if (bluetooth_rx_data >= '0' && bluetooth_rx_data <= '5') {
    target_direction = bluetooth_rx_data - '0';
}
```

**Python程序对应**：
```python
self.commands = {
    'STOP': '0',      # DIR_STOP
    'FORWARD': '1',   # DIR_FORWARD  
    'BACKWARD': '2',  # DIR_BACKWARD
    'LEFT': '3',      # DIR_LEFT
    'RIGHT': '4',     # DIR_RIGHT
    'UTURN': '5'      # DIR_UTURN
}
```

#### 2. **激光雷达数据格式** - 匹配hardware.c的SendLidarToBluetooth
```c
// hardware.c中的雷达数据发送
void SendLidarToBluetooth(float angle, float distance, uint8_t quality) {
    char buffer[32];
    int len = snprintf(buffer, sizeof(buffer), "A:%.2f,D:%.2f,Q:%d\n", angle, distance, quality);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, 100);
}
```

**Python程序解析**：
```python
# 解析雷达数据（格式：A:123.45, D:0.678, Q:100）
radar_matches = re.findall(r'A:([-+]?\d*\.\d+),\s*D:([-+]?\d*\.\d+),\s*Q:(\d+)', self.data_buffer)
```

#### 3. **电机数据格式** - 匹配hardware.c的电机数据
```c
// hardware.c中的电机数据发送
snprintf(uart_buf, sizeof(uart_buf), "MotorA:%.1f,RPMB:%.1f\n", rpmA, rpmB);
HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, strlen(uart_buf), 100);
```

**Python程序解析**：
```python
# 解析电机数据（格式：MotorA:123.45,RPMB:67.89）
motor_matches = re.findall(r'MotorA:([-+]?\d*\.\d+),RPMB:([-+]?\d*\.\d+)', self.data_buffer)
```

#### 4. **IMU数据格式** - 匹配hardware.c的MPU6500数据
```c
// hardware.c中的IMU数据发送
snprintf(uart_buf, sizeof(uart_buf),
    "AccX:%.2f,AccY:%.2f,AccZ:%.2f,GyroX:%.2f,GyroY:%.2f,GyroZ:%.2f\n",
    mpu_data.accel_x, mpu_data.accel_y, mpu_data.accel_z,
    mpu_data.gyro_x, mpu_data.gyro_y, mpu_data.gyro_z);
```

**Python程序解析**：
```python
# 解析IMU数据（格式：AccX:1.23,AccY:4.56,AccZ:7.89,GyroX:0.12,GyroY:0.34,GyroZ:0.56）
imu_matches = re.findall(r'AccX:([-+]?\d*\.\d+),AccY:([-+]?\d*\.\d+),AccZ:([-+]?\d*\.\d+),GyroX:([-+]?\d*\.\d+),GyroY:([-+]?\d*\.\d+),GyroZ:([-+]?\d*\.\d+)', self.data_buffer)
```

### 🎨 显示方式借鉴bluetooth2.py

#### 1. **极坐标雷达显示**
- 完全借鉴`bluetooth2.py`的极坐标显示方式
- 支持鼠标滚轮缩放
- 实时更新雷达扫描点

#### 2. **三轴角度显示**
- 实时显示roll、pitch、yaw角度
- 与`bluetooth2.py`的显示方式一致

#### 3. **数据过滤和异常检测**
- 借鉴`bluetooth2.py`的异常值检测算法
- 质量过滤和距离范围检查

## 🚀 使用方法

### 1. **启动程序**
```bash
cd pc_control
python start_hardware_matched.py
```

### 2. **连接设置**
- 选择串口号（如COM3）
- 波特率115200（匹配hardware.c）
- 点击"Connect"

### 3. **控制小车**
- **键盘控制**：方向键 + 空格
- **按钮控制**：点击界面按钮
- **命令格式**：完全匹配hardware.c

### 4. **数据显示**
- **雷达扫描**：极坐标实时显示
- **三轴角度**：roll、pitch、yaw
- **电机数据**：rpmA、rpmB
- **IMU数据**：加速度和陀螺仪

## 🔧 技术特性

### 1. **数据解析完全匹配**
- 激光雷达：`A:角度,D:距离,Q:质量`
- 电机数据：`MotorA:RPM,RPMB:RPM`
- IMU数据：`AccX:值,AccY:值,AccZ:值,GyroX:值,GyroY:值,GyroZ:值`

### 2. **控制命令完全匹配**
- 发送单字符命令：'0'-'5'
- 完全匹配hardware.c的命令处理逻辑

### 3. **显示方式借鉴bluetooth2.py**
- 极坐标雷达显示
- 鼠标滚轮缩放
- 异常值检测和过滤
- 实时数据更新

### 4. **线程安全设计**
- 串口读取线程
- GUI更新线程
- 数据解析线程

## 📊 功能对比

| 功能 | hardware.c | 本程序 | bluetooth2.py |
|------|------------|--------|----------------|
| **控制命令** | ✅ 单字符 | ✅ 完全匹配 | ❌ 无 |
| **雷达数据** | ✅ 发送 | ✅ 解析显示 | ✅ 显示 |
| **电机数据** | ✅ 发送 | ✅ 解析显示 | ❌ 无 |
| **IMU数据** | ✅ 发送 | ✅ 解析显示 | ✅ 显示 |
| **极坐标显示** | ❌ 无 | ✅ 借鉴 | ✅ 原生 |
| **键盘控制** | ❌ 无 | ✅ 支持 | ❌ 无 |

## 🎯 优势总结

1. **完全兼容**：与hardware.c的数据格式100%匹配
2. **显示优秀**：借鉴bluetooth2.py的优秀显示方式
3. **功能完整**：支持所有hardware.c的功能
4. **易于使用**：图形界面 + 键盘控制
5. **实时性好**：50ms更新频率

## 🔍 技术细节

### 1. **数据流处理**
```
hardware.c → 串口 → Python程序 → 解析 → 显示
```

### 2. **命令流处理**
```
键盘/按钮 → Python程序 → 串口 → hardware.c → 电机控制
```

### 3. **显示更新**
```
数据解析 → 雷达点更新 → 极坐标显示 → 角度文本更新
```

这个程序完美结合了你的hardware.c代码和bluetooth2.py的显示优势，提供了一个功能完整、易于使用的蓝牙控制界面！
