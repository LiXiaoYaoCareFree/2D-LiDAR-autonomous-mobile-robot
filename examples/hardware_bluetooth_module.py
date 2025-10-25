"""
硬件适配蓝牙连接模块
基于hardware.c和bluetooth2.py开发，类似HC蓝牙助手功能
支持激光雷达数据接收、IMU数据解析、电机控制
"""

import serial
import threading
import time
import re
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import logging
from typing import Dict, List, Tuple, Optional, Callable
from dataclasses import dataclass
import json
import queue

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

@dataclass
class LidarPoint:
    """激光雷达数据点"""
    angle: float      # 角度（度）
    distance: float   # 距离（米）
    quality: int      # 质量
    timestamp: float  # 时间戳

@dataclass
class IMUData:
    """IMU数据"""
    roll: float       # 横滚角
    pitch: float      # 俯仰角
    yaw: float        # 偏航角
    timestamp: float  # 时间戳

@dataclass
class MotorData:
    """电机数据"""
    rpm_a: float      # A电机转速
    rpm_b: float      # B电机转速
    direction: int    # 方向（0-5）
    speed: int       # 速度
    timestamp: float  # 时间戳

class HardwareBluetoothInterface:
    """硬件适配蓝牙接口"""
    
    def __init__(self, port: str = "COM3", baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.ser: Optional[serial.Serial] = None
        self.is_connected = False
        self.is_running = False
        
        # 数据缓存
        self.lidar_data = deque(maxlen=1000)
        self.imu_data = deque(maxlen=100)
        self.motor_data = deque(maxlen=100)
        self.raw_data_buffer = ""
        
        # 线程控制
        self.read_thread: Optional[threading.Thread] = None
        self.data_lock = threading.Lock()
        
        # 回调函数
        self.data_callbacks: List[Callable] = []
        
        # 连接状态
        self.connection_status = {
            'connected': False,
            'last_data_time': 0,
            'data_count': 0,
            'error_count': 0
        }
        
        logger.info(f"硬件蓝牙接口初始化完成 - 端口: {port}, 波特率: {baudrate}")
    
    def add_data_callback(self, callback: Callable):
        """添加数据回调函数"""
        self.data_callbacks.append(callback)
    
    def connect(self) -> bool:
        """连接蓝牙设备"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.1,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
            
            if self.ser.is_open:
                self.is_connected = True
                self.is_running = True
                self.connection_status['connected'] = True
                
                # 启动数据读取线程
                self.read_thread = threading.Thread(target=self._read_data_thread, daemon=True)
                self.read_thread.start()
                
                logger.info(f"✅ 蓝牙连接成功: {self.port}")
                return True
            else:
                logger.error(f"❌ 蓝牙连接失败: {self.port}")
                return False
                
        except serial.SerialException as e:
            logger.error(f"❌ 串口异常: {e}")
            return False
        except Exception as e:
            logger.error(f"❌ 连接异常: {e}")
            return False
    
    def disconnect(self):
        """断开蓝牙连接"""
        self.is_running = False
        self.is_connected = False
        self.connection_status['connected'] = False
        
        if self.read_thread and self.read_thread.is_alive():
            self.read_thread.join(timeout=1.0)
        
        if self.ser and self.ser.is_open:
            self.ser.close()
            logger.info("📌 蓝牙连接已断开")
    
    def send_motor_command(self, direction: int) -> bool:
        """发送电机控制命令（0-5）"""
        if not self.is_connected or not self.ser:
            logger.warning("蓝牙未连接，无法发送命令")
            return False
        
        try:
            command = str(direction)
            self.ser.write(command.encode('utf-8'))
            logger.info(f"发送电机命令: {direction}")
            return True
        except Exception as e:
            logger.error(f"发送命令失败: {e}")
            return False
    
    def _read_data_thread(self):
        """数据读取线程"""
        logger.info("数据读取线程启动")
        
        while self.is_running and self.is_connected:
            try:
                if self.ser and self.ser.in_waiting > 0:
                    # 读取数据
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self._process_data(line)
                        self.connection_status['last_data_time'] = time.time()
                        self.connection_status['data_count'] += 1
                
                time.sleep(0.01)  # 降低CPU占用
                
            except Exception as e:
                logger.error(f"数据读取异常: {e}")
                self.connection_status['error_count'] += 1
                time.sleep(0.1)
    
    def _process_data(self, line: str):
        """处理接收到的数据"""
        with self.data_lock:
            self.raw_data_buffer += line + "\n"
            
            # 解析激光雷达数据
            self._parse_lidar_data(line)
            
            # 解析IMU数据
            self._parse_imu_data(line)
            
            # 解析电机数据
            self._parse_motor_data(line)
            
            # 触发回调函数
            for callback in self.data_callbacks:
                try:
                    callback(line)
                except Exception as e:
                    logger.error(f"回调函数异常: {e}")
    
    def _parse_lidar_data(self, line: str):
        """解析激光雷达数据"""
        # 格式: A:123.45,D:0.678,Q:10
        pattern = r'A:([-+]?\d*\.?\d+),\s*D:([-+]?\d*\.?\d+)m?,\s*Q:(\d+)'
        matches = re.findall(pattern, line)
        
        for angle_str, dist_str, quality_str in matches:
            try:
                angle = float(angle_str)
                distance = float(dist_str)
                quality = int(quality_str)
                
                point = LidarPoint(
                    angle=angle,
                    distance=distance,
                    quality=quality,
                    timestamp=time.time()
                )
                
                self.lidar_data.append(point)
                logger.debug(f"激光雷达数据: 角度={angle:.2f}°, 距离={distance:.3f}m, 质量={quality}")
                
            except ValueError as e:
                logger.warning(f"激光雷达数据解析失败: {angle_str}, {dist_str}, {quality_str} - {e}")
    
    def _parse_imu_data(self, line: str):
        """解析IMU数据"""
        # 格式: roll:12.34, pitch:-5.67, yaw:90.0
        roll_match = re.search(r'roll:([-+]?\d*\.?\d+)', line)
        pitch_match = re.search(r'pitch:([-+]?\d*\.?\d+)', line)
        yaw_match = re.search(r'yaw:([-+]?\d*\.?\d+)', line)
        
        if roll_match or pitch_match or yaw_match:
            try:
                roll = float(roll_match.group(1)) if roll_match else 0.0
                pitch = float(pitch_match.group(1)) if pitch_match else 0.0
                yaw = float(yaw_match.group(1)) if yaw_match else 0.0
                
                imu = IMUData(
                    roll=roll,
                    pitch=pitch,
                    yaw=yaw,
                    timestamp=time.time()
                )
                
                self.imu_data.append(imu)
                logger.debug(f"IMU数据: roll={roll:.2f}°, pitch={pitch:.2f}°, yaw={yaw:.2f}°")
                
            except ValueError as e:
                logger.warning(f"IMU数据解析失败: {e}")
    
    def _parse_motor_data(self, line: str):
        """解析电机数据"""
        # 格式: MotorA:123.4,RPMB:567.8
        motor_match = re.search(r'MotorA:([-+]?\d*\.?\d+),RPMB:([-+]?\d*\.?\d+)', line)
        
        if motor_match:
            try:
                rpm_a = float(motor_match.group(1))
                rpm_b = float(motor_match.group(2))
                
                motor = MotorData(
                    rpm_a=rpm_a,
                    rpm_b=rpm_b,
                    direction=0,  # 需要从其他数据获取
                    speed=0,      # 需要从其他数据获取
                    timestamp=time.time()
                )
                
                self.motor_data.append(motor)
                logger.debug(f"电机数据: RPM_A={rpm_a:.1f}, RPM_B={rpm_b:.1f}")
                
            except ValueError as e:
                logger.warning(f"电机数据解析失败: {e}")
    
    def get_latest_lidar_data(self) -> List[LidarPoint]:
        """获取最新的激光雷达数据"""
        with self.data_lock:
            return list(self.lidar_data)
    
    def get_latest_imu_data(self) -> Optional[IMUData]:
        """获取最新的IMU数据"""
        with self.data_lock:
            return self.imu_data[-1] if self.imu_data else None
    
    def get_latest_motor_data(self) -> Optional[MotorData]:
        """获取最新的电机数据"""
        with self.data_lock:
            return self.motor_data[-1] if self.motor_data else None
    
    def get_connection_status(self) -> Dict:
        """获取连接状态"""
        return self.connection_status.copy()
    
    def clear_data(self):
        """清空数据缓存"""
        with self.data_lock:
            self.lidar_data.clear()
            self.imu_data.clear()
            self.motor_data.clear()
            self.raw_data_buffer = ""


class HardwareBluetoothVisualizer:
    """硬件蓝牙数据可视化器"""
    
    def __init__(self, bluetooth_interface: HardwareBluetoothInterface):
        self.bt_interface = bluetooth_interface
        self.fig = None
        self.ax_lidar = None
        self.ax_imu = None
        self.lidar_line = None
        self.imu_text = None
        self.motor_text = None
        self.is_running = False
        
        # 数据缓存
        self.lidar_angles = deque(maxlen=360)
        self.lidar_distances = deque(maxlen=360)
        
        # 配置matplotlib
        plt.rcParams['font.sans-serif'] = ['SimHei', 'Arial Unicode MS']
        plt.rcParams['axes.unicode_minus'] = False
        
    def setup_plots(self):
        """设置绘图界面"""
        self.fig, (self.ax_lidar, self.ax_imu) = plt.subplots(1, 2, figsize=(16, 8))
        
        # 激光雷达极坐标图
        self.ax_lidar = plt.subplot(121, projection='polar')
        self.ax_lidar.set_theta_zero_location('N')
        self.ax_lidar.set_theta_direction(-1)
        self.ax_lidar.set_ylim(0, 10)
        self.ax_lidar.set_title("激光雷达实时扫描", fontsize=14, fontweight='bold')
        
        # IMU和电机数据显示
        self.ax_imu = plt.subplot(122)
        self.ax_imu.axis('off')
        self.ax_imu.set_title("传感器数据", fontsize=14, fontweight='bold')
        
        # 初始化绘图元素
        self.lidar_line, = self.ax_lidar.plot([], [], 'bo', markersize=2, alpha=0.7)
        
        # 文本显示
        self.imu_text = self.ax_imu.text(0.1, 0.8, "等待数据...", 
                                       transform=self.ax_imu.transAxes, fontsize=12,
                                       bbox=dict(boxstyle="round,pad=0.3", facecolor="lightblue", alpha=0.8))
        
        self.motor_text = self.ax_imu.text(0.1, 0.4, "等待数据...", 
                                        transform=self.ax_imu.transAxes, fontsize=12,
                                        bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgreen", alpha=0.8))
        
        # 连接数据回调
        self.bt_interface.add_data_callback(self._on_data_received)
        
    def _on_data_received(self, data: str):
        """数据接收回调"""
        # 更新激光雷达数据
        lidar_data = self.bt_interface.get_latest_lidar_data()
        if lidar_data:
            angles = [np.radians(point.angle) for point in lidar_data[-360:]]
            distances = [point.distance for point in lidar_data[-360:]]
            
            self.lidar_angles.extend(angles)
            self.lidar_distances.extend(distances)
    
    def update_plot(self, frame):
        """更新绘图"""
        if not self.is_running:
            return
        
        # 更新激光雷达数据
        if self.lidar_angles and self.lidar_distances:
            self.lidar_line.set_data(list(self.lidar_angles), list(self.lidar_distances))
            
            # 自动调整距离范围
            if self.lidar_distances:
                max_dist = max(self.lidar_distances)
                self.ax_lidar.set_ylim(0, max_dist * 1.1)
        
        # 更新IMU数据
        imu_data = self.bt_interface.get_latest_imu_data()
        if imu_data:
            imu_text = f"IMU数据 (实时)\n" \
                       f"横滚角(roll): {imu_data.roll:.2f}°\n" \
                       f"俯仰角(pitch): {imu_data.pitch:.2f}°\n" \
                       f"偏航角(yaw): {imu_data.yaw:.2f}°"
            self.imu_text.set_text(imu_text)
        
        # 更新电机数据
        motor_data = self.bt_interface.get_latest_motor_data()
        if motor_data:
            motor_text = f"电机数据 (实时)\n" \
                        f"电机A转速: {motor_data.rpm_a:.1f} RPM\n" \
                        f"电机B转速: {motor_data.rpm_b:.1f} RPM\n" \
                        f"方向: {motor_data.direction}\n" \
                        f"速度: {motor_data.speed}"
            self.motor_text.set_text(motor_text)
        
        # 更新连接状态
        status = self.bt_interface.get_connection_status()
        status_text = f"连接状态\n" \
                     f"已连接: {'是' if status['connected'] else '否'}\n" \
                     f"数据计数: {status['data_count']}\n" \
                     f"错误计数: {status['error_count']}"
        
        return self.lidar_line, self.imu_text, self.motor_text
    
    def start_visualization(self):
        """启动可视化"""
        self.setup_plots()
        self.is_running = True
        
        # 启动动画
        ani = FuncAnimation(self.fig, self.update_plot, interval=50, blit=False)
        
        # 添加键盘控制
        def on_key(event):
            if event.key == '0':
                self.bt_interface.send_motor_command(0)  # 停止
            elif event.key == '1':
                self.bt_interface.send_motor_command(1)  # 前进
            elif event.key == '2':
                self.bt_interface.send_motor_command(2)  # 后退
            elif event.key == '3':
                self.bt_interface.send_motor_command(3)  # 左转
            elif event.key == '4':
                self.bt_interface.send_motor_command(4)  # 右转
            elif event.key == '5':
                self.bt_interface.send_motor_command(5)  # 掉头
        
        self.fig.canvas.mpl_connect('key_press_event', on_key)
        
        # 显示帮助信息
        help_text = "键盘控制:\n0-停止, 1-前进, 2-后退, 3-左转, 4-右转, 5-掉头"
        plt.figtext(0.02, 0.02, help_text, fontsize=10, bbox=dict(boxstyle="round,pad=0.3", facecolor="yellow", alpha=0.7))
        
        plt.tight_layout()
        plt.show()
        
        self.is_running = False


def main():
    """主函数"""
    print("🚀 硬件蓝牙连接模块启动")
    print("=" * 50)
    
    # 配置参数
    PORT = "COM3"  # 根据实际情况修改
    BAUDRATE = 115200
    
    # 创建蓝牙接口
    bt_interface = HardwareBluetoothInterface(port=PORT, baudrate=BAUDRATE)
    
    # 连接蓝牙
    if not bt_interface.connect():
        print(f"❌ 无法连接到蓝牙设备: {PORT}")
        print("请检查:")
        print("1. 蓝牙设备是否已连接")
        print("2. 串口是否被其他程序占用")
        print("3. 波特率是否与硬件一致")
        return
    
    try:
        # 创建可视化器
        visualizer = HardwareBluetoothVisualizer(bt_interface)
        
        # 启动可视化
        print("✅ 蓝牙连接成功，启动数据可视化...")
        print("键盘控制: 0-停止, 1-前进, 2-后退, 3-左转, 4-右转, 5-掉头")
        visualizer.start_visualization()
        
    except KeyboardInterrupt:
        print("\n📌 程序被用户中断")
    except Exception as e:
        print(f"❌ 程序异常: {e}")
    finally:
        # 清理资源
        bt_interface.disconnect()
        print("📌 程序已退出")


if __name__ == "__main__":
    main()
