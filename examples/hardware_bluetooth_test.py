"""
硬件蓝牙连接模块 - 简化测试版本
用于快速验证蓝牙连接和数据接收功能
"""

import serial
import time
import re
import threading
from collections import deque

class SimpleBluetoothTest:
    """简化的蓝牙测试类"""
    
    def __init__(self, port="COM3", baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.is_connected = False
        self.is_running = False
        
        # 数据统计
        self.lidar_count = 0
        self.imu_count = 0
        self.motor_count = 0
        self.total_data = 0
        
        # 最新数据
        self.latest_lidar = None
        self.latest_imu = None
        self.latest_motor = None
        
    def connect(self):
        """连接蓝牙"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.1
            )
            self.is_connected = True
            self.is_running = True
            print(f"✅ 蓝牙连接成功: {self.port}")
            return True
        except Exception as e:
            print(f"❌ 蓝牙连接失败: {e}")
            return False
    
    def disconnect(self):
        """断开连接"""
        self.is_running = False
        if self.ser:
            self.ser.close()
        print("📌 蓝牙连接已断开")
    
    def send_command(self, cmd):
        """发送控制命令"""
        if not self.is_connected:
            print("❌ 蓝牙未连接")
            return False
        
        try:
            self.ser.write(str(cmd).encode('utf-8'))
            print(f"📤 发送命令: {cmd}")
            return True
        except Exception as e:
            print(f"❌ 发送命令失败: {e}")
            return False
    
    def read_data_loop(self, duration=30):
        """读取数据循环"""
        print(f"📡 开始读取数据，持续 {duration} 秒...")
        print("按 Ctrl+C 停止")
        
        start_time = time.time()
        
        try:
            while self.is_running and (time.time() - start_time) < duration:
                if self.ser and self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self._parse_line(line)
                        self.total_data += 1
                
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            print("\n📌 用户中断")
        except Exception as e:
            print(f"❌ 读取数据异常: {e}")
    
    def _parse_line(self, line):
        """解析数据行"""
        print(f"📥 收到数据: {line}")
        
        # 解析激光雷达数据
        lidar_match = re.search(r'A:([-+]?\d*\.?\d+),\s*D:([-+]?\d*\.?\d+)m?,\s*Q:(\d+)', line)
        if lidar_match:
            angle = float(lidar_match.group(1))
            distance = float(lidar_match.group(2))
            quality = int(lidar_match.group(3))
            self.latest_lidar = (angle, distance, quality)
            self.lidar_count += 1
            print(f"  🔍 激光雷达: 角度={angle:.2f}°, 距离={distance:.3f}m, 质量={quality}")
        
        # 解析IMU数据
        roll_match = re.search(r'roll:([-+]?\d*\.?\d+)', line)
        pitch_match = re.search(r'pitch:([-+]?\d*\.?\d+)', line)
        yaw_match = re.search(r'yaw:([-+]?\d*\.?\d+)', line)
        
        if roll_match or pitch_match or yaw_match:
            roll = float(roll_match.group(1)) if roll_match else 0.0
            pitch = float(pitch_match.group(1)) if pitch_match else 0.0
            yaw = float(yaw_match.group(1)) if yaw_match else 0.0
            self.latest_imu = (roll, pitch, yaw)
            self.imu_count += 1
            print(f"  📱 IMU: roll={roll:.2f}°, pitch={pitch:.2f}°, yaw={yaw:.2f}°")
        
        # 解析电机数据
        motor_match = re.search(r'MotorA:([-+]?\d*\.?\d+),RPMB:([-+]?\d*\.?\d+)', line)
        if motor_match:
            rpm_a = float(motor_match.group(1))
            rpm_b = float(motor_match.group(2))
            self.latest_motor = (rpm_a, rpm_b)
            self.motor_count += 1
            print(f"  ⚙️ 电机: A={rpm_a:.1f}RPM, B={rpm_b:.1f}RPM")
    
    def print_statistics(self):
        """打印统计信息"""
        print("\n" + "="*50)
        print("📊 数据统计")
        print("="*50)
        print(f"总数据包: {self.total_data}")
        print(f"激光雷达数据: {self.lidar_count}")
        print(f"IMU数据: {self.imu_count}")
        print(f"电机数据: {self.motor_count}")
        
        if self.latest_lidar:
            print(f"\n🔍 最新激光雷达数据: 角度={self.latest_lidar[0]:.2f}°, 距离={self.latest_lidar[1]:.3f}m, 质量={self.latest_lidar[2]}")
        
        if self.latest_imu:
            print(f"📱 最新IMU数据: roll={self.latest_imu[0]:.2f}°, pitch={self.latest_imu[1]:.2f}°, yaw={self.latest_imu[2]:.2f}°")
        
        if self.latest_motor:
            print(f"⚙️ 最新电机数据: A={self.latest_motor[0]:.1f}RPM, B={self.latest_motor[1]:.1f}RPM")


def main():
    """主函数"""
    print("🚀 硬件蓝牙连接测试")
    print("="*50)
    
    # 配置参数
    PORT = "COM3"  # 根据实际情况修改
    BAUDRATE = 115200
    
    # 创建测试实例
    test = SimpleBluetoothTest(port=PORT, baudrate=BAUDRATE)
    
    # 连接蓝牙
    if not test.connect():
        print("❌ 无法连接蓝牙设备")
        print("请检查:")
        print("1. 蓝牙设备是否已连接")
        print("2. 串口是否被其他程序占用")
        print("3. 波特率是否与硬件一致")
        return
    
    try:
        # 测试控制命令
        print("\n🎮 测试控制命令...")
        for cmd in [0, 1, 2, 0]:  # 停止->前进->后退->停止
            test.send_command(cmd)
            time.sleep(1)
        
        # 读取数据
        print("\n📡 开始读取数据...")
        test.read_data_loop(duration=30)
        
    except KeyboardInterrupt:
        print("\n📌 用户中断测试")
    except Exception as e:
        print(f"❌ 测试异常: {e}")
    finally:
        # 显示统计信息
        test.print_statistics()
        
        # 断开连接
        test.disconnect()
        print("📌 测试完成")


if __name__ == "__main__":
    main()
