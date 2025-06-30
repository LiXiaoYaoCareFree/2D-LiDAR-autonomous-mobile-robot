#!/usr/bin/env python3
'''
communication.py - 通信模块，使用PySerial通过蓝牙与机器人进行数据交换

该模块负责与机器人进行数据交换，接收传感器数据并发送控制命令。
'''

import time
import json
import struct
import threading
import serial
from queue import Queue, Empty

class BluetoothCommunication:
    """蓝牙通信类，用于迷宫求解器演示"""
    
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        """
        初始化蓝牙通信
        
        参数:
            port: 串口设备
            baudrate: 波特率
        """
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.connected = False
        
        # 模拟数据
        self.last_command = None
        self.last_lidar_data = None
    
    def connect(self):
        """连接到设备"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1.0
            )
            self.connected = True
            print(f"已连接到设备，端口: {self.port}, 波特率: {self.baudrate}")
            return True
        except Exception as e:
            print(f"连接设备时出错: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """断开与设备的连接"""
        if self.serial and self.serial.is_open:
            self.serial.close()
            self.connected = False
            print("已断开与设备的连接")
    
    def is_connected(self):
        """检查是否已连接"""
        return self.connected
    
    def get_lidar_data(self):
        """
        获取激光雷达数据
        
        返回:
            scan_data: 激光扫描数据，包含距离和角度
        """
        # 在实际应用中，这里应该从设备读取数据
        # 这里返回模拟数据
        if self.last_lidar_data:
            return self.last_lidar_data
        
        # 创建模拟数据
        scan_size = 360
        distances = [5.0] * scan_size  # 默认5米距离
        angles = list(range(0, 360))
        
        scan_data = {
            'distances': distances,
            'angles': angles
        }
        
        self.last_lidar_data = scan_data
        return scan_data
    
    def send_navigation_command(self, target_pose):
        """
        发送导航命令
        
        参数:
            target_pose: 目标位置，(x, y, theta)，单位为米和度
        """
        # 在实际应用中，这里应该向设备发送命令
        # 这里只记录命令
        self.last_command = {
            'type': 'navigate',
            'target_pose': target_pose
        }
        
        print(f"发送导航命令: 目标位置=({target_pose[0]:.2f}, {target_pose[1]:.2f}, {target_pose[2]:.2f})")
        return True
    
    def send_stop_command(self):
        """发送停止命令"""
        # 在实际应用中，这里应该向设备发送命令
        # 这里只记录命令
        self.last_command = {
            'type': 'stop'
        }
        
        print("发送停止命令")
        return True
    
    def send_reset_command(self):
        """发送重置命令"""
        # 在实际应用中，这里应该向设备发送命令
        # 这里只记录命令
        self.last_command = {
            'type': 'reset'
        }
        
        print("发送重置命令")
        return True
    
    def set_lidar_data(self, scan_data):
        """
        设置模拟的激光雷达数据
        
        参数:
            scan_data: 激光扫描数据，包含距离和角度
        """
        self.last_lidar_data = scan_data

class RobotCommunication:
    """机器人通信类，使用PySerial通过蓝牙与机器人进行数据交换"""
    
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, 
                 sensor_queue=None, command_queue=None):
        """
        初始化机器人通信
        
        参数:
            port: 串口设备
            baudrate: 波特率
            sensor_queue: 传感器数据队列
            command_queue: 命令队列
        """
        self.port = port
        self.baudrate = baudrate
        self.sensor_queue = sensor_queue if sensor_queue else Queue()
        self.command_queue = command_queue if command_queue else Queue()
        
        # 串口对象
        self.serial = None
        
        # 线程锁，用于保护串口访问
        self.lock = threading.Lock()
        
        # 运行标志
        self.running = False
        
        # 数据包计数
        self.packet_count = 0
        
        # 数据包格式定义
        self.PACKET_HEADER = b'\xAA\x55'
        self.PACKET_FOOTER = b'\x55\xAA'
        
        # 命令类型
        self.CMD_NAVIGATE = 0x01
        self.CMD_STOP = 0x02
        self.CMD_RESET = 0x03
    
    def connect(self):
        """连接到机器人"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1.0
            )
            
            if self.serial.is_open:
                print(f"已连接到机器人，端口: {self.port}, 波特率: {self.baudrate}")
                return True
            else:
                print(f"无法打开串口: {self.port}")
                return False
                
        except Exception as e:
            print(f"连接机器人时出错: {e}")
            return False
    
    def disconnect(self):
        """断开与机器人的连接"""
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("已断开与机器人的连接")
    
    def run(self):
        """运行通信循环"""
        self.running = True
        
        # 连接到机器人
        if not self.connect():
            print("无法连接到机器人，通信模块停止")
            self.running = False
            return
        
        # 创建接收线程
        receive_thread = threading.Thread(target=self._receive_loop)
        receive_thread.daemon = True
        receive_thread.start()
        
        # 主线程处理发送命令
        try:
            while self.running:
                # 检查是否有命令要发送
                try:
                    command = self.command_queue.get(timeout=0.1)
                    self._send_command(command)
                except Empty:
                    pass
                
                time.sleep(0.01)  # 小延迟以避免CPU占用过高
                
        except Exception as e:
            print(f"通信循环错误: {e}")
        finally:
            self.disconnect()
    
    def close(self):
        """关闭通信"""
        self.running = False
    
    def _receive_loop(self):
        """接收数据循环"""
        buffer = bytearray()
        
        while self.running:
            try:
                # 读取数据
                if self.serial and self.serial.is_open and self.serial.in_waiting > 0:
                    with self.lock:
                        data = self.serial.read(self.serial.in_waiting)
                    
                    # 将数据添加到缓冲区
                    buffer.extend(data)
                    
                    # 处理缓冲区中的完整数据包
                    buffer = self._process_buffer(buffer)
                
                time.sleep(0.01)  # 小延迟以避免CPU占用过高
                
            except Exception as e:
                print(f"接收数据错误: {e}")
    
    def _process_buffer(self, buffer):
        """
        处理接收缓冲区，提取完整数据包
        
        参数:
            buffer: 接收缓冲区
            
        返回:
            remaining_buffer: 处理后的剩余缓冲区
        """
        # 查找数据包头
        start_idx = buffer.find(self.PACKET_HEADER)
        
        if start_idx == -1:
            # 没有找到数据包头，清空缓冲区
            return bytearray()
        
        # 如果数据包头不在开始位置，丢弃之前的数据
        if start_idx > 0:
            buffer = buffer[start_idx:]
        
        # 确保缓冲区足够长，至少包含头部和长度字段
        if len(buffer) < 4:
            return buffer
        
        # 获取数据包长度
        packet_length = struct.unpack('<H', buffer[2:4])[0]
        
        # 计算完整数据包的长度（头部2字节 + 长度2字节 + 数据包长度 + 尾部2字节）
        total_length = 2 + 2 + packet_length + 2
        
        # 如果缓冲区不足以包含完整数据包，等待更多数据
        if len(buffer) < total_length:
            return buffer
        
        # 提取完整数据包
        packet = buffer[:total_length]
        
        # 验证数据包尾部
        if packet[-2:] == self.PACKET_FOOTER:
            # 处理有效数据包
            self._handle_packet(packet[4:-2])  # 去掉头部、长度和尾部
            self.packet_count += 1
        else:
            print("数据包格式错误，丢弃")
        
        # 返回剩余的缓冲区
        return buffer[total_length:]
    
    def _handle_packet(self, data):
        """
        处理接收到的数据包
        
        参数:
            data: 数据包内容
        """
        try:
            # 解析数据包
            packet_type = data[0]
            
            if packet_type == 0x01:  # 激光雷达数据
                sensor_data = self._parse_lidar_data(data[1:])
                if self.sensor_queue:
                    self.sensor_queue.put(sensor_data)
                    
            elif packet_type == 0x02:  # 里程计数据
                sensor_data = self._parse_odometry_data(data[1:])
                if self.sensor_queue:
                    self.sensor_queue.put(sensor_data)
                    
            elif packet_type == 0x03:  # 综合数据（激光雷达+里程计）
                sensor_data = self._parse_combined_data(data[1:])
                if self.sensor_queue:
                    self.sensor_queue.put(sensor_data)
                    
            else:
                print(f"未知数据包类型: {packet_type}")
                
        except Exception as e:
            print(f"处理数据包错误: {e}")
    
    def _parse_lidar_data(self, data):
        """
        解析激光雷达数据
        
        参数:
            data: 激光雷达数据字节
            
        返回:
            sensor_data: 解析后的传感器数据字典
        """
        # 假设数据格式为：
        # - 2字节时间戳
        # - 2字节扫描点数
        # - 每个扫描点2字节（距离，单位为毫米）
        
        timestamp = struct.unpack('<H', data[0:2])[0]
        num_points = struct.unpack('<H', data[2:4])[0]
        
        # 确保数据长度足够
        expected_length = 4 + num_points * 2
        if len(data) < expected_length:
            print(f"激光雷达数据长度不足，期望 {expected_length}，实际 {len(data)}")
            return None
        
        # 解析扫描点
        lidar_data = []
        for i in range(num_points):
            distance = struct.unpack('<H', data[4+i*2:6+i*2])[0]
            lidar_data.append(distance)
        
        return {
            'type': 'lidar',
            'timestamp': timestamp,
            'lidar': lidar_data
        }
    
    def _parse_odometry_data(self, data):
        """
        解析里程计数据
        
        参数:
            data: 里程计数据字节
            
        返回:
            sensor_data: 解析后的传感器数据字典
        """
        # 假设数据格式为：
        # - 2字节时间戳
        # - 4字节x位置（浮点数，单位为米）
        # - 4字节y位置（浮点数，单位为米）
        # - 4字节theta角度（浮点数，单位为度）
        # - 4字节线速度（浮点数，单位为米/秒）
        # - 4字节角速度（浮点数，单位为度/秒）
        
        timestamp = struct.unpack('<H', data[0:2])[0]
        x = struct.unpack('<f', data[2:6])[0]
        y = struct.unpack('<f', data[6:10])[0]
        theta = struct.unpack('<f', data[10:14])[0]
        linear_vel = struct.unpack('<f', data[14:18])[0]
        angular_vel = struct.unpack('<f', data[18:22])[0]
        
        return {
            'type': 'odometry',
            'timestamp': timestamp,
            'position': (x, y, theta),
            'velocity': (linear_vel, angular_vel),
            'odometry': (x, y, theta)
        }
    
    def _parse_combined_data(self, data):
        """
        解析综合数据（激光雷达+里程计）
        
        参数:
            data: 综合数据字节
            
        返回:
            sensor_data: 解析后的传感器数据字典
        """
        # 假设数据格式为：
        # - 2字节时间戳
        # - 4字节x位置（浮点数，单位为米）
        # - 4字节y位置（浮点数，单位为米）
        # - 4字节theta角度（浮点数，单位为度）
        # - 2字节扫描点数
        # - 每个扫描点2字节（距离，单位为毫米）
        
        timestamp = struct.unpack('<H', data[0:2])[0]
        x = struct.unpack('<f', data[2:6])[0]
        y = struct.unpack('<f', data[6:10])[0]
        theta = struct.unpack('<f', data[10:14])[0]
        num_points = struct.unpack('<H', data[14:16])[0]
        
        # 确保数据长度足够
        expected_length = 16 + num_points * 2
        if len(data) < expected_length:
            print(f"综合数据长度不足，期望 {expected_length}，实际 {len(data)}")
            return None
        
        # 解析扫描点
        lidar_data = []
        for i in range(num_points):
            distance = struct.unpack('<H', data[16+i*2:18+i*2])[0]
            lidar_data.append(distance)
        
        return {
            'type': 'combined',
            'timestamp': timestamp,
            'position': (x, y, theta),
            'lidar': lidar_data,
            'odometry': (x, y, theta)
        }
    
    def _send_command(self, command):
        """
        发送命令到机器人
        
        参数:
            command: 命令字典
        """
        try:
            # 根据命令类型构建数据包
            if command['type'] == 'navigate':
                self._send_navigate_command(command['position'])
                
            elif command['type'] == 'stop':
                self._send_stop_command()
                
            elif command['type'] == 'reset':
                self._send_reset_command()
                
            else:
                print(f"未知命令类型: {command['type']}")
                
        except Exception as e:
            print(f"发送命令错误: {e}")
    
    def _send_navigate_command(self, position):
        """
        发送导航命令
        
        参数:
            position: 目标位置，(x, y)或(x, y, theta)，单位为米和度
        """
        # 提取位置信息
        if len(position) >= 3:
            x, y, theta = position
        else:
            x, y = position
            theta = 0.0
        
        # 构建数据包
        data = bytearray()
        data.append(self.CMD_NAVIGATE)  # 命令类型
        data.extend(struct.pack('<f', x))  # x位置
        data.extend(struct.pack('<f', y))  # y位置
        data.extend(struct.pack('<f', theta))  # theta角度
        
        # 发送数据包
        self._send_packet(data)
        
        print(f"发送导航命令: 位置=({x:.2f}, {y:.2f}, {theta:.2f})")
    
    def _send_stop_command(self):
        """发送停止命令"""
        # 构建数据包
        data = bytearray([self.CMD_STOP])
        
        # 发送数据包
        self._send_packet(data)
        
        print("发送停止命令")
    
    def _send_reset_command(self):
        """发送重置命令"""
        # 构建数据包
        data = bytearray([self.CMD_RESET])
        
        # 发送数据包
        self._send_packet(data)
        
        print("发送重置命令")
    
    def _send_packet(self, data):
        """
        发送数据包
        
        参数:
            data: 数据包内容
        """
        # 构建完整数据包
        packet = bytearray()
        packet.extend(self.PACKET_HEADER)  # 数据包头
        packet.extend(struct.pack('<H', len(data)))  # 数据包长度
        packet.extend(data)  # 数据包内容
        packet.extend(self.PACKET_FOOTER)  # 数据包尾
        
        # 发送数据包
        if self.serial and self.serial.is_open:
            with self.lock:
                self.serial.write(packet)
        else:
            print("串口未打开，无法发送数据包") 