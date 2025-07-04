#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
串口通信接口模块
提供与Arduino等串口设备的通信功能
"""

import time
import json
import threading
import queue
import logging
from .base_interface import HardwareInterface

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
    logging.warning("PySerial库未安装，串口功能不可用。请使用 'pip install pyserial' 安装。")

class SerialInterface(HardwareInterface):
    """串口通信接口类"""
    
    def __init__(self, port=None, baudrate=115200, timeout=1.0, 
                 device_name="串口设备", auto_reconnect=True):
        """初始化串口通信接口
        
        参数:
            port: 串口名称，如 'COM3' 或 '/dev/ttyUSB0'
            baudrate: 波特率
            timeout: 超时时间（秒）
            device_name: 设备名称
            auto_reconnect: 是否自动重连
        """
        super().__init__(device_name=device_name)
        
        if not SERIAL_AVAILABLE:
            self.logger.error("PySerial库未安装，串口功能不可用")
            return
            
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.auto_reconnect = auto_reconnect
        self.serial = None
        
        # 读取线程相关
        self.read_thread = None
        self.read_thread_running = False
        self.read_queue = queue.Queue()
        
        # 命令响应映射
        self.response_map = {}
        self.response_lock = threading.Lock()
        
        # 自动发现串口
        if self.port is None:
            self.port = self._find_arduino_port()
            if self.port:
                self.logger.info(f"自动发现串口设备: {self.port}")
    
    def connect(self):
        """连接到串口设备"""
        if not SERIAL_AVAILABLE:
            self.logger.error("PySerial库未安装，无法连接")
            return False
            
        if self.connected:
            self.logger.info(f"已经连接到 {self.port}")
            return True
            
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            
            # 等待Arduino重置
            time.sleep(2.0)
            
            # 清空缓冲区
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            
            self.connected = True
            self.stats["connect_time"] = time.time()
            self.stats["last_activity"] = time.time()
            
            # 启动读取线程
            self._start_read_thread()
            
            self.logger.info(f"已连接到 {self.port}，波特率: {self.baudrate}")
            return True
            
        except serial.SerialException as e:
            self.logger.error(f"连接到 {self.port} 失败: {e}")
            self.error_count += 1
            self.stats["errors"] += 1
            return False
    
    def disconnect(self):
        """断开与串口设备的连接"""
        if not self.connected:
            return
            
        # 停止读取线程
        self._stop_read_thread()
        
        try:
            if self.serial and self.serial.is_open:
                self.serial.close()
            
            self.connected = False
            self.logger.info(f"已断开与 {self.port} 的连接")
            
        except serial.SerialException as e:
            self.logger.error(f"断开连接时出错: {e}")
            self.error_count += 1
            self.stats["errors"] += 1
    
    def send_command(self, command, *args, wait_response=True, timeout=2.0, **kwargs):
        """发送命令到串口设备
        
        参数:
            command: 命令字符串
            args: 命令参数
            wait_response: 是否等待响应
            timeout: 等待响应的超时时间（秒）
            kwargs: 其他参数（未使用）
            
        返回:
            如果wait_response为True，返回响应数据；否则返回None
        """
        if not self.connected:
            if self.auto_reconnect:
                self.logger.info("尝试重新连接...")
                if not self.connect():
                    self.logger.error("重新连接失败")
                    return None
            else:
                self.logger.error("未连接到设备")
                return None
        
        try:
            # 编码命令
            cmd_str = self.encode_command(command, *args)
            cmd_bytes = (cmd_str + '\n').encode('utf-8')
            
            # 发送命令
            self.serial.write(cmd_bytes)
            self.serial.flush()
            
            # 更新统计信息
            self._update_stats(
                bytes_sent=len(cmd_bytes),
                commands_sent=1
            )
            
            self.last_command_time = time.time()
            self.logger.debug(f"已发送命令: {cmd_str}")
            
            # 等待响应
            if wait_response:
                return self._wait_for_response(self.command_count - 1, timeout)
            return None
            
        except serial.SerialException as e:
            self.logger.error(f"发送命令失败: {e}")
            self.error_count += 1
            self.stats["errors"] += 1
            
            if self.auto_reconnect:
                self.logger.info("尝试重新连接...")
                self.disconnect()
                if self.connect():
                    self.logger.info("重新连接成功")
                    # 重试发送
                    return self.send_command(command, *args, 
                                            wait_response=wait_response, 
                                            timeout=timeout)
            return None
    
    def read_data(self, timeout=1.0):
        """从串口读取数据
        
        参数:
            timeout: 超时时间（秒）
            
        返回:
            读取的数据字符串，如果超时则返回None
        """
        if not self.connected:
            self.logger.error("未连接到设备")
            return None
            
        try:
            # 尝试从队列中获取数据
            try:
                data = self.read_queue.get(timeout=timeout)
                self.read_queue.task_done()
                return data
            except queue.Empty:
                return None
                
        except Exception as e:
            self.logger.error(f"读取数据失败: {e}")
            self.error_count += 1
            self.stats["errors"] += 1
            return None
    
    def _start_read_thread(self):
        """启动读取线程"""
        if self.read_thread_running:
            return
            
        self.read_thread_running = True
        self.read_thread = threading.Thread(
            target=self._read_thread_func,
            daemon=True
        )
        self.read_thread.start()
        self.logger.debug("读取线程已启动")
    
    def _stop_read_thread(self):
        """停止读取线程"""
        self.read_thread_running = False
        if self.read_thread:
            self.read_thread.join(timeout=1.0)
            self.read_thread = None
        self.logger.debug("读取线程已停止")
    
    def _read_thread_func(self):
        """读取线程函数"""
        while self.read_thread_running and self.connected:
            try:
                if self.serial and self.serial.in_waiting > 0:
                    line = self.serial.readline().decode('utf-8').strip()
                    if line:
                        self._process_received_line(line)
                else:
                    # 短暂休眠，减少CPU使用
                    time.sleep(0.01)
            except serial.SerialException as e:
                self.logger.error(f"读取线程错误: {e}")
                self.error_count += 1
                self.stats["errors"] += 1
                
                # 如果自动重连，尝试重新连接
                if self.auto_reconnect:
                    self.logger.info("读取线程尝试重新连接...")
                    self.connected = False
                    try:
                        if self.serial:
                            self.serial.close()
                        time.sleep(1.0)
                        self.connect()
                    except Exception as reconnect_error:
                        self.logger.error(f"重新连接失败: {reconnect_error}")
                        time.sleep(5.0)  # 等待一段时间再次尝试
                else:
                    # 不自动重连，直接退出线程
                    break
            except Exception as e:
                self.logger.error(f"读取线程未知错误: {e}")
                time.sleep(0.1)
    
    def _process_received_line(self, line):
        """处理接收到的行数据
        
        参数:
            line: 接收到的行数据
        """
        self._update_stats(
            bytes_received=len(line) + 2,  # +2 for \r\n
            responses_received=1
        )
        
        self.logger.debug(f"接收到数据: {line}")
        
        try:
            # 尝试解析为JSON
            data = json.loads(line)
            
            # 检查是否是命令响应
            if "id" in data and "result" in data:
                cmd_id = data["id"]
                with self.response_lock:
                    self.response_map[cmd_id] = data
            
            # 无论是否是命令响应，都放入队列
            self.read_queue.put(data)
            
        except json.JSONDecodeError:
            # 非JSON数据，直接放入队列
            self.logger.debug(f"接收到非JSON数据: {line}")
            self.read_queue.put(line)
    
    def _wait_for_response(self, cmd_id, timeout=2.0):
        """等待特定命令的响应
        
        参数:
            cmd_id: 命令ID
            timeout: 超时时间（秒）
            
        返回:
            响应数据，如果超时则返回None
        """
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            with self.response_lock:
                if cmd_id in self.response_map:
                    response = self.response_map[cmd_id]
                    del self.response_map[cmd_id]
                    return response
            
            # 短暂休眠，减少CPU使用
            time.sleep(0.01)
        
        self.logger.warning(f"等待命令 {cmd_id} 的响应超时")
        return None
    
    def _find_arduino_port(self):
        """自动查找Arduino串口
        
        返回:
            找到的串口名称，如果未找到则返回None
        """
        if not SERIAL_AVAILABLE:
            return None
            
        try:
            ports = list(serial.tools.list_ports.comports())
            for port in ports:
                # Arduino设备通常包含'Arduino'或'CH340'或'USB'等关键字
                if ('arduino' in port.description.lower() or 
                    'ch340' in port.description.lower() or 
                    'usb' in port.description.lower()):
                    return port.device
            
            # 如果没有找到明确的Arduino设备，但有串口设备，返回第一个
            if ports:
                return ports[0].device
                
        except Exception as e:
            self.logger.error(f"查找Arduino串口失败: {e}")
            
        return None
    
    def list_available_ports(self):
        """列出可用的串口设备
        
        返回:
            串口设备列表
        """
        if not SERIAL_AVAILABLE:
            return []
            
        try:
            ports = list(serial.tools.list_ports.comports())
            return [(port.device, port.description) for port in ports]
        except Exception as e:
            self.logger.error(f"列出串口设备失败: {e}")
            return []
    
    def send_motor_command(self, left_speed, right_speed, duration=None):
        """发送电机控制命令
        
        参数:
            left_speed: 左电机速度 (-255 到 255)
            right_speed: 右电机速度 (-255 到 255)
            duration: 持续时间（秒），如果为None则持续运行
            
        返回:
            响应数据
        """
        # 限制速度范围
        left_speed = max(-255, min(255, int(left_speed)))
        right_speed = max(-255, min(255, int(right_speed)))
        
        if duration is not None:
            return self.send_command("motor", left_speed, right_speed, duration)
        else:
            return self.send_command("motor", left_speed, right_speed)
    
    def send_servo_command(self, servo_id, angle):
        """发送舵机控制命令
        
        参数:
            servo_id: 舵机ID
            angle: 角度 (0-180)
            
        返回:
            响应数据
        """
        # 限制角度范围
        angle = max(0, min(180, int(angle)))
        return self.send_command("servo", servo_id, angle)
    
    def read_sensors(self):
        """读取传感器数据
        
        返回:
            传感器数据字典
        """
        return self.send_command("sensors")
    
    def reset_device(self):
        """重置设备"""
        return self.send_command("reset", wait_response=False) 