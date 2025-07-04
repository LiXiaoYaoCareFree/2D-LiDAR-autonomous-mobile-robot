#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
基础硬件接口模块
定义了与硬件通信的抽象接口
"""

from abc import ABC, abstractmethod
import time
import logging
import json
import numpy as np

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)

class HardwareInterface(ABC):
    """硬件通信接口抽象基类"""
    
    def __init__(self, device_name="未命名设备", log_level=logging.INFO):
        """初始化硬件接口
        
        参数:
            device_name: 设备名称
            log_level: 日志级别
        """
        self.device_name = device_name
        self.connected = False
        self.last_command_time = 0
        self.command_count = 0
        self.error_count = 0
        
        # 设置日志
        self.logger = logging.getLogger(f"HW:{device_name}")
        self.logger.setLevel(log_level)
        
        # 通信统计信息
        self.stats = {
            "bytes_sent": 0,
            "bytes_received": 0,
            "commands_sent": 0,
            "responses_received": 0,
            "errors": 0,
            "connect_time": None,
            "last_activity": None
        }
    
    @abstractmethod
    def connect(self):
        """连接到硬件设备"""
        pass
    
    @abstractmethod
    def disconnect(self):
        """断开与硬件设备的连接"""
        pass
    
    @abstractmethod
    def send_command(self, command, *args, **kwargs):
        """发送命令到硬件设备
        
        参数:
            command: 命令字符串
            args: 位置参数
            kwargs: 关键字参数
            
        返回:
            响应数据
        """
        pass
    
    @abstractmethod
    def read_data(self, *args, **kwargs):
        """从硬件设备读取数据
        
        参数:
            args: 位置参数
            kwargs: 关键字参数
            
        返回:
            读取的数据
        """
        pass
    
    def is_connected(self):
        """检查是否已连接到硬件设备
        
        返回:
            布尔值，表示是否已连接
        """
        return self.connected
    
    def get_stats(self):
        """获取通信统计信息
        
        返回:
            统计信息字典
        """
        return self.stats
    
    def reset_stats(self):
        """重置通信统计信息"""
        self.stats = {
            "bytes_sent": 0,
            "bytes_received": 0,
            "commands_sent": 0,
            "responses_received": 0,
            "errors": 0,
            "connect_time": self.stats["connect_time"],
            "last_activity": self.stats["last_activity"]
        }
        
    def _update_stats(self, bytes_sent=0, bytes_received=0, 
                     commands_sent=0, responses_received=0, errors=0):
        """更新通信统计信息
        
        参数:
            bytes_sent: 发送的字节数
            bytes_received: 接收的字节数
            commands_sent: 发送的命令数
            responses_received: 接收的响应数
            errors: 错误数
        """
        self.stats["bytes_sent"] += bytes_sent
        self.stats["bytes_received"] += bytes_received
        self.stats["commands_sent"] += commands_sent
        self.stats["responses_received"] += responses_received
        self.stats["errors"] += errors
        self.stats["last_activity"] = time.time()
    
    def encode_command(self, command, *args):
        """编码命令为JSON格式
        
        参数:
            command: 命令字符串
            args: 命令参数
            
        返回:
            编码后的命令字符串
        """
        cmd_dict = {
            "cmd": command,
            "args": args,
            "id": self.command_count
        }
        self.command_count += 1
        return json.dumps(cmd_dict)
    
    def decode_response(self, response_str):
        """解码JSON格式的响应
        
        参数:
            response_str: 响应字符串
            
        返回:
            解码后的响应字典
        """
        try:
            return json.loads(response_str)
        except json.JSONDecodeError as e:
            self.logger.error(f"解码响应失败: {e}")
            self.error_count += 1
            return {"error": "解码失败", "raw": response_str}
            
    def __str__(self):
        """返回设备的字符串表示"""
        status = "已连接" if self.connected else "未连接"
        return f"{self.device_name} ({status})"
    
    def __enter__(self):
        """上下文管理器入口"""
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """上下文管理器退出"""
        self.disconnect() 