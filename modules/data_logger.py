#!/usr/bin/env python3
'''
data_logger.py - 数据记录模块，用于记录传感器数据和导航命令

该模块负责记录传感器数据、导航命令和地图，以便后续分析和调试。
'''

import os
import time
import json
import pickle
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

class DataLogger:
    """数据记录类，用于记录传感器数据和导航命令"""
    
    def __init__(self, log_dir='./logs'):
        """
        初始化数据记录器
        
        参数:
            log_dir: 日志目录
        """
        self.log_dir = log_dir
        
        # 创建日志目录
        os.makedirs(log_dir, exist_ok=True)
        
        # 创建会话目录
        self.session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_dir = os.path.join(log_dir, self.session_id)
        os.makedirs(self.session_dir, exist_ok=True)
        
        # 创建各类数据的子目录
        self.sensor_dir = os.path.join(self.session_dir, 'sensor_data')
        self.command_dir = os.path.join(self.session_dir, 'commands')
        self.map_dir = os.path.join(self.session_dir, 'maps')
        
        os.makedirs(self.sensor_dir, exist_ok=True)
        os.makedirs(self.command_dir, exist_ok=True)
        os.makedirs(self.map_dir, exist_ok=True)
        
        # 初始化数据计数器
        self.sensor_count = 0
        self.command_count = 0
        self.map_count = 0
        
        # 创建日志文件
        self.log_file = os.path.join(self.session_dir, 'session.log')
        with open(self.log_file, 'w') as f:
            f.write(f"会话开始: {self.session_id}\n")
            f.write(f"时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")
        
        print(f"数据记录器初始化完成，会话ID: {self.session_id}")
    
    def log_sensor_data(self, sensor_data):
        """
        记录传感器数据
        
        参数:
            sensor_data: 传感器数据字典
        """
        if not sensor_data:
            return
            
        # 增加计数器
        self.sensor_count += 1
        
        # 记录时间戳
        timestamp = time.time()
        sensor_data['system_timestamp'] = timestamp
        
        # 根据数据类型选择不同的记录方式
        data_type = sensor_data.get('type', 'unknown')
        
        if data_type == 'lidar':
            self._log_lidar_data(sensor_data)
        elif data_type == 'odometry':
            self._log_odometry_data(sensor_data)
        elif data_type == 'combined':
            self._log_combined_data(sensor_data)
        else:
            # 通用记录方式
            filename = os.path.join(self.sensor_dir, f"sensor_{self.sensor_count:06d}.json")
            with open(filename, 'w') as f:
                json.dump(sensor_data, f)
    
    def log_navigation_commands(self, path):
        """
        记录导航命令
        
        参数:
            path: 路径点列表，每个路径点是(x, y)坐标，单位为米
        """
        if not path:
            return
            
        # 增加计数器
        self.command_count += 1
        
        # 记录时间戳
        timestamp = time.time()
        
        # 创建命令数据
        command_data = {
            'type': 'navigation',
            'timestamp': timestamp,
            'path': path
        }
        
        # 保存为JSON文件
        filename = os.path.join(self.command_dir, f"command_{self.command_count:06d}.json")
        with open(filename, 'w') as f:
            json.dump(command_data, f)
        
        # 记录到日志文件
        with open(self.log_file, 'a') as f:
            f.write(f"导航命令 {self.command_count}: {len(path)} 个路径点, 时间: {datetime.now().strftime('%H:%M:%S')}\n")
    
    def save_map(self, map_data, filename=None):
        """
        保存地图
        
        参数:
            map_data: 地图数据，numpy数组
            filename: 文件名，如果为None，则自动生成
        """
        # 增加计数器
        self.map_count += 1
        
        # 如果没有指定文件名，自动生成
        if filename is None:
            filename = os.path.join(self.map_dir, f"map_{self.map_count:06d}")
        else:
            # 确保文件在map_dir目录下
            if not os.path.dirname(filename):
                filename = os.path.join(self.map_dir, filename)
        
        # 保存为numpy文件
        np.save(filename + '.npy', map_data)
        
        # 保存为图像文件
        plt.figure(figsize=(10, 10))
        plt.imshow(map_data, cmap='gray')
        plt.colorbar(label='占用概率')
        plt.title(f'地图 {self.map_count}')
        plt.savefig(filename + '.png')
        plt.close()
        
        # 记录到日志文件
        with open(self.log_file, 'a') as f:
            f.write(f"保存地图 {self.map_count}: {filename}, 时间: {datetime.now().strftime('%H:%M:%S')}\n")
    
    def log_event(self, event_type, description):
        """
        记录事件
        
        参数:
            event_type: 事件类型
            description: 事件描述
        """
        # 记录到日志文件
        with open(self.log_file, 'a') as f:
            f.write(f"事件 [{event_type}]: {description}, 时间: {datetime.now().strftime('%H:%M:%S')}\n")
    
    def _log_lidar_data(self, sensor_data):
        """
        记录激光雷达数据
        
        参数:
            sensor_data: 传感器数据字典
        """
        # 提取激光雷达数据
        lidar_data = sensor_data.get('lidar', [])
        
        # 保存为numpy文件
        filename = os.path.join(self.sensor_dir, f"lidar_{self.sensor_count:06d}")
        np.save(filename + '.npy', np.array(lidar_data))
        
        # 保存元数据为JSON文件
        metadata = {
            'timestamp': sensor_data.get('timestamp', 0),
            'system_timestamp': sensor_data.get('system_timestamp', 0),
            'num_points': len(lidar_data)
        }
        
        with open(filename + '.json', 'w') as f:
            json.dump(metadata, f)
    
    def _log_odometry_data(self, sensor_data):
        """
        记录里程计数据
        
        参数:
            sensor_data: 传感器数据字典
        """
        # 提取里程计数据
        position = sensor_data.get('position', (0, 0, 0))
        velocity = sensor_data.get('velocity', (0, 0))
        
        # 保存为JSON文件
        filename = os.path.join(self.sensor_dir, f"odometry_{self.sensor_count:06d}.json")
        
        data = {
            'timestamp': sensor_data.get('timestamp', 0),
            'system_timestamp': sensor_data.get('system_timestamp', 0),
            'position': position,
            'velocity': velocity
        }
        
        with open(filename, 'w') as f:
            json.dump(data, f)
    
    def _log_combined_data(self, sensor_data):
        """
        记录综合数据（激光雷达+里程计）
        
        参数:
            sensor_data: 传感器数据字典
        """
        # 提取数据
        lidar_data = sensor_data.get('lidar', [])
        position = sensor_data.get('position', (0, 0, 0))
        
        # 保存激光雷达数据为numpy文件
        lidar_filename = os.path.join(self.sensor_dir, f"combined_lidar_{self.sensor_count:06d}")
        np.save(lidar_filename + '.npy', np.array(lidar_data))
        
        # 保存元数据和里程计数据为JSON文件
        metadata = {
            'timestamp': sensor_data.get('timestamp', 0),
            'system_timestamp': sensor_data.get('system_timestamp', 0),
            'position': position,
            'num_lidar_points': len(lidar_data)
        }
        
        with open(lidar_filename + '.json', 'w') as f:
            json.dump(metadata, f)
    
    def close(self):
        """关闭数据记录器"""
        # 记录会话结束
        with open(self.log_file, 'a') as f:
            f.write(f"\n会话结束: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"传感器数据: {self.sensor_count} 条\n")
            f.write(f"导航命令: {self.command_count} 条\n")
            f.write(f"地图: {self.map_count} 张\n")
        
        print(f"数据记录器关闭，会话ID: {self.session_id}")
        print(f"传感器数据: {self.sensor_count} 条")
        print(f"导航命令: {self.command_count} 条")
        print(f"地图: {self.map_count} 张") 