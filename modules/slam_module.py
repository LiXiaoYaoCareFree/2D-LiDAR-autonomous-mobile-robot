#!/usr/bin/env python3
'''
slam_module.py - SLAM系统模块，使用BreezySLAM处理激光雷达和里程计数据

该模块负责处理激光雷达和里程计数据，构建环境地图，并估计机器人位置。
'''

import numpy as np
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import Laser

class LidarModel(Laser):
    """自定义激光雷达模型类，继承自BreezySLAM的Laser类"""
    
    def __init__(self, scan_size=360, scan_rate_hz=10, 
                 detection_angle_degrees=360, distance_no_detection_mm=5000,
                 detection_margin=70, offset_mm=0):
        """
        初始化激光雷达模型
        
        参数:
            scan_size: 一次扫描的点数
            scan_rate_hz: 扫描频率(Hz)
            detection_angle_degrees: 扫描角度范围(度)
            distance_no_detection_mm: 无检测时的距离值(毫米)
            detection_margin: 检测边缘(毫米)
            offset_mm: 偏移量(毫米)
        """
        # 调用父类构造函数
        Laser.__init__(self, 
                      scan_size, 
                      scan_rate_hz, 
                      detection_angle_degrees, 
                      distance_no_detection_mm, 
                      detection_margin, 
                      offset_mm)

class SLAMSystem:
    """SLAM系统类，使用BreezySLAM处理激光雷达和里程计数据"""
    
    def __init__(self, map_size_pixels=800, map_size_meters=32, 
                 use_odometry=True, random_seed=None):
        """
        初始化SLAM系统
        
        参数:
            map_size_pixels: 地图大小(像素)
            map_size_meters: 地图大小(米)
            use_odometry: 是否使用里程计
            random_seed: 随机种子，用于粒子滤波
        """
        # 创建激光雷达模型
        self.lidar = LidarModel()
        
        # 创建SLAM对象
        self.slam = RMHC_SLAM(self.lidar, map_size_pixels, map_size_meters, random_seed=random_seed) \
                    if random_seed is not None else \
                    RMHC_SLAM(self.lidar, map_size_pixels, map_size_meters)
        
        # 初始化地图数据
        self.map_size_pixels = map_size_pixels
        self.mapbytes = bytearray(map_size_pixels * map_size_pixels)
        
        # 记录是否使用里程计
        self.use_odometry = use_odometry
        
        # 当前位置
        self.current_position = (0, 0, 0)  # (x, y, theta)
    
    def update(self, lidar_data, odometry=None):
        """
        使用激光雷达和里程计数据更新SLAM
        
        参数:
            lidar_data: 激光雷达数据，距离值列表(毫米)
            odometry: 里程计数据，格式为(dx, dy, dtheta)，单位为毫米和度
        """
        # 确保激光雷达数据是整数数组
        scan_data = np.array(lidar_data, dtype=np.int32)
        
        if self.use_odometry and odometry is not None:
            # 使用里程计数据更新SLAM
            self.slam.update(scan_data, odometry)
        else:
            # 不使用里程计数据更新SLAM
            self.slam.update(scan_data)
        
        # 更新当前位置
        x_mm, y_mm, theta_degrees = self.slam.getpos()
        self.current_position = (x_mm / 1000.0, y_mm / 1000.0, theta_degrees)  # 转换为米和度
        
        # 更新地图
        self.slam.getmap(self.mapbytes)
    
    def get_position(self):
        """
        获取当前位置
        
        返回:
            (x, y, theta): 位置和方向，单位为米和度
        """
        return self.current_position
    
    def get_map(self):
        """
        获取当前地图
        
        返回:
            map_array: 地图数据的numpy数组
        """
        # 将地图字节转换为numpy数组
        map_array = np.reshape(np.frombuffer(self.mapbytes, dtype=np.uint8), 
                              (self.map_size_pixels, self.map_size_pixels))
        return map_array
    
    def reset(self):
        """重置SLAM系统"""
        self.slam.reset()
        self.current_position = (0, 0, 0)
        self.mapbytes = bytearray(self.map_size_pixels * self.map_size_pixels) 