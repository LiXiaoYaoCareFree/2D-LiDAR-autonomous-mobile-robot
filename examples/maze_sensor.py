#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
激光传感器模块
提供激光传感器类，用于模拟机器人的激光雷达传感器
"""

import math
import numpy as np

class LaserSensor:
    """激光传感器类，模拟激光雷达"""
    
    def __init__(self, max_range=5.0, num_rays=36):
        """初始化激光传感器
        
        参数:
            max_range: 激光的最大探测范围
            num_rays: 激光束的数量
        """
        self.max_range = max_range
        self.num_rays = num_rays
        
    def scan(self, pose, env):
        """扫描环境，返回激光点的坐标
        
        参数:
            pose: 机器人的位置和朝向 (x, y, theta)
            env: 环境对象，包含障碍物信息
            
        返回:
            激光点的坐标列表
        """
        x, y, theta = pose
        laser_points = []
        
        # 计算每个激光束的角度
        angles = np.linspace(theta - math.pi, theta + math.pi, self.num_rays)
        
        for angle in angles:
            # 计算激光束的终点
            end_x = x + self.max_range * math.cos(angle)
            end_y = y + self.max_range * math.sin(angle)
            
            # 使用射线检测找到最近的障碍物
            hit_point = self._ray_cast(x, y, end_x, end_y, env)
            if hit_point:
                laser_points.append(hit_point)
                
        return laser_points
    
    def _ray_cast(self, x1, y1, x2, y2, env):
        """射线检测，找到从(x1,y1)到(x2,y2)的射线与障碍物的交点
        
        参数:
            x1, y1: 射线起点
            x2, y2: 射线终点
            env: 环境对象，包含障碍物信息
            
        返回:
            如果有交点，返回交点坐标；否则返回射线终点
        """
        # 使用Bresenham算法获取射线上的所有点
        line_points = self._bresenham_line(int(round(x1)), int(round(y1)), 
                                          int(round(x2)), int(round(y2)))
        
        # 检查射线上的每个点是否是障碍物
        for x, y in line_points:
            if (x, y) in env.obstacles:
                return (x, y)
                
        # 如果没有交点，返回射线终点
        return (x2, y2)
    
    def _bresenham_line(self, x0, y0, x1, y1):
        """Bresenham算法，获取从(x0,y0)到(x1,y1)的线上的所有点
        
        参数:
            x0, y0: 线的起点
            x1, y1: 线的终点
            
        返回:
            线上的所有点的坐标列表
        """
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        while True:
            points.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
                
        return points 