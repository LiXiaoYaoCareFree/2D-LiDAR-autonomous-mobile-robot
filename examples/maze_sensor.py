#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
迷宫传感器模块
包含激光传感器的实现
"""

import numpy as np

class LaserSensor:
    """激光传感器"""
    def __init__(self, env, range_max=5.0, angle_min=-np.pi/2, angle_max=np.pi/2, angle_increment=np.pi/180):
        self.env = env
        self.range_max = range_max  # 最大测量距离
        self.angle_min = angle_min  # 最小角度
        self.angle_max = angle_max  # 最大角度
        self.angle_increment = angle_increment  # 角度增量
        self.angles = np.arange(angle_min, angle_max + angle_increment, angle_increment)
    
    def scan(self, pose):
        """扫描环境，返回激光点的坐标"""
        x, y, theta = pose
        laser_points = []
        
        for angle in self.angles:
            # 计算激光束的全局角度
            global_angle = theta + angle
            
            # 初始化激光束终点
            end_x = x + self.range_max * np.cos(global_angle)
            end_y = y + self.range_max * np.sin(global_angle)
            
            # 检查激光束是否与障碍物相交
            min_dist = self.range_max
            
            # 遍历所有可能的障碍物
            for obs_x in range(max(0, int(x - self.range_max - 1)), min(self.env.x_range, int(x + self.range_max + 1))):
                for obs_y in range(max(0, int(y - self.range_max - 1)), min(self.env.y_range, int(y + self.range_max + 1))):
                    if (obs_x, obs_y) in self.env.obstacles:
                        # 计算障碍物的四个角点
                        corners = [
                            (obs_x - 0.5, obs_y - 0.5),
                            (obs_x + 0.5, obs_y - 0.5),
                            (obs_x + 0.5, obs_y + 0.5),
                            (obs_x - 0.5, obs_y + 0.5)
                        ]
                        
                        # 检查激光束是否与障碍物相交
                        for i in range(4):
                            x1, y1 = corners[i]
                            x2, y2 = corners[(i + 1) % 4]
                            
                            # 检查线段相交
                            intersection = self.line_intersection((x, y), (end_x, end_y), (x1, y1), (x2, y2))
                            if intersection:
                                ix, iy = intersection
                                # 计算距离
                                dist = np.sqrt((ix - x) ** 2 + (iy - y) ** 2)
                                if dist < min_dist:
                                    min_dist = dist
                                    end_x = ix
                                    end_y = iy
            
            # 添加激光点
            laser_points.append((end_x, end_y))
        
        return laser_points
    
    def line_intersection(self, p1, p2, p3, p4):
        """计算两条线段的交点"""
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = p3
        x4, y4 = p4
        
        # 计算分母
        denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1)
        if denom == 0:
            return None  # 平行线
        
        # 计算参数
        ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denom
        ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denom
        
        # 检查参数是否在[0,1]范围内
        if 0 <= ua <= 1 and 0 <= ub <= 1:
            # 计算交点
            x = x1 + ua * (x2 - x1)
            y = y1 + ua * (y2 - y1)
            return (x, y)
        
        return None  # 不相交 