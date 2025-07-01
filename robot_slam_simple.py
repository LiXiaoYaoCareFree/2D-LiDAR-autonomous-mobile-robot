#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""

主要功能：
1. 15x15迷宫环境仿真
2. 激光雷达模拟和SLAM建图
3. A*全局路径规划 (使用PythonRobotics)
4. DWA局部避障 (使用PythonRobotics)
5. 实时可视化 (使用PyRoboViz优化)
"""

import math
import numpy as np
import matplotlib.pyplot as plt
import time
import random
from collections import defaultdict
import sys
import os

# 添加工具包路径到Python路径
sys.path.append('PythonRobotics/PathPlanning/AStar')
sys.path.append('PythonRobotics/PathPlanning/DynamicWindowApproach')
sys.path.append('PyRoboViz')

# 导入PythonRobotics算法
from a_star import AStarPlanner
from dynamic_window_approach import dwa_control, Config as DWAConfig, RobotType
import roboviz

# 设置matplotlib显示
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

class MapEnvironment:
    """地图环境管理类"""
    
    def __init__(self, map_size=15):
        self.map_size = map_size
        self.obstacle_map = np.zeros((map_size, map_size))
        self.wall_segments = self._parse_wall_data()
        self._create_obstacle_map()

    def _parse_wall_data(self):
        """解析墙壁数据"""
        wall_data = [
            ((0,0), (2,0)), ((0,0), (0,15)), ((2,0), (2,7)), ((4,0), (15,0)),
            ((4,0), (4,5)), ((2,7), (4,7)), ((0,9), (2,9)), ((2,9), (2,13)),
            ((4,9), (6,9)), ((6,9), (6,7)), ((6,7), (11,7)), ((11,2), (11,7)),
            ((13,0), (13,2)), ((13,2), (6,2)), ((6,2), (6,5)), ((6,5), (9,5)),
            ((9,5), (9,2)), ((15,0), (15,15)), ((0,15), (11,15)), ((11,15), (11,13)),
            ((4,15), (4,13)), ((4,13), (9,13)), ((6,13), (6,11)), ((9,13), (9,9)),
            ((9,11), (13,11)), ((11,9), (13,9)), ((13,9), (13,4)), ((13,4), (15,4)),
            ((13,15), (13,13)), ((13,15), (15,15))
        ]
        return wall_data

    def _create_obstacle_map(self):
        """创建障碍物地图"""
        self.obstacle_map = np.zeros((self.map_size, self.map_size))
        for start, end in self.wall_segments:
            self._draw_line(start, end)

    def _draw_line(self, start, end):
        """使用Bresenham算法绘制直线"""
        x1, y1 = start
        x2, y2 = end
        
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1
        err = dx - dy

        x, y = x1, y1
        while True:
            if 0 <= x < self.map_size and 0 <= y < self.map_size:
                self.obstacle_map[y][x] = 1
            
            if x == x2 and y == y2:
                break
                
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

    def get_obstacles_for_planning(self):
        """获取障碍物点列表"""
        obstacles = []
        for y in range(self.map_size):
            for x in range(self.map_size):
                if self.obstacle_map[y][x] == 1:
                    obstacles.append([x, y])
        return np.array(obstacles) if obstacles else np.array([]).reshape(0, 2)

    def is_collision(self, x, y, radius=0.3):
        """检查碰撞 - 改进版防止穿墙"""
        grid_x, grid_y = int(round(x)), int(round(y))
        
        # 检查半径范围内的所有网格
        check_radius = max(1, int(math.ceil(radius)))
        
        for dx in range(-check_radius, check_radius + 1):
            for dy in range(-check_radius, check_radius + 1):
                check_x, check_y = grid_x + dx, grid_y + dy
                if 0 <= check_x < self.map_size and 0 <= check_y < self.map_size:
                    if self.obstacle_map[check_y][check_x] == 1:
                        # 计算到障碍物网格中心的距离
                        obstacle_center_x = check_x + 0.5
                        obstacle_center_y = check_y + 0.5
                        dist = math.hypot(x - obstacle_center_x, y - obstacle_center_y)
                        
                        # 如果距离小于半径+网格半径，则认为碰撞
                        if dist <= radius + 0.5:  # 0.5是网格半径
                            return True
        return False

class LidarSimulator:
    """激光雷达模拟器"""
    
    def __init__(self, environment, scan_size=360, max_range=8.0):
        self.environment = environment
        self.scan_size = scan_size
        self.max_range = max_range
        self.angle_step = math.radians(360 / scan_size)

    def scan(self, robot_x, robot_y, robot_theta):
        """执行激光扫描"""
        distances = []
        
        for i in range(self.scan_size):
            angle = robot_theta + (i - self.scan_size // 2) * self.angle_step
            distance = self._ray_cast(robot_x, robot_y, angle)
            distances.append(min(distance, self.max_range * 1000))
            
        return distances

    def _ray_cast(self, start_x, start_y, angle):
        """射线检测 - 优化版"""
        step = 0.05  # 进一步减小步长，提高精度和稳定性
        max_steps = int(self.max_range / step)
        
        # 从机器人位置开始，跳过机器人自身半径
        start_step = int(0.3 / step)  # 跳过机器人半径
        
        for i in range(start_step, max_steps):
            x = start_x + i * step * math.cos(angle)
            y = start_y + i * step * math.sin(angle)
            
            # 边界检测
            if x < 0 or x >= self.environment.map_size or y < 0 or y >= self.environment.map_size:
                return i * step * 1000
            
            # 障碍物检测
            grid_x, grid_y = int(x), int(y)
            if 0 <= grid_x < self.environment.map_size and 0 <= grid_y < self.environment.map_size:
                if self.environment.obstacle_map[grid_y][grid_x] == 1:
                    return i * step * 1000
                
        return self.max_range * 1000

class SimpleSLAM:
    """简化SLAM实现"""
    
    def __init__(self, map_size_pixels=400, map_size_meters=15):
        self.map_size_pixels = map_size_pixels
        self.map_size_meters = map_size_meters
        self.resolution = map_size_meters / map_size_pixels
        
        self.occupancy_map = np.ones((map_size_pixels, map_size_pixels)) * 0.5
        self.hit_count = np.zeros((map_size_pixels, map_size_pixels))
        self.miss_count = np.zeros((map_size_pixels, map_size_pixels))
        
    def update(self, scan_data, robot_x, robot_y, robot_theta):
        """更新地图 - 修复版"""
        robot_px = int((robot_x / self.map_size_meters) * self.map_size_pixels)
        robot_py = int((robot_y / self.map_size_meters) * self.map_size_pixels)
        
        if not (0 <= robot_px < self.map_size_pixels and 0 <= robot_py < self.map_size_pixels):
            return
        
        # 只处理有效的扫描数据，减少噪声
        angle_step = 2 * math.pi / len(scan_data)
        
        for i in range(0, len(scan_data), 5):  # 跳跃采样，减少数据量
            distance_mm = scan_data[i]
            if distance_mm >= 80000 or distance_mm < 1000:  # 过滤过远和过近的数据
                continue
                
            # 修正角度计算
            angle = robot_theta + (i - len(scan_data) // 2) * angle_step
            distance_m = distance_mm / 1000.0
            
            # 限制扫描距离，避免过长射线
            if distance_m > 6.0:
                distance_m = 6.0
            
            end_x = robot_x + distance_m * math.cos(angle)
            end_y = robot_y + distance_m * math.sin(angle)
            
            # 确保终点在地图范围内
            if 0 <= end_x < self.map_size_meters and 0 <= end_y < self.map_size_meters:
                end_px = int((end_x / self.map_size_meters) * self.map_size_pixels)
                end_py = int((end_y / self.map_size_meters) * self.map_size_pixels)
                
                self._update_line(robot_px, robot_py, end_px, end_py)
    
    def _update_line(self, x0, y0, x1, y1):
        """更新激光线路径"""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        
        while True:
            if 0 <= x < self.map_size_pixels and 0 <= y < self.map_size_pixels:
                self.miss_count[y, x] += 1
                total = self.hit_count[y, x] + self.miss_count[y, x]
                if total > 0:
                    self.occupancy_map[y, x] = self.hit_count[y, x] / total
            
            if x == x1 and y == y1:
                if 0 <= x < self.map_size_pixels and 0 <= y < self.map_size_pixels:
                    self.hit_count[y, x] += 1
                    total = self.hit_count[y, x] + self.miss_count[y, x]
                    self.occupancy_map[y, x] = self.hit_count[y, x] / total
                break
                
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
    
    def get_map_data(self):
        """获取地图数据"""
        map_data = (1 - self.occupancy_map) * 255
        return map_data.astype(np.uint8)

# A*规划器现在使用PythonRobotics中的高效实现
# AStarPlanner已从上面的导入语句中导入

class OptimizedDWAPlanner:
    """优化的DWA规划器，基于PythonRobotics"""
    
    def __init__(self):
        self.config = DWAConfig()
        # 调整参数以适应我们的环境
        self.config.max_speed = 0.5
        self.config.min_speed = 0.0
        self.config.max_yaw_rate = 40.0 * math.pi / 180.0
        self.config.max_accel = 0.2
        self.config.max_delta_yaw_rate = 40.0 * math.pi / 180.0
        self.config.v_resolution = 0.02
        self.config.yaw_rate_resolution = 0.2 * math.pi / 180.0
        self.config.dt = 0.1
        self.config.predict_time = 2.0
        self.config.to_goal_cost_gain = 0.15
        self.config.speed_cost_gain = 1.0
        self.config.obstacle_cost_gain = 1.0
        self.config.robot_radius = 0.3
        self.config.robot_type = RobotType.circle

    def dwa_control(self, x, goal, ob):
        """DWA控制接口"""
        # 转换状态格式
        state = np.array([x[0], x[1], x[2], x[3], x[4]])
        goal_point = [goal[0], goal[1]]
        
        # 调用PythonRobotics的DWA算法
        u, trajectory = dwa_control(state, self.config, goal_point, ob)
        
        return u, trajectory

class Robot:
    """机器人主类"""
    
    def __init__(self, environment, start_x=3, start_y=0):
        self.environment = environment
        self.x = start_x
        self.y = start_y
        self.theta = 0.0
        self.v = 0.0
        self.omega = 0.0
        
        # 初始化组件
        self.lidar = LidarSimulator(environment)
        self.slam = SimpleSLAM()
        self.local_planner = OptimizedDWAPlanner()
        
        # 状态变量
        self.current_path = []
        self.path_index = 0
        self.is_mapping = True
        self.mapping_complete = False
        self.exploration_complete = False
        
        # 轨迹记录
        self.trajectory = [(self.x, self.y)]
        self.visited_cells = set()
        
        # 碰撞状态 - 新增
        self.collision_detected = False
        self.collision_recovery_steps = 0
        self.recovery_direction = 0  # 恢复转向方向：1为顺时针，-1为逆时针
        self.recovery_target_angle = 0  # 目标转向角度
        self.recovery_turned_angle = 0  # 已转向角度
        
        # 探索目标 - 基于地图分析的安全路径点
        self.exploration_targets = [
            (5, 1),   # 第一步：向右移动到开阔区
            (8, 3),   # 进入中央开放区域
            (3, 8),   # 探索左上区域
            (10, 8),  # 探索右上区域
            (14, 12), # 接近终点区域
            (12, 14)  # 最终探索点
        ]
        self.target_index = 0
        self.exploration_target = None
        
    def update(self, dt=0.2):  # 增加时间步长，提高移动速度
        """更新机器人状态"""
        if self.is_mapping and not self.mapping_complete:
            self._exploration_behavior()
        elif self.mapping_complete and not self.exploration_complete:
            self._path_following_behavior()
        
        # 更新位置
        new_x = self.x + self.v * math.cos(self.theta) * dt
        new_y = self.y + self.v * math.sin(self.theta) * dt
        new_theta = self.theta + self.omega * dt
        
        # 碰撞检测和处理 - 修复穿墙问题
        collision = self.environment.is_collision(new_x, new_y, 0.2)  # 使用20cm合理半径
        
        # 碰撞调试信息（每20步打印一次）
        if not hasattr(self, '_collision_debug_counter'):
            self._collision_debug_counter = 0
        self._collision_debug_counter += 1
        
        if self._collision_debug_counter % 20 == 0:
            print(f"碰撞检测: ({self.x:.2f},{self.y:.2f}) -> ({new_x:.2f},{new_y:.2f}) 碰撞:{collision}")
        
        if not collision:
            self.x = new_x
            self.y = new_y
            self.theta = new_theta
            # 没有碰撞时减少恢复计数
            if self.collision_recovery_steps > 0:
                self.collision_recovery_steps -= 1
            if self.collision_recovery_steps == 0:
                self.collision_detected = False
                self.recovery_direction = 0  # 重置转向方向
                self.recovery_target_angle = 0  # 重置目标角度
                self.recovery_turned_angle = 0  # 重置已转角度
        else:
            # 碰撞时设置碰撞状态
            if not self.collision_detected:
                # 首次碰撞
                self.collision_detected = True
                self.collision_recovery_steps = 15  # 增加恢复时间到15步
                self.recovery_direction = 0  # 重置，会重新选择方向和角度
                self.recovery_target_angle = 0
                self.recovery_turned_angle = 0
            else:
                # 恢复过程中再次碰撞，重新开始避障
                self.recovery_direction = 0  # 重新选择避障策略
                self.recovery_target_angle = 0
                self.recovery_turned_angle = 0
                self.collision_recovery_steps = 15  # 重置恢复时间
            
            if self.v > 0:
                self.v = 0.0
                if self._collision_debug_counter % 20 == 0:
                    print(f"碰撞！启动避障模式")
            # 允许转向，不重置omega
        
        # 边界限制
        self.x = max(0.15, min(self.environment.map_size - 0.15, self.x))
        self.y = max(0.15, min(self.environment.map_size - 0.15, self.y))
        
        # 角度规范化
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # 记录轨迹
        self.trajectory.append((self.x, self.y))
        self.visited_cells.add((int(self.x), int(self.y)))
        
        # 更新SLAM
        scan_data = self.lidar.scan(self.x, self.y, self.theta)
        self.slam.update(scan_data, self.x, self.y, self.theta)
        
    def _exploration_behavior(self):
        """探索行为 - 简化高效版"""
        if self.exploration_target is None or self._reached_target():
            if self.target_index < len(self.exploration_targets):
                self.exploration_target = self.exploration_targets[self.target_index]
                self.target_index += 1
                print(f"设定新探索目标: {self.exploration_target}")
                print(f"当前位置: ({self.x:.2f}, {self.y:.2f})")
            else:
                print("探索完成，开始路径规划...")
                self.mapping_complete = True
                self.is_mapping = False
                self._plan_optimal_path()
                return
        
        if self.exploration_target:
            goal_x, goal_y = self.exploration_target
            
            # 简化导航：直接朝目标前进，只在必要时避障
            self._simple_navigation_to_goal(goal_x, goal_y)
    
    def _simple_navigation_to_goal(self, goal_x, goal_y):
        """简化的目标导航 - 碰撞感知版"""
        # 计算到目标的距离和角度
        dist_to_goal = math.hypot(goal_x - self.x, goal_y - self.y)
        angle_to_goal = math.atan2(goal_y - self.y, goal_x - self.x)
        angle_diff = angle_to_goal - self.theta
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
        
        # 获取激光雷达扫描数据
        scan_data = self.lidar.scan(self.x, self.y, self.theta)
        
        # 前方障碍检测
        front_range = 15
        front_start = len(scan_data) // 2 - front_range // 2
        front_end = len(scan_data) // 2 + front_range // 2
        front_distances = [scan_data[i] / 1000.0 for i in range(front_start, front_end)]
        front_min_distance = min(front_distances) if front_distances else float('inf')
        
        # 调试信息
        if hasattr(self, '_debug_counter'):
            self._debug_counter += 1
        else:
            self._debug_counter = 0
            
        if self._debug_counter % 10 == 0:
            collision_status = "碰撞中" if self.collision_detected else "正常"
            print(f"位置:({self.x:.2f},{self.y:.2f}) -> 目标:({goal_x},{goal_y}) 状态:{collision_status}")
            print(f"距离:{dist_to_goal:.2f} 角度差:{math.degrees(angle_diff):.1f}° 前方距离:{front_min_distance:.2f}")
        
        # 碰撞恢复模式 - 优先级最高
        if self.collision_detected:
            # 如果刚进入碰撞恢复模式，选择转向角度和方向
            if self.recovery_direction == 0:
                # 检查左右侧空间，选择更开阔的一侧
                left_range = slice(len(scan_data) // 4, len(scan_data) // 4 + 30)
                right_range = slice(3 * len(scan_data) // 4, 3 * len(scan_data) // 4 + 30)
                left_distance = min([scan_data[i] / 1000.0 for i in range(*left_range.indices(len(scan_data)))]) if scan_data else 0.1
                right_distance = min([scan_data[i] / 1000.0 for i in range(*right_range.indices(len(scan_data)))]) if scan_data else 0.1
                
                # 选择更开阔的一侧
                if left_distance > right_distance + 0.1:
                    self.recovery_direction = -1  # 逆时针（向左）
                else:
                    self.recovery_direction = 1   # 顺时针（向右）
                
                # 设置随机转向角度（45-120度）
                self.recovery_target_angle = random.uniform(math.radians(45), math.radians(120))
                self.recovery_turned_angle = 0
                
                if self._debug_counter % 10 == 0:
                    direction_str = "逆时针" if self.recovery_direction == -1 else "顺时针"
                    print(f"选择恢复策略: {direction_str}转{math.degrees(self.recovery_target_angle):.1f}度")
            
            # 检查是否已经转向足够的角度
            if self.recovery_turned_angle < self.recovery_target_angle:
                # 继续转向
                self.v = 0.0
                self.omega = 1.5 * self.recovery_direction
                self.recovery_turned_angle += abs(self.omega) * 0.2  # dt=0.2
                
                if self._debug_counter % 10 == 0:
                    remaining = math.degrees(self.recovery_target_angle - self.recovery_turned_angle)
                    direction_str = "逆时针" if self.recovery_direction == -1 else "顺时针"
                    print(f"模式：转向中({direction_str}) 剩余{remaining:.1f}度")
            else:
                # 转向完成，尝试前进
                self.v = 0.8  # 慢速前进
                self.omega = 0.0
                
                if self._debug_counter % 10 == 0:
                    print(f"模式：碰撞恢复前进 v={self.v:.2f} ω={self.omega:.2f}")
            
            return
        
        # 正常导航逻辑
        safe_distance = 0.15
        abs_angle_diff = abs(angle_diff)
        
        if abs_angle_diff > math.radians(20):
            # 需要转向
            if front_min_distance > 0.12:
                self.v = 0.8
                self.omega = angle_diff * 2.0
                if self._debug_counter % 10 == 0:
                    print(f"模式：转向前进 v={self.v:.2f} ω={self.omega:.2f}")
            else:
                self.v = 0.0
                self.omega = 2.0 if angle_diff > 0 else -2.0
                if self._debug_counter % 10 == 0:
                    print(f"模式：原地转向 v={self.v:.2f} ω={self.omega:.2f}")
        elif front_min_distance > safe_distance:
            self.v = 1.5
            self.omega = angle_diff * 1.5
            if self._debug_counter % 10 == 0:
                print(f"模式：积极前进 v={self.v:.2f} ω={self.omega:.2f}")
        else:
            self.v = 0.6
            self.omega = 2.0 if random.random() > 0.5 else -2.0
            if self._debug_counter % 10 == 0:
                print(f"模式：慢速避障 v={self.v:.2f} ω={self.omega:.2f}")
        
        # 限制速度
        self.v = max(-0.5, min(2.0, self.v))
        self.omega = max(-2.0, min(2.0, self.omega))
    
    def _obstacle_avoidance(self, left_distance, right_distance, safe_distance):
        """障碍物避障逻辑"""
        if left_distance > right_distance and left_distance > safe_distance:
            # 左侧更开阔，向左避障
            self.v = 1.0
            self.omega = 1.0
        elif right_distance > safe_distance:
            # 右侧开阔，向右避障
            self.v = 1.0
            self.omega = -1.0
        else:
            # 两侧都有障碍，后退并转向较开阔的一侧
            self.v = -0.3
            self.omega = 1.5 if left_distance > right_distance else -1.5
    
    def _reached_target(self):
        """检查是否到达目标"""
        if self.exploration_target:
            dist = math.hypot(self.exploration_target[0] - self.x, 
                             self.exploration_target[1] - self.y)
            reached = dist < 1.0  # 放宽到达判断距离
            if reached:
                print(f"到达探索目标 {self.exploration_target}, 距离: {dist:.2f}")
            return reached
        return False
    
    def _path_following_behavior(self):
        """路径跟踪行为"""
        if not self.current_path or self.path_index >= len(self.current_path):
            self.v = 0.0
            self.omega = 0.0
            self.exploration_complete = True
            print("任务完成！")
            return
        
        goal = self.current_path[self.path_index]
        dist_to_goal = math.hypot(goal[0] - self.x, goal[1] - self.y)
        
        if dist_to_goal < 0.5:
            self.path_index += 1
            print(f"到达路径点 {self.path_index}/{len(self.current_path)}")
            return
        
        # 使用DWA进行局部路径规划
        state = np.array([self.x, self.y, self.theta, self.v, self.omega])
        obstacles = self.environment.get_obstacles_for_planning()
        
        try:
            u, trajectory = self.local_planner.dwa_control(state, goal, obstacles)
            self.v, self.omega = u
        except:
            # 简单控制作为备选
            angle_to_goal = math.atan2(goal[1] - self.y, goal[0] - self.x)
            angle_diff = angle_to_goal - self.theta
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
            
            self.v = 0.2
            self.omega = angle_diff * 0.5
    
    def _plan_optimal_path(self):
        """A*路径规划"""
        obstacles = self.environment.get_obstacles_for_planning()
        
        ox = []
        oy = []
        
        if len(obstacles) > 0:
            ox = obstacles[:, 0].tolist()
            oy = obstacles[:, 1].tolist()
        
        # 添加边界
        for i in range(self.environment.map_size + 2):
            ox.extend([i, i, -1, self.environment.map_size])
            oy.extend([-1, self.environment.map_size, i, i])
        
        try:
            path_planner = AStarPlanner(ox, oy, 0.5, 0.3)
            goal_x, goal_y = 12, 15  # 修正为用户指定的终点
            path_x, path_y = path_planner.planning(self.x, self.y, goal_x, goal_y)
            
            if path_x and path_y:
                self.current_path = list(zip(path_x, path_y))
                self.current_path.reverse()
                print(f"路径规划成功，路径长度：{len(self.current_path)}")
            else:
                print("路径规划失败")
                self.current_path = [(goal_x, goal_y)]
        except Exception as e:
            print(f"路径规划错误：{e}")
            self.current_path = [(12, 15)]  # 修正备用路径的终点
    
    def get_map_data(self):
        """获取SLAM地图数据"""
        return self.slam.get_map_data()

class RobotVisualizer:
    """可视化类"""
    
    def __init__(self, robot, environment):
        self.robot = robot
        self.environment = environment
        
        # 创建图形
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(16, 8))
        self.fig.suptitle('扫地机器人建图和导航仿真 (使用PythonRobotics算法)', fontsize=16, fontweight='bold')
        
        # 左侧：真实环境
        self.ax1.set_title('真实环境和机器人轨迹')
        self.ax1.set_xlim(-1, 16)
        self.ax1.set_ylim(-1, 16)
        self.ax1.set_aspect('equal')
        self.ax1.grid(True, alpha=0.3)
        
        # 右侧：SLAM地图
        self.ax2.set_title('SLAM建图结果')
        self.ax2.set_aspect('equal')
        
        # 绘制环境
        self._draw_environment()
        
        # 初始化动态元素
        self.robot_patch = None
        self.trajectory_line = None
        self.slam_image = None
        self.path_line = None
        
    def _draw_environment(self):
        """绘制环境地图"""
        # 绘制墙壁
        for start, end in self.environment.wall_segments:
            self.ax1.plot([start[0], end[0]], [start[1], end[1]], 'k-', linewidth=3)
        
        # 起点和终点
        self.ax1.plot(3, 0, 'go', markersize=10, label='起点(3,0)')
        self.ax1.plot(12, 15, 'ro', markersize=10, label='终点(12,15)')
        
        # 网格
        for i in range(16):
            self.ax1.axhline(y=i, color='gray', linestyle=':', alpha=0.5)
            self.ax1.axvline(x=i, color='gray', linestyle=':', alpha=0.5)
        
        self.ax1.legend()
    
    def update(self):
        """更新可视化"""
        # 清除动态元素
        if self.robot_patch:
            self.robot_patch.remove()
        if self.trajectory_line:
            for line in self.trajectory_line:
                line.remove()
        if self.path_line:
            for line in self.path_line:
                line.remove()
        
        # 绘制机器人
        robot_x, robot_y = self.robot.x, self.robot.y
        robot_size = 0.3
        
        circle = plt.Circle((robot_x, robot_y), robot_size, color='red', alpha=0.7)
        self.ax1.add_patch(circle)
        self.robot_patch = circle
        
        # 方向指示
        arrow_length = 0.5
        arrow_x = robot_x + arrow_length * math.cos(self.robot.theta)
        arrow_y = robot_y + arrow_length * math.sin(self.robot.theta)
        self.ax1.annotate('', xy=(arrow_x, arrow_y), xytext=(robot_x, robot_y),
                         arrowprops=dict(arrowstyle='->', color='blue', lw=2))
        
        # 绘制轨迹
        if len(self.robot.trajectory) > 1:
            traj_x, traj_y = zip(*self.robot.trajectory)
            self.trajectory_line = self.ax1.plot(traj_x, traj_y, 'b-', alpha=0.6, linewidth=1)
        
        # 绘制路径
        if hasattr(self.robot, 'current_path') and self.robot.current_path:
            path_x, path_y = zip(*self.robot.current_path)
            self.path_line = self.ax1.plot(path_x, path_y, 'g--', linewidth=2, alpha=0.7, label='A*路径(PythonRobotics)')
        
        # 探索目标
        if self.robot.exploration_target and self.robot.is_mapping:
            target_x, target_y = self.robot.exploration_target
            self.ax1.plot(target_x, target_y, 'r*', markersize=15)
        
        # SLAM地图
        slam_map = self.robot.get_map_data()
        if self.slam_image is None:
            self.slam_image = self.ax2.imshow(slam_map, cmap='gray', origin='lower', extent=[0, 15, 0, 15])
        else:
            self.slam_image.set_array(slam_map)
        
        # 状态信息
        if self.robot.is_mapping:
            target_info = f"目标{self.robot.exploration_target}" if self.robot.exploration_target else "无目标"
            status = f"建图探索中 - {target_info}"
        elif self.robot.exploration_complete:
            status = "任务完成！"
        else:
            status = "执行规划路径中(DWA避障)"
        
        visited_count = len(self.robot.visited_cells)
        speed_info = f"v={self.robot.v:.2f}m/s ω={self.robot.omega:.2f}rad/s"
        self.fig.suptitle(f'机器人仿真 - {status} | {speed_info} | 已访问:{visited_count}格', 
                         fontsize=13, fontweight='bold')
        
        plt.pause(0.01)

def main():
    """主程序"""
    print("启动扫地机器人建图和导航仿真程序...")
    print("程序功能：")
    print("1. 15x15迷宫环境仿真")
    print("2. 激光雷达扫描和SLAM建图")
    print("3. A*全局路径规划 (PythonRobotics)")
    print("4. DWA局部避障 (PythonRobotics)")
    print("5. 实时可视化显示")
    
    # 创建环境和机器人
    environment = MapEnvironment()
    robot = Robot(environment)
    visualizer = RobotVisualizer(robot, environment)
    
    # 仿真循环
    dt = 0.2  # 增加时间步长，提高移动速度
    step_count = 0
    max_steps = 3000  # 减少最大步数以适应更大的时间步长
    
    try:
        while step_count < max_steps and not robot.exploration_complete:
            robot.update(dt)
            
            # 每1步更新一次可视化，保持实时性
            if step_count % 1 == 0:
                visualizer.update()
            
            step_count += 1
            time.sleep(0.01)  # 稍微增加睡眠时间以便观察
            
            # 检查窗口状态
            if not plt.get_fignums():
                break
            
        print("仿真完成")
        print(f"总步数: {step_count}")
        print(f"访问的格子数: {len(robot.visited_cells)}")
        print(f"轨迹长度: {len(robot.trajectory)}")
        print("使用的算法：")
        print("- A*路径规划：PythonRobotics实现")
        print("- DWA局部避障：PythonRobotics实现")
        print("- SLAM建图：保持原始实现")
        
        # 保持窗口显示
        input("按回车键退出...")
        
    except KeyboardInterrupt:
        print("\n用户中断仿真")
    except Exception as e:
        print(f"仿真错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        plt.close('all')

if __name__ == "__main__":
    main() 