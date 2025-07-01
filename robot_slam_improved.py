#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
优化版机器人SLAM和导航系统
- 更高效的SLAM算法
- 改进的路径规划
- 动态目标选择
- 并行处理提高性能
"""

import math
import numpy as np
import matplotlib.pyplot as plt
import time
import random
from collections import defaultdict
import sys
import os
import threading
from queue import Queue

# 添加工具包路径到Python路径
sys.path.append('PythonRobotics/PathPlanning/AStar')
sys.path.append('PythonRobotics/PathPlanning/DynamicWindowApproach')
sys.path.append('PyRoboViz')

# 导入PythonRobotics算法
from a_star import AStarPlanner
from dynamic_window_approach import dwa_control, Config as DWAConfig, RobotType
import fixed_roboviz as roboviz

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

class FastLidarSimulator:
    """高性能激光雷达模拟器"""
    
    def __init__(self, environment, scan_size=180, max_range=8.0):  # 减少扫描点，提高速度
        self.environment = environment
        self.scan_size = scan_size
        self.max_range = max_range
        self.angle_step = math.radians(360 / scan_size)
        # 预计算角度
        self.angles = [i * self.angle_step for i in range(scan_size)]

    def scan(self, robot_x, robot_y, robot_theta):
        """并行执行激光扫描"""
        distances = []
        
        # 批量处理以减少函数调用开销
        for angle_idx in range(self.scan_size):
            angle = robot_theta + self.angles[angle_idx] - math.pi  # 均匀分布在360度
            distance = self._ray_cast(robot_x, robot_y, angle)
            distances.append(min(distance, self.max_range * 1000))
            
        return distances

    def _ray_cast(self, start_x, start_y, angle):
        """优化版射线检测 - 使用更大步长，减少计算量"""
        step = 0.1  # 增大步长，提高速度
        max_steps = int(self.max_range / step)
        cos_angle, sin_angle = math.cos(angle), math.sin(angle)
        
        # 从机器人位置开始，跳过机器人自身半径
        start_step = int(0.3 / step)  # 跳过机器人半径
        
        for i in range(start_step, max_steps):
            x = start_x + i * step * cos_angle
            y = start_y + i * step * sin_angle
            
            # 边界检测
            if x < 0 or x >= self.environment.map_size or y < 0 or y >= self.environment.map_size:
                return i * step * 1000
            
            # 障碍物检测 - 整数化坐标，减少类型转换
            grid_x, grid_y = int(x), int(y)
            if self.environment.obstacle_map[grid_y][grid_x] == 1:
                return i * step * 1000
                
        return self.max_range * 1000

class OptimizedSLAM:
    """优化的SLAM实现"""
    
    def __init__(self, map_size_pixels=300, map_size_meters=15):  # 减小地图大小提高速度
        self.map_size_pixels = map_size_pixels
        self.map_size_meters = map_size_meters
        self.resolution = map_size_meters / map_size_pixels
        
        # 使用整数计数并后期计算概率，更高效
        self.occupancy_map = np.ones((map_size_pixels, map_size_pixels)) * 0.5
        self.hit_count = np.zeros((map_size_pixels, map_size_pixels), dtype=np.uint16)
        self.miss_count = np.zeros((map_size_pixels, map_size_pixels), dtype=np.uint16)
        
        # 预计算常量，减少除法操作
        self.meters_to_pixels = map_size_pixels / map_size_meters
        
    def update(self, scan_data, robot_x, robot_y, robot_theta):
        """更新地图 - 高性能版"""
        robot_px = int(robot_x * self.meters_to_pixels)
        robot_py = int(robot_y * self.meters_to_pixels)
        
        if not (0 <= robot_px < self.map_size_pixels and 0 <= robot_py < self.map_size_pixels):
            return
        
        # 只处理部分扫描数据，大幅减少计算量
        angle_step = 2 * math.pi / len(scan_data)
        
        for i in range(0, len(scan_data), 10):  # 大幅增加采样间隔，提高性能
            distance_mm = scan_data[i]
            if distance_mm >= 7000 or distance_mm < 500:  # 调整过滤范围
                continue
                
            # 计算激光终点
            angle = robot_theta + (i * angle_step - math.pi)
            distance_m = min(distance_mm / 1000.0, 6.0)  # 限制最大距离
            
            cos_angle, sin_angle = math.cos(angle), math.sin(angle)
            end_x = robot_x + distance_m * cos_angle
            end_y = robot_y + distance_m * sin_angle
            
            # 确保终点在地图范围内
            if 0 <= end_x < self.map_size_meters and 0 <= end_y < self.map_size_meters:
                end_px = int(end_x * self.meters_to_pixels)
                end_py = int(end_y * self.meters_to_pixels)
                
                # 使用更高效的Bresenham算法
                self._update_line_fast(robot_px, robot_py, end_px, end_py)
    
    def _update_line_fast(self, x0, y0, x1, y1):
        """高效版本的Bresenham直线算法"""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        hit_end = False
        
        while not hit_end:
            if x == x1 and y == y1:
                hit_end = True
                # 只在终点标记为障碍物
                if 0 <= x < self.map_size_pixels and 0 <= y < self.map_size_pixels:
                    self.hit_count[y, x] = min(self.hit_count[y, x] + 2, 65000)  # 终点权重加倍
                    # 动态计算阈值，避免频繁更新整个地图
                    if self.hit_count[y, x] + self.miss_count[y, x] > 10:
                        self.occupancy_map[y, x] = min(0.95, self.hit_count[y, x] / 
                                             (self.hit_count[y, x] + self.miss_count[y, x]))
            else:
                # 中间点标记为自由空间
                if 0 <= x < self.map_size_pixels and 0 <= y < self.map_size_pixels:
                    self.miss_count[y, x] = min(self.miss_count[y, x] + 1, 65000)
                    # 只在计数足够时更新
                    if self.hit_count[y, x] + self.miss_count[y, x] > 5:
                        self.occupancy_map[y, x] = max(0.05, self.hit_count[y, x] / 
                                             (self.hit_count[y, x] + self.miss_count[y, x]))
            
            if hit_end:
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
        # 应用阈值化，加速地图识别
        thresholded_map = np.where(self.occupancy_map > 0.65, 0, 
                         np.where(self.occupancy_map < 0.30, 255, 128))
        return thresholded_map.astype(np.uint8)
    
    def get_binary_map(self):
        """获取二值地图，用于路径规划"""
        binary_map = np.where(self.occupancy_map > 0.55, 1, 0)
        return binary_map

class ImprovedRobot:
    """改进版机器人主类"""
    
    def __init__(self, environment, start_x=3, start_y=0):
        self.environment = environment
        self.x = start_x
        self.y = start_y
        self.theta = 0.0
        self.v = 0.0
        self.omega = 0.0
        
        # 初始化组件 - 使用更高效的版本
        self.lidar = FastLidarSimulator(environment)
        self.slam = OptimizedSLAM()
        
        # 创建DWA配置
        self.dwa_config = DWAConfig()
        self.dwa_config.max_speed = 1.0  # 提高最大速度
        self.dwa_config.max_yaw_rate = 100.0 * math.pi / 180.0  # 更灵活的转向
        self.dwa_config.max_accel = 0.5  # 提高加速度
        self.dwa_config.dt = 0.1  # 更快的控制周期
        self.dwa_config.predict_time = 1.5  # 减少预测时间，更积极避障
        self.dwa_config.to_goal_cost_gain = 0.8  # 提高目标吸引力
        self.dwa_config.obstacle_cost_gain = 1.0
        self.dwa_config.robot_radius = 0.3
        
        # 状态变量
        self.current_path = []
        self.path_index = 0
        self.is_mapping = True
        self.mapping_complete = False
        self.exploration_complete = False
        
        # 轨迹记录
        self.trajectory = [(self.x, self.y)]
        self.visited_cells = set()
        
        # 碰撞状态
        self.collision_detected = False
        self.collision_recovery_steps = 0
        self.recovery_direction = 0
        
        # 动态探索目标系统 - 使用更好的目标序列
        self.exploration_targets = [
            (5, 1),   # 右侧走廊入口
            (9, 1),   # 右下开放区域
            (12, 4),  # 右侧中部
            (8, 6),   # 中央区域
            (3, 8),   # 左上走廊
            (7, 12),  # 上部走廊
            (12, 14), # 终点附近
            (13, 14)  # 接近终点
        ]
        self.target_index = 0
        self.exploration_target = None
        
        # 创建线程安全队列，用于SLAM和路径规划
        self.slam_queue = Queue()
        self.path_queue = Queue()
        
        # 启动SLAM线程
        self.slam_thread_active = True
        self.slam_thread = threading.Thread(target=self._slam_worker)
        self.slam_thread.daemon = True
        self.slam_thread.start()
        
    def _slam_worker(self):
        """SLAM后台工作线程 - 处理激光数据"""
        while self.slam_thread_active:
            if not self.slam_queue.empty():
                scan_data, robot_x, robot_y, robot_theta = self.slam_queue.get()
                self.slam.update(scan_data, robot_x, robot_y, robot_theta)
            else:
                time.sleep(0.01)  # 减少CPU使用
        
    def update(self, dt=0.3):  # 增加时间步长，提高移动速度
        """更新机器人状态"""
        # 导航行为决策
        if self.is_mapping and not self.mapping_complete:
            self._exploration_behavior()
        elif self.mapping_complete and not self.exploration_complete:
            self._path_following_behavior()
        
        # 更新位置
        new_x = self.x + self.v * math.cos(self.theta) * dt
        new_y = self.y + self.v * math.sin(self.theta) * dt
        new_theta = self.theta + self.omega * dt
        
        # 碰撞检测和处理
        collision = self.environment.is_collision(new_x, new_y, 0.25)
        
        if not collision:
            self.x = new_x
            self.y = new_y
            self.theta = new_theta
            if self.collision_recovery_steps > 0:
                self.collision_recovery_steps -= 1
            if self.collision_recovery_steps == 0:
                self.collision_detected = False
        else:
            if not self.collision_detected:
                self.collision_detected = True
                self.collision_recovery_steps = 10
                self.recovery_direction = 1 if random.random() > 0.5 else -1
            self.v = 0.0  # 碰撞时停止前进
        
        # 边界限制
        self.x = max(0.25, min(self.environment.map_size - 0.25, self.x))
        self.y = max(0.25, min(self.environment.map_size - 0.25, self.y))
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # 记录轨迹
        self.trajectory.append((self.x, self.y))
        self.visited_cells.add((int(self.x), int(self.y)))
        
        # 异步更新SLAM - 减轻主线程负担
        scan_data = self.lidar.scan(self.x, self.y, self.theta)
        if self.slam_queue.qsize() < 2:  # 限制队列大小防止延迟积累
            self.slam_queue.put((scan_data, self.x, self.y, self.theta))
        
    def _exploration_behavior(self):
        """智能探索行为"""
        if self.exploration_target is None or self._reached_target():
            if self.target_index < len(self.exploration_targets):
                self.exploration_target = self.exploration_targets[self.target_index]
                self.target_index += 1
                print(f"设定新探索目标: {self.exploration_target}")
            else:
                print("探索完成，开始终点路径规划...")
                self.mapping_complete = True
                self.is_mapping = False
                self._direct_to_goal()
                return
        
        if self.exploration_target:
            goal_x, goal_y = self.exploration_target
            self._navigate_to_goal(goal_x, goal_y)
    
    def _navigate_to_goal(self, goal_x, goal_y):
        """改进的目标导航 - 结合DWA和行为控制"""
        # 计算到目标的距离和角度
        dist_to_goal = math.hypot(goal_x - self.x, goal_y - self.y)
        angle_to_goal = math.atan2(goal_y - self.y, goal_x - self.x)
        angle_diff = angle_to_goal - self.theta
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
        
        # 获取激光雷达数据
        scan_data = self.lidar.scan(self.x, self.y, self.theta)
        
        # 简化前方障碍检测
        front_start = len(scan_data) // 4
        front_end = 3 * len(scan_data) // 4
        front_distances = [scan_data[i] / 1000.0 for i in range(front_start, front_end)]
        front_min_distance = min(front_distances) if front_distances else float('inf')
        
        # 碰撞恢复模式
        if self.collision_detected:
            self.v = 0.0
            self.omega = 2.0 * self.recovery_direction
            return
            
        # 正常导航 - 基于距离和障碍物调整速度
        if abs(angle_diff) > math.radians(30):
            # 需要大角度转向，减速转向
            self.v = 0.5
            self.omega = angle_diff * 3.0
        elif front_min_distance < 0.5:
            # 前方障碍物，减速并寻找开放空间
            self.v = 0.3
            
            # 查找左右侧最开阔方向
            left_scan = scan_data[:len(scan_data)//2]
            right_scan = scan_data[len(scan_data)//2:]
            left_space = sum(d/1000.0 for d in left_scan) / len(left_scan)
            right_space = sum(d/1000.0 for d in right_scan) / len(right_scan)
            
            # 向更开阔的方向转向
            if left_space > right_space:
                self.omega = 1.5  # 左转
            else:
                self.omega = -1.5  # 右转
        else:
            # 正常导航，高速前进
            self.v = 1.2
            self.omega = angle_diff * 2.0
            
        # 限制速度
        self.v = max(-0.5, min(2.0, self.v))
        self.omega = max(-3.0, min(3.0, self.omega))
    
    def _reached_target(self):
        """检查是否到达目标"""
        if self.exploration_target:
            dist = math.hypot(self.exploration_target[0] - self.x, 
                             self.exploration_target[1] - self.y)
            reached = dist < 1.0
            if reached:
                print(f"到达探索目标 {self.exploration_target}, 距离: {dist:.2f}")
            return reached
        return False
    
    def _direct_to_goal(self):
        """直接规划到终点的路径"""
        print("开始规划到终点的最优路径...")
        
        # 获取障碍物地图
        binary_map = self.slam.get_binary_map()
        
        # 使用A*算法规划路径
        path_planner = EnhancedAStarPlanner(binary_map)
        start = (int(self.x), int(self.y))
        goal = (13, 14)  # 终点位置
        
        path = path_planner.plan(start, goal)
        
        if path:
            self.current_path = path
            self.path_index = 0
            print(f"成功规划到终点路径，长度: {len(path)}")
        else:
            print("路径规划失败，使用备用导航点")
            # 如果规划失败，使用简单路径点序列
            self.current_path = [(8, 8), (12, 12), (13, 14)]
            self.path_index = 0
    
    def _path_following_behavior(self):
        """路径跟踪行为"""
        if not self.current_path or self.path_index >= len(self.current_path):
            self.v = 0.0
            self.omega = 0.0
            self.exploration_complete = True
            print("到达终点！任务完成！")
            return
        
        # 获取当前目标点
        goal = self.current_path[self.path_index]
        dist_to_goal = math.hypot(goal[0] - self.x, goal[1] - self.y)
        
        # 到达路径点后前进到下一点
        if dist_to_goal < 0.5:
            self.path_index += 1
            print(f"到达路径点 {self.path_index}/{len(self.current_path)}")
            return
        
        # 简单导航到路径点
        self._navigate_to_goal(goal[0], goal[1])
    
    def get_map_data(self):
        """获取SLAM地图数据"""
        return self.slam.get_map_data()
    
    def shutdown(self):
        """关闭线程等资源"""
        self.slam_thread_active = False
        if hasattr(self, 'slam_thread'):
            self.slam_thread.join(timeout=1.0)

class EnhancedAStarPlanner:
    """增强版A*路径规划器"""
    
    def __init__(self, binary_map, grid_size=1.0):
        self.grid_map = binary_map
        self.height, self.width = binary_map.shape
        self.grid_size = grid_size
        
        # 膨胀障碍物，增加安全距离
        self._inflate_obstacles()
        
    def _inflate_obstacles(self, radius=2):
        """膨胀障碍物，确保路径安全"""
        inflated_map = np.copy(self.grid_map)
        
        for y in range(self.height):
            for x in range(self.width):
                if self.grid_map[y, x] == 1:  # 是障碍物
                    # 膨胀周围区域
                    for dy in range(-radius, radius+1):
                        for dx in range(-radius, radius+1):
                            if dx*dx + dy*dy <= radius*radius:  # 圆形膨胀
                                ny, nx = y + dy, x + dx
                                if 0 <= ny < self.height and 0 <= nx < self.width:
                                    inflated_map[ny, nx] = 1
        
        self.grid_map = inflated_map
    
    def plan(self, start, goal):
        """规划从起点到终点的路径"""
        # 检查起点和终点是否可行
        if not self._is_valid(start) or not self._is_valid(goal):
            print("起点或终点在障碍物内或超出地图范围")
            return None
            
        # 使用改进的A*算法
        open_set = {start}
        closed_set = set()
        
        came_from = {}
        
        g_score = {start: 0}
        f_score = {start: self._heuristic(start, goal)}
        
        while open_set:
            current = min(open_set, key=lambda pos: f_score.get(pos, float('inf')))
            
            if current == goal:
                return self._reconstruct_path(came_from, current)
                
            open_set.remove(current)
            closed_set.add(current)
            
            # 八方向移动
            for dx, dy in [(0,1), (1,0), (0,-1), (-1,0), (1,1), (1,-1), (-1,1), (-1,-1)]:
                neighbor = (current[0] + dx, current[1] + dy)
                
                if not self._is_valid(neighbor) or neighbor in closed_set:
                    continue
                    
                # 计算移动代价
                move_cost = 1.0 if dx*dy == 0 else 1.414  # 斜向移动代价更高
                tentative_g = g_score[current] + move_cost
                
                if neighbor not in open_set:
                    open_set.add(neighbor)
                elif tentative_g >= g_score.get(neighbor, float('inf')):
                    continue
                    
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = g_score[neighbor] + self._heuristic(neighbor, goal)
                
        print("无法找到有效路径")
        return None
    
    def _is_valid(self, point):
        """检查点是否有效（在地图内且不是障碍物）"""
        x, y = point
        if 0 <= y < self.height and 0 <= x < self.width:
            return self.grid_map[y, x] == 0  # 0表示自由空间
        return False
    
    def _heuristic(self, a, b):
        """启发式函数 - 使用欧几里得距离"""
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
    
    def _reconstruct_path(self, came_from, current):
        """重建路径"""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]  # 反转路径，从起点到终点

class FastVisualizer:
    """高性能可视化类"""
    
    def __init__(self, robot, environment):
        self.robot = robot
        self.environment = environment
        
        # 创建图形 - 使用更小的尺寸提高渲染速度
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(12, 6))
        self.fig.suptitle('优化版扫地机器人SLAM和导航系统', fontsize=14)
        
        # 设置坐标轴
        self.ax1.set_title('真实环境和机器人轨迹')
        self.ax1.set_xlim(-1, 16)
        self.ax1.set_ylim(-1, 16)
        self.ax1.set_aspect('equal')
        self.ax1.grid(True, alpha=0.3)
        
        self.ax2.set_title('SLAM建图结果')
        self.ax2.set_aspect('equal')
        
        # 绘制环境
        self._draw_environment()
        
        # 初始化动态元素
        self.robot_patch = None
        self.trajectory_line = None
        self.slam_image = None
        self.path_line = None
        
        # 更新率控制
        self.last_update_time = time.time()
        self.update_interval = 0.1  # 10Hz更新率
    
    def _draw_environment(self):
        """绘制环境地图"""
        # 绘制墙壁
        for start, end in self.environment.wall_segments:
            self.ax1.plot([start[0], end[0]], [start[1], end[1]], 'k-', linewidth=2)
        
        # 起点和终点
        self.ax1.plot(3, 0, 'go', markersize=8, label='起点')
        self.ax1.plot(13, 14, 'ro', markersize=8, label='终点')
        
        self.ax1.legend(loc='upper left')
    
    def update(self):
        """更新可视化 - 加入更新率限制"""
        current_time = time.time()
        if current_time - self.last_update_time < self.update_interval:
            return
        self.last_update_time = current_time
        
        # 清除动态元素
        if self.robot_patch:
            self.robot_patch.remove()
        if hasattr(self, 'direction_arrow') and self.direction_arrow:
            self.direction_arrow.remove()
        if self.trajectory_line:
            for line in self.trajectory_line:
                line.remove()
        if self.path_line:
            for line in self.path_line:
                line.remove()
        
        # 绘制机器人
        robot_x, robot_y = self.robot.x, self.robot.y
        circle = plt.Circle((robot_x, robot_y), 0.3, color='blue', alpha=0.7)
        self.ax1.add_patch(circle)
        self.robot_patch = circle
        
        # 方向指示
        arrow_length = 0.5
        arrow_x = robot_x + arrow_length * math.cos(self.robot.theta)
        arrow_y = robot_y + arrow_length * math.sin(self.robot.theta)
        self.direction_arrow = self.ax1.arrow(robot_x, robot_y, 
                                arrow_x-robot_x, arrow_y-robot_y,
                                head_width=0.2, head_length=0.3, fc='red', ec='red')
        
        # 绘制轨迹 - 每5个点采样一次，减少点数提高性能
        if len(self.robot.trajectory) > 5:
            traj_x, traj_y = zip(*self.robot.trajectory[::5])
            self.trajectory_line = self.ax1.plot(traj_x, traj_y, 'b-', alpha=0.6, linewidth=1)
        
        # 绘制路径
        if hasattr(self.robot, 'current_path') and self.robot.current_path:
            path_x, path_y = zip(*self.robot.current_path)
            self.path_line = self.ax1.plot(path_x, path_y, 'g--', linewidth=2, alpha=0.7)
        
        # 探索目标
        if self.robot.exploration_target and self.robot.is_mapping:
            target_x, target_y = self.robot.exploration_target
            self.ax1.plot(target_x, target_y, 'y*', markersize=10)
        
        # SLAM地图
        slam_map = self.robot.get_map_data()
        if self.slam_image is None:
            self.slam_image = self.ax2.imshow(slam_map, cmap='gray', origin='lower', extent=[0, 15, 0, 15])
        else:
            self.slam_image.set_array(slam_map)
        
        # 状态信息
        if self.robot.is_mapping:
            status = f"正在建图探索中 (目标: {self.robot.exploration_target})"
        elif self.robot.exploration_complete:
            status = "任务完成！"
        else:
            status = "正在执行最终路径"
        
        visited_count = len(self.robot.visited_cells)
        self.fig.suptitle(f'优化版SLAM系统 - {status} | 已探索: {visited_count}格 | v={self.robot.v:.2f} ω={self.robot.omega:.2f}', 
                         fontsize=12)
        
        plt.pause(0.01)

def main():
    """主程序"""
    print("启动优化版SLAM和导航系统...")
    print("系统改进：")
    print("1. 更高效的SLAM算法")
    print("2. 优化的路径规划")
    print("3. 更智能的导航决策")
    print("4. 多线程加速处理")
    
    # 创建环境和机器人
    environment = MapEnvironment()
    robot = ImprovedRobot(environment)
    visualizer = FastVisualizer(robot, environment)
    
    try:
        # 启动模拟循环
        while not robot.exploration_complete:
            # 更新机器人状态
            robot.update()
            
            # 更新可视化
            visualizer.update()
            
            # 限制循环速度，减少CPU使用
            time.sleep(0.05)
            
        # 导航完成后等待用户关闭
        print("导航完成! 显示最终结果...")
        plt.ioff()
        plt.show()
        
    except KeyboardInterrupt:
        print("程序被用户中断")
    finally:
        # 确保正常关闭线程
        robot.shutdown()
        print("程序已安全退出")

if __name__ == "__main__":
    main() 