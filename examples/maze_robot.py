#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
迷宫机器人模块
包含机器人的实现及其探索算法
"""

import time
import math
import random
import numpy as np
from python_motion_planning.global_planner.graph_search.a_star import AStar
from maze_sensor import LaserSensor
import heapq

class Robot:
    """自主移动机器人"""
    def __init__(self, start_pos, env):
        # 初始化位置和方向
        self.x, self.y = start_pos[:2]
        self.theta = start_pos[2] if len(start_pos) > 2 else 0.0
        
        # 环境
        self.env = env
        
        # 传感器
        self.sensor = LaserSensor(max_range=5.0, num_rays=36)
        
        # 探索相关
        self.visited_cells = set()  # 已访问的单元格
        self.frontier = set()  # 边界单元格
        self.exploration_stack = []  # 探索栈
        self.failed_targets = set()  # 无法到达的目标点
        self.path = []  # 路径
        self.goal_path = []  # 到目标的路径
        self.last_positions = []  # 最近的位置，用于检测是否被卡住
        self.random_move_count = 0  # 随机移动的次数
        self.stuck_threshold = 5  # 卡住的阈值
        
        # 添加前进方向记录
        self.forward_direction = (1, 0)  # 默认向右为前进方向
        self.last_move_direction = None  # 上一次移动的方向
        self.direction_preference = [(1, 0), (0, 1), (0, -1), (-1, 0)]  # 方向优先级：右、上、下、左
        
        # 探索方向 - 只允许上下左右四个方向
        self.directions = [
            (1, 0),   # 右
            (0, 1),   # 上
            (-1, 0),  # 左
            (0, -1)   # 下
        ]
        
        # 标记起始位置为已访问
        self.visited_cells.add((int(round(self.x)), int(round(self.y))))
        
        # 初始化边界
        self.update_frontier()
        
        # 传感器数据
        self.sensor_data = []
        
        # 探索进度
        self.exploration_progress = 0.0
        
        # 添加更新计数器
        self.update_count = 0
        
        # 计算并缓存可访问单元格的总数
        self.accessible_cells_count = self._calculate_accessible_cells()
        
        # 调试信息
        print(f"机器人初始化在位置 ({self.x}, {self.y})")
        print(f"环境大小: {self.env.x_range}x{self.env.y_range}")
        print(f"可访问单元格总数: {self.accessible_cells_count}")
        
        # 添加迷宫格子探索相关参数
        self.maze_cells = {}  # 用于记录迷宫的每个格子的状态：0=未知，1=通路，2=墙壁
        self.cell_size = 1    # 格子大小
        self.initialize_maze_cells()
        
        # 初始化路径
        self.path.append((self.x, self.y))
        
        # 设置初始移动方向
        self.first_move_done = False
        
    def _calculate_accessible_cells(self):
        """计算地图中的可访问单元格总数（排除障碍物）"""
        accessible_cells = 0
        for x in range(self.env.x_range):
            for y in range(self.env.y_range):
                if (x, y) not in self.env.obstacles:
                    accessible_cells += 1
        return accessible_cells
    
    def initialize_maze_cells(self):
        """初始化迷宫格子状态"""
        # 将整个环境划分为格子
        for x in range(0, self.env.x_range, self.cell_size):
            for y in range(0, self.env.y_range, self.cell_size):
                self.maze_cells[(x, y)] = 0  # 0表示未知
        
        # 将起始位置标记为通路
        start_x, start_y = int(round(self.x)), int(round(self.y))
        self.maze_cells[(start_x, start_y)] = 1
        
    def update_maze_cells(self):
        """更新迷宫格子状态"""
        # 更新已探索区域为通路
        for x, y in self.visited_cells:
            cell_x = (x // self.cell_size) * self.cell_size
            cell_y = (y // self.cell_size) * self.cell_size
            self.maze_cells[(cell_x, cell_y)] = 1  # 1表示通路
        
        # 更新障碍物为墙壁
        for x, y in self.env.obstacles:
            cell_x = (x // self.cell_size) * self.cell_size
            cell_y = (y // self.cell_size) * self.cell_size
            self.maze_cells[(cell_x, cell_y)] = 2  # 2表示墙壁
    
    def update_frontier_cells(self):
        """更新未探索的边界单元格"""
        # 清空边界集合
        self.frontier = set()
        
        # 遍历所有已探索的单元格
        for cell in self.visited_cells:
            # 检查四个方向的邻居
            for dx, dy in self.directions:
                nx, ny = cell[0] + dx, cell[1] + dy
                neighbor = (nx, ny)
                
                # 如果邻居在地图范围内，不是障碍物，且未被探索，则是边界
                if (0 <= nx < self.env.x_range and 0 <= ny < self.env.y_range and
                    neighbor not in self.env.obstacles and
                    neighbor not in self.visited_cells):
                    self.frontier.add(neighbor)
    
    def update_position(self, new_pos):
        """更新机器人位置"""
        new_x, new_y = new_pos[:2]
        new_theta = new_pos[2] if len(new_pos) > 2 else self.theta
        
        # 检查新位置是否有效
        if not (0 <= new_x < self.env.x_range and 0 <= new_y < self.env.y_range):
            print(f"警告：位置 ({new_x}, {new_y}) 超出地图范围")
            return False
        
        # 检查新位置是否是障碍物
        if (int(round(new_x)), int(round(new_y))) in self.env.obstacles:
            print(f"警告：位置 ({new_x}, {new_y}) 是障碍物")
            return False
        
        # 确保移动是连续的（不跳格子）
        old_x, old_y = self.x, self.y
        dx, dy = new_x - old_x, new_y - old_y
        distance = math.sqrt(dx**2 + dy**2)
        
        # 如果距离大于1.5，说明可能会跳格子，需要进行插值移动
        if distance > 1.5:
            print(f"检测到跳格子移动，距离: {distance:.2f}，进行插值移动")
            # 计算需要插入的中间点数量
            steps = math.ceil(distance)
            # 进行插值移动
            for i in range(1, steps):
                # 计算中间点位置
                intermediate_x = old_x + (dx * i) / steps
                intermediate_y = old_y + (dy * i) / steps
                intermediate_theta = self.theta + ((new_theta - self.theta) * i) / steps
                
                # 检查中间点是否有效
                if not (0 <= intermediate_x < self.env.x_range and 0 <= intermediate_y < self.env.y_range):
                    continue
                    
                # 检查中间点是否是障碍物
                if (int(round(intermediate_x)), int(round(intermediate_y))) in self.env.obstacles:
                    continue
                    
                # 更新到中间点位置
                self.x, self.y = intermediate_x, intermediate_y
                self.theta = intermediate_theta
                
                # 添加到路径
                self.path.append((self.x, self.y))
                if len(self.path) > 1000:  # 限制路径长度
                    self.path = self.path[-1000:]
                    
                # 标记当前位置为已访问
                current_cell = (int(round(self.x)), int(round(self.y)))
                self.visited_cells.add(current_cell)
                
                # 更新迷宫单元格状态（对中间点也更新）
                self.update_maze_cells()
        
        # 更新到最终位置
        self.x, self.y = new_x, new_y
        self.theta = new_theta
            
        # 添加到路径
        self.path.append((self.x, self.y))
        if len(self.path) > 1000:  # 限制路径长度
            self.path = self.path[-1000:]
            
        # 标记当前位置为已访问
        current_cell = (int(round(self.x)), int(round(self.y)))
        self.visited_cells.add(current_cell)
        
        # 更新前进方向
        if old_x != new_x or old_y != new_y:
            move_dx = int(round(new_x - old_x))
            move_dy = int(round(new_y - old_y))
            if move_dx != 0 or move_dy != 0:  # 确保有实际移动
                self.last_move_direction = (move_dx, move_dy)
                # 如果是有意义的移动，更新前进方向
                if abs(move_dx) + abs(move_dy) == 1:  # 确保是单位向量
                    self.forward_direction = (move_dx, move_dy)
        
        # 更新边界
        self.update_frontier()
        
        # 更新最近的位置列表，用于检测是否被卡住
        self.last_positions.append(current_cell)
        if len(self.last_positions) > self.stuck_threshold:
            self.last_positions.pop(0)
            
        # 检测是否被卡住
        if len(self.last_positions) == self.stuck_threshold:
            if all(pos == self.last_positions[0] for pos in self.last_positions):
                print(f"警告: 机器人可能被卡住在 {current_cell}")
                self.random_move_count = 3  # 设置随机移动次数
        
        # 更新迷宫单元格状态
        self.update_maze_cells()
        
        # 打印移动信息
        print(f"机器人从 ({old_x:.1f}, {old_y:.1f}) 移动到 ({self.x:.1f}, {self.y:.1f})")
        
        # 更新探索进度 - 使用缓存的可访问单元格总数
        explored_cells = len(self.visited_cells)
        
        # 计算探索进度百分比，限制最大值为99.9%，确保不会过早达到100%
        if self.accessible_cells_count > 0:
            progress = (explored_cells / self.accessible_cells_count) * 100
            self.exploration_progress = min(progress, 99.9)
        else:
            self.exploration_progress = 0
        
        return True
    
    def scan_environment(self):
        """使用激光雷达扫描环境，更新探索地图"""
        # 递增更新计数器
        self.update_count += 1
        
        # 使用激光传感器扫描环境
        self.sensor_data = self.sensor.scan((self.x, self.y, self.theta), self.env)
        
        # 更新已探索区域
        current_x, current_y = int(round(self.x)), int(round(self.y))
        
        # 将当前位置标记为已探索
        self.visited_cells.add((current_x, current_y))
        
        # 更新周围的探索区域（模拟视野）
        view_range = 2
        for dx in range(-view_range, view_range + 1):
            for dy in range(-view_range, view_range + 1):
                nx, ny = current_x + dx, current_y + dy
                if 0 <= nx < self.env.x_range and 0 <= ny < self.env.y_range:
                    # 只添加非障碍物区域
                    if (nx, ny) not in self.env.obstacles:
                        self.visited_cells.add((nx, ny))
        
        # 处理激光点
        for point in self.sensor_data:
            # 计算激光点的栅格坐标
            grid_x, grid_y = int(round(point[0])), int(round(point[1]))
            
            # 将激光点标记为已探索
            if 0 <= grid_x < self.env.x_range and 0 <= grid_y < self.env.y_range:
                self.visited_cells.add((grid_x, grid_y))
                
                # 如果是障碍物，确保它在障碍物集合中
                if (grid_x, grid_y) in self.env.obstacles:
                    self.env.obstacles.add((grid_x, grid_y))
                
                # 激光点沿途的点也标记为已探索
                points = self.bresenham_line(current_x, current_y, grid_x, grid_y)
                for px, py in points:
                    if 0 <= px < self.env.x_range and 0 <= py < self.env.y_range:
                        self.visited_cells.add((px, py))
        
        # 更新未探索的边界单元格
        self.update_frontier_cells()
        
        # 计算探索进度 - 使用缓存的可访问单元格总数
        explored_cells = len(self.visited_cells)
        
        # 计算探索进度百分比，限制最大值为99.9%，确保不会过早达到100%
        if self.accessible_cells_count > 0:
            progress = (explored_cells / self.accessible_cells_count) * 100
            self.exploration_progress = min(progress, 99.9)
        else:
            self.exploration_progress = 0
        
        # 打印详细的探索统计信息
        if self.update_count % 20 == 0:  # 每20次更新打印一次统计信息
            print(f"探索统计: 已探索 {explored_cells} 个单元格，可访问单元格总数 {self.accessible_cells_count}")
            print(f"当前探索进度: {self.exploration_progress:.2f}%")
            print(f"边界单元格数量: {len(self.frontier)}")
            print(f"未探索区域比例: {(self.accessible_cells_count - explored_cells) / self.accessible_cells_count * 100:.2f}%")
        
        # 更新迷宫格子状态
        self.update_maze_cells()
    
    def explore_maze(self):
        """探索迷宫，确保完整遍历所有可达区域"""
        # 第一次移动时，强制向右移动一步
        if not self.first_move_done:
            self.first_move_done = True
            next_x, next_y = int(round(self.x)) + 1, int(round(self.y))
            if (0 < next_x < self.env.x_range - 1 and 
                0 < next_y < self.env.y_range - 1 and 
                (next_x, next_y) not in self.env.obstacles):
                
                print(f"初始移动到 ({next_x}, {next_y})")
                self.update_position((next_x, next_y, 0.0))
                # 设置初始前进方向为向右
                self.forward_direction = (1, 0)
                return True
            else:
                # 如果右边不能走，尝试向下
                next_x, next_y = int(round(self.x)), int(round(self.y)) + 1
                if (0 < next_x < self.env.x_range - 1 and 
                    0 < next_y < self.env.y_range - 1 and 
                    (next_x, next_y) not in self.env.obstacles):
                    
                    print(f"初始移动到 ({next_x}, {next_y})")
                    self.update_position((next_x, next_y, math.pi/2))
                    # 设置初始前进方向为向下
                    self.forward_direction = (0, 1)
                    return True
                else:
                    # 如果向下也不能走，尝试其他两个方向
                    for dx, dy in [(0, -1), (-1, 0)]:  # 上、左
                        next_x, next_y = int(round(self.x)) + dx, int(round(self.y)) + dy
                        if (0 < next_x < self.env.x_range - 1 and 
                            0 < next_y < self.env.y_range - 1 and 
                            (next_x, next_y) not in self.env.obstacles):
                            
                            target_theta = math.atan2(dy, dx)
                            print(f"初始移动到 ({next_x}, {next_y})")
                            self.update_position((next_x, next_y, target_theta))
                            # 设置初始前进方向
                            self.forward_direction = (dx, dy)
                            return True
        
        # 检查是否需要更新边界单元格
        if len(self.frontier) == 0:
            self.update_frontier()
            
        # 如果已经探索了大部分区域，检查是否可以直接前往终点
        if self.exploration_progress > 80.0 and hasattr(self.env, 'goal_pos') and self.env.goal_pos is not None:
            goal_cell = (int(round(self.env.goal_pos[0])), int(round(self.env.goal_pos[1])))
            current_cell = (int(round(self.x)), int(round(self.y)))
            
            # 如果终点已知且未被标记为失败目标，尝试规划到终点的路径
            if goal_cell not in self.failed_targets:
                print(f"探索进度已达到 {self.exploration_progress:.2f}%，尝试直接前往终点 {goal_cell}")
                path = self.plan_path(current_cell, goal_cell)
                
                if path:
                    print(f"找到到终点的路径，长度: {len(path)}")
                    # 确保路径连续
                    processed_path = self.process_path_to_cardinal_directions(path)
                    self.goal_path = processed_path[1:]  # 跳过起点
                    return True
        
        # 优先尝试沿着当前前进方向继续探索
        current_cell = (int(round(self.x)), int(round(self.y)))
        forward_cell = (current_cell[0] + self.forward_direction[0], 
                        current_cell[1] + self.forward_direction[1])
        
        # 检查前方单元格是否可行
        if (0 < forward_cell[0] < self.env.x_range - 1 and 
            0 < forward_cell[1] < self.env.y_range - 1 and 
            forward_cell not in self.env.obstacles and
            forward_cell not in self.visited_cells):
            
            print(f"沿着当前方向 {self.forward_direction} 继续探索到 {forward_cell}")
            self.update_position((forward_cell[0], forward_cell[1], 
                                 math.atan2(self.forward_direction[1], self.forward_direction[0])))
            return True
            
        # 尝试查看前方稍微偏离的方向
        for angle_offset in [0, math.pi/4, -math.pi/4, math.pi/2, -math.pi/2]:
            if angle_offset == 0:
                continue  # 已经检查过正前方了
                
            # 计算偏移后的方向
            current_angle = math.atan2(self.forward_direction[1], self.forward_direction[0])
            new_angle = current_angle + angle_offset
            new_dx = round(math.cos(new_angle))
            new_dy = round(math.sin(new_angle))
            
            # 确保是单位向量
            if abs(new_dx) + abs(new_dy) == 2:  # 对角线方向
                if random.random() < 0.5:
                    new_dx = 0
                else:
                    new_dy = 0
                    
            if (new_dx, new_dy) in self.directions:  # 确保是上下左右四个方向之一
                new_cell = (current_cell[0] + new_dx, current_cell[1] + new_dy)
                
                if (0 < new_cell[0] < self.env.x_range - 1 and 
                    0 < new_cell[1] < self.env.y_range - 1 and 
                    new_cell not in self.env.obstacles and
                    new_cell not in self.visited_cells):
                    
                    print(f"沿着偏移方向 ({new_dx}, {new_dy}) 探索到 {new_cell}")
                    self.update_position((new_cell[0], new_cell[1], new_angle))
                    # 更新前进方向
                    self.forward_direction = (new_dx, new_dy)
                    return True
        
        # 移动到下一个目标
        return self.move_to_next_target()
    
    def find_path_to_goal(self, goal_pos):
        """找到从当前位置到目标的路径"""
        current_pos = (int(round(self.x)), int(round(self.y)))
        goal_cell = (int(round(goal_pos[0])), int(round(goal_pos[1])))
        
        print(f"寻找从 {current_pos} 到 {goal_cell} 的路径")
        
        # 使用A*算法规划路径
        path = self.plan_path(current_pos, goal_cell)
        
        if path and len(path) > 1:
            print(f"找到路径，长度: {len(path)}")
            # 确保路径连续
            processed_path = self.process_path_to_cardinal_directions(path)
            # 设置目标路径
            self.goal_path = processed_path
            return True
        elif len(path) == 1 and path[0] == current_pos:
            print(f"当前位置就是目标位置，无需移动")
            # 创建一个只包含当前位置和目标位置的路径
            self.goal_path = [current_pos, goal_cell]
            return True
        else:
            print(f"无法找到到 {goal_cell} 的路径")
            return False
    
    def find_intermediate_path(self, start, goal):
        """寻找两点之间的中间路径，避开障碍物"""
        # 使用更细致的网格搜索
        open_set = [start]
        closed_set = set()
        came_from = {}
        
        # 使用曼哈顿距离作为启发式
        def manhattan_distance(p1, p2):
            return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])
        
        # 初始化g和f值
        g_score = {start: 0}
        f_score = {start: manhattan_distance(start, goal)}
        
        while open_set:
            # 找到f值最小的节点
            current = min(open_set, key=lambda x: f_score.get(x, float('inf')))
            
            # 如果到达目标，重建路径
            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                # 反转路径并确保连续性
                raw_path = path[::-1]
                return self.process_path_to_cardinal_directions(raw_path)
            
            open_set.remove(current)
            closed_set.add(current)
            
            # 检查四个方向的邻居
            for dx, dy in self.directions:
                neighbor = (current[0] + dx, current[1] + dy)
                
                # 检查是否是有效位置
                if (0 <= neighbor[0] < self.env.x_range and 
                    0 <= neighbor[1] < self.env.y_range and 
                    neighbor not in self.env.obstacles and
                    neighbor not in closed_set and
                    self.is_path_clear(current[0], current[1], neighbor[0], neighbor[1])):
                    
                    # 计算新的g值
                    tentative_g_score = g_score[current] + 1
                    
                    if neighbor not in open_set:
                        open_set.append(neighbor)
                    elif tentative_g_score >= g_score.get(neighbor, float('inf')):
                        continue
                    
                    # 这是最佳路径，记录下来
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + manhattan_distance(neighbor, goal)
        
        return None  # 没有找到路径
    
    def find_fine_path(self, start, goal):
        """寻找一条更细致的路径，避免穿过障碍物"""
        # 使用细分网格的A*搜索
        open_set = [start]
        closed_set = set()
        came_from = {}
        
        # 使用欧几里得距离作为启发式
        def euclidean_distance(p1, p2):
            return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
        
        # 初始化g和f值
        g_score = {start: 0}
        f_score = {start: euclidean_distance(start, goal)}
        
        # 只使用上下左右四个方向
        fine_directions = [
            (0, 1),   # 上
            (1, 0),   # 右
            (0, -1),  # 下
            (-1, 0)   # 左
        ]
        
        while open_set:
            # 找到f值最小的节点
            current = min(open_set, key=lambda x: f_score.get(x, float('inf')))
            
            # 如果到达目标，重建路径
            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                # 反转路径并确保连续性
                raw_path = path[::-1]
                return self.process_path_to_cardinal_directions(raw_path)
            
            open_set.remove(current)
            closed_set.add(current)
            
            # 检查四个方向的邻居
            for dx, dy in fine_directions:
                neighbor = (current[0] + dx, current[1] + dy)
                
                # 检查是否是有效位置
                if (0 <= neighbor[0] < self.env.x_range and 
                    0 <= neighbor[1] < self.env.y_range and 
                    neighbor not in self.env.obstacles and
                    neighbor not in closed_set and
                    self.is_path_clear(current[0], current[1], neighbor[0], neighbor[1])):
                    
                    # 计算移动代价
                    move_cost = 1.0
                    
                    # 计算新的g值
                    tentative_g_score = g_score[current] + move_cost
                    
                    if neighbor not in open_set:
                        open_set.append(neighbor)
                    elif tentative_g_score >= g_score.get(neighbor, float('inf')):
                        continue
                    
                    # 这是最佳路径，记录下来
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + euclidean_distance(neighbor, goal)
        
        # 如果无法直接找到路径，尝试分段寻路
        if start != goal:
            # 计算中间点
            mid_x = (start[0] + goal[0]) // 2
            mid_y = (start[1] + goal[1]) // 2
            mid_point = (mid_x, mid_y)
            
            # 如果中间点是障碍物，尝试周围的点
            if mid_point in self.env.obstacles:
                for dx, dy in fine_directions:
                    alt_mid = (mid_x + dx, mid_y + dy)
                    if (0 <= alt_mid[0] < self.env.x_range and 
                        0 <= alt_mid[1] < self.env.y_range and 
                        alt_mid not in self.env.obstacles):
                        mid_point = alt_mid
                        break
            
            # 如果中间点不是障碍物，尝试分段寻路
            if mid_point not in self.env.obstacles:
                path1 = self.find_intermediate_path(start, mid_point)
                path2 = self.find_intermediate_path(mid_point, goal)
                if path1 and path2:
                    # 合并路径（去掉重复的中间点）
                    combined_path = path1[:-1] + path2
                    # 确保合并后的路径连续性
                    return self.process_path_to_cardinal_directions(combined_path)
        
        return None  # 没有找到路径
    
    def process_path_to_cardinal_directions(self, path):
        """处理路径，确保只有上下左右四个方向的移动"""
        if not path or len(path) <= 1:
            return path
            
        processed_path = [path[0]]  # 起点
        
        for i in range(1, len(path)):
            prev = processed_path[-1]
            current = path[i]
            
            # 计算方向
            dx = current[0] - prev[0]
            dy = current[1] - prev[1]
            
            # 如果是对角线移动，拆分为两个基本移动
            if dx != 0 and dy != 0:
                # 先水平移动
                mid_point = (prev[0] + dx, prev[1])
                # 检查中间点是否是障碍物
                if mid_point not in self.env.obstacles:
                    processed_path.append(mid_point)
                    processed_path.append(current)
                else:
                    # 尝试先垂直移动
                    mid_point = (prev[0], prev[1] + dy)
                    if mid_point not in self.env.obstacles:
                        processed_path.append(mid_point)
                        processed_path.append(current)
                    else:
                        # 两个方向都不行，尝试找到另一条路径
                        # 这里简单地跳过这个点
                        continue
            else:
                # 如果是基本移动（上下左右），直接添加
                processed_path.append(current)
        
        # 确保路径中相邻点之间的距离不超过1（防止跳格子）
        i = 0
        while i < len(processed_path) - 1:
            p1 = processed_path[i]
            p2 = processed_path[i + 1]
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            distance = abs(dx) + abs(dy)
            
            # 如果相邻点之间的曼哈顿距离大于1，插入中间点
            if distance > 1:
                # 确定需要插入的中间点数量
                steps = distance
                new_points = []
                
                for step in range(1, steps):
                    # 计算中间点位置（按比例插值）
                    ratio = step / steps
                    mid_x = int(round(p1[0] + dx * ratio))
                    mid_y = int(round(p1[1] + dy * ratio))
                    mid_point = (mid_x, mid_y)
                    
                    # 确保中间点不是障碍物
                    if mid_point not in self.env.obstacles:
                        new_points.append(mid_point)
                
                # 插入中间点
                if new_points:
                    processed_path[i+1:i+1] = new_points
                    i += len(new_points)
            
            i += 1
        
        return processed_path
    
    def navigate_to_goal(self):
        """沿着已规划的路径导航到目标"""
        # 如果没有路径，无法导航
        if not self.goal_path or len(self.goal_path) < 2:
            print("没有有效的导航路径")
            return False
        
        # 获取当前位置
        current_x, current_y = int(round(self.x)), int(round(self.y))
        current_pos = (current_x, current_y)
        
        # 检查当前位置是否在路径上
        if current_pos not in self.goal_path:
            print(f"当前位置 {current_pos} 不在规划的路径上，寻找最近的路径点")
            
            # 寻找最近的路径点
            min_dist = float('inf')
            closest_idx = 0
            
            for i, pos in enumerate(self.goal_path):
                dist = abs(pos[0] - current_x) + abs(pos[1] - current_y)  # 曼哈顿距离
                if dist < min_dist:
                    min_dist = dist
                    closest_idx = i
            
            # 如果找到了最近的点，从那里继续导航
            if min_dist < 5:  # 设置一个合理的阈值
                print(f"找到最近的路径点: {self.goal_path[closest_idx]}，距离: {min_dist}")
                
                # 如果距离大于1，需要先规划到该点的路径
                if min_dist > 1:
                    print(f"距离大于1，需要先规划到路径点的路径")
                    closest_point = self.goal_path[closest_idx]
                    path_to_closest = self.plan_path(current_pos, closest_point)
                    
                    if path_to_closest and len(path_to_closest) > 1:
                        print(f"找到到最近路径点的路径，长度: {len(path_to_closest)}")
                        # 使用新规划的路径（跳过起点）加上原始路径的剩余部分
                        self.goal_path = path_to_closest[1:] + self.goal_path[closest_idx:]
                        return True
                    else:
                        print(f"无法规划到最近路径点的路径")
                
                # 从最近的点开始导航
                self.goal_path = self.goal_path[closest_idx:]
                if len(self.goal_path) <= 1:
                    print("已经到达目标点附近")
                    return False
                return True
            else:
                # 如果最近的点也很远，尝试直接移动到目标方向
                print("最近的路径点也很远，尝试直接移动到目标方向")
                goal = self.goal_path[-1]  # 目标位置
                
                # 规划一条新的路径到目标
                print(f"重新规划到目标 {goal} 的路径")
                new_path = self.plan_path(current_pos, goal)
                
                if new_path and len(new_path) > 1:
                    print(f"找到新的到目标的路径，长度: {len(new_path)}")
                    self.goal_path = new_path[1:]  # 跳过起点
                    return True
                
                # 如果无法规划新路径，尝试单步移动到目标方向
                dx = goal[0] - current_x
                dy = goal[1] - current_y
                
                # 确定主要移动方向（只取一个方向，避免对角线移动）
                if abs(dx) > abs(dy):
                    # 水平方向移动
                    next_x = current_x + (1 if dx > 0 else -1)
                    next_y = current_y
                else:
                    # 垂直方向移动
                    next_x = current_x
                    next_y = current_y + (1 if dy > 0 else -1)
                
                next_pos = (next_x, next_y)
                
                # 检查下一个位置是否可行
                if (0 <= next_x < self.env.x_range and 
                    0 <= next_y < self.env.y_range and 
                    next_pos not in self.env.obstacles):
                    
                    # 计算朝向
                    target_theta = math.atan2(next_y - current_y, next_x - current_x)
                    
                    # 移动到下一个位置
                    if self.update_position((next_x, next_y, target_theta)):
                        print(f"直接移动到 {next_pos}")
                        # 重新规划路径，确保导航可以继续
                        goal_pos = self.goal_path[-1]
                        if self.find_path_to_goal(goal_pos):
                            print("重新规划路径成功")
                        return True
                
                # 如果直接移动失败，重新规划路径
                return False
        
        # 尝试沿着路径移动到下一个点
        try:
            # 获取路径中的当前位置索引
            current_index = self.goal_path.index(current_pos)
            
            # 检查是否已经到达目标
            if current_index == len(self.goal_path) - 1:
                print("已经到达目标点")
                return False
            
            # 获取下一个位置
            next_pos = self.goal_path[current_index + 1]
        except ValueError:
            # 如果当前位置不在路径中，找最近的点并从那里继续
            print(f"无法在路径中找到当前位置 {current_pos}，寻找最近的点")
            min_dist = float('inf')
            closest_idx = 0
            
            for i, pos in enumerate(self.goal_path):
                dist = abs(pos[0] - current_x) + abs(pos[1] - current_y)
                if dist < min_dist:
                    min_dist = dist
                    closest_idx = i
            
            # 如果找到了最近的点，从那里继续导航
            if min_dist < 3 and closest_idx < len(self.goal_path) - 1:
                # 如果距离大于1，需要先规划到该点的路径
                if min_dist > 1:
                    print(f"距离大于1，需要先规划到路径点的路径")
                    closest_point = self.goal_path[closest_idx]
                    path_to_closest = self.plan_path(current_pos, closest_point)
                    
                    if path_to_closest and len(path_to_closest) > 1:
                        print(f"找到到最近路径点的路径，长度: {len(path_to_closest)}")
                        # 使用新规划的路径（跳过起点）加上原始路径的剩余部分
                        self.goal_path = path_to_closest[1:] + self.goal_path[closest_idx:]
                        return True
                    else:
                        print(f"无法规划到最近路径点的路径")
                
                next_pos = self.goal_path[closest_idx + 1]
                print(f"从最近的路径点 {self.goal_path[closest_idx]} 继续导航到 {next_pos}")
            else:
                # 如果最近的点也很远或已经是终点，直接移动向目标
                if closest_idx < len(self.goal_path) - 1:
                    next_pos = self.goal_path[closest_idx + 1]
                else:
                    print("已经到达路径末尾")
                    return False
        
        # 检查下一个位置是否是相邻的单元格
        dx = next_pos[0] - current_x
        dy = next_pos[1] - current_y
        
        # 确保只沿着上下左右四个方向移动，不要对角线移动
        if abs(dx) + abs(dy) > 1:
            # 如果是对角线移动，分解为两步
            print(f"检测到对角线移动 ({dx}, {dy})，分解为单步移动")
            # 优先水平移动
            if dx != 0:
                next_pos = (current_x + (1 if dx > 0 else -1), current_y)
            else:
                next_pos = (current_x, current_y + (1 if dy > 0 else -1))
            dx = next_pos[0] - current_x
            dy = next_pos[1] - current_y
        
        # 检查下一步是否有效
        if (dx, dy) not in self.directions:
            print(f"下一个位置 {next_pos} 不是有效的移动方向，尝试重新规划")
            return False
        
        # 计算朝向
        target_theta = math.atan2(dy, dx)
        
        # 移动到下一个位置
        if self.update_position((next_pos[0], next_pos[1], target_theta)):
            print(f"导航：成功移动到 {next_pos}")
            return True
        else:
            print(f"导航：移动到 {next_pos} 失败，可能是障碍物")
            # 尝试从路径中删除这个点
            try:
                idx = self.goal_path.index(next_pos)
                self.goal_path.pop(idx)
                print(f"从路径中删除障碍点 {next_pos}")
                # 如果路径太短，重新规划
                if len(self.goal_path) < 2:
                    print("路径太短，需要重新规划")
                    return False
            except ValueError:
                pass
            return False
    
    def is_path_clear(self, x1, y1, x2, y2):
        """检查从(x1,y1)到(x2,y2)的路径是否通畅"""
        # 如果是相邻的格子，直接检查目标格子是否是障碍物
        if abs(x2 - x1) <= 1 and abs(y2 - y1) <= 1:
            return (x2, y2) not in self.env.obstacles
            
        # 使用Bresenham算法检查路径上的每个格子
        line = self.get_line(x1, y1, x2, y2)
        
        # 跳过起点和终点
        for x, y in line[1:-1]:
            if (x, y) in self.env.obstacles:
                return False
                
        return True
        
    def get_line(self, x1, y1, x2, y2):
        """使用Bresenham算法获取从(x1,y1)到(x2,y2)的线上的所有点"""
        points = []
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1
        err = dx - dy
        
        while True:
            points.append((x1, y1))
            if x1 == x2 and y1 == y2:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x1 += sx
            if e2 < dx:
                err += dx
                y1 += sy
                
        return points
    
    def bresenham_line(self, x0, y0, x1, y1):
        """Bresenham直线算法，返回从(x0,y0)到(x1,y1)的所有点"""
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
    
    def update_sensor_data(self):
        """更新传感器数据"""
        # 使用激光传感器扫描环境
        self.sensor_data = self.sensor.scan((self.x, self.y, self.theta), self.env)
        
        # 更新迷宫单元格状态
        self.update_maze_cells()
    
    def update_frontier(self):
        """更新边界单元格"""
        self.frontier = set()
        for cell in self.visited_cells:
            for dx, dy in self.directions:
                nx, ny = cell[0] + dx, cell[1] + dy
                neighbor = (nx, ny)
                if (0 <= nx < self.env.x_range and 0 <= ny < self.env.y_range and
                    neighbor not in self.env.obstacles and
                    neighbor not in self.visited_cells):
                    self.frontier.add(neighbor) 

    def move_to_next_target(self):
        """移动到下一个目标位置"""
        # 如果需要随机移动（被卡住）
        if self.random_move_count > 0:
            self.random_move_count -= 1
            print(f"执行随机移动 (剩余 {self.random_move_count} 次)")
            
            # 获取所有可能的移动方向
            valid_moves = []
            current_cell = (int(round(self.x)), int(round(self.y)))
            
            # 根据前进方向对方向进行排序
            sorted_directions = self.get_sorted_directions()
            
            for dx, dy in sorted_directions:
                next_x, next_y = current_cell[0] + dx, current_cell[1] + dy
                next_cell = (next_x, next_y)
                
                # 检查是否是有效移动
                if (0 < next_x < self.env.x_range - 1 and 
                    0 < next_y < self.env.y_range - 1 and 
                    next_cell not in self.env.obstacles):
                    
                    # 优先选择未访问的单元格
                    if next_cell not in self.visited_cells:
                        print(f"随机移动到未访问的单元格 {next_cell}")
                        self.update_position((next_x, next_y, math.atan2(dy, dx)))
                        # 更新前进方向
                        self.forward_direction = (dx, dy)
                        return True
                    
                    valid_moves.append((next_x, next_y, math.atan2(dy, dx)))
            
            # 如果没有未访问的单元格，选择一个有效移动
            if valid_moves:
                next_pos = valid_moves[0]  # 选择第一个（优先级最高的）
                print(f"随机移动到 {(next_pos[0], next_pos[1])}")
                self.update_position(next_pos)
                # 更新前进方向
                dx = next_pos[0] - current_cell[0]
                dy = next_pos[1] - current_cell[1]
                if abs(dx) + abs(dy) == 1:  # 确保是单位向量
                    self.forward_direction = (dx, dy)
                return True
            else:
                print("没有有效的随机移动")
                return False
                
        # 如果有目标路径，沿着路径移动
        if self.goal_path:
            # 获取路径上的下一个点
            next_point = self.goal_path[0]
            
            # 检查下一个点是否可达
            if (next_point[0], next_point[1]) in self.env.obstacles:
                print(f"警告: 路径点 {next_point} 是障碍物，重新规划路径")
                self.goal_path = []  # 清空路径，下次会重新规划
                return False
            
            # 检查当前位置到下一个点的距离
            current_cell = (int(round(self.x)), int(round(self.y)))
            distance_to_next = abs(next_point[0] - current_cell[0]) + abs(next_point[1] - current_cell[1])
            
            # 如果距离大于1，说明不是相邻点，需要先规划到该点的路径
            if distance_to_next > 1:
                print(f"当前位置 {current_cell} 与路径点 {next_point} 距离为 {distance_to_next}，需要先规划到该点的路径")
                # 保存原始路径
                original_path = self.goal_path.copy()
                # 规划到第一个点的路径
                path_to_first = self.plan_path(current_cell, next_point)
                
                if path_to_first and len(path_to_first) > 1:
                    print(f"找到到路径点的路径，长度: {len(path_to_first)}")
                    # 使用新规划的路径（跳过起点）加上原始路径的剩余部分
                    self.goal_path = path_to_first[1:] + original_path[1:]
                    # 重新尝试移动
                    return self.move_to_next_target()
                else:
                    print(f"无法规划到路径点 {next_point} 的路径，跳过该点")
                    self.goal_path.pop(0)  # 移除无法到达的点
                    if not self.goal_path:
                        return False
                    return self.move_to_next_target()
            
            # 移除已处理的点
            self.goal_path.pop(0)
            
            print(f"移动到路径的下一个点 {next_point}")
            
            # 更新位置
            dx = next_point[0] - current_cell[0]
            dy = next_point[1] - current_cell[1]
            target_theta = math.atan2(dy, dx)
            
            if self.update_position((next_point[0], next_point[1], target_theta)):
                # 更新前进方向
                if abs(dx) + abs(dy) == 1:  # 确保是单位向量
                    self.forward_direction = (dx, dy)
                return True
            return False
            
        # 如果没有目标路径，但有探索栈，规划到栈顶目标的路径
        elif self.exploration_stack:
            target = self.exploration_stack[-1]
            current_cell = (int(round(self.x)), int(round(self.y)))
            
            print(f"当前位置: {current_cell}, 目标位置: {target}")
            
            # 如果已经到达目标，弹出栈顶
            if current_cell == target:
                self.exploration_stack.pop()
                # 立即尝试沿着当前方向继续探索，而不是递归调用
                # 检查前方单元格是否可行
                forward_cell = (current_cell[0] + self.forward_direction[0], 
                               current_cell[1] + self.forward_direction[1])
                
                if (0 < forward_cell[0] < self.env.x_range - 1 and 
                    0 < forward_cell[1] < self.env.y_range - 1 and 
                    forward_cell not in self.env.obstacles and
                    forward_cell not in self.visited_cells):
                    
                    print(f"到达目标后，继续沿着当前方向 {self.forward_direction} 探索到 {forward_cell}")
                    self.update_position((forward_cell[0], forward_cell[1], 
                                         math.atan2(self.forward_direction[1], self.forward_direction[0])))
                    return True
                
                # 如果前方不可行，再尝试移动到下一个目标
                if self.exploration_stack:
                    return self.move_to_next_target()  # 递归调用，移动到下一个目标
                else:
                    # 如果栈为空，更新边界并添加新目标
                    self.update_frontier()
                    added = self.add_frontier_to_stack()
                    if added > 0:
                        return self.move_to_next_target()
                
            # 规划到目标的路径
            path = self.plan_path(current_cell, target)
            
            if path:
                print(f"找到路径，长度: {len(path)}")
                # 确保路径连续
                processed_path = self.process_path_to_cardinal_directions(path)
                self.goal_path = processed_path[1:]  # 跳过起点
                return self.move_to_next_target()  # 递归调用，沿着路径移动
            else:
                print(f"无法找到到 {target} 的路径，标记为失败目标")
                self.failed_targets.add(target)
                self.exploration_stack.pop()  # 弹出无法到达的目标
                
                # 如果探索栈为空，尝试添加新的边界单元格
                if not self.exploration_stack:
                    self.add_frontier_to_stack()
                    
                # 如果探索栈仍然为空，但还有未探索区域，尝试随机移动
                if not self.exploration_stack and self.frontier:
                    print("探索栈为空，但还有未探索区域，尝试随机移动")
                    self.random_move_count = 3
                    return self.move_to_next_target()
                    
                return False
        
        # 如果没有目标路径也没有探索栈，但有边界单元格，添加到探索栈
        elif self.frontier:
            added = self.add_frontier_to_stack()
            if added > 0:
                print(f"添加了 {added} 个边界单元格到探索栈")
                return self.move_to_next_target()  # 递归调用，规划到新目标的路径
            else:
                # 如果没有可添加的边界单元格，尝试随机移动
                print("没有可添加的边界单元格，尝试随机移动")
                self.random_move_count = 3
                return self.move_to_next_target()
        
        # 如果既没有目标路径，也没有探索栈，也没有边界单元格，说明探索完成
        else:
            print("探索完成，没有更多区域可探索")
            return False

    def get_sorted_directions(self):
        """根据前进方向对方向进行排序"""
        # 计算每个方向与前进方向的相似度
        direction_scores = []
        
        for dx, dy in self.directions:
            # 计算方向相似度 (点积)
            similarity = dx * self.forward_direction[0] + dy * self.forward_direction[1]
            
            # 额外的惩罚项：如果是与上次移动方向相反的方向，给予更大的惩罚
            if self.last_move_direction and (dx == -self.last_move_direction[0] and dy == -self.last_move_direction[1]):
                similarity -= 2.0
                
            # 如果是前进方向，给予额外奖励
            if (dx, dy) == self.forward_direction:
                similarity += 0.5
                
            direction_scores.append((similarity, (dx, dy)))
            
        # 按相似度从高到低排序
        direction_scores.sort(reverse=True)
        
        # 返回排序后的方向列表
        return [direction for _, direction in direction_scores]

    def add_frontier_to_stack(self):
        """将边界单元格添加到探索栈中"""
        if not self.frontier:
            return 0
            
        # 获取当前位置
        current_cell = (int(round(self.x)), int(round(self.y)))
        
        # 计算到每个边界单元格的方向得分
        frontier_scores = []
        for cell in self.frontier:
            # 如果单元格在失败目标集合中，跳过
            if cell in self.failed_targets:
                continue
                
            # 计算方向向量
            dx = cell[0] - current_cell[0]
            dy = cell[1] - current_cell[1]
            
            # 计算距离
            distance = abs(dx) + abs(dy)
            
            # 计算方向相似度 (归一化后的点积)
            if distance > 0:
                norm_dx, norm_dy = dx / distance, dy / distance
                direction_similarity = norm_dx * self.forward_direction[0] + norm_dy * self.forward_direction[1]
            else:
                direction_similarity = 0
                
            # 大幅提高方向相似度的权重，确保优先沿着当前方向探索
            score = direction_similarity * 20 - distance
            
            # 如果是已访问过的单元格的邻居，降低其优先级
            visited_neighbors = 0
            for ndx, ndy in self.directions:
                neighbor = (cell[0] + ndx, cell[1] + ndy)
                if neighbor in self.visited_cells:
                    visited_neighbors += 1
            
            # 如果周围已访问单元格太多，大幅降低优先级
            score -= visited_neighbors * 1.0
            
            frontier_scores.append((score, cell))
            
        # 按得分从高到低排序
        frontier_scores.sort(reverse=True)
        
        # 选择得分最高的边界单元格添加到探索栈
        # 修改为只添加一个得分最高的单元格，确保机器人专注于一条路径
        added_count = 0
        if frontier_scores:
            best_cell = frontier_scores[0][1]
            if best_cell not in self.exploration_stack and best_cell not in self.failed_targets:
                self.exploration_stack.append(best_cell)
                added_count += 1
                print(f"添加边界单元格 {best_cell} 到探索栈，优先沿着当前方向探索")
                
        return added_count

    def plan_path(self, start, goal):
        """使用A*算法规划从起点到目标的路径"""
        print(f"规划从 {start} 到 {goal} 的路径")
        
        # 如果起点和目标相同，直接返回
        if start == goal:
            return [start]
            
        # 如果目标是障碍物，返回空路径
        if goal in self.env.obstacles:
            print(f"目标 {goal} 是障碍物，无法规划路径")
            return []
            
        # 如果目标在失败目标集合中，返回空路径
        if goal in self.failed_targets:
            print(f"目标 {goal} 在失败目标集合中，跳过")
            return []
            
        # 初始化开放列表和关闭列表
        open_list = []
        closed_set = set()
        
        # 起点的g值和f值
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        
        # 父节点字典，用于重建路径
        came_from = {}
        
        # 将起点加入开放列表
        heapq.heappush(open_list, (f_score[start], start))
        
        # 设置最大迭代次数，避免无限循环
        max_iterations = 10000
        iteration_count = 0
        
        # A*搜索
        while open_list and iteration_count < max_iterations:
            iteration_count += 1
            
            # 获取f值最小的节点
            _, current = heapq.heappop(open_list)
            
            # 如果到达目标，重建路径
            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                print(f"找到路径，长度: {len(path)}")
                # 处理路径，确保连续性
                processed_path = self.process_path_to_cardinal_directions(path)
                return processed_path
                
            # 将当前节点加入关闭列表
            closed_set.add(current)
            
            # 遍历相邻节点 - 只考虑上下左右四个方向
            for dx, dy in self.directions:
                neighbor = (current[0] + dx, current[1] + dy)
                
                # 如果邻居在关闭列表中，跳过
                if neighbor in closed_set:
                    continue
                    
                # 如果邻居是障碍物，跳过
                if neighbor in self.env.obstacles:
                    continue
                    
                # 如果邻居超出地图范围，跳过
                if not (0 <= neighbor[0] < self.env.x_range and 0 <= neighbor[1] < self.env.y_range):
                    continue
                
                # 计算从起点到邻居的代价
                tentative_g_score = g_score[current] + 1
                
                # 如果是目标点附近，降低代价以提高找到路径的可能性
                if self.heuristic(neighbor, goal) < 3:
                    tentative_g_score -= 0.5
                
                # 如果找到了更好的路径或是新节点
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    # 更新路径和代价
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    
                    # 将邻居加入开放列表（如果不在的话）
                    if not any(neighbor == item[1] for item in open_list):
                        heapq.heappush(open_list, (f_score[neighbor], neighbor))
        
        if iteration_count >= max_iterations:
            print(f"警告: A*搜索达到最大迭代次数 {max_iterations}")
        
        # 如果无法找到完整路径，尝试使用松弛版本的路径规划
        print(f"无法直接找到从 {start} 到 {goal} 的路径，尝试松弛版本...")
        
        # 重置数据结构
        open_list = []
        closed_set = set()
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        heapq.heappush(open_list, (f_score[start], start))
        
        # 允许经过更多点
        visited_set = set(self.visited_cells)  # 使用已探索区域的副本
        
        # 松弛版本的A*搜索
        iteration_count = 0
        while open_list and iteration_count < max_iterations:
            iteration_count += 1
            
            # 获取f值最小的节点
            _, current = heapq.heappop(open_list)
            
            # 如果接近目标，提前结束
            if self.heuristic(current, goal) < 2:
                path = [current, goal]  # 直接添加目标
                while current in came_from:
                    current = came_from[current]
                    path.insert(0, current)
                print(f"找到接近目标的路径，长度: {len(path)}")
                # 处理路径，确保连续性
                processed_path = self.process_path_to_cardinal_directions(path)
                return processed_path
                
            # 将当前节点加入关闭列表
            closed_set.add(current)
            
            # 遍历所有方向（包括对角线）
            all_directions = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]
            for dx, dy in all_directions:
                neighbor = (current[0] + dx, current[1] + dy)
                
                # 如果邻居在关闭列表中，跳过
                if neighbor in closed_set:
                    continue
                    
                # 如果邻居是障碍物，跳过
                if neighbor in self.env.obstacles:
                    continue
                    
                # 如果邻居超出地图范围，跳过
                if not (0 <= neighbor[0] < self.env.x_range and 0 <= neighbor[1] < self.env.y_range):
                    continue
                
                # 如果是对角线移动，检查两个相邻的格子是否是障碍物
                if abs(dx) + abs(dy) == 2:
                    if (current[0] + dx, current[1]) in self.env.obstacles or (current[0], current[1] + dy) in self.env.obstacles:
                        continue
                
                # 计算代价
                move_cost = 1.0 if (abs(dx) + abs(dy) == 1) else 1.414  # 对角线移动代价更高
                
                # 计算从起点到邻居的代价
                tentative_g_score = g_score[current] + move_cost
                
                # 如果找到了更好的路径，更新代价
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal) * 0.8  # 降低启发式权重
                    
                    # 将邻居加入开放列表
                    if not any(neighbor == item[1] for item in open_list):
                        heapq.heappush(open_list, (f_score[neighbor], neighbor))
        
        # 如果仍然找不到路径，返回空
        print(f"无法找到从 {start} 到 {goal} 的路径")
        return []

    def heuristic(self, current, goal):
        """计算两个点之间的曼哈顿距离作为启发式函数"""
        # 使用曼哈顿距离而不是欧几里得距离，因为机器人只能上下左右移动
        return abs(current[0] - goal[0]) + abs(current[1] - goal[1]) 