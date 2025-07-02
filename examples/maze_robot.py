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
        
        # 探索方向
        self.directions = [
            (1, 0),   # 右
            (0, 1),   # 上
            (-1, 0),  # 左
            (0, -1),  # 下
            (1, 1),   # 右上
            (-1, 1),  # 左上
            (-1, -1), # 左下
            (1, -1)   # 右下
        ]
        
        # 标记起始位置为已访问
        self.visited_cells.add((int(round(self.x)), int(round(self.y))))
        
        # 初始化边界
        self.update_frontier()
        
        # 传感器数据
        self.sensor_data = []
        
        # 探索进度
        self.exploration_progress = 0.0
        
        # 调试信息
        print(f"机器人初始化在位置 ({self.x}, {self.y})")
        print(f"环境大小: {self.env.x_range}x{self.env.y_range}")
        
        # 添加迷宫格子探索相关参数
        self.maze_cells = {}  # 用于记录迷宫的每个格子的状态：0=未知，1=通路，2=墙壁
        self.cell_size = 1    # 格子大小
        self.initialize_maze_cells()
        
        # 初始化路径
        self.path.append((self.x, self.y))
        
        # 设置初始移动方向
        self.first_move_done = False
        
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
        
        old_x, old_y = self.x, self.y
        self.x, self.y = new_x, new_y
        self.theta = new_theta
            
        # 添加到路径
        self.path.append((self.x, self.y))
        if len(self.path) > 1000:  # 限制路径长度
            self.path = self.path[-1000:]
            
        # 标记当前位置为已访问
        current_cell = (int(round(self.x)), int(round(self.y)))
        self.visited_cells.add(current_cell)
        
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
        
        # 更新探索进度
        total_cells = (self.env.x_range - 2) * (self.env.y_range - 2) - len(self.env.obstacles)
        explored_cells = len(self.visited_cells)
        self.exploration_progress = explored_cells / total_cells * 100 if total_cells > 0 else 0
        
        return True
    
    def scan_environment(self):
        """使用激光雷达扫描环境，更新探索地图"""
        # 使用激光传感器扫描环境
        self.sensor_data = self.sensor.scan((self.x, self.y, self.theta))
        
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
        
        # 计算探索进度
        total_cells = (self.env.x_range - 2) * (self.env.y_range - 2) - len(self.env.obstacles)
        explored_cells = len(self.visited_cells - self.env.obstacles)
        self.exploration_progress = explored_cells / total_cells * 100 if total_cells > 0 else 100
        
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
                return True
            else:
                # 如果右边不能走，尝试向下
                next_x, next_y = int(round(self.x)), int(round(self.y)) + 1
                if (0 < next_x < self.env.x_range - 1 and 
                    0 < next_y < self.env.y_range - 1 and 
                    (next_x, next_y) not in self.env.obstacles):
                    
                    print(f"初始移动到 ({next_x}, {next_y})")
                    self.update_position((next_x, next_y, math.pi/2))
                    return True
                
        # 如果需要随机移动，执行随机移动
        if self.random_move_count > 0:
            print(f"执行随机移动，剩余随机移动次数: {self.random_move_count}")
            self.random_move_count -= 1
            
            # 获取所有可能的移动方向
            possible_moves = []
            current_x, current_y = int(round(self.x)), int(round(self.y))
            
            for dx, dy in self.directions:
                next_x, next_y = current_x + dx, current_y + dy
                if (0 < next_x < self.env.x_range - 1 and 
                    0 < next_y < self.env.y_range - 1 and 
                    (next_x, next_y) not in self.env.obstacles and
                    (next_x, next_y) not in self.failed_targets):
                    possible_moves.append((next_x, next_y))
            
            # 如果有可能的移动，随机选择一个
            if possible_moves:
                next_pos = random.choice(possible_moves)
                dx = next_pos[0] - current_x
                dy = next_pos[1] - current_y
                target_theta = math.atan2(dy, dx)
                
                print(f"随机移动到 {next_pos}")
                if self.update_position((next_pos[0], next_pos[1], target_theta)):
                    return True
            
            # 如果随机移动失败，尝试移动到未访问的邻居单元格
            for dx, dy in self.directions:
                next_x, next_y = current_x + dx, current_y + dy
                if (0 < next_x < self.env.x_range - 1 and 
                    0 < next_y < self.env.y_range - 1 and 
                    (next_x, next_y) not in self.env.obstacles and
                    (next_x, next_y) not in self.visited_cells):
                    
                    target_theta = math.atan2(dy, dx)
                    print(f"移动到未访问的邻居单元格 ({next_x}, {next_y})")
                    if self.update_position((next_x, next_y, target_theta)):
                        return True
            
            # 如果仍然无法移动，尝试移动到任何可行的邻居单元格
            for dx, dy in self.directions:
                next_x, next_y = current_x + dx, current_y + dy
                if (0 < next_x < self.env.x_range - 1 and 
                    0 < next_y < self.env.y_range - 1 and 
                    (next_x, next_y) not in self.env.obstacles):
                    
                    target_theta = math.atan2(dy, dx)
                    print(f"移动到任何可行的邻居单元格 ({next_x}, {next_y})")
                    if self.update_position((next_x, next_y, target_theta)):
                        return True
            
            print("无法执行随机移动，继续正常探索")
            
        # 更新传感器数据
        self.update_sensor_data()
        
        # 更新边界单元格
        self.update_frontier()
        
        # 如果探索栈为空，添加未探索的边界单元格
        if not self.exploration_stack:
            # 获取未访问的边界单元格
            unvisited_frontier = self.frontier - self.visited_cells - self.failed_targets
            
            if unvisited_frontier:
                # 按照到当前位置的距离排序
                current_pos = (int(round(self.x)), int(round(self.y)))
                sorted_frontier = sorted(unvisited_frontier, 
                                        key=lambda pos: abs(pos[0] - current_pos[0]) + abs(pos[1] - current_pos[1]))
                
                # 添加到探索栈
                for pos in sorted_frontier[:min(10, len(sorted_frontier))]:  # 最多添加10个
                    if pos not in self.exploration_stack and pos not in self.failed_targets:
                        self.exploration_stack.append(pos)
                        
                print(f"添加了 {len(self.exploration_stack)} 个边界单元格到探索栈")
                
                # 如果栈中有目标，尝试规划路径
                if self.exploration_stack:
                    target = self.exploration_stack[-1]  # 获取栈顶元素
                    print(f"规划到未探索区域 {target} 的路径")
                    
                    # 使用A*算法规划路径
                    current_pos = (int(round(self.x)), int(round(self.y)))
                    a_star = AStar(start=current_pos, goal=target, env=self.env)
                    cost, path, _ = a_star.plan()
                    
                    if path and len(path) > 1:
                        print(f"找到路径，长度: {len(path)}")
                        return True
                    else:
                        print(f"无法规划到 {target} 的路径，标记为失败目标")
                        if target in self.exploration_stack:
                            self.exploration_stack.remove(target)
                        self.failed_targets.add(target)
                        return True
            else:
                # 所有可达区域都已探索完毕
                print("所有可达区域都已完整遍历！")
                return False
        
        # 从栈中取出下一个目标点
        if self.exploration_stack:
            next_pos = self.exploration_stack.pop()
            
            # 检查目标点是否已经被访问过
            if next_pos in self.visited_cells:
                print(f"目标点 {next_pos} 已被访问，跳过")
                return True  # 继续探索
                
            # 检查是否可以直接移动到目标点
            current_x, current_y = int(round(self.x)), int(round(self.y))
            
            # 调试信息
            print(f"当前位置: ({current_x}, {current_y}), 目标位置: {next_pos}")
            
            # 检查目标点是否是障碍物
            if next_pos in self.env.obstacles:
                print(f"警告：目标点 {next_pos} 是障碍物！标记为失败目标")
                self.failed_targets.add(next_pos)
                return True
            
            # 检查目标点是否在地图边缘
            if next_pos[0] <= 0 or next_pos[1] <= 0 or next_pos[0] >= self.env.x_range - 1 or next_pos[1] >= self.env.y_range - 1:
                print(f"警告：目标点 {next_pos} 在地图边缘，标记为失败目标")
                self.failed_targets.add(next_pos)
                return True
            
            # 检查路径是否通畅
            is_clear = self.is_path_clear(current_x, current_y, next_pos[0], next_pos[1])
            print(f"路径是否通畅: {is_clear}")
            
            if is_clear:
                # 计算朝向
                dx = next_pos[0] - current_x
                dy = next_pos[1] - current_y
                target_theta = math.atan2(dy, dx)
                
                # 更新位置
                if self.update_position((next_pos[0], next_pos[1], target_theta)):
                    print(f"移动到 {next_pos}")
                    return True
                else:
                    print(f"移动到 {next_pos} 失败")
                    # 如果移动失败，标记为失败目标
                    self.failed_targets.add(next_pos)
                    # 尝试随机移动
                    self.random_move_count = 5
            else:
                print(f"到 {next_pos} 的路径被阻塞")
                # 重新规划路径
                current_pos = (current_x, current_y)
                a_star = AStar(start=current_pos, goal=next_pos, env=self.env)
                cost, path, _ = a_star.plan()
                
                if path and len(path) > 1:
                    # 移动到路径的下一个点
                    next_step = path[1]
                    dx = next_step[0] - current_x
                    dy = next_step[1] - current_y
                    target_theta = math.atan2(dy, dx)
                    
                    if self.update_position((next_step[0], next_step[1], target_theta)):
                        print(f"移动到 {next_step}（重新规划的路径）")
                        # 将剩余路径点加回栈中
                        for point in reversed(path[2:]):
                            self.exploration_stack.append(point)
                        return True
                    else:
                        print(f"移动到 {next_step} 失败")
                        self.failed_targets.add(next_pos)
                        # 尝试随机移动
                        self.random_move_count = 5
                else:
                    print(f"无法规划到 {next_pos} 的路径，标记为失败目标")
                    self.failed_targets.add(next_pos)
                    
                    # 尝试随机移动来摆脱困境
                    self.random_move_count = 5
        
        # 如果栈为空且没有可探索的边界，则探索完成
        if not self.exploration_stack and not self.frontier - self.visited_cells - self.failed_targets:
            # 检查是否还有任何未访问的单元格
            all_cells = set()
            for x in range(1, self.env.x_range - 1):
                for y in range(1, self.env.y_range - 1):
                    if (x, y) not in self.env.obstacles:
                        all_cells.add((x, y))
            
            unvisited_cells = all_cells - self.visited_cells - self.failed_targets
            
            if not unvisited_cells:
                print("迷宫完整遍历完成！")
                return False
            else:
                # 还有未访问的单元格，但可能无法到达
                print(f"还有 {len(unvisited_cells)} 个单元格未访问，但可能无法到达")
                # 标记这些单元格为失败目标，避免继续尝试
                self.failed_targets.update(unvisited_cells)
                return False
            
        return True
    
    def find_path_to_goal(self, goal_pos):
        """寻找到目标点的路径"""
        start = (int(round(self.x)), int(round(self.y)))
        goal = (int(round(goal_pos[0])), int(round(goal_pos[1])))
        
        # 使用A*算法规划路径
        a_star = AStar(start=start, goal=goal, env=self.env)
        cost, path, _ = a_star.plan()
        
        if path:
            # 验证路径是否可行（不穿过障碍物）
            valid_path = []
            current = start
            valid_path.append(current)
            
            for i in range(1, len(path)):
                next_pos = path[i]
                # 检查从当前点到下一个点的路径是否穿过障碍物
                if self.is_path_clear(current[0], current[1], next_pos[0], next_pos[1]):
                    valid_path.append(next_pos)
                    current = next_pos
                else:
                    # 如果路径被阻塞，尝试寻找一条从当前点到下一个点的可行路径
                    intermediate_path = self.find_intermediate_path(current, next_pos)
                    if intermediate_path:
                        # 添加中间路径点（跳过起点，因为已经添加过了）
                        valid_path.extend(intermediate_path[1:])
                        current = intermediate_path[-1]
                    else:
                        # 如果无法找到中间路径，停止在当前点
                        break
            
            # 检查最终路径是否到达目标
            if valid_path and valid_path[-1] == goal:
                self.goal_path = valid_path
                print(f"找到到目标点的有效路径，长度: {len(valid_path)}")
                return True
            else:
                # 尝试使用更细致的路径规划
                fine_path = self.find_fine_path(start, goal)
                if fine_path:
                    self.goal_path = fine_path
                    print(f"使用细致路径规划找到路径，长度: {len(fine_path)}")
                    return True
                else:
                    print("无法找到到目标点的有效路径")
                    return False
        else:
            print("无法找到到目标点的路径")
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
                return path[::-1]  # 反转路径
            
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
        
        # 定义更多方向（8个方向）
        fine_directions = [
            (0, 1), (1, 0), (0, -1), (-1, 0),  # 上、右、下、左
            (1, 1), (1, -1), (-1, -1), (-1, 1)  # 对角线方向
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
                return path[::-1]  # 反转路径
            
            open_set.remove(current)
            closed_set.add(current)
            
            # 检查8个方向的邻居
            for dx, dy in fine_directions:
                neighbor = (current[0] + dx, current[1] + dy)
                
                # 检查是否是有效位置
                if (0 <= neighbor[0] < self.env.x_range and 
                    0 <= neighbor[1] < self.env.y_range and 
                    neighbor not in self.env.obstacles and
                    neighbor not in closed_set and
                    self.is_path_clear(current[0], current[1], neighbor[0], neighbor[1])):
                    
                    # 计算移动代价（对角线移动代价更高）
                    move_cost = math.sqrt(dx*dx + dy*dy)
                    
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
                    return path1[:-1] + path2
        
        return None  # 没有找到路径
    
    def navigate_to_goal(self):
        """沿着规划的路径导航到目标点"""
        if not self.goal_path or len(self.goal_path) <= 1:
            print("没有可用的路径或已到达目标")
            return False
        
        # 获取下一个路径点
        next_pos = self.goal_path[1]  # 跳过当前位置
        
        # 计算朝向
        current_x, current_y = int(round(self.x)), int(round(self.y))
        dx = next_pos[0] - current_x
        dy = next_pos[1] - current_y
        target_theta = math.atan2(dy, dx)
        
        # 更新位置
        if self.update_position((next_pos[0], next_pos[1], target_theta)):
            # 更新路径
            self.goal_path = self.goal_path[1:]
            return True
        else:
            print(f"导航到 {next_pos} 失败，重新规划路径")
            # 重新规划路径
            start = (current_x, current_y)
            goal = self.goal_path[-1]
            a_star = AStar(start=start, goal=goal, env=self.env)
            cost, path, _ = a_star.plan()
            
            if path and len(path) > 1:
                self.goal_path = path
                return True
            else:
                print("无法重新规划路径")
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
        
        # 更新已探索区域
        current_x, current_y = int(round(self.x)), int(round(self.y))
        
        # 将当前位置标记为已访问
        self.visited_cells.add((current_x, current_y))
        
        # 更新周围的探索区域（模拟视野）
        vision_range = 2  # 视野范围
        for dx in range(-vision_range, vision_range + 1):
            for dy in range(-vision_range, vision_range + 1):
                # 计算距离
                dist = math.sqrt(dx**2 + dy**2)
                if dist <= vision_range:
                    nx, ny = current_x + dx, current_y + dy
                    # 检查是否在地图范围内
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
                    continue
                
                # 标记从机器人到激光点之间的所有点为已探索
                points = self.get_line(int(round(self.x)), int(round(self.y)), grid_x, grid_y)
                for px, py in points:
                    if 0 <= px < self.env.x_range and 0 <= py < self.env.y_range:
                        self.visited_cells.add((px, py))
        
        # 更新未探索的边界单元格
        self.update_frontier()
        
        # 计算探索进度
        total_cells = (self.env.x_range - 2) * (self.env.y_range - 2) - len(self.env.obstacles)
        explored_cells = len(self.visited_cells - self.env.obstacles)
        self.exploration_progress = explored_cells / total_cells * 100 if total_cells > 0 else 100
        
        # 更新迷宫格子状态
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