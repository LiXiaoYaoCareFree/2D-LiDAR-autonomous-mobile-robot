#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
迷宫探索与路径规划
使用基于栅格的DFS算法探索未知迷宫，并使用A*算法规划最优路径
"""

import sys, os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, Wedge
from matplotlib.gridspec import GridSpec
import time
import math
from matplotlib.font_manager import FontProperties
from matplotlib.animation import FuncAnimation
import random

# 导入路径规划算法
from python_motion_planning.global_planner.graph_search.a_star import AStar
from python_motion_planning.utils.environment.env import Grid

# 尝试设置中文字体
try:
    plt.rcParams['font.sans-serif'] = ['SimHei']  # 指定默认字体为黑体
    plt.rcParams['axes.unicode_minus'] = False  # 解决保存图像是负号'-'显示为方块的问题
    font = FontProperties(fname=r"c:\windows\fonts\simsun.ttc", size=14)
except:
    print("警告: 无法设置中文字体，将使用默认字体")
    font = None

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from python_motion_planning import *

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

class Robot:
    """自主移动机器人"""
    def __init__(self, start_pos, env, goal_pos=None, animation_speed=0.05):
        self.x, self.y, self.theta = start_pos
        self.env = env
        self.goal_pos = goal_pos
        self.path = []  # 机器人走过的路径
        self.laser_sensor = LaserSensor(env)
        self.animation_speed = animation_speed  # 动画速度控制参数
        
        # 为栅格地图创建探索地图
        self.explored_map = set()  # 已经探索的区域
        self.frontier = set()  # 边界区域
        self.unexplored_cells = set()  # 未探索的单元格
        
        # 添加起始位置
        self.path.append((self.x, self.y))
        self.explored_map.add((int(self.x), int(self.y)))
        
        # DFS探索需要的数据结构
        self.dfs_stack = []
        self.visited = set()
        self.grid_visited = set()  # 栅格DFS已访问节点
        self.grid_stack = []       # 栅格DFS栈
        
        # 初始化起始位置
        start_grid = (int(round(self.x)), int(round(self.y)))
        self.grid_stack.append(start_grid)
        self.grid_visited.add(start_grid)
        
        # 卡住检测
        self.last_positions = []  # 记录最近的位置
        self.stuck_threshold = 20  # 如果20步内位置没有明显变化，认为卡住了
        self.random_move_count = 0  # 随机移动计数
        self.max_random_moves = 10  # 最大随机移动次数
        self.failed_targets = set()  # 记录失败的目标点
        self.failed_count = {}  # 记录每个目标点的失败次数
        self.max_retry = 3  # 最大重试次数
        self.backtrack_count = 0  # 回溯计数
        self.last_exploration_target = None  # 上一个探索目标
        
        # 方向：上、右、下、左
        self.directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        
        # 激光传感器数据
        self.laser_points = []
        
        # 探索进度统计
        self.update_count = 0
        self.exploration_progress = 0
        
        # 为A*算法创建路径
        self.optimal_path = []
        
        # 移动方向 (上, 右, 下, 左)
        self.directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        
        # 跟踪未探索的区域
        self.exploration_step = 1  # 每次探索的步长，设为1以便按格子遍历
        
        # 添加迷宫格子探索相关参数
        self.maze_cells = {}  # 用于记录迷宫的每个格子的状态：0=未知，1=通路，2=墙壁
        self.cell_size = 1  # 格子大小
        self.initialize_maze_cells()
        
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
        for x, y in self.explored_map:
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
        for cell in self.explored_map:
            # 检查四个方向的邻居
            for dx, dy in self.directions:
                nx, ny = cell[0] + dx, cell[1] + dy
                neighbor = (nx, ny)
                
                # 如果邻居在地图范围内，不是障碍物，且未被探索，则是边界
                if (0 <= nx < self.env.x_range and 0 <= ny < self.env.y_range and
                    neighbor not in self.env.obstacles and
                    neighbor not in self.explored_map):
                    self.frontier.add(neighbor)
        
        # 更新未探索的单元格
        self.unexplored_cells = self.frontier.copy()
    
    def get_frontier_cells(self):
        """获取未探索的边界单元格"""
        return self.frontier.copy()
        
    def update_position(self, new_pos):
        """更新机器人位置"""
        new_x, new_y, new_theta = new_pos
        
        # 检查新位置是否在地图范围内
        if not (0 <= new_x < self.env.x_range and 0 <= new_y < self.env.y_range):
            print(f"位置 ({new_x}, {new_y}) 超出地图范围")
            return False
        
        # 检查新位置是否是障碍物
        if (int(round(new_x)), int(round(new_y))) in self.env.obstacles:
            print(f"位置 ({new_x}, {new_y}) 是障碍物")
            return False
        
        # 检查从当前位置到新位置的路径是否穿过障碍物
        if not self.is_path_clear(int(round(self.x)), int(round(self.y)), 
                                int(round(new_x)), int(round(new_y))):
            print(f"从 ({self.x}, {self.y}) 到 ({new_x}, {new_y}) 的路径穿过障碍物")
            return False
        
        # 计算移动距离
        dist = math.sqrt((new_x - self.x) ** 2 + (new_y - self.y) ** 2)
        
        # 如果距离太远，可能会穿过障碍物，拒绝移动
        if dist > 2.0:
            print(f"移动距离 {dist:.2f} 太远，可能会穿过障碍物")
            return False
            
        # 更新位置和角度
        self.x, self.y, self.theta = new_x, new_y, new_theta
        
        # 记录路径
        self.path.append((self.x, self.y))
        
        # 将当前位置标记为已探索
        grid_pos = (int(round(self.x)), int(round(self.y)))
        self.explored_map.add(grid_pos)
        
        # 如果当前位置在未探索列表中，将其移除
        if grid_pos in self.unexplored_cells:
            self.unexplored_cells.remove(grid_pos)
        
        # 扫描环境
        self.scan_environment()
        
        # 模拟移动时间
        time.sleep(self.animation_speed)
        
        return True
    
    def grid_dfs_explore(self):
        """基于栅格的DFS探索算法"""
        # 更新未探索的单元格
        self.update_count += 1
        if (not self.unexplored_cells or self.update_count % 10 == 0):  # 初始化或每10步更新一次未探索区域
            # 更新未探索的单元格为边界格子
            self.unexplored_cells = self.get_frontier_cells()
            if self.unexplored_cells:
                print(f"更新未探索区域，共有{len(self.unexplored_cells)}个未探索单元格")
        
        # 检查是否被卡住
        current_pos = (int(round(self.x)), int(round(self.y)))
        self.last_positions.append(current_pos)
        if len(self.last_positions) > self.stuck_threshold:
            self.last_positions.pop(0)
            
        # 如果最近的位置都相同或来回移动，认为被卡住
        if len(self.last_positions) == self.stuck_threshold:
            unique_positions = set(self.last_positions)
            if len(unique_positions) <= 2:
                print(f"机器人可能被卡住在位置{current_pos}，尝试随机移动")
                self.grid_stack = []  # 清空栈，强制寻找新目标
                
                # 增加随机移动次数，更有效地摆脱困境
                self.random_move_count = self.max_random_moves * 2
                return True
        
        # 如果需要随机移动，执行随机移动
        if self.random_move_count > 0:
            self.random_move_count -= 1
            print(f"执行随机移动，剩余{self.random_move_count}次")
            
            # 随机选择一个方向
            directions = list(self.directions)
            random.shuffle(directions)
            
            # 尝试每个方向，增加移动距离
            for dx, dy in directions:
                for step in range(1, 4):  # 尝试不同的步长
                    next_x, next_y = int(round(self.x)) + dx * step, int(round(self.y)) + dy * step
                    if (0 < next_x < self.env.x_range - 1 and 
                        0 < next_y < self.env.y_range - 1 and 
                        (next_x, next_y) not in self.env.obstacles and
                        self.is_path_clear(int(round(self.x)), int(round(self.y)), next_x, next_y)):
                        
                        target_theta = math.atan2(dy, dx)
                        print(f"随机移动到 ({next_x}, {next_y})，步长: {step}")
                        self.update_position((next_x, next_y, target_theta))
                        return True
            
            print("随机移动失败，所有方向都被阻塞")
            # 如果所有方向都被阻塞，尝试清除一些障碍物记录（可能是错误检测的障碍物）
            if self.random_move_count % 5 == 0:  # 每5次尝试清除一次
                current_x, current_y = int(round(self.x)), int(round(self.y))
                for dx in range(-2, 3):
                    for dy in range(-2, 3):
                        pos = (current_x + dx, current_y + dy)
                        if pos in self.failed_targets:
                            self.failed_targets.remove(pos)
                            print(f"清除失败目标点: {pos}")
            
        # 如果栈为空，尝试寻找未探索的区域
        if not self.grid_stack:
            # 检查是否还有未探索的区域
            unexplored = list(self.unexplored_cells - self.explored_map - self.env.obstacles - self.failed_targets)
            
            if unexplored:
                # 找到距离当前位置最近的未探索单元格
                current_pos = (int(round(self.x)), int(round(self.y)))
                
                # 使用曼哈顿距离作为启发式
                def manhattan_distance(p1, p2):
                    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])
                
                # 按距离排序
                unexplored.sort(key=lambda cell: manhattan_distance(cell, current_pos))
                
                # 选择最近的几个点中的随机一个，增加探索的随机性
                closest_idx = random.randint(0, min(5, len(unexplored) - 1))
                closest_unexplored = unexplored[closest_idx]
                
                # 如果与上一个目标相同，尝试选择不同的目标
                if closest_unexplored == self.last_exploration_target and len(unexplored) > 1:
                    closest_idx = (closest_idx + 1) % min(5, len(unexplored))
                    closest_unexplored = unexplored[closest_idx]
                
                self.last_exploration_target = closest_unexplored
                
                # 将最近的未探索单元格加入栈中
                self.grid_stack.append(closest_unexplored)
                print(f"找到新的未探索区域: {closest_unexplored}，距离: {manhattan_distance(closest_unexplored, current_pos)}")
                
                # 重置访问记录，开始新的探索
                self.grid_visited = set([closest_unexplored])
                
                # 清除一些未探索的单元格，减少计算量
                if len(self.unexplored_cells) > 1000:
                    # 随机删除一些远离当前位置的单元格
                    cells_to_remove = sorted(list(self.unexplored_cells), 
                                           key=lambda cell: -manhattan_distance(cell, current_pos))
                    for cell in cells_to_remove[:len(cells_to_remove)//2]:
                        self.unexplored_cells.remove(cell)
            elif len(self.failed_targets) > 0 and len(self.unexplored_cells) > 0:
                # 如果所有未探索区域都被标记为失败，但仍有未探索单元格，重置失败记录
                print("重置失败记录，重新尝试所有未探索区域")
                self.failed_targets = set()
                self.failed_count = {}
                return True
            else:
                # 所有区域都已探索
                print("所有区域已完成探索!")
                return False
        
        # 从栈中弹出当前位置
        current = self.grid_stack.pop()
        
        # 计算当前位置到机器人位置的方向
        current_x, current_y = current
        robot_x, robot_y = int(round(self.x)), int(round(self.y))
        
        # 如果当前位置不是机器人位置，需要先移动到该位置
        if current != (robot_x, robot_y):
            # 使用A*算法规划从当前位置到目标位置的路径
            a_star = AStar(start=(robot_x, robot_y), goal=current, env=self.env)
            cost, path, _ = a_star.plan()
            
            if path and len(path) > 1:
                # 移动到路径的下一个点
                next_pos = path[1]
                
                # 检查是否可以直接移动到下一个点（不穿过障碍物）
                if self.is_path_clear(robot_x, robot_y, next_pos[0], next_pos[1]):
                    dx = next_pos[0] - robot_x
                    dy = next_pos[1] - robot_y
                    
                    # 设置朝向角度
                    target_theta = math.atan2(dy, dx)
                    
                    # 更新位置
                    if self.update_position((next_pos[0], next_pos[1], target_theta)):
                        return True
                    else:
                        print(f"移动到{next_pos}失败")
                else:
                    # 如果不能直接移动，尝试找一条不穿过障碍物的路径
                    # 重新规划一条更细致的路径
                    fine_path = self.find_fine_path((robot_x, robot_y), next_pos)
                    if fine_path and len(fine_path) > 1:
                        next_fine_pos = fine_path[1]
                        dx = next_fine_pos[0] - robot_x
                        dy = next_fine_pos[1] - robot_y
                        target_theta = math.atan2(dy, dx)
                        if self.update_position((next_fine_pos[0], next_fine_pos[1], target_theta)):
                            return True
                        else:
                            print(f"移动到{next_fine_pos}失败")
            
            # 如果无法规划路径，记录失败的目标点
            if current not in self.failed_count:
                self.failed_count[current] = 0
            self.failed_count[current] += 1
            
            if self.failed_count[current] >= self.max_retry:
                print(f"目标点{current}尝试{self.max_retry}次仍然失败，标记为不可达")
                self.failed_targets.add(current)
                
                # 如果栈中还有其他目标，继续尝试
                if self.grid_stack:
                    return True
                
                # 尝试随机移动来摆脱困境
                self.random_move_count = self.max_random_moves
                return True
            
            # 尝试回溯
            self.backtrack_count += 1
            if self.backtrack_count > 5 and self.grid_stack:
                # 如果多次尝试失败，直接跳到栈中的下一个位置
                self.backtrack_count = 0
                return True
            
            # 尝试直接移动（但确保不穿过障碍物）
            dx = current_x - robot_x
            dy = current_y - robot_y
            
            # 规范化移动方向，每次只移动一格
            if abs(dx) > 0:
                dx = dx // abs(dx)
            if abs(dy) > 0:
                dy = dy // abs(dy)
            
            next_x, next_y = robot_x + dx, robot_y + dy
            
            # 检查是否可以直接移动
            if (next_x, next_y) not in self.env.obstacles and self.is_path_clear(robot_x, robot_y, next_x, next_y):
                # 设置朝向角度
                target_theta = math.atan2(dy, dx)
                
                # 更新位置
                if self.update_position((next_x, next_y, target_theta)):
                    return True
                else:
                    print(f"移动到{(next_x, next_y)}失败")
            else:
                # 如果直接移动也不行，尝试其他方向
                for dx, dy in self.directions:
                    next_x, next_y = robot_x + dx, robot_y + dy
                    if ((next_x, next_y) not in self.env.obstacles and 
                        0 <= next_x < self.env.x_range and 
                        0 <= next_y < self.env.y_range and
                        self.is_path_clear(robot_x, robot_y, next_x, next_y)):
                        target_theta = math.atan2(dy, dx)
                        if self.update_position((next_x, next_y, target_theta)):
                            return True
                        else:
                            print(f"移动到{(next_x, next_y)}失败")
                
                # 如果所有方向都不行，放弃当前目标
                if current not in self.failed_count:
                    self.failed_count[current] = 0
                self.failed_count[current] += 1
                
                if self.failed_count[current] >= self.max_retry:
                    print(f"目标点{current}尝试{self.max_retry}次仍然失败，标记为不可达")
                    self.failed_targets.add(current)
                
                if self.grid_stack:
                    return True
                return False
        
        # 将当前位置标记为已探索
        self.explored_map.add(current)
        if current in self.unexplored_cells:
            self.unexplored_cells.remove(current)
        
        # 探索周围的四个方向
        neighbors = []
        for dx, dy in self.directions:
            next_x, next_y = current_x + dx, current_y + dy
            next_pos = (next_x, next_y)
            
            # 检查是否是有效位置（在地图内，不是障碍物，且未访问过）
            if (0 < next_x < self.env.x_range - 1 and 
                0 < next_y < self.env.y_range - 1 and 
                next_pos not in self.env.obstacles and
                next_pos not in self.grid_visited and
                self.is_path_clear(current_x, current_y, next_x, next_y)):  # 确保路径不穿过障碍物
                
                # 计算启发式值（优先探索未探索区域）
                heuristic = 1 if next_pos not in self.explored_map else 10
                neighbors.append((next_pos, heuristic))
        
        # 按启发式值排序，优先探索未探索区域
        neighbors.sort(key=lambda x: x[1])
        
        # 将排序后的邻居加入栈中
        for next_pos, _ in neighbors:
            self.grid_stack.append(next_pos)
            self.grid_visited.add(next_pos)
        
        # 如果有邻居，移动到第一个邻居
        if neighbors:
            next_pos = neighbors[0][0]
            next_x, next_y = next_pos
            dx = next_x - current_x
            dy = next_y - current_y
            
            # 设置朝向
            target_theta = math.atan2(dy, dx)
            
            # 更新位置
            if self.update_position((next_x, next_y, target_theta)):
                return True
            else:
                print(f"移动到{next_pos}失败")
                # 如果移动失败，但栈不为空，继续尝试
                if self.grid_stack:
                    return True
        
        # 如果四个方向都探索过，但栈不为空，则继续
        if self.grid_stack:
            return True
        
        # 如果栈为空且没有未探索区域，则尝试随机移动
        if not self.grid_stack and not self.unexplored_cells:
            self.random_move_count = self.max_random_moves
            return True
            
        return False
    
    def scan_environment(self):
        """使用激光雷达扫描环境，更新探索地图"""
        # 使用激光传感器扫描环境
        self.laser_points = self.laser_sensor.scan((self.x, self.y, self.theta))
        
        # 更新已探索区域
        current_x, current_y = int(round(self.x)), int(round(self.y))
        
        # 将当前位置标记为已探索
        self.explored_map.add((current_x, current_y))
        
        # 更新周围的探索区域（模拟视野）
        view_range = 2
        for dx in range(-view_range, view_range + 1):
            for dy in range(-view_range, view_range + 1):
                nx, ny = current_x + dx, current_y + dy
                if 0 <= nx < self.env.x_range and 0 <= ny < self.env.y_range:
                    # 只添加非障碍物区域
                    if (nx, ny) not in self.env.obstacles:
                        self.explored_map.add((nx, ny))
        
        # 处理激光点
        for point in self.laser_points:
            # 计算激光点的栅格坐标
            grid_x, grid_y = int(round(point[0])), int(round(point[1]))
            
            # 将激光点标记为已探索
            if 0 <= grid_x < self.env.x_range and 0 <= grid_y < self.env.y_range:
                self.explored_map.add((grid_x, grid_y))
                
                # 如果是障碍物，确保它在障碍物集合中
                if (grid_x, grid_y) in self.env.obstacles:
                    self.env.obstacles.add((grid_x, grid_y))
                
                # 激光点沿途的点也标记为已探索
                points = self.bresenham_line(current_x, current_y, grid_x, grid_y)
                for px, py in points:
                    if 0 <= px < self.env.x_range and 0 <= py < self.env.y_range:
                        self.explored_map.add((px, py))
        
        # 更新未探索的边界单元格
        self.update_frontier_cells()
        
        # 计算探索进度
        total_cells = (self.env.x_range - 2) * (self.env.y_range - 2) - len(self.env.obstacles)
        explored_cells = len(self.explored_map - self.env.obstacles)
        self.exploration_progress = explored_cells / total_cells * 100 if total_cells > 0 else 100
    
    def find_optimal_path(self, start, goal):
        """使用A*算法找到最优路径"""
        a_star = AStar(start=start, goal=goal, env=self.env)
        cost, path, _ = a_star.plan()
        
        if path:
            self.optimal_path = path
            return True
        return False

    def is_path_clear(self, x1, y1, x2, y2):
        """检查从(x1,y1)到(x2,y2)的路径是否穿过障碍物"""
        # 使用Bresenham算法检查路径上的每个点
        points = self.bresenham_line(x1, y1, x2, y2)
        for x, y in points:
            if (x, y) in self.env.obstacles:
                return False
        return True
    
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
    
    def find_fine_path(self, start, goal):
        """寻找一条更细致的路径，避免穿过障碍物"""
        # 使用A*算法，但增加更多的检查点
        a_star = AStar(start=start, goal=goal, env=self.env)
        _, path, _ = a_star.plan()
        
        if not path:
            return None
            
        # 检查路径是否穿过障碍物
        for i in range(len(path) - 1):
            if not self.is_path_clear(path[i][0], path[i][1], path[i+1][0], path[i+1][1]):
                return None
                
        return path

class MazeExploration:
    """迷宫测绘与导航系统"""
    def __init__(self):
        # 创建环境
        self.grid_env = self.create_environment()
        
        # 定义起点和目标点
        self.start_pos = (5, 5, 0)  # (x, y, theta)
        self.goal_pos = (45, 25)    # 目标位置，初始不知道
        
        # 创建机器人
        self.robot = Robot(self.start_pos, self.grid_env)
        
        # 设置可视化
        self.setup_visualization()
        
        # 设置状态
        self.exploration_complete = False
        self.path_planning_complete = False
        self.goal_found = False
        self.animation_running = True
        self.exploration_paused = False
        self.current_state = "exploration"  # 当前状态：exploration, path_planning, complete
        
        # 添加目标检测距离
        self.goal_detection_distance = 3  # 当机器人靠近目标3个单位时，视为找到目标
        
        # 添加帧率控制
        self.last_update_time = time.time()
        self.update_interval = 0.05  # 更新间隔，单位秒
        
        # 探索完成度阈值
        self.exploration_threshold = 0.70  # 探索70%的区域认为完成
        
        # 计数器
        self.step_count = 0
        self.exploration_start_time = time.time()
        
    def create_environment(self):
        """创建迷宫环境"""
        # 设置迷宫大小，确保不超过16x16
        width, height = 15, 15  # 改为15x15，确保不超过16x16的要求
        grid_env = Grid(width, height)
        obstacles = grid_env.obstacles
        
        # 初始化迷宫，先将所有单元格设为墙壁
        for x in range(width):
            for y in range(height):
                obstacles.add((x, y))
        
        # 使用DFS算法生成迷宫
        # 迷宫生成从(1,1)开始，每次走2步，确保墙壁和通道的分隔
        start_x, start_y = 1, 1
        # 移除起点的障碍物
        obstacles.discard((start_x, start_y))
        
        # 定义DFS栈和已访问集合
        stack = [(start_x, start_y)]
        visited = set([(start_x, start_y)])
        
        # 定义移动方向：上、右、下、左
        directions = [(0, 2), (2, 0), (0, -2), (-2, 0)]
        
        # DFS生成迷宫
        while stack:
            current_x, current_y = stack[-1]
            
            # 获取可能的移动方向
            possible_directions = []
            for dx, dy in directions:
                nx, ny = current_x + dx, current_y + dy
                # 检查是否在边界内且未访问过
                if 0 < nx < width-1 and 0 < ny < height-1 and (nx, ny) not in visited:
                    possible_directions.append((dx, dy))
            
            if possible_directions:
                # 随机选择一个方向
                dx, dy = random.choice(possible_directions)
                nx, ny = current_x + dx, current_y + dy
                
                # 移除墙壁（路径上的点和中间点）
                obstacles.discard((nx, ny))
                obstacles.discard((current_x + dx//2, current_y + dy//2))
                
                # 标记为已访问并加入栈
                visited.add((nx, ny))
                stack.append((nx, ny))
            else:
                # 如果没有可行方向，回溯
                stack.pop()
        
        # 设置起点和终点
        self.start_pos = (1, 1, 0)  # (x, y, theta)
        
        # 设置终点在迷宫的右下角
        goal_x, goal_y = width-2, height-2
        self.goal_pos = (goal_x, goal_y)
        
        # 确保起点和终点不是障碍物
        obstacles.discard((1, 1))
        obstacles.discard((goal_x, goal_y))
        
        # 确保起点和终点周围的区域也是可通行的
        for dx in range(-1, 2):
            for dy in range(-1, 2):
                nx, ny = 1 + dx, 1 + dy
                if 0 <= nx < width and 0 <= ny < height:
                    obstacles.discard((nx, ny))
                
                nx, ny = goal_x + dx, goal_y + dy
                if 0 <= nx < width and 0 <= ny < height:
                    obstacles.discard((nx, ny))
        
        # 更新环境的障碍物
        grid_env.update(obstacles)
        
        # 为Grid类添加is_obstacle方法
        def is_obstacle(x, y):
            return (x, y) in obstacles
        
        # 动态添加方法到grid_env实例
        grid_env.is_obstacle = is_obstacle
        
        return grid_env
        
    def setup_visualization(self):
        """设置可视化"""
        # 创建图形和子图
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(15, 7))
        
        # 设置第一个子图（实时探索）
        self.ax1.set_xlim(0, self.grid_env.x_range)
        self.ax1.set_ylim(0, self.grid_env.y_range)
        self.ax1.set_title("Robot Exploration")
        self.ax1.set_aspect('equal')
        
        # 设置第二个子图（地图）
        self.ax2.set_xlim(0, self.grid_env.x_range)
        self.ax2.set_ylim(0, self.grid_env.y_range)
        self.ax2.set_title("Grid Map")
        self.ax2.set_aspect('equal')
        
        # 绘制障碍物
        for obs in self.grid_env.obstacles:
            self.ax1.add_patch(Rectangle((obs[0]-0.5, obs[1]-0.5), 1, 1, color='black'))
            self.ax2.add_patch(Rectangle((obs[0]-0.5, obs[1]-0.5), 1, 1, color='black'))
        
        # 绘制起点和终点
        self.ax1.plot(self.start_pos[0], self.start_pos[1], 'go', markersize=8, label='Start')
        self.ax2.plot(self.start_pos[0], self.start_pos[1], 'go', markersize=8, label='Start')
        self.ax1.plot(self.goal_pos[0], self.goal_pos[1], 'ro', markersize=8, label='Goal')
        self.ax2.plot(self.goal_pos[0], self.goal_pos[1], 'ro', markersize=8, label='Goal')
        
        # 添加图例
        self.ax1.legend()
        self.ax2.legend()
        
        # 初始化机器人位置标记
        start_x, start_y = self.robot.x, self.robot.y
        self.robot_marker1, = self.ax1.plot([start_x], [start_y], 'bo', markersize=8)
        self.robot_marker2, = self.ax2.plot([start_x], [start_y], 'bo', markersize=8)
        
        # 初始化机器人方向指示
        length = 1.0
        dx = length * np.cos(self.robot.theta)
        dy = length * np.sin(self.robot.theta)
        self.direction_line1, = self.ax1.plot([start_x, start_x + dx], [start_y, start_y + dy], 'b-', linewidth=2)
        self.direction_line2, = self.ax2.plot([start_x, start_x + dx], [start_y, start_y + dy], 'b-', linewidth=2)
        
        # 初始化路径
        self.path_line1, = self.ax1.plot([], [], 'c-', linewidth=1, alpha=0.5)
        self.path_line2, = self.ax2.plot([], [], 'c-', linewidth=1, alpha=0.5)
        
        # 初始化最优路径
        self.optimal_path_line1, = self.ax1.plot([], [], 'g-', linewidth=2)
        self.optimal_path_line2, = self.ax2.plot([], [], 'g-', linewidth=2)
        
        # 初始化激光线
        self.laser_lines = []
        
        # 初始化阴影（表示未探索区域）
        self.unexplored = np.ones((self.grid_env.x_range, self.grid_env.y_range))
        # 起点周围区域设为已探索
        x, y = int(self.robot.x), int(self.robot.y)
        view_range = 2
        for dx in range(-view_range, view_range + 1):
            for dy in range(-view_range, view_range + 1):
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.grid_env.x_range and 0 <= ny < self.grid_env.y_range:
                    self.unexplored[nx, ny] = 0
                    
        self.shadow = self.ax1.imshow(self.unexplored.T, origin='lower', extent=(0, self.grid_env.x_range, 0, self.grid_env.y_range), 
                                     cmap='Greys', alpha=0.3, vmin=0, vmax=1)
        
        # 初始化探索地图散点图
        self.explored_scatter = self.ax2.scatter([], [], c='lightblue', s=5, alpha=0.8)
        
        # 初始化迷宫格子状态散点图
        self.unknown_cells_scatter = self.ax2.scatter([], [], c='lightgray', s=5, alpha=0.5)
        self.path_cells_scatter = self.ax2.scatter([], [], c='lightblue', s=5, alpha=0.8)
        
        # 添加信息文本
        self.info_text = self.ax1.text(0.02, 0.98, '', transform=self.ax1.transAxes, 
                                      verticalalignment='top', fontsize=10, 
                                      bbox=dict(boxstyle='round', facecolor='white', alpha=0.7))
        
        # 添加状态文本
        self.status_text = self.ax1.text(0.02, 0.02, '', transform=self.ax1.transAxes, 
                                        verticalalignment='bottom', fontsize=10, 
                                        bbox=dict(boxstyle='round', facecolor='white', alpha=0.7))
        
        # 添加控制按钮
        button_axes = plt.axes([0.45, 0.01, 0.1, 0.04])
        self.pause_button = plt.Button(button_axes, 'Pause', color='lightgoldenrodyellow')
        self.pause_button.on_clicked(self.toggle_pause)
        
        plt.tight_layout()
        
    def toggle_pause(self, event):
        """暂停/继续动画"""
        self.exploration_paused = not self.exploration_paused
        if self.exploration_paused:
            self.pause_button.label.set_text('Continue')
        else:
            self.pause_button.label.set_text('Pause')
        
    def update_visualization(self):
        """更新可视化"""
        # 清除之前的激光线
        for line in self.laser_lines:
            line.remove() if hasattr(line, 'remove') else None
        self.laser_lines = []
        
        # 添加新的激光线
        for point in self.robot.laser_points:
            line, = self.ax1.plot([self.robot.x, point[0]], [self.robot.y, point[1]], 'r-', alpha=0.3, linewidth=0.5)
            self.laser_lines.append(line)
        
        # 更新机器人位置和方向
        self.robot_marker1.set_data([self.robot.x], [self.robot.y])
        self.robot_marker2.set_data([self.robot.x], [self.robot.y])
        
        dx = math.cos(self.robot.theta) * 1.0
        dy = math.sin(self.robot.theta) * 1.0
        self.direction_line1.set_data([self.robot.x, self.robot.x + dx], [self.robot.y, self.robot.y + dy])
        self.direction_line2.set_data([self.robot.x, self.robot.x + dx], [self.robot.y, self.robot.y + dy])
        
        # 更新路径
        if self.robot.path:
            path_x, path_y = zip(*self.robot.path)
            self.path_line1.set_data(path_x, path_y)
            self.path_line2.set_data(path_x, path_y)
        
        # 更新探索地图
        if hasattr(self, 'explored_scatter'):
            self.explored_scatter.remove()
        
        if self.robot.explored_map:
            explored_x, explored_y = zip(*self.robot.explored_map)
            self.explored_scatter = self.ax2.scatter(explored_x, explored_y, c='lightblue', s=5, alpha=0.8)
        
        # 更新迷宫格子状态
        # 清除之前的格子状态
        if hasattr(self, 'unknown_cells_scatter'):
            self.unknown_cells_scatter.remove()
        if hasattr(self, 'path_cells_scatter'):
            self.path_cells_scatter.remove()
            
        # 显示未知区域（灰色）
        unknown_cells = [(x, y) for (x, y), status in self.robot.maze_cells.items() if status == 0]
        if unknown_cells:
            unknown_x, unknown_y = zip(*unknown_cells)
            self.unknown_cells_scatter = self.ax2.scatter(unknown_x, unknown_y, c='lightgray', s=5, alpha=0.5)
            
        # 显示已探索的通路（蓝色）
        path_cells = [(x, y) for (x, y), status in self.robot.maze_cells.items() if status == 1]
        if path_cells:
            path_x, path_y = zip(*path_cells)
            self.path_cells_scatter = self.ax2.scatter(path_x, path_y, c='lightblue', s=5, alpha=0.8)
        
        # 更新最优路径
        if self.robot.optimal_path:
            path_x, path_y = zip(*self.robot.optimal_path)
            self.optimal_path_line1.set_data(path_x, path_y)
            self.optimal_path_line2.set_data(path_x, path_y)
        
        # 更新阴影
        for pos in self.robot.explored_map:
            if 0 <= pos[0] < self.grid_env.x_range and 0 <= pos[1] < self.grid_env.y_range:
                self.unexplored[pos[0], pos[1]] = 0
        
        self.shadow.set_data(self.unexplored.T)
        
        # 计算探索进度
        total_cells = (self.grid_env.x_range - 2) * (self.grid_env.y_range - 2) - len(self.grid_env.obstacles)
        explored_cells = len(self.robot.explored_map)
        explored_percent = explored_cells / total_cells * 100 if total_cells > 0 else 0
        
        # 检查是否达到探索阈值
        if explored_percent >= self.exploration_threshold * 100 and not self.exploration_complete:
            self.exploration_complete = True
            print(f"Exploration threshold reached: {explored_percent:.2f}% of map explored")
        
        # 更新信息显示
        dist_to_goal = math.sqrt((self.robot.x - self.goal_pos[0])**2 + (self.robot.y - self.goal_pos[1])**2)
        elapsed_time = time.time() - self.exploration_start_time
        info_str = f"Position: ({self.robot.x:.1f}, {self.robot.y:.1f})\nDistance to goal: {dist_to_goal:.1f}\nExplored: {explored_percent:.1f}%\nSteps: {self.step_count}\nTime: {elapsed_time:.1f}s"
        self.info_text.set_text(info_str)
        
        # 更新状态显示
        status_str = f"Status: {self.current_state.capitalize()}"
        if self.exploration_paused:
            status_str += " (Paused)"
        self.status_text.set_text(status_str)
        
        # 检查是否找到目标
        if not self.goal_found and dist_to_goal <= self.goal_detection_distance:
            self.goal_found = True
            print("Goal found! Distance:", dist_to_goal)
        
        # 刷新图形
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
    
    def update(self, frame):
        """更新动画帧"""
        # 控制帧率
        current_time = time.time()
        if current_time - self.last_update_time < self.update_interval:
            return
        self.last_update_time = current_time
        
        # 如果动画已暂停，不进行更新
        if self.exploration_paused:
            return
        
        # 更新步数
        self.step_count += 1
        
        # 根据当前状态执行不同的操作
        if self.current_state == "exploration":
            # 探索阶段
            # 扫描环境，更新探索地图
            self.robot.scan_environment()
            
            # 使用基于栅格的DFS探索
            if not self.robot.grid_dfs_explore():
                print("Grid DFS exploration completed!")
                self.exploration_complete = True
                self.current_state = "path_planning"
                
            # 显示探索进度
            if self.step_count % 10 == 0:
                print(f"探索进度: {self.robot.exploration_progress:.2f}%")
                print(f"已探索: {len(self.robot.explored_map)}, 未探索: {len(self.robot.unexplored_cells)}")
                
            # 检查是否卡住
            if self.step_count % 50 == 0:
                # 检查最近50步是否有明显进展
                if len(self.robot.unexplored_cells) > 0 and self.robot.random_move_count == 0:
                    print("检测到可能卡住，尝试随机移动")
                    self.robot.random_move_count = self.robot.max_random_moves
                    
        elif self.current_state == "path_planning":
            # 路径规划阶段
            print("Planning path to goal...")
            
            # 如果目标位置不存在，设置为地图右下角
            if not self.robot.goal_pos:
                self.robot.goal_pos = (self.grid_env.x_range - 2, self.grid_env.y_range - 2)
                
            # 使用A*算法规划从当前位置到目标位置的路径
            start = (int(round(self.robot.x)), int(round(self.robot.y)))
            goal = (int(round(self.robot.goal_pos[0])), int(round(self.robot.goal_pos[1])))
            
            # 确保目标不是障碍物
            if goal in self.grid_env.obstacles:
                # 寻找附近非障碍物的位置
                for dx in range(-2, 3):
                    for dy in range(-2, 3):
                        new_goal = (goal[0] + dx, goal[1] + dy)
                        if (new_goal not in self.grid_env.obstacles and 
                            0 <= new_goal[0] < self.grid_env.x_range and 
                            0 <= new_goal[1] < self.grid_env.y_range):
                            goal = new_goal
                            break
            
            # 规划路径
            cost, path, _ = self.robot.find_optimal_path(start, goal)
            
            if path:
                print(f"Path found with cost: {cost}")
                self.planned_path = path
                self.current_state = "navigation"
            else:
                print("No path found to goal. Trying to find a closer goal...")
                # 尝试找一个更接近的目标
                self.robot.random_move_count = self.robot.max_random_moves
                self.current_state = "exploration"
                
        elif self.current_state == "navigation":
            # 导航阶段
            if not self.planned_path:
                print("No planned path available. Returning to exploration...")
                self.current_state = "exploration"
                return
                
            # 如果已到达目标，结束导航
            current_pos = (int(round(self.robot.x)), int(round(self.robot.y)))
            goal_pos = (int(round(self.robot.goal_pos[0])), int(round(self.robot.goal_pos[1])))
            
            if current_pos == goal_pos:
                print("Goal reached!")
                self.current_state = "completed"
                self.exploration_paused = True
                self.exploration_end_time = time.time()
                exploration_time = self.exploration_end_time - self.exploration_start_time
                print(f"Total exploration time: {exploration_time:.2f} seconds")
                print(f"Total steps: {self.step_count}")
                return
                
            # 获取下一个路径点
            if len(self.planned_path) > 1:
                next_pos = self.planned_path[1]  # 跳过当前位置
                
                # 检查是否可以直接移动到下一个点
                if self.robot.is_path_clear(current_pos[0], current_pos[1], next_pos[0], next_pos[1]):
                    # 计算朝向
                    dx = next_pos[0] - current_pos[0]
                    dy = next_pos[1] - current_pos[1]
                    target_theta = math.atan2(dy, dx)
                    
                    # 更新位置
                    if self.robot.update_position((next_pos[0], next_pos[1], target_theta)):
                        # 更新路径
                        self.planned_path = self.planned_path[1:]
                    else:
                        print("Failed to move to next position. Replanning...")
                        self.current_state = "path_planning"
                else:
                    print("Path to next position is blocked. Replanning...")
                    self.current_state = "path_planning"
            else:
                # 路径已完成，但未到达目标
                print("Path completed but goal not reached. Replanning...")
                self.current_state = "path_planning"
        
        elif self.current_state == "completed":
            # 探索完成，不再更新
            pass
            
        # 更新可视化
        self.update_visualization()
        
    def run(self):
        """运行迷宫测绘与导航"""
        print("Starting maze exploration with real-time animation...")
        self.exploration_start_time = time.time()
        
        # 创建动画
        self.anim = FuncAnimation(
            self.fig, 
            self.update,
            frames=None,  # 无限帧
            interval=50,  # 每50毫秒更新一次
            repeat=False,
            blit=False
        )
        
        # 显示图形
        plt.show(block=True)

if __name__ == '__main__':
    maze_exploration = MazeExploration()
    maze_exploration.run() 