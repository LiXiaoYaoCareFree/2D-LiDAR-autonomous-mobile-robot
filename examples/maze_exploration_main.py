#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
迷宫探索与路径规划主程序
"""

import time
import math
import sys
import os
import argparse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib import colors

# 导入自定义模块
from maze_env import MazeEnvironment
from maze_robot import Robot
from maze_visualization import MazeVisualization

class SLAMVisualizer:
    """SLAM地图可视化器"""
    def __init__(self, maze_env, robot):
        self.maze_env = maze_env
        self.robot = robot
        self.display_size = max(maze_env.width, maze_env.height) + 4  # 增加边缘空间
        self.resolution = 0.1  # 地图分辨率
        self.grid_size = int(self.display_size / self.resolution)
        
        # 初始化SLAM地图 (0=未知/灰色, 1=空闲/白色, 2=障碍物/黑色)
        self.slam_map = np.zeros((self.grid_size, self.grid_size))
        
        # 机器人路径 - 仅内部使用，不显示
        self.robot_path = []
        
        # 激光扫描数据
        self.scan_points = []
        self.obstacle_points = []
        
        # 道路宽度（与机器人大小相关）
        self.road_width = 0.5  # 单位：网格单元
        
    def world_to_grid(self, world_pos):
        """将世界坐标转换为网格坐标"""
        x = int((world_pos[0] + 2) / self.resolution)
        y = int((world_pos[1] + 2) / self.resolution)
        return max(0, min(x, self.grid_size - 1)), max(0, min(y, self.grid_size - 1))
        
    def update_map(self, robot_pos, scan_data):
        """更新SLAM地图"""
        # 添加机器人位置到路径（内部记录，不显示）
        self.robot_path.append((robot_pos[0], robot_pos[1]))
        
        # 更新激光扫描数据
        self.scan_points = []
        self.obstacle_points = []
        
        # 获取机器人位置的网格坐标
        robot_grid_x, robot_grid_y = self.world_to_grid(robot_pos)
        
        # 将机器人当前位置标记为已探索（空闲）
        road_width_grid = int(self.road_width / self.resolution)
        for dx in range(-road_width_grid, road_width_grid + 1):
            for dy in range(-road_width_grid, road_width_grid + 1):
                nx, ny = robot_grid_x + dx, robot_grid_y + dy
                if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                    # 只有未知区域才标记为空闲，避免覆盖已标记的障碍物
                    if self.slam_map[nx, ny] == 0:
                        self.slam_map[nx, ny] = 1  # 标记为空闲
        
        # 处理激光扫描数据
        if scan_data:
            for point in scan_data:
                self.scan_points.append(point)
                
                # 将激光点标记为障碍物
                grid_x, grid_y = self.world_to_grid(point)
                
                # 将障碍物点周围小范围标记为障碍物，使墙壁更明显
                obstacle_width = 1  # 障碍物宽度
                for dx in range(-obstacle_width, obstacle_width + 1):
                    for dy in range(-obstacle_width, obstacle_width + 1):
                        nx, ny = grid_x + dx, grid_y + dy
                        if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                            self.slam_map[nx, ny] = 2  # 障碍物
                
                # 将机器人到激光点的路径标记为空闲
                self.mark_line_as_free(robot_grid_x, robot_grid_y, grid_x, grid_y)
                
                # 记录障碍物点
                self.obstacle_points.append(point)
    
    def mark_line_as_free(self, x0, y0, x1, y1):
        """使用Bresenham算法将线段标记为空闲，但不包括终点（障碍物）"""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        # 记录当前位置
        current_x, current_y = x0, y0
        
        # 标记起点为空闲
        road_width_grid = int(self.road_width / self.resolution)
        
        while True:
            # 检查是否到达终点前一个位置
            if (sx > 0 and current_x >= x1 - 1) or (sx < 0 and current_x <= x1 + 1) or \
               (sy > 0 and current_y >= y1 - 1) or (sy < 0 and current_y <= y1 + 1):
                # 已接近终点，停止标记
                break
            
            # 标记当前点及其周围为空闲（形成宽度一致的道路）
            for dx in range(-road_width_grid, road_width_grid + 1):
                for dy in range(-road_width_grid, road_width_grid + 1):
                    nx, ny = current_x + dx, current_y + dy
                    if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                        # 只有未知区域才标记为空闲，避免覆盖已标记的障碍物
                        if self.slam_map[nx, ny] == 0:
                            self.slam_map[nx, ny] = 1  # 标记为空闲
            
            # 沿着Bresenham线移动
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                current_x += sx
            if e2 < dx:
                err += dx
                current_y += sy

class MazeExplorationController:
    """迷宫探索控制器"""
    def __init__(self, maze_env):
        # 设置环境
        self.maze_env = maze_env
        
        # 创建机器人
        self.robot = Robot(self.maze_env.start_pos, self.maze_env.grid_env)
        
        # 创建可视化
        self.visualizer = MazeVisualization(self.maze_env, self.robot)
        self.visualizer.controller = self  # 设置控制器引用
        
        # 创建SLAM可视化器
        self.slam_visualizer = SLAMVisualizer(self.maze_env, self.robot)
        
        # 设置状态
        self.exploration_complete = False
        self.path_planning_complete = False
        self.goal_found = False
        self.current_state = "exploration"  # 当前状态：exploration, path_planning, navigation, completed
        
        # 添加目标检测距离
        self.goal_detection_distance = 3  # 当机器人靠近目标3个单位时，视为找到目标
        
        # 探索完成度阈值
        self.exploration_threshold = 0.99  # 设置为99%，确保合理的遍历但不过高
        
        # 计数器
        self.step_count = 0
        
        # 标记是否已找到起点和终点
        self.start_found = True  # 起点默认已知
        self.goal_found = False  # 终点初始未知
        
        # 标记是否已到达终点
        self.reached_goal = False
        
        # 标记是否已返回起点
        self.returned_to_start = False
        
        # 导航失败重试次数
        self.navigation_retry_count = 0
        self.max_navigation_retries = 5  # 减少最大重试次数，加快阶段转换
        
        # 添加探索时间限制，避免无限探索
        self.exploration_start_time = time.time()
        self.max_exploration_time = 300  # 减少最大探索时间，确保能进入后续阶段
        
    def update(self, visualization):
        """更新一帧"""
        # 如果动画已暂停，不进行更新
        if visualization.exploration_paused:
            return
        
        # 控制帧率
        current_time = time.time()
        if hasattr(visualization, 'last_update_time') and hasattr(visualization, 'update_interval'):
            if current_time - visualization.last_update_time < visualization.update_interval:
                return
            visualization.last_update_time = current_time
        else:
            visualization.last_update_time = current_time
            visualization.update_interval = 0.05  # 默认更新间隔
        
        # 更新步数
        self.step_count += 1
        
        # 更新SLAM地图
        self.slam_visualizer.update_map((self.robot.x, self.robot.y), self.robot.sensor_data)
        
        # 检查探索时间是否超时
        exploration_time = current_time - self.exploration_start_time
        if exploration_time > self.max_exploration_time and self.current_state == "exploration":
            print(f"\n===== 阶段转换：探索时间已达到限制 ({self.max_exploration_time}秒)，强制结束探索阶段 =====\n")
            self.exploration_complete = True
            
            # 如果已经找到目标但还没到达，现在导航到目标
            if self.goal_found and not self.reached_goal:
                print("\n===== 阶段转换：开始导航到目标点 =====\n")
                self.current_state = "navigate_to_goal"
                if self.robot.find_path_to_goal(self.maze_env.goal_pos):
                    print(f"找到到目标点的路径，长度: {len(self.robot.goal_path)}")
                else:
                    print("无法找到到目标点的路径，尝试直接进入路径规划阶段")
                    self.current_state = "path_planning"
            else:
                # 如果没找到目标，继续搜索
                print("\n===== 阶段转换：开始搜索目标点 =====\n")
                self.current_state = "search_goal"
        
        # 根据当前状态执行不同的操作
        if self.current_state == "exploration":
            # 探索阶段 - 完整遍历迷宫
            if not self.robot.explore_maze():
                print("\n===== 阶段转换：迷宫遍历完成 =====\n")
                self.exploration_complete = True
                
                # 如果已经找到目标但还没到达，现在导航到目标
                if self.goal_found and not self.reached_goal:
                    print("\n===== 阶段转换：开始导航到目标点 =====\n")
                    self.current_state = "navigate_to_goal"
                    if self.robot.find_path_to_goal(self.maze_env.goal_pos):
                        print(f"找到到目标点的路径，长度: {len(self.robot.goal_path)}")
                    else:
                        print("无法找到到目标点的路径，尝试直接进入路径规划阶段")
                        self.current_state = "path_planning"
                else:
                    # 如果没找到目标，继续搜索
                    print("\n===== 阶段转换：开始搜索目标点 =====\n")
                    self.current_state = "search_goal"
            
            # 显示探索进度
            if self.step_count % 10 == 0:
                print(f"探索进度: {self.robot.exploration_progress:.2f}%")
                # 显示探索时间
                elapsed_time = current_time - self.exploration_start_time
                print(f"探索时间: {elapsed_time:.1f}秒 / {self.max_exploration_time}秒")
            
            # 检查是否已经找到目标
            # 计算当前位置到目标的距离
            goal_x, goal_y = self.maze_env.goal_pos[0], self.maze_env.goal_pos[1]
            dist_to_goal = math.sqrt((self.robot.x - goal_x)**2 + (self.robot.y - goal_y)**2)
            
            # 当机器人靠近目标时，视为找到目标
            if dist_to_goal <= self.goal_detection_distance and not self.goal_found:
                print(f"\n===== 发现目标点！({goal_x}, {goal_y}) =====\n")
                self.goal_found = True
                
            # 检查是否达到探索阈值
            if self.robot.exploration_progress >= self.exploration_threshold * 100 and not self.exploration_complete:
                print(f"\n===== 阶段转换：达到探索阈值 {self.robot.exploration_progress:.2f}% >= {self.exploration_threshold * 100}% =====\n")
                self.exploration_complete = True
                
                # 如果已经找到目标但还没到达，现在导航到目标
                if self.goal_found and not self.reached_goal:
                    print("\n===== 阶段转换：开始导航到目标点 =====\n")
                    self.current_state = "navigate_to_goal"
                    if self.robot.find_path_to_goal(self.maze_env.goal_pos):
                        print(f"找到到目标点的路径，长度: {len(self.robot.goal_path)}")
                    else:
                        print("无法找到到目标点的路径，尝试直接进入路径规划阶段")
                        self.current_state = "path_planning"
                else:
                    # 如果没找到目标，继续搜索
                    print("\n===== 阶段转换：开始搜索目标点 =====\n")
                    self.current_state = "search_goal"
                
        elif self.current_state == "search_goal":
            # 搜索目标点阶段
            if not self.goal_found:
                # 尝试在地图中寻找目标点
                print("在地图中搜索目标点...")
                
                # 使用A*算法规划从当前位置到目标位置的路径
                if self.robot.find_path_to_goal(self.maze_env.goal_pos):
                    self.goal_found = True
                    print("\n===== 找到目标点路径！=====\n")
                    self.current_state = "navigate_to_goal"
                else:
                    print("\n===== 无法找到目标点，直接进入路径规划阶段 =====\n")
                    # 即使找不到目标，也强制进入路径规划阶段
                    self.current_state = "path_planning"
            else:
                # 已经找到目标点，导航过去
                print("\n===== 阶段转换：开始导航到目标点 =====\n")
                self.current_state = "navigate_to_goal"
                
        elif self.current_state == "navigate_to_goal":
            # 导航到目标点阶段
            # 检查是否已到达目标
            goal_x, goal_y = self.maze_env.goal_pos[0], self.maze_env.goal_pos[1]
            dist_to_goal = math.sqrt((self.robot.x - goal_x)**2 + (self.robot.y - goal_y)**2)
            
            if dist_to_goal <= 1.0:
                print(f"\n===== 阶段转换：已到达目标点！({goal_x}, {goal_y}) =====\n")
                self.reached_goal = True
                self.current_state = "path_planning"
                # 重置导航重试计数
                self.navigation_retry_count = 0
                return
                
            # 沿着规划的路径导航
            if not self.robot.navigate_to_goal():
                print("导航失败，重新规划路径...")
                self.navigation_retry_count += 1
                
                if self.navigation_retry_count >= self.max_navigation_retries:
                    print(f"\n===== 导航失败次数达到最大值({self.max_navigation_retries})，强制进入下一阶段 =====\n")
                    # 直接进入下一阶段
                    self.reached_goal = True
                    self.current_state = "path_planning"
                    self.navigation_retry_count = 0
                    return
                
                if self.robot.find_path_to_goal(self.maze_env.goal_pos):
                    print("重新规划路径成功")
                else:
                    print("无法找到到目标的路径，强制进入下一阶段")
                    self.current_state = "path_planning"
                
        elif self.current_state == "path_planning":
            # 路径规划阶段 - 从终点返回起点
            print("\n===== 阶段转换：规划从当前位置返回起点的路径 =====\n")
            
            # 使用A*算法规划从当前位置到起点的路径
            start_pos = self.maze_env.start_pos
            print(f"准备从当前位置 ({self.robot.x}, {self.robot.y}) 返回起点 {start_pos}")
            
            if self.robot.find_path_to_goal(start_pos):
                self.path_planning_complete = True
                print(f"找到返回起点的路径，长度: {len(self.robot.goal_path)}")
                print(f"路径详情: {self.robot.goal_path[:min(10, len(self.robot.goal_path))]}...")
                
                # 确保路径有效
                if len(self.robot.goal_path) <= 1:
                    print("警告: 路径太短，可能无效")
                    # 尝试强制创建一个简单路径
                    current_pos = (int(round(self.robot.x)), int(round(self.robot.y)))
                    if current_pos != start_pos:
                        print("尝试创建直接路径...")
                        dx = start_pos[0] - current_pos[0]
                        dy = start_pos[1] - current_pos[1]
                        
                        # 创建简单路径
                        path = [current_pos]
                        cx, cy = current_pos
                        
                        # 先水平移动
                        steps = abs(dx)
                        for i in range(steps):
                            cx += 1 if dx > 0 else -1
                            path.append((cx, cy))
                            
                        # 再垂直移动
                        steps = abs(dy)
                        for i in range(steps):
                            cy += 1 if dy > 0 else -1
                            path.append((cx, cy))
                        
                        self.robot.goal_path = path
                        print(f"创建了直接路径，长度: {len(self.robot.goal_path)}")
                        
                self.current_state = "navigation"
            else:
                print("无法找到返回起点的路径，探索结束")
                self.current_state = "completed"
                
        elif self.current_state == "navigation":
            # 导航阶段 - 从终点返回起点
            # 检查是否已到达起点
            dist_to_start = math.sqrt((self.robot.x - self.maze_env.start_pos[0])**2 + 
                                     (self.robot.y - self.maze_env.start_pos[1])**2)
            if dist_to_start <= 1.0:
                print("\n===== 任务完成：已返回起点！=====\n")
                self.returned_to_start = True
                self.current_state = "completed"
                visualization.exploration_paused = True
                return
            
            # 检查是否有有效路径
            if not self.robot.goal_path or len(self.robot.goal_path) < 2:
                print("警告: 返回起点的路径无效或为空，尝试重新规划")
                if self.robot.find_path_to_goal(self.maze_env.start_pos):
                    print(f"重新规划成功，路径长度: {len(self.robot.goal_path)}")
                else:
                    print("重新规划失败，放弃导航")
                    self.current_state = "completed"
                    visualization.exploration_paused = True
                    return
                
            # 沿着规划的路径导航
            print(f"当前位置: ({self.robot.x:.1f}, {self.robot.y:.1f}), 下一个路径点: {self.robot.goal_path[1] if len(self.robot.goal_path) > 1 else '无'}")
            if not self.robot.navigate_to_goal():
                print("导航失败，重新规划路径...")
                self.navigation_retry_count += 1
                
                if self.navigation_retry_count >= self.max_navigation_retries:
                    print(f"\n===== 导航失败次数达到最大值({self.max_navigation_retries})，放弃导航 =====\n")
                    self.current_state = "completed"
                    visualization.exploration_paused = True
                    return
                
                if self.robot.find_path_to_goal(self.maze_env.start_pos):
                    print("重新规划路径成功")
                else:
                    print("无法找到到起点的路径，探索结束")
                    self.current_state = "completed"
                    visualization.exploration_paused = True
                
        elif self.current_state == "completed":
            # 探索完成，不再更新
            print("\n===== 任务完成 =====\n")
            visualization.exploration_paused = True
            return
        
        # 更新可视化
        visualization.current_state = self.current_state
        visualization.update_visualization()
        
    def run(self):
        """运行控制器"""
        # 创建可视化
        visualization = self.visualizer
        
        # 启动动画
        if hasattr(visualization, 'start_animation'):
            visualization.start_animation()
        else:
            visualization.run_animation(self)

    def run_exploration(self):
        """运行迷宫探索"""
        # 初始化可视化
        self.visualizer.initialize()
        
        # 设置状态
        self.state = "exploring"
        
        # 开始探索
        while True:
            # 更新可视化
            self.visualizer.update(self.robot, self.env)
            
            # 根据当前状态执行不同的操作
            if self.state == "exploring":
                # 探索阶段：机器人探索迷宫，寻找起点和终点
                if not self.robot.explore_maze():
                    print("探索完成，进入规划阶段")
                    self.state = "planning_to_goal"
                    
                    # 更新探索完成后的地图
                    self.visualizer.update(self.robot, self.env)
                    
            elif self.state == "planning_to_goal":
                # 规划阶段：规划从当前位置到终点的路径
                if self.env.goal is not None:
                    print(f"规划到终点 {self.env.goal} 的路径")
                    goal_path = self.robot.find_path_to_goal(self.env.goal)
                    
                    if goal_path:
                        print(f"找到到终点的路径，长度: {len(goal_path)}")
                        self.robot.goal_path = goal_path[1:]  # 跳过起点
                        self.state = "moving_to_goal"
                    else:
                        print("无法找到到终点的路径，尝试继续探索")
                        self.state = "exploring"
                else:
                    print("终点未知，继续探索")
                    self.state = "exploring"
                    
            elif self.state == "moving_to_goal":
                # 移动阶段：机器人沿着规划的路径移动到终点
                if self.robot.goal_path:
                    # 获取路径上的下一个点
                    next_point = self.robot.goal_path[0]
                    self.robot.goal_path.pop(0)
                    
                    print(f"移动到终点路径的下一个点 {next_point}")
                    
                    # 更新位置
                    self.robot.update_position((next_point[0], next_point[1], self.robot.theta))
                else:
                    # 到达终点
                    current_pos = (int(round(self.robot.x)), int(round(self.robot.y)))
                    goal_pos = (int(round(self.env.goal[0])), int(round(self.env.goal[1])))
                    
                    if current_pos == goal_pos:
                        print("已到达终点！")
                        self.state = "planning_to_start"
                    else:
                        print("路径执行完毕但未到达终点，重新规划")
                        self.state = "planning_to_goal"
                        
            elif self.state == "planning_to_start":
                # 规划阶段：规划从终点回到起点的路径
                if self.env.start is not None:
                    print(f"规划回到起点 {self.env.start} 的路径")
                    
                    # 使用A*算法直接规划从当前位置到起点的最优路径
                    current_pos = (int(round(self.robot.x)), int(round(self.robot.y)))
                    start_pos = (int(round(self.env.start[0])), int(round(self.env.start[1])))
                    
                    # 使用已知的地图信息规划最短路径
                    start_path = self.robot.plan_path(current_pos, start_pos)
                    
                    if start_path:
                        print(f"找到回到起点的路径，长度: {len(start_path)}")
                        self.robot.goal_path = start_path[1:]  # 跳过起点
                        self.state = "moving_to_start"
                    else:
                        print("无法找到回到起点的路径")
                        # 尝试随机移动
                        self.robot.random_move_count = 3
                        self.state = "exploring"  # 回到探索状态
                else:
                    print("起点未知，继续探索")
                    self.state = "exploring"
                    
            elif self.state == "moving_to_start":
                # 移动阶段：机器人沿着规划的路径移动回起点
                if self.robot.goal_path:
                    # 获取路径上的下一个点
                    next_point = self.robot.goal_path[0]
                    self.robot.goal_path.pop(0)
                    
                    print(f"移动到起点路径的下一个点 {next_point}")
                    
                    # 更新位置
                    self.robot.update_position((next_point[0], next_point[1], self.robot.theta))
                else:
                    # 到达起点
                    current_pos = (int(round(self.robot.x)), int(round(self.robot.y)))
                    start_pos = (int(round(self.env.start[0])), int(round(self.env.start[1])))
                    
                    if current_pos == start_pos:
                        print("已回到起点！任务完成！")
                        self.state = "finished"
                    else:
                        print("路径执行完毕但未回到起点，重新规划")
                        self.state = "planning_to_start"
                        
            elif self.state == "finished":
                # 完成状态：任务完成
                print("迷宫探索任务完成！")
                break
                
            # 更新探索进度
            exploration_progress = self.robot.exploration_progress
            print(f"探索进度: {exploration_progress:.2f}%")
            
            # 暂停一下，以便观察
            time.sleep(0.1)

def main():
    """主函数"""
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='迷宫探索与路径规划')
    parser.add_argument('--json', type=str, help='JSON迷宫文件路径，如果不提供则使用随机生成的迷宫')
    parser.add_argument('--maze-id', type=int, choices=[1, 2, 3], help='使用预定义的迷宫ID (1, 2, 3)')
    parser.add_argument('--width', type=int, default=15, help='迷宫宽度，默认15')
    parser.add_argument('--height', type=int, default=15, help='迷宫高度，默认15')
    args = parser.parse_args()
    
    # 确定迷宫文件路径
    json_file = None
    
    if args.maze_id:
        # 获取当前文件所在目录
        current_dir = os.path.dirname(os.path.abspath(__file__))
        json_file = os.path.join(current_dir, 'json_data', f'{args.maze_id}.json')
        print(f"使用预定义迷宫 {args.maze_id}: {json_file}")
        
        # 根据迷宫ID调整宽度和高度
        if args.maze_id == 2:
            args.width = 16
            args.height = 16
            print(f"将迷宫2的大小调整为 {args.width}x{args.height}")
        elif args.maze_id == 3:
            args.width = 21
            args.height = 21
            print(f"将迷宫3的大小调整为 {args.width}x{args.height}")
    elif args.json:
        json_file = args.json
        print(f"使用自定义迷宫文件: {json_file}")
    
    # 创建迷宫环境
    maze_env = MazeEnvironment(width=args.width, height=args.height, json_file=json_file)
    
    # 输出环境信息
    print(f"机器人初始化在位置 ({maze_env.start_pos[0]}, {maze_env.start_pos[1]})")
    print(f"环境大小: {maze_env.width}x{maze_env.height}")
    print(f"目标位置: ({maze_env.goal_pos[0]}, {maze_env.goal_pos[1]})")
    print(f"可访问单元格总数: {len([1 for x in range(maze_env.width) for y in range(maze_env.height) if (x, y) not in maze_env.grid_env.obstacles])}")
    
    # 创建并运行控制器
    controller = MazeExplorationController(maze_env)
    controller.run()

if __name__ == "__main__":
    main() 