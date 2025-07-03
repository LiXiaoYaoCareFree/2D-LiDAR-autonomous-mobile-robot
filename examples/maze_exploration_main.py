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
import matplotlib.animation as animation

# 导入自定义模块
from maze_env import MazeEnvironment
from maze_robot import Robot
from maze_visualization import MazeVisualization

class MazeSLAMVisualizer:
    """迷宫SLAM可视化类，独立于主程序逻辑"""
    
    def __init__(self, maze_env, robot):
        """初始化SLAM可视化器
        
        参数:
            maze_env: 迷宫环境
            robot: 机器人对象
        """
        self.maze_env = maze_env
        self.robot = robot
        
        # SLAM地图相关
        self.map_resolution = 0.1  # 地图分辨率，每个格子代表0.1个单位
        self.map_size = int(max(maze_env.width, maze_env.height) / self.map_resolution) + 20  # 地图大小，加上边界
        self.slam_map = np.zeros((self.map_size, self.map_size))  # 0=未知，1=空闲，2=障碍物
        
        # 机器人路径
        self.robot_path = []
        
        # 激光数据
        self.scan_points = []
        self.obstacle_points = []
        
        # 创建图形和子图
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.fig.suptitle('迷宫SLAM可视化')
        
        # 设置坐标轴
        self.ax.set_xlim(0, self.maze_env.width)
        self.ax.set_ylim(0, self.maze_env.height)
        self.ax.set_title("迷宫SLAM地图")
        self.ax.set_aspect('equal')
        
        # 初始化地图图像
        self.map_image = self.ax.imshow(
            self.slam_map, 
            cmap=ListedColormap(['lightgray', 'white', 'black']), 
            origin='lower',
            extent=[0, self.map_size * self.map_resolution, 0, self.map_size * self.map_resolution],
            alpha=0.7
        )
        
        # 初始化机器人位置标记
        self.robot_marker, = self.ax.plot([], [], 'bo', markersize=8)
        
        # 初始化机器人路径
        self.path_line, = self.ax.plot([], [], 'b-', linewidth=1, alpha=0.6)
        
        # 初始化激光线
        self.laser_lines = []
        
        # 初始化文本信息
        self.info_text = self.ax.text(
            0.02, 0.98, '', transform=self.ax.transAxes,
            verticalalignment='top', fontsize=10,
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.7)
        )
        
        # 使用定时器更新，而不是FuncAnimation
        self.timer = self.fig.canvas.new_timer(interval=100)
        self.timer.add_callback(self.update_callback)
        self.timer.start()
        
        # 显示图形
        plt.ion()  # 开启交互模式
        plt.show(block=False)
    
    def update_callback(self):
        """定时器回调函数，更新SLAM地图"""
        try:
            # 检查图形是否仍然存在
            if plt.fignum_exists(self.fig.number):
                self.update()
                self.fig.canvas.draw_idle()
                self.fig.canvas.flush_events()
        except Exception as e:
            print(f"SLAM更新错误: {e}")
            # 出错时停止定时器
            if hasattr(self, 'timer'):
                self.timer.stop()
        
    def update(self):
        """更新SLAM地图可视化
        """
        # 更新机器人路径
        self.robot_path.append((self.robot.x, self.robot.y))
        if len(self.robot_path) > 1000:  # 限制路径长度
            self.robot_path = self.robot_path[-1000:]
        
        # 更新SLAM地图
        self.update_slam_map()
        
        # 更新地图图像
        self.map_image.set_data(self.slam_map)
        
        # 更新机器人位置
        self.robot_marker.set_data([self.robot.x], [self.robot.y])
        
        # 更新机器人路径
        if self.robot_path:
            path_x = [p[0] for p in self.robot_path]
            path_y = [p[1] for p in self.robot_path]
            self.path_line.set_data(path_x, path_y)
        
        # 清除旧的激光线
        for line in self.laser_lines:
            line.remove()
        self.laser_lines = []
        
        # 绘制新的激光线
        if hasattr(self.robot, 'sensor_data') and self.robot.sensor_data:
            for point in self.robot.sensor_data:
                line, = self.ax.plot(
                    [self.robot.x, point[0]], [self.robot.y, point[1]],
                    'y-', alpha=0.3
                )
                self.laser_lines.append(line)
        
        # 更新信息文本
        status_text = f"位置: ({self.robot.x:.1f}, {self.robot.y:.1f})\n"
        if hasattr(self.robot, 'exploration_progress'):
            status_text += f"探索进度: {self.robot.exploration_progress:.1f}%\n"
        if hasattr(self.maze_env, 'goal_pos'):
            dist = math.sqrt((self.robot.x - self.maze_env.goal_pos[0])**2 + 
                           (self.robot.y - self.maze_env.goal_pos[1])**2)
            status_text += f"距离目标: {dist:.1f}"
        self.info_text.set_text(status_text)
        
    def update_slam_map(self):
        """更新SLAM地图"""
        # 获取机器人当前位置
        robot_pos = (self.robot.x, self.robot.y)
        
        # 确保机器人已扫描环境
        self.robot.scan_environment()
        scan_data = self.robot.sensor_data
        
        # 将机器人位置转换为地图坐标
        robot_map_x, robot_map_y = self.world_to_map(robot_pos[0], robot_pos[1])
        
        # 将机器人位置周围标记为空闲区域
        view_range = int(1.5 / self.map_resolution)  # 视野范围
        for dx in range(-view_range, view_range + 1):
            for dy in range(-view_range, view_range + 1):
                mx, my = robot_map_x + dx, robot_map_y + dy
                if 0 <= mx < self.map_size and 0 <= my < self.map_size:
                    # 计算到机器人的距离
                    dist = math.sqrt(dx**2 + dy**2) * self.map_resolution
                    if dist <= 1.0:  # 只标记1.0单位范围内的区域
                        self.slam_map[my, mx] = 1  # 标记为空闲区域
        
        # 处理激光扫描数据
        for point in scan_data:
            # 将扫描点转换为地图坐标
            scan_map_x, scan_map_y = self.world_to_map(point[0], point[1])
            
            # 检查是否在地图范围内
            if 0 <= scan_map_x < self.map_size and 0 <= scan_map_y < self.map_size:
                # 标记扫描点为障碍物
                self.slam_map[scan_map_y, scan_map_x] = 2
                
                # 使用Bresenham算法绘制从机器人到扫描点的线
                line_points = self.bresenham_line(robot_map_x, robot_map_y, scan_map_x, scan_map_y)
                
                # 标记线上的点为空闲区域，除了最后一个点（障碍物）
                for i, (mx, my) in enumerate(line_points[:-1]):
                    if 0 <= mx < self.map_size and 0 <= my < self.map_size:
                        self.slam_map[my, mx] = 1  # 标记为空闲区域
    
    def world_to_map(self, world_x, world_y):
        """将世界坐标转换为地图坐标"""
        # 添加偏移量，确保负坐标也能映射到地图上
        offset = 10
        map_x = int((world_x + offset) / self.map_resolution)
        map_y = int((world_y + offset) / self.map_resolution)
        return map_x, map_y
    
    def map_to_world(self, map_x, map_y):
        """将地图坐标转换为世界坐标"""
        offset = 10
        world_x = map_x * self.map_resolution - offset
        world_y = map_y * self.map_resolution - offset
        return world_x, world_y
    
    def bresenham_line(self, x0, y0, x1, y1):
        """Bresenham算法，获取从(x0,y0)到(x1,y1)的线上的所有点"""
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
    
    def close(self):
        """关闭可视化窗口"""
        # 停止定时器
        if hasattr(self, 'timer'):
            self.timer.stop()
        plt.close(self.fig)

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
        
        # 设置状态
        self.exploration_complete = False
        self.path_planning_complete = False
        self.goal_found = False
        self.current_state = "exploration"  # 当前状态：exploration, path_planning, navigation, completed
        
        # 添加目标检测距离
        self.goal_detection_distance = 3  # 当机器人靠近目标3个单位时，视为找到目标
        
        # 探索完成度阈值
        self.exploration_threshold = 0.99  # 设置为85%，确保合理的遍历但不过高
        
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
    parser.add_argument('--slam', action='store_true', help='启用SLAM可视化')
    args = parser.parse_args()
    
    # 确定迷宫文件路径
    json_file = None
    
    if args.maze_id:
        # 获取当前文件所在目录
        current_dir = os.path.dirname(os.path.abspath(__file__))
        json_file = os.path.join(current_dir, 'json_data', f'{args.maze_id}.json')
        print(f"使用预定义迷宫 {args.maze_id}: {json_file}")
        
        # 根据迷宫ID调整宽度和高度
        if args.maze_id == 3:
            args.width = 21
            args.height = 21
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
    
    # 如果启用SLAM可视化，创建SLAM可视化器
    slam_visualizer = None
    if args.slam:
        slam_visualizer = MazeSLAMVisualizer(maze_env, controller.robot)
    
    try:
        # 运行控制器
        controller.run()
    finally:
        # 如果SLAM可视化器存在，保持窗口打开直到用户关闭
        if slam_visualizer:
            # 停止定时器更新，但保持窗口打开
            slam_visualizer.timer.stop()
            plt.ioff()
            plt.show(block=True)
            # 最后关闭可视化器
            slam_visualizer.close()

if __name__ == "__main__":
    main() 