#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
迷宫探索与路径规划主程序
"""

import time
import math
import sys
import os

# 导入自定义模块
from maze_env import MazeEnvironment
from maze_robot import Robot
from maze_visualization import MazeVisualization

class MazeExploration:
    """迷宫测绘与导航系统"""
    def __init__(self):
        # 创建环境
        self.maze_env = MazeEnvironment()
        
        # 创建机器人
        self.robot = Robot(self.maze_env.start_pos, self.maze_env.grid_env)
        
        # 设置状态
        self.exploration_complete = False
        self.path_planning_complete = False
        self.goal_found = False
        self.current_state = "exploration"  # 当前状态：exploration, path_planning, navigation, completed
        
        # 添加目标检测距离
        self.goal_detection_distance = 3  # 当机器人靠近目标3个单位时，视为找到目标
        
        # 探索完成度阈值
        self.exploration_threshold = 0.70  # 探索70%的区域认为完成
        
        # 计数器
        self.step_count = 0
        
    def update(self, visualization):
        """更新一帧"""
        # 如果动画已暂停，不进行更新
        if visualization.exploration_paused:
            return
        
        # 控制帧率
        current_time = time.time()
        if current_time - visualization.last_update_time < visualization.update_interval:
            return
        visualization.last_update_time = current_time
        
        # 更新步数
        self.step_count += 1
        
        # 根据当前状态执行不同的操作
        if self.current_state == "exploration":
            # 探索阶段
            if not self.robot.explore_maze():
                print("探索完成！")
                self.exploration_complete = True
                self.current_state = "path_planning"
                
            # 显示探索进度
            if self.step_count % 10 == 0:
                print(f"探索进度: {self.robot.exploration_progress:.2f}%")
            
            # 检查是否已经找到目标
            dist_to_goal = math.sqrt((self.robot.x - self.maze_env.goal_pos[0])**2 + 
                                     (self.robot.y - self.maze_env.goal_pos[1])**2)
            if dist_to_goal <= self.goal_detection_distance:
                print("找到目标点！")
                self.goal_found = True
                self.current_state = "path_planning"
                
            # 检查是否达到探索阈值
            if self.robot.exploration_progress >= self.exploration_threshold * 100 and not self.exploration_complete:
                print(f"达到探索阈值: {self.robot.exploration_progress:.2f}% 的地图已探索")
                self.exploration_complete = True
                self.current_state = "path_planning"
                
        elif self.current_state == "path_planning":
            # 路径规划阶段
            print("规划到目标的路径...")
            
            # 使用A*算法规划从当前位置到目标位置的路径
            if self.robot.find_path_to_goal(self.maze_env.goal_pos):
                self.path_planning_complete = True
                self.current_state = "navigation"
            else:
                print("无法找到到目标的路径，继续探索...")
                self.current_state = "exploration"
                
        elif self.current_state == "navigation":
            # 导航阶段
            # 检查是否已到达目标
            dist_to_goal = math.sqrt((self.robot.x - self.maze_env.goal_pos[0])**2 + 
                                     (self.robot.y - self.maze_env.goal_pos[1])**2)
            if dist_to_goal <= 1.0:
                print("到达目标点！")
                self.current_state = "completed"
                visualization.exploration_paused = True
                return
                
            # 沿着规划的路径导航
            if not self.robot.navigate_to_goal():
                print("导航失败，重新规划路径...")
                self.current_state = "path_planning"
                
        elif self.current_state == "completed":
            # 探索完成，不再更新
            pass
            
        # 更新可视化
        visualization.current_state = self.current_state
        visualization.update_visualization()
        
    def run(self):
        """运行迷宫测绘与导航"""
        # 创建可视化
        visualization = MazeVisualization(self.maze_env, self.robot)
        
        # 运行动画
        visualization.run_animation(self)

if __name__ == '__main__':
    maze_exploration = MazeExploration()
    maze_exploration.run() 