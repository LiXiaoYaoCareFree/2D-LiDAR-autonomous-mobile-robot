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
        
        # 创建可视化
        self.visualizer = MazeVisualization(self.maze_env, self.robot)
        
        # 设置状态
        self.exploration_complete = False
        self.path_planning_complete = False
        self.goal_found = False
        self.current_state = "exploration"  # 当前状态：exploration, path_planning, navigation, completed
        
        # 添加目标检测距离
        self.goal_detection_distance = 3  # 当机器人靠近目标3个单位时，视为找到目标
        
        # 探索完成度阈值
        self.exploration_threshold = 0.95  # 提高探索阈值到95%，确保更完整的遍历
        
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
        self.max_navigation_retries = 10  # 最大重试次数
        
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
        
        # 根据当前状态执行不同的操作
        if self.current_state == "exploration":
            # 探索阶段 - 完整遍历迷宫
            if not self.robot.explore_maze():
                print("迷宫遍历完成！")
                self.exploration_complete = True
                
                # 如果已经找到目标但还没到达，现在导航到目标
                if self.goal_found and not self.reached_goal:
                    print("导航到目标点...")
                    self.current_state = "navigate_to_goal"
                    if self.robot.find_path_to_goal(self.maze_env.goal_pos):
                        print(f"找到到目标点的路径，长度: {len(self.robot.goal_path)}")
                    else:
                        print("无法找到到目标点的路径，探索结束")
                        self.current_state = "completed"
                else:
                    # 如果没找到目标，继续搜索
                    print("尝试寻找目标点...")
                    self.current_state = "search_goal"
            
            # 显示探索进度
            if self.step_count % 10 == 0:
                print(f"探索进度: {self.robot.exploration_progress:.2f}%")
            
            # 检查是否已经找到目标
            dist_to_goal = math.sqrt((self.robot.x - self.maze_env.goal_pos[0])**2 + 
                                     (self.robot.y - self.maze_env.goal_pos[1])**2)
            if dist_to_goal <= self.goal_detection_distance:
                print("找到目标点！")
                self.goal_found = True
                
            # 检查是否达到探索阈值
            if self.robot.exploration_progress >= self.exploration_threshold * 100 and not self.exploration_complete:
                print(f"达到探索阈值: {self.robot.exploration_progress:.2f}% 的地图已探索")
                self.exploration_complete = True
                
                # 如果已经找到目标但还没到达，现在导航到目标
                if self.goal_found and not self.reached_goal:
                    print("导航到目标点...")
                    self.current_state = "navigate_to_goal"
                    if self.robot.find_path_to_goal(self.maze_env.goal_pos):
                        print(f"找到到目标点的路径，长度: {len(self.robot.goal_path)}")
                    else:
                        print("无法找到到目标点的路径，探索结束")
                        self.current_state = "completed"
                else:
                    # 如果没找到目标，继续搜索
                    print("尝试寻找目标点...")
                    self.current_state = "search_goal"
                
        elif self.current_state == "search_goal":
            # 搜索目标点阶段
            if not self.goal_found:
                # 尝试在地图中寻找目标点
                print("在地图中搜索目标点...")
                
                # 使用A*算法规划从当前位置到目标位置的路径
                if self.robot.find_path_to_goal(self.maze_env.goal_pos):
                    self.goal_found = True
                    print("找到目标点路径！")
                    self.current_state = "navigate_to_goal"
                else:
                    print("无法找到目标点，探索结束")
                    self.current_state = "completed"
            else:
                # 已经找到目标点，导航过去
                self.current_state = "navigate_to_goal"
                
        elif self.current_state == "navigate_to_goal":
            # 导航到目标点阶段
            # 检查是否已到达目标
            dist_to_goal = math.sqrt((self.robot.x - self.maze_env.goal_pos[0])**2 + 
                                     (self.robot.y - self.maze_env.goal_pos[1])**2)
            if dist_to_goal <= 1.0:
                print("到达目标点！")
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
                    print(f"导航失败次数达到最大值({self.max_navigation_retries})，放弃导航")
                    # 直接进入下一阶段
                    self.reached_goal = True
                    self.current_state = "path_planning"
                    self.navigation_retry_count = 0
                    return
                
                if self.robot.find_path_to_goal(self.maze_env.goal_pos):
                    print("重新规划路径成功")
                else:
                    print("无法找到到目标的路径，探索结束")
                    self.current_state = "completed"
                
        elif self.current_state == "path_planning":
            # 路径规划阶段 - 从终点返回起点
            print("规划从终点返回起点的路径...")
            
            # 使用A*算法规划从当前位置(终点)到起点的路径
            if self.robot.find_path_to_goal(self.maze_env.start_pos):
                self.path_planning_complete = True
                print(f"找到返回起点的路径，长度: {len(self.robot.goal_path)}")
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
                print("已返回起点！任务完成！")
                self.returned_to_start = True
                self.current_state = "completed"
                visualization.exploration_paused = True
                return
                
            # 沿着规划的路径导航
            if not self.robot.navigate_to_goal():
                print("导航失败，重新规划路径...")
                self.navigation_retry_count += 1
                
                if self.navigation_retry_count >= self.max_navigation_retries:
                    print(f"导航失败次数达到最大值({self.max_navigation_retries})，放弃导航")
                    # 直接完成任务
                    self.returned_to_start = True
                    self.current_state = "completed"
                    visualization.exploration_paused = True
                    return
                
                if self.robot.find_path_to_goal(self.maze_env.start_pos):
                    print("重新规划路径成功")
                else:
                    print("无法找到返回起点的路径，探索结束")
                    self.current_state = "completed"
                
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

if __name__ == '__main__':
    maze_exploration = MazeExploration()
    maze_exploration.run() 