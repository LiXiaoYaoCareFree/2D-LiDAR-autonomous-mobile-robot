#!/usr/bin/env python3
'''
maze_solver.py - 迷宫求解器模块

该模块实现了迷宫探索和解决策略，包括探索未知区域、检测出口、规划最优路径等功能。
'''

import numpy as np
import time
from enum import Enum
from modules.advanced_navigation import InfoGainFrontierExploration, OptimalPathPlanner

class MazeState(Enum):
    """迷宫求解状态枚举"""
    EXPLORING = 0       # 探索阶段
    EXIT_FOUND = 1      # 已找到出口
    NAVIGATING_EXIT = 2 # 正在前往出口
    RETURNING = 3       # 正在返回起点
    COMPLETED = 4       # 任务完成

class MazeSolver:
    """迷宫求解器类，实现迷宫探索和解决策略"""
    
    def __init__(self, map_size_pixels=800, map_size_meters=32):
        """
        初始化迷宫求解器
        
        参数:
            map_size_pixels: 地图大小(像素)
            map_size_meters: 地图大小(米)
        """
        # 初始化导航组件
        self.explorer = InfoGainFrontierExploration(map_size_pixels, map_size_meters)
        self.path_planner = OptimalPathPlanner(map_size_pixels, map_size_meters)
        
        # 迷宫状态
        self.state = MazeState.EXPLORING
        
        # 位置记录
        self.start_position = None  # 起始位置 (x, y)，单位为米
        self.exit_position = None   # 出口位置 (x, y)，单位为米
        
        # 路径规划
        self.current_path = []      # 当前规划的路径
        self.current_target = None  # 当前导航目标
        
        # 性能统计
        self.exploration_start_time = None
        self.exit_found_time = None
        self.exit_reached_time = None
        self.return_start_time = None
        self.completion_time = None
        
        # 调试信息
        self.debug_info = {
            'frontiers_detected': [],  # 探测到的边界点列表
            'selected_frontiers': [],  # 选择的边界点列表
            'path_history': []         # 路径历史
        }
    
    def initialize(self, start_pose):
        """
        初始化迷宫求解器，记录起始位置
        
        参数:
            start_pose: 起始位置，(x, y, theta)，单位为米和度
        """
        self.start_position = (start_pose[0], start_pose[1])
        self.state = MazeState.EXPLORING
        self.exploration_start_time = time.time()
        
        print(f"迷宫求解器初始化，起始位置: ({self.start_position[0]:.2f}, {self.start_position[1]:.2f})")
    
    def update(self, occupancy_grid, current_pose):
        """
        更新迷宫求解器状态
        
        参数:
            occupancy_grid: 占用栅格地图，numpy数组
            current_pose: 当前位置，(x, y, theta)，单位为米和度
            
        返回:
            (target_pose, is_complete): 目标位置和是否完成任务
        """
        # 根据当前状态执行不同的操作
        if self.state == MazeState.EXPLORING:
            return self._handle_exploring(occupancy_grid, current_pose)
        
        elif self.state == MazeState.EXIT_FOUND:
            return self._handle_exit_found(occupancy_grid, current_pose)
        
        elif self.state == MazeState.NAVIGATING_EXIT:
            return self._handle_navigating_exit(occupancy_grid, current_pose)
        
        elif self.state == MazeState.RETURNING:
            return self._handle_returning(occupancy_grid, current_pose)
        
        elif self.state == MazeState.COMPLETED:
            return None, True
        
        return None, False
    
    def _handle_exploring(self, occupancy_grid, current_pose):
        """处理探索状态"""
        # 检查是否找到出口
        exit_found, exit_position = self.explorer.detect_exit(occupancy_grid, current_pose)
        
        if exit_found:
            self.exit_position = exit_position
            self.state = MazeState.EXIT_FOUND
            self.exit_found_time = time.time()
            
            exploration_time = self.exit_found_time - self.exploration_start_time
            print(f"出口已找到! 位置: ({exit_position[0]:.2f}, {exit_position[1]:.2f}), 探索耗时: {exploration_time:.2f}秒")
            
            # 转到处理找到出口的状态
            return self._handle_exit_found(occupancy_grid, current_pose)
        
        # 检查是否已完成探索（如果未找到出口但已探索完毕）
        if self.explorer.is_exploration_complete(occupancy_grid):
            print("迷宫探索完成，但未找到出口。返回起点...")
            self.state = MazeState.RETURNING
            self.return_start_time = time.time()
            
            # 转到处理返回的状态
            return self._handle_returning(occupancy_grid, current_pose)
        
        # 继续探索
        # 检测边界点
        frontiers = self.explorer.detect_frontiers(occupancy_grid)
        self.debug_info['frontiers_detected'] = frontiers
        
        if not frontiers:
            print("未检测到边界点，返回起点...")
            self.state = MazeState.RETURNING
            self.return_start_time = time.time()
            return self._handle_returning(occupancy_grid, current_pose)
        
        # 选择最佳边界点
        best_frontier = self.explorer.select_best_frontier(frontiers, current_pose)
        self.debug_info['selected_frontiers'].append(best_frontier)
        
        if best_frontier is None:
            print("无法选择边界点，返回起点...")
            self.state = MazeState.RETURNING
            self.return_start_time = time.time()
            return self._handle_returning(occupancy_grid, current_pose)
        
        # 规划到边界点的路径
        path = self.path_planner.plan_path(occupancy_grid, current_pose, best_frontier)
        
        if not path:
            print("无法规划到边界点的路径，尝试下一个边界点...")
            # 从列表中移除当前边界点
            frontiers = [f for f in frontiers if f != best_frontier]
            if frontiers:
                best_frontier = self.explorer.select_best_frontier(frontiers, current_pose)
                path = self.path_planner.plan_path(occupancy_grid, current_pose, best_frontier)
            
            if not path:
                print("无法规划到任何边界点的路径，返回起点...")
                self.state = MazeState.RETURNING
                self.return_start_time = time.time()
                return self._handle_returning(occupancy_grid, current_pose)
        
        # 更新当前路径和目标
        self.current_path = path
        self.current_target = best_frontier
        self.debug_info['path_history'].append(path)
        
        # 返回下一个导航目标
        if len(path) > 1:
            next_point = path[1]  # 第一个点是当前位置
            # 计算朝向角度
            dx = next_point[0] - current_pose[0]
            dy = next_point[1] - current_pose[1]
            theta = np.arctan2(dy, dx) * 180 / np.pi
            
            return (next_point[0], next_point[1], theta), False
        else:
            # 如果路径只有一个点，直接返回当前位置
            return (current_pose[0], current_pose[1], current_pose[2]), False
    
    def _handle_exit_found(self, occupancy_grid, current_pose):
        """处理找到出口的状态"""
        # 规划到出口的最优路径
        optimal_path = self.path_planner.plan_optimal_path(occupancy_grid, current_pose, self.exit_position)
        
        if not optimal_path:
            print("无法规划到出口的路径，继续探索...")
            self.state = MazeState.EXPLORING
            return self._handle_exploring(occupancy_grid, current_pose)
        
        # 更新当前路径和目标
        self.current_path = optimal_path
        self.current_target = self.exit_position
        self.state = MazeState.NAVIGATING_EXIT
        
        print(f"已规划到出口的路径，路径长度: {len(optimal_path)}点")
        
        # 返回下一个导航目标
        if len(optimal_path) > 1:
            next_point = optimal_path[1]  # 第一个点是当前位置
            # 计算朝向角度
            dx = next_point[0] - current_pose[0]
            dy = next_point[1] - current_pose[1]
            theta = np.arctan2(dy, dx) * 180 / np.pi
            
            return (next_point[0], next_point[1], theta), False
        else:
            # 如果路径只有一个点，直接返回当前位置
            return (current_pose[0], current_pose[1], current_pose[2]), False
    
    def _handle_navigating_exit(self, occupancy_grid, current_pose):
        """处理前往出口的状态"""
        # 检查是否已到达出口
        current_x, current_y, _ = current_pose
        exit_x, exit_y = self.exit_position
        
        distance_to_exit = np.sqrt((current_x - exit_x)**2 + (current_y - exit_y)**2)
        
        if distance_to_exit < 0.5:  # 如果距离出口小于0.5米，认为已到达
            print("已到达出口!")
            self.state = MazeState.RETURNING
            self.exit_reached_time = time.time()
            
            exit_navigation_time = self.exit_reached_time - self.exit_found_time
            print(f"到达出口耗时: {exit_navigation_time:.2f}秒")
            
            # 转到处理返回的状态
            return self._handle_returning(occupancy_grid, current_pose)
        
        # 更新路径规划（如有必要）
        if len(self.current_path) <= 1:
            # 重新规划路径
            optimal_path = self.path_planner.plan_optimal_path(occupancy_grid, current_pose, self.exit_position)
            
            if not optimal_path:
                print("无法重新规划到出口的路径，继续探索...")
                self.state = MazeState.EXPLORING
                return self._handle_exploring(occupancy_grid, current_pose)
            
            self.current_path = optimal_path
        
        # 返回下一个导航目标
        if len(self.current_path) > 1:
            next_point = self.current_path[1]  # 第一个点是当前位置
            # 从路径中移除已访问的点
            self.current_path = self.current_path[1:]
            
            # 计算朝向角度
            dx = next_point[0] - current_pose[0]
            dy = next_point[1] - current_pose[1]
            theta = np.arctan2(dy, dx) * 180 / np.pi
            
            return (next_point[0], next_point[1], theta), False
        else:
            # 如果路径只有一个点，直接返回当前位置
            return (current_pose[0], current_pose[1], current_pose[2]), False
    
    def _handle_returning(self, occupancy_grid, current_pose):
        """处理返回起点的状态"""
        # 检查是否已返回起点
        current_x, current_y, _ = current_pose
        start_x, start_y = self.start_position
        
        distance_to_start = np.sqrt((current_x - start_x)**2 + (current_y - start_y)**2)
        
        if distance_to_start < 0.5:  # 如果距离起点小于0.5米，认为已返回
            print("已返回起点!")
            self.state = MazeState.COMPLETED
            self.completion_time = time.time()
            
            total_time = self.completion_time - self.exploration_start_time
            print(f"任务完成! 总耗时: {total_time:.2f}秒")
            
            # 返回任务完成状态
            return None, True
        
        # 规划返回起点的最短路径
        return_path = self.path_planner.find_shortest_return_path(occupancy_grid, current_pose, self.start_position)
        
        if not return_path:
            print("无法规划返回起点的路径，尝试继续探索...")
            self.state = MazeState.EXPLORING
            return self._handle_exploring(occupancy_grid, current_pose)
        
        # 更新当前路径和目标
        self.current_path = return_path
        self.current_target = self.start_position
        
        # 返回下一个导航目标
        if len(return_path) > 1:
            next_point = return_path[1]  # 第一个点是当前位置
            # 计算朝向角度
            dx = next_point[0] - current_pose[0]
            dy = next_point[1] - current_pose[1]
            theta = np.arctan2(dy, dx) * 180 / np.pi
            
            return (next_point[0], next_point[1], theta), False
        else:
            # 如果路径只有一个点，直接返回当前位置
            return (current_pose[0], current_pose[1], current_pose[2]), False
    
    def get_state_name(self):
        """获取当前状态的名称"""
        return self.state.name
    
    def get_performance_stats(self):
        """获取性能统计信息"""
        stats = {
            'state': self.state.name,
            'exploration_time': None,
            'exit_navigation_time': None,
            'return_time': None,
            'total_time': None
        }
        
        current_time = time.time()
        
        if self.exploration_start_time:
            if self.exit_found_time:
                stats['exploration_time'] = self.exit_found_time - self.exploration_start_time
            else:
                stats['exploration_time'] = current_time - self.exploration_start_time
        
        if self.exit_found_time:
            if self.exit_reached_time:
                stats['exit_navigation_time'] = self.exit_reached_time - self.exit_found_time
            else:
                stats['exit_navigation_time'] = current_time - self.exit_found_time
        
        if self.exit_reached_time:
            if self.completion_time:
                stats['return_time'] = self.completion_time - self.exit_reached_time
            else:
                stats['return_time'] = current_time - self.exit_reached_time
        
        if self.exploration_start_time:
            if self.completion_time:
                stats['total_time'] = self.completion_time - self.exploration_start_time
            else:
                stats['total_time'] = current_time - self.exploration_start_time
        
        return stats 