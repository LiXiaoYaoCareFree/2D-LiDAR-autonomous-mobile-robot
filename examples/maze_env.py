#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
迷宫环境模块
包含迷宫环境的创建和初始化
"""

import numpy as np
import random
from python_motion_planning.utils.environment.env import Grid

class MazeEnvironment:
    """迷宫环境类"""
    def __init__(self, width=15, height=15):
        """初始化迷宫环境
        
        Args:
            width: 迷宫宽度，默认15
            height: 迷宫高度，默认15
        """
        # 确保迷宫大小不超过16x16
        self.width = min(width, 15)
        self.height = min(height, 15)
        self.grid_env = self.create_grid_env()
        self.start_pos = (1, 1, 0)  # (x, y, theta)
        self.goal_pos = (self.width-2, self.height-2)  # 目标位置在右下角
        
    def create_grid_env(self):
        """创建栅格环境"""
        grid_env = Grid(self.width, self.height)
        obstacles = grid_env.obstacles
        
        # 初始化迷宫，先将所有单元格设为墙壁
        for x in range(self.width):
            for y in range(self.height):
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
                if 0 < nx < self.width-1 and 0 < ny < self.height-1 and (nx, ny) not in visited:
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
        
        # 确保起点和终点不是障碍物
        goal_x, goal_y = self.width-2, self.height-2
        obstacles.discard((1, 1))
        obstacles.discard((goal_x, goal_y))
        
        # 确保起点和终点周围的区域也是可通行的
        for dx in range(-1, 2):
            for dy in range(-1, 2):
                nx, ny = 1 + dx, 1 + dy
                if 0 <= nx < self.width and 0 <= ny < self.height:
                    obstacles.discard((nx, ny))
                
                nx, ny = goal_x + dx, goal_y + dy
                if 0 <= nx < self.width and 0 <= ny < self.height:
                    obstacles.discard((nx, ny))
        
        # 更新环境的障碍物
        grid_env.update(obstacles)
        
        # 为Grid类添加is_obstacle方法
        def is_obstacle(x, y):
            return (x, y) in obstacles
        
        # 动态添加方法到grid_env实例
        grid_env.is_obstacle = is_obstacle
        
        return grid_env 