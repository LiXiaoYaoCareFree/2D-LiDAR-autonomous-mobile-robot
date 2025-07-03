#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
迷宫JSON加载器
用于从JSON文件加载预定义的迷宫
"""

import json
import numpy as np

def load_maze_from_json(file_path, width=15, height=15):
    """从JSON文件加载迷宫数据
    
    Args:
        file_path: JSON文件路径
        width: 迷宫宽度
        height: 迷宫高度
        
    Returns:
        obstacles: 障碍物集合
        start_pos: 起始位置
    """
    # 读取JSON文件
    with open(file_path, 'r') as f:
        data = json.load(f)
    
    # 获取线段和起点
    segments = data.get('segments', [])
    start_point = data.get('start_point', [1, 1])
    
    # 创建网格表示
    grid = np.zeros((width, height), dtype=int)
    
    # 将线段转换为障碍物
    obstacles = set()
    for segment in segments:
        start = segment.get('start')
        end = segment.get('end')
        
        # 水平线段
        if start[1] == end[1]:
            y = start[1]
            # 确保坐标在有效范围内
            x_start = max(0, min(start[0], end[0]))
            x_end = min(width-1, max(start[0], end[0]))
            
            for x in range(x_start, x_end + 1):
                if 0 <= x < width and 0 <= y < height:
                    obstacles.add((x, y))
                    grid[x, y] = 1
        
        # 垂直线段
        elif start[0] == end[0]:
            x = start[0]
            # 确保坐标在有效范围内
            y_start = max(0, min(start[1], end[1]))
            y_end = min(height-1, max(start[1], end[1]))
            
            for y in range(y_start, y_end + 1):
                if 0 <= x < width and 0 <= y < height:
                    obstacles.add((x, y))
                    grid[x, y] = 1
        
        # 斜线段 - 使用Bresenham算法
        else:
            x0, y0 = start
            x1, y1 = end
            
            # Bresenham算法
            dx = abs(x1 - x0)
            dy = abs(y1 - y0)
            sx = 1 if x0 < x1 else -1
            sy = 1 if y0 < y1 else -1
            err = dx - dy
            
            while True:
                # 只添加在范围内的点
                if 0 <= x0 < width and 0 <= y0 < height:
                    obstacles.add((x0, y0))
                    grid[x0, y0] = 1
                
                # 检查是否到达终点
                if x0 == x1 and y0 == y1:
                    break
                
                # 计算下一个点
                e2 = 2 * err
                if e2 > -dy:
                    err -= dy
                    x0 += sx
                if e2 < dx:
                    err += dx
                    y0 += sy
    
    # 添加外围墙壁 - 确保迷宫边界是封闭的
    # 添加上下边界
    for x in range(width):
        obstacles.add((x, 0))
        obstacles.add((x, height-1))
        grid[x, 0] = 1
        grid[x, height-1] = 1
    
    # 添加左右边界
    for y in range(height):
        obstacles.add((0, y))
        obstacles.add((width-1, y))
        grid[0, y] = 1
        grid[width-1, y] = 1
    
    # 设置起点
    start_pos = (start_point[0], start_point[1], 0)  # (x, y, theta)
    
    # 确保起点不是障碍物
    if (start_pos[0], start_pos[1]) in obstacles:
        obstacles.remove((start_pos[0], start_pos[1]))
        if 0 <= start_pos[0] < width and 0 <= start_pos[1] < height:
            grid[start_pos[0], start_pos[1]] = 0
    
    return obstacles, start_pos, grid

def visualize_maze_grid(grid):
    """可视化迷宫网格
    
    Args:
        grid: 迷宫网格
    """
    import matplotlib.pyplot as plt
    
    plt.figure(figsize=(10, 10))
    plt.imshow(grid.T, cmap='binary', origin='lower')
    plt.grid(True, color='gray', linestyle='-', linewidth=0.5)
    plt.title('Maze Grid')
    plt.show()

if __name__ == "__main__":
    # 测试加载迷宫
    import os
    
    # 获取当前文件所在目录
    current_dir = os.path.dirname(os.path.abspath(__file__))
    
    # 测试加载三个迷宫
    for i in range(1, 4):
        json_file = os.path.join(current_dir, 'json_data', f'{i}.json')
        
        # 根据迷宫大小调整宽度和高度
        width, height = 15, 15
        if i == 3:
            width, height = 21, 21
        
        obstacles, start_pos, grid = load_maze_from_json(json_file, width, height)
        print(f"迷宫 {i}:")
        print(f"  障碍物数量: {len(obstacles)}")
        print(f"  起点: {start_pos}")
        
        # 可视化迷宫
        visualize_maze_grid(grid) 