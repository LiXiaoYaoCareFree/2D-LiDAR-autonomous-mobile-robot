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
        
        # 确保坐标在范围内
        if not (0 <= start[0] < width and 0 <= start[1] < height and 
                0 <= end[0] < width and 0 <= end[1] < height):
            continue
        
        # 水平线段
        if start[1] == end[1]:
            y = start[1]
            for x in range(min(start[0], end[0]), max(start[0], end[0]) + 1):
                obstacles.add((x, y))
                if 0 <= x < width and 0 <= y < height:
                    grid[x, y] = 1
        
        # 垂直线段
        elif start[0] == end[0]:
            x = start[0]
            for y in range(min(start[1], end[1]), max(start[1], end[1]) + 1):
                obstacles.add((x, y))
                if 0 <= x < width and 0 <= y < height:
                    grid[x, y] = 1
    
    # 设置起点
    start_pos = (start_point[0], start_point[1], 0)  # (x, y, theta)
    
    # 确保起点不是障碍物
    if (start_pos[0], start_pos[1]) in obstacles:
        obstacles.remove((start_pos[0], start_pos[1]))
    
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