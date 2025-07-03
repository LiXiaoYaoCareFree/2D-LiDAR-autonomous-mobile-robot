#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
测试从JSON文件加载迷宫并可视化
"""

import os
import matplotlib.pyplot as plt
import numpy as np
from maze_env import MazeEnvironment
from maze_json_loader import load_maze_from_json

def visualize_maze_environment(env, title="迷宫环境"):
    """可视化迷宫环境
    
    Args:
        env: 迷宫环境
        title: 图表标题
    """
    # 创建网格表示
    grid = np.zeros((env.width, env.height))
    
    # 标记障碍物
    for x in range(env.width):
        for y in range(env.height):
            if (x, y) in env.grid_env.obstacles:
                grid[x, y] = 1
    
    # 绘制迷宫
    plt.figure(figsize=(10, 10))
    plt.imshow(grid.T, cmap='binary', origin='lower')
    plt.grid(True, color='gray', linestyle='-', linewidth=0.5)
    
    # 标记起点和终点
    start_x, start_y = env.start_pos[0], env.start_pos[1]
    goal_x, goal_y = env.goal_pos[0], env.goal_pos[1]
    
    plt.plot(start_x, start_y, 'go', markersize=10, label='起点')
    plt.plot(goal_x, goal_y, 'ro', markersize=10, label='终点')
    
    plt.title(title)
    plt.legend()
    plt.tight_layout()
    plt.show()

def main():
    """主函数"""
    # 获取当前文件所在目录
    current_dir = os.path.dirname(os.path.abspath(__file__))
    
    # 测试加载三个迷宫
    for i in range(1, 4):
        json_file = os.path.join(current_dir, 'json_data', f'{i}.json')
        
        # 根据迷宫大小调整宽度和高度
        width, height = 15, 15
        if i == 3:
            width, height = 21, 21
        
        # 创建迷宫环境
        env = MazeEnvironment(width=width, height=height, json_file=json_file)
        
        # 可视化迷宫
        visualize_maze_environment(env, f"迷宫 {i} (从JSON加载)")

if __name__ == "__main__":
    main() 