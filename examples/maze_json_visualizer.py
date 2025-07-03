#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
迷宫JSON可视化工具
用于直接可视化JSON文件中定义的迷宫
"""

import os
import sys
import argparse
import matplotlib.pyplot as plt
import numpy as np
from maze_json_loader import load_maze_from_json

def visualize_json_maze(json_file, width=15, height=15, save_path=None):
    """可视化JSON迷宫文件
    
    Args:
        json_file: JSON迷宫文件路径
        width: 迷宫宽度
        height: 迷宫高度
        save_path: 如果提供，则将可视化结果保存到该路径
    """
    # 加载迷宫数据
    obstacles, start_pos, grid = load_maze_from_json(json_file, width, height)
    
    # 绘制迷宫
    plt.figure(figsize=(10, 10))
    plt.imshow(grid.T, cmap='binary', origin='lower')
    plt.grid(True, color='gray', linestyle='-', linewidth=0.5)
    
    # 标记起点 - 只显示起点，终点不预先标注
    plt.plot(start_pos[0], start_pos[1], 'go', markersize=10, label='起点')
    plt.legend()
    
    # 添加标题和标签
    plt.title(f"迷宫从JSON文件加载: {os.path.basename(json_file)}")
    plt.xlabel("X坐标")
    plt.ylabel("Y坐标")
    
    # 添加网格线
    plt.xticks(np.arange(-0.5, width, 1), [])
    plt.yticks(np.arange(-0.5, height, 1), [])
    plt.grid(True, color='gray', linestyle='-', linewidth=0.5)
    
    # 调整布局
    plt.tight_layout()
    
    # 保存图像或显示
    if save_path:
        plt.savefig(save_path)
        print(f"迷宫图像已保存至: {save_path}")
    else:
        plt.show()

def main():
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='可视化JSON迷宫文件')
    parser.add_argument('--json', type=str, help='JSON迷宫文件路径')
    parser.add_argument('--maze-id', type=int, choices=[1, 2, 3], help='使用预定义的迷宫ID (1, 2, 3)')
    parser.add_argument('--width', type=int, default=15, help='迷宫宽度，默认15')
    parser.add_argument('--height', type=int, default=15, help='迷宫高度，默认15')
    parser.add_argument('--save', type=str, help='保存图像的路径')
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
    else:
        print("错误：必须指定--json或--maze-id参数")
        parser.print_help()
        sys.exit(1)
    
    # 可视化迷宫
    visualize_json_maze(json_file, args.width, args.height, args.save)

if __name__ == "__main__":
    main() 