#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
maze_slam_gui.py - 直接用matplotlib GUI展示迷宫SLAM地图
地图数据来自examples/json_data/1.json
"""

import os
import sys
import argparse

# 保证能找到best目录下的核心类
sys.path.append(os.path.join(os.path.dirname(__file__), 'best'))

from maze_slam_visual_new2 import GlobalMazeSLAMSystem

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="迷宫SLAM GUI可视化，支持自定义地图文件")
    parser.add_argument('--map_file', type=str, default=os.path.join(os.path.dirname(__file__), "json_data", "1.json"),
                        help='地图json文件路径，默认: examples/json_data/1.json')
    args = parser.parse_args()
    map_file = args.map_file
    print(f"使用地图文件: {map_file}")
    
    # 创建SLAM系统实例
    slam_system = GlobalMazeSLAMSystem(map_file=map_file)
    
    # 运行探索并用matplotlib GUI展示
    slam_system.run_exploration() 