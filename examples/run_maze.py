#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
迷宫探索启动脚本
简单的启动脚本，用于运行迷宫探索程序
"""

import sys
import os

# 确保可以导入模块
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    # 导入主程序
    from maze_exploration_main import MazeExploration
    
    print("="*50)
    print("迷宫探索与路径规划演示程序")
    print("="*50)
    print("程序将模拟机器人在未知迷宫中探索并寻找目标点的过程")
    print("1. 机器人会自动探索迷宫")
    print("2. 找到目标点后会规划最优路径")
    print("3. 然后沿着规划的路径导航到目标点")
    print("\n按Enter键开始...")
    input()
    
    # 创建并运行迷宫探索
    maze_exploration = MazeExploration()
    maze_exploration.run()
    
except Exception as e:
    print(f"程序运行出错: {e}")
    import traceback
    traceback.print_exc()
    print("\n按Enter键退出...")
    input() 