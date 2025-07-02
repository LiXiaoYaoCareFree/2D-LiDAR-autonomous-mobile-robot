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
    
    print("="*60)
    print("迷宫探索与路径规划演示程序")
    print("="*60)
    print("程序将模拟机器人在未知迷宫中探索并寻找目标点的过程")
    print("\n探索过程分为三个阶段：")
    print("1. 完整遍历迷宫阶段：")
    print("   - 机器人会尽可能遍历迷宫中所有可达区域")
    print("   - 避免重复走相同的路径，提高探索效率")
    print("   - 在遍历过程中会寻找起点和终点")
    print("   - 当所有可达区域都被遍历后，进入下一阶段")
    print("\n2. 路径规划阶段：")
    print("   - 使用A*算法规划从当前位置到目标点的最优路径")
    print("   - 确保路径不会穿过障碍物")
    print("\n3. 导航阶段：")
    print("   - 机器人沿着规划的路径导航到目标点")
    print("\n可视化说明：")
    print("- 左侧窗口：显示机器人实时探索视图")
    print("- 右侧窗口：显示迷宫地图和已探索区域")
    print("- 蓝色点：机器人当前位置")
    print("- 绿色点：起点")
    print("- 红色点：终点")
    print("- 黑色方块：障碍物")
    print("- 浅蓝色区域：已探索区域")
    print("- 绿色线：规划的最优路径")
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