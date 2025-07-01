#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
迷宫可视化模块
包含迷宫探索的可视化功能
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, Wedge
from matplotlib.animation import FuncAnimation
import time
import math
from matplotlib.font_manager import FontProperties

# 尝试设置中文字体
try:
    plt.rcParams['font.sans-serif'] = ['SimHei']  # 指定默认字体为黑体
    plt.rcParams['axes.unicode_minus'] = False  # 解决保存图像是负号'-'显示为方块的问题
    font = FontProperties(fname=r"c:\windows\fonts\simsun.ttc", size=14)
except:
    print("警告: 无法设置中文字体，将使用默认字体")
    font = None

class MazeVisualization:
    """迷宫可视化类"""
    def __init__(self, maze_env, robot):
        """初始化可视化"""
        self.maze_env = maze_env
        self.robot = robot
        self.grid_env = maze_env.grid_env
        self.start_pos = maze_env.start_pos
        self.goal_pos = maze_env.goal_pos
        
        # 设置可视化参数
        self.animation_running = True
        self.exploration_paused = False
        self.update_interval = 0.05  # 更新间隔，单位秒
        self.last_update_time = time.time()
        
        # 设置图形
        self.setup_visualization()
        
    def setup_visualization(self):
        """设置可视化"""
        # 创建图形和子图
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(15, 7))
        
        # 设置第一个子图（实时探索）
        self.ax1.set_xlim(0, self.grid_env.x_range)
        self.ax1.set_ylim(0, self.grid_env.y_range)
        self.ax1.set_title("机器人实时探索")
        self.ax1.set_aspect('equal')
        
        # 设置第二个子图（地图）
        self.ax2.set_xlim(0, self.grid_env.x_range)
        self.ax2.set_ylim(0, self.grid_env.y_range)
        self.ax2.set_title("迷宫地图")
        self.ax2.set_aspect('equal')
        
        # 绘制障碍物
        for obs in self.grid_env.obstacles:
            self.ax1.add_patch(Rectangle((obs[0]-0.5, obs[1]-0.5), 1, 1, color='black'))
            self.ax2.add_patch(Rectangle((obs[0]-0.5, obs[1]-0.5), 1, 1, color='black'))
        
        # 绘制起点和终点
        self.ax1.plot(self.start_pos[0], self.start_pos[1], 'go', markersize=8, label='起点')
        self.ax2.plot(self.start_pos[0], self.start_pos[1], 'go', markersize=8, label='起点')
        self.ax1.plot(self.goal_pos[0], self.goal_pos[1], 'ro', markersize=8, label='终点')
        self.ax2.plot(self.goal_pos[0], self.goal_pos[1], 'ro', markersize=8, label='终点')
        
        # 添加图例
        self.ax1.legend()
        self.ax2.legend()
        
        # 初始化机器人位置标记
        start_x, start_y = self.robot.x, self.robot.y
        self.robot_marker1, = self.ax1.plot([start_x], [start_y], 'bo', markersize=8)
        self.robot_marker2, = self.ax2.plot([start_x], [start_y], 'bo', markersize=8)
        
        # 初始化机器人方向指示
        length = 1.0
        dx = length * np.cos(self.robot.theta)
        dy = length * np.sin(self.robot.theta)
        self.direction_line1, = self.ax1.plot([start_x, start_x + dx], [start_y, start_y + dy], 'b-', linewidth=2)
        self.direction_line2, = self.ax2.plot([start_x, start_x + dx], [start_y, start_y + dy], 'b-', linewidth=2)
        
        # 初始化路径
        self.path_line1, = self.ax1.plot([], [], 'c-', linewidth=1, alpha=0.5)
        self.path_line2, = self.ax2.plot([], [], 'c-', linewidth=1, alpha=0.5)
        
        # 初始化最优路径
        self.optimal_path_line1, = self.ax1.plot([], [], 'g-', linewidth=2)
        self.optimal_path_line2, = self.ax2.plot([], [], 'g-', linewidth=2)
        
        # 初始化激光线
        self.laser_lines = []
        
        # 初始化阴影（表示未探索区域）
        self.unexplored = np.ones((self.grid_env.x_range, self.grid_env.y_range))
        # 起点周围区域设为已探索
        x, y = int(self.robot.x), int(self.robot.y)
        view_range = 2
        for dx in range(-view_range, view_range + 1):
            for dy in range(-view_range, view_range + 1):
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.grid_env.x_range and 0 <= ny < self.grid_env.y_range:
                    self.unexplored[nx, ny] = 0
                    
        self.shadow = self.ax1.imshow(self.unexplored.T, origin='lower', extent=(0, self.grid_env.x_range, 0, self.grid_env.y_range), 
                                     cmap='Greys', alpha=0.3, vmin=0, vmax=1)
        
        # 初始化探索地图散点图
        self.explored_scatter = self.ax2.scatter([], [], c='lightblue', s=5, alpha=0.8)
        
        # 初始化迷宫格子状态散点图
        self.unknown_cells_scatter = self.ax2.scatter([], [], c='lightgray', s=5, alpha=0.5)
        self.path_cells_scatter = self.ax2.scatter([], [], c='lightblue', s=5, alpha=0.8)
        
        # 添加信息文本
        self.info_text = self.ax1.text(0.02, 0.98, '', transform=self.ax1.transAxes, 
                                      verticalalignment='top', fontsize=10, 
                                      bbox=dict(boxstyle='round', facecolor='white', alpha=0.7))
        
        # 添加状态文本
        self.status_text = self.ax1.text(0.02, 0.02, '', transform=self.ax1.transAxes, 
                                        verticalalignment='bottom', fontsize=10, 
                                        bbox=dict(boxstyle='round', facecolor='white', alpha=0.7))
        
        # 添加控制按钮
        button_axes = plt.axes([0.45, 0.01, 0.1, 0.04])
        self.pause_button = plt.Button(button_axes, '暂停', color='lightgoldenrodyellow')
        self.pause_button.on_clicked(self.toggle_pause)
        
        plt.tight_layout()
        
    def toggle_pause(self, event):
        """暂停/继续动画"""
        self.exploration_paused = not self.exploration_paused
        if self.exploration_paused:
            self.pause_button.label.set_text('继续')
        else:
            self.pause_button.label.set_text('暂停')
        
    def update_visualization(self):
        """更新可视化"""
        # 清除之前的激光线
        for line in self.laser_lines:
            line.remove() if hasattr(line, 'remove') else None
        self.laser_lines = []
        
        # 添加新的激光线
        for point in self.robot.laser_points:
            line, = self.ax1.plot([self.robot.x, point[0]], [self.robot.y, point[1]], 'r-', alpha=0.3, linewidth=0.5)
            self.laser_lines.append(line)
        
        # 更新机器人位置和方向
        self.robot_marker1.set_data([self.robot.x], [self.robot.y])
        self.robot_marker2.set_data([self.robot.x], [self.robot.y])
        
        dx = math.cos(self.robot.theta) * 1.0
        dy = math.sin(self.robot.theta) * 1.0
        self.direction_line1.set_data([self.robot.x, self.robot.x + dx], [self.robot.y, self.robot.y + dy])
        self.direction_line2.set_data([self.robot.x, self.robot.x + dx], [self.robot.y, self.robot.y + dy])
        
        # 更新路径
        if self.robot.path:
            path_x, path_y = zip(*self.robot.path)
            self.path_line1.set_data(path_x, path_y)
            self.path_line2.set_data(path_x, path_y)
        
        # 更新探索地图
        if hasattr(self, 'explored_scatter'):
            self.explored_scatter.remove()
        
        if self.robot.explored_map:
            explored_x, explored_y = zip(*self.robot.explored_map)
            self.explored_scatter = self.ax2.scatter(explored_x, explored_y, c='lightblue', s=5, alpha=0.8)
        
        # 更新迷宫格子状态
        # 清除之前的格子状态
        if hasattr(self, 'unknown_cells_scatter'):
            self.unknown_cells_scatter.remove()
        if hasattr(self, 'path_cells_scatter'):
            self.path_cells_scatter.remove()
            
        # 显示未知区域（灰色）
        unknown_cells = [(x, y) for (x, y), status in self.robot.maze_cells.items() if status == 0]
        if unknown_cells:
            unknown_x, unknown_y = zip(*unknown_cells)
            self.unknown_cells_scatter = self.ax2.scatter(unknown_x, unknown_y, c='lightgray', s=5, alpha=0.5)
            
        # 显示已探索的通路（蓝色）
        path_cells = [(x, y) for (x, y), status in self.robot.maze_cells.items() if status == 1]
        if path_cells:
            path_x, path_y = zip(*path_cells)
            self.path_cells_scatter = self.ax2.scatter(path_x, path_y, c='lightblue', s=5, alpha=0.8)
        
        # 更新最优路径
        if self.robot.optimal_path:
            path_x, path_y = zip(*self.robot.optimal_path)
            self.optimal_path_line1.set_data(path_x, path_y)
            self.optimal_path_line2.set_data(path_x, path_y)
        
        # 更新阴影
        for pos in self.robot.explored_map:
            if 0 <= pos[0] < self.grid_env.x_range and 0 <= pos[1] < self.grid_env.y_range:
                self.unexplored[pos[0], pos[1]] = 0
        
        self.shadow.set_data(self.unexplored.T)
        
        # 计算探索进度
        total_cells = (self.grid_env.x_range - 2) * (self.grid_env.y_range - 2) - len(self.grid_env.obstacles)
        explored_cells = len(self.robot.explored_map)
        explored_percent = explored_cells / total_cells * 100 if total_cells > 0 else 0
        
        # 计算到目标的距离
        dist_to_goal = math.sqrt((self.robot.x - self.goal_pos[0])**2 + (self.robot.y - self.goal_pos[1])**2)
        
        # 更新信息显示
        info_str = f"位置: ({self.robot.x:.1f}, {self.robot.y:.1f})\n到终点距离: {dist_to_goal:.1f}\n探索进度: {explored_percent:.1f}%"
        self.info_text.set_text(info_str)
        
        # 更新状态显示
        if self.current_state == "exploration":
            status = "探索中"
        elif self.current_state == "path_planning":
            status = "路径规划中"
        elif self.current_state == "navigation":
            status = "导航中"
        else:
            status = "完成"
            
        status_str = f"状态: {status}"
        if self.exploration_paused:
            status_str += " (已暂停)"
        self.status_text.set_text(status_str)
        
        # 刷新图形
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
    
    def run_animation(self, maze_exploration):
        """运行动画"""
        print("开始迷宫探索动画...")
        print("请等待图形窗口出现，然后点击窗口中的按钮控制动画")
        self.current_state = maze_exploration.current_state
        self.exploration_start_time = time.time()
        
        # 创建动画
        self.anim = FuncAnimation(
            self.fig, 
            lambda frame: maze_exploration.update(self),
            frames=None,  # 无限帧
            interval=50,  # 每50毫秒更新一次
            repeat=False,
            blit=False,
            cache_frame_data=False  # 避免缓存警告
        )
        
        # 显示图形
        plt.show(block=True) 