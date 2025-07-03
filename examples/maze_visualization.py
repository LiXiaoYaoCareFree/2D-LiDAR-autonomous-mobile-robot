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
from matplotlib import colors

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
        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(1, 3, figsize=(20, 7))
        
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
        
        # 设置第三个子图（SLAM地图）
        self.ax3.set_xlim(-2, self.grid_env.x_range + 2)
        self.ax3.set_ylim(-2, self.grid_env.y_range + 2)
        self.ax3.set_title("SLAM地图")
        self.ax3.set_aspect('equal')
        
        # 绘制障碍物
        for obs in self.grid_env.obstacles:
            self.ax1.add_patch(Rectangle((obs[0]-0.5, obs[1]-0.5), 1, 1, color='black'))
            self.ax2.add_patch(Rectangle((obs[0]-0.5, obs[1]-0.5), 1, 1, color='black'))
        
        # 绘制起点 - 只显示起点，终点在探索过程中发现后才显示
        self.ax1.plot(self.start_pos[0], self.start_pos[1], 'go', markersize=8, label='起点')
        self.ax2.plot(self.start_pos[0], self.start_pos[1], 'go', markersize=8, label='起点')
        
        # 添加图例
        self.ax1.legend()
        self.ax2.legend()
        
        # 初始化机器人位置标记
        start_x, start_y = self.robot.x, self.robot.y
        self.robot_marker1, = self.ax1.plot([start_x], [start_y], 'bo', markersize=8)
        self.robot_marker2, = self.ax2.plot([start_x], [start_y], 'bo', markersize=8)
        self.robot_marker3, = self.ax3.plot([start_x], [start_y], 'bo', markersize=8)
        
        # 初始化机器人方向指示
        length = 1.0
        dx = length * np.cos(self.robot.theta)
        dy = length * np.sin(self.robot.theta)
        self.direction_line1, = self.ax1.plot([start_x, start_x + dx], [start_y, start_y + dy], 'b-', linewidth=2)
        self.direction_line2, = self.ax2.plot([start_x, start_x + dx], [start_y, start_y + dy], 'b-', linewidth=2)
        
        # 初始化路径
        self.path_line1, = self.ax1.plot([], [], 'c-', linewidth=1, alpha=0.5)
        self.path_line2, = self.ax2.plot([], [], 'c-', linewidth=1, alpha=0.5)
        self.path_line3, = self.ax3.plot([], [], 'c-', linewidth=1, alpha=0.5)
        
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
        # 清除之前的绘图
        self.ax1.clear()
        self.ax2.clear()
        self.ax3.clear()
        
        # 设置坐标轴范围
        self.ax1.set_xlim(0, self.grid_env.x_range)
        self.ax1.set_ylim(0, self.grid_env.y_range)
        self.ax2.set_xlim(0, self.grid_env.x_range)
        self.ax2.set_ylim(0, self.grid_env.y_range)
        self.ax3.set_xlim(-2, self.grid_env.x_range + 2)
        self.ax3.set_ylim(-2, self.grid_env.y_range + 2)
        
        # 设置SLAM地图的背景为灰色
        self.ax3.set_facecolor('#e0e0e0')
        
        # 绘制障碍物
        for obs in self.grid_env.obstacles:
            self.ax1.add_patch(Rectangle((obs[0] - 0.5, obs[1] - 0.5), 1, 1, color='black'))
            self.ax2.add_patch(Rectangle((obs[0] - 0.5, obs[1] - 0.5), 1, 1, color='black'))
        
        # 绘制起点
        self.ax1.plot(self.start_pos[0], self.start_pos[1], 'go', markersize=10)  # 绿色起点
        self.ax2.plot(self.start_pos[0], self.start_pos[1], 'go', markersize=10)  # 绿色起点
        self.ax3.plot(self.start_pos[0], self.start_pos[1], 'go', markersize=10)  # 绿色起点
        
        # 只有在目标被发现后才绘制终点
        if hasattr(self, 'controller') and hasattr(self.controller, 'goal_found') and self.controller.goal_found:
            self.ax1.plot(self.goal_pos[0], self.goal_pos[1], 'ro', markersize=10)  # 红色终点
            self.ax2.plot(self.goal_pos[0], self.goal_pos[1], 'ro', markersize=10)  # 红色终点
            self.ax3.plot(self.goal_pos[0], self.goal_pos[1], 'ro', markersize=10)  # 红色终点
        
        # 绘制机器人位置
        self.ax1.plot(self.robot.x, self.robot.y, 'bo', markersize=8)  # 蓝色机器人
        self.ax2.plot(self.robot.x, self.robot.y, 'bo', markersize=8)  # 蓝色机器人
        self.ax3.plot(self.robot.x, self.robot.y, 'bo', markersize=8)  # 蓝色机器人
        
        # 绘制机器人朝向
        direction_length = 1.0
        dx = direction_length * math.cos(self.robot.theta)
        dy = direction_length * math.sin(self.robot.theta)
        self.ax1.arrow(self.robot.x, self.robot.y, dx, dy, head_width=0.3, head_length=0.3, fc='blue', ec='blue')
        self.ax3.arrow(self.robot.x, self.robot.y, dx, dy, head_width=0.3, head_length=0.3, fc='blue', ec='blue')
        
        # 绘制激光传感器数据
        if hasattr(self.robot, 'sensor_data') and self.robot.sensor_data:
            for point in self.robot.sensor_data:
                self.ax1.plot([self.robot.x, point[0]], [self.robot.y, point[1]], 'y-', alpha=0.3)  # 黄色激光线
                self.ax1.plot(point[0], point[1], 'rx', markersize=3)  # 红色激光点
        
        # 绘制雷达射线
        if hasattr(self, 'controller') and hasattr(self.controller, 'slam_visualizer') and hasattr(self.controller.slam_visualizer, 'radar_rays'):
            for ray in self.controller.slam_visualizer.radar_rays:
                start_pos, end_pos = ray
                # 提取起点坐标（忽略角度）
                if len(start_pos) == 3:
                    start_x, start_y, _ = start_pos
                else:
                    start_x, start_y = start_pos
                
                # 提取终点坐标
                if isinstance(end_pos, tuple):
                    end_x, end_y = end_pos[:2]  # 只取前两个坐标
                else:
                    end_x, end_y = end_pos, end_pos
                
                # 在SLAM地图上绘制雷达射线
                self.ax3.plot([start_x, end_x], [start_y, end_y], 'g-', alpha=0.5, linewidth=0.8)
                # 在雷达射线终点绘制小点
                self.ax3.plot(end_x, end_y, 'g.', markersize=3)
        
        # 绘制已探索区域
        if hasattr(self.robot, 'visited_cells'):
            for cell in self.robot.visited_cells:
                if cell not in self.grid_env.obstacles:
                    self.ax2.add_patch(Rectangle((cell[0] - 0.5, cell[1] - 0.5), 1, 1, color='lightblue', alpha=0.5))
        
        # 绘制边界单元格
        if hasattr(self.robot, 'frontier'):
            for cell in self.robot.frontier:
                if cell not in self.robot.visited_cells and cell not in self.grid_env.obstacles:
                    self.ax2.add_patch(Rectangle((cell[0] - 0.5, cell[1] - 0.5), 1, 1, color='yellow', alpha=0.3))
        
        # 绘制规划的路径
        if hasattr(self, 'controller') and hasattr(self.controller, 'current_state') and self.controller.current_state in ["navigation", "navigate_to_goal"] and hasattr(self.robot, 'goal_path') and self.robot.goal_path:
            path_x = [p[0] for p in self.robot.goal_path]
            path_y = [p[1] for p in self.robot.goal_path]
            self.ax1.plot(path_x, path_y, 'g-', linewidth=2)  # 绿色路径
            self.ax2.plot(path_x, path_y, 'g-', linewidth=2)  # 绿色路径
            # 不在SLAM地图上显示路径
        
        # 绘制SLAM地图
        if hasattr(self, 'controller') and hasattr(self.controller, 'slam_visualizer'):
            slam_viz = self.controller.slam_visualizer
            
            # 创建自定义颜色映射：灰色(未探索)、白色(空闲)、黑色(障碍物)
            custom_cmap = colors.ListedColormap(['#e0e0e0', 'white', 'black'])
            
            # 显示SLAM地图
            self.ax3.imshow(slam_viz.slam_map.T, origin='lower', 
                          extent=[-2, self.grid_env.x_range + 2, -2, self.grid_env.y_range + 2], 
                          cmap=custom_cmap, alpha=1.0, vmin=0, vmax=2)
            
            # 不显示轨迹
        
        # 设置子图标题
        self.ax1.set_title("机器人实时探索", fontsize=12, fontweight='bold')
        self.ax2.set_title("迷宫地图", fontsize=12, fontweight='bold')
        self.ax3.set_title("SLAM地图", fontsize=12, fontweight='bold')
        
        # 更新状态信息
        if hasattr(self, 'controller'):
            # 计算探索进度
            exploration_progress = self.robot.exploration_progress if hasattr(self.robot, 'exploration_progress') else 0
            
            # 显示状态信息
            status_text = f"状态: {self.controller.current_state}\n"
            status_text += f"探索进度: {exploration_progress:.2f}%\n"
            status_text += f"步数: {self.controller.step_count}\n"
            
            if hasattr(self.controller, 'goal_found'):
                status_text += f"目标发现: {'是' if self.controller.goal_found else '否'}\n"
                
            if hasattr(self.controller, 'reached_goal'):
                status_text += f"到达目标: {'是' if self.controller.reached_goal else '否'}\n"
                
            if hasattr(self.controller, 'returned_to_start'):
                status_text += f"返回起点: {'是' if self.controller.returned_to_start else '否'}\n"
                
            self.status_text = self.ax1.text(0.02, 0.02, status_text, transform=self.ax1.transAxes, 
                                          verticalalignment='bottom', fontsize=10, 
                                          bbox=dict(boxstyle='round', facecolor='white', alpha=0.7))
        
        plt.tight_layout()
        plt.draw()
    
    def run_animation(self, maze_exploration):
        """运行动画"""
        print("开始迷宫探索动画...")
        print("请等待图形窗口出现，然后点击窗口中的按钮控制动画")
        self.current_state = maze_exploration.current_state
        self.exploration_start_time = time.time()
        
        try:
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
        except Exception as e:
            print(f"动画运行出错: {e}")
            import traceback
            traceback.print_exc() 

    def update_plot(self):
        """更新绘图"""
        # 更新机器人位置
        self.robot_marker1.set_data([self.robot.x], [self.robot.y])
        self.robot_marker2.set_data([self.robot.x], [self.robot.y])
        
        # 更新机器人方向
        length = 1.0
        dx = length * np.cos(self.robot.theta)
        dy = length * np.sin(self.robot.theta)
        self.direction_line1.set_data([self.robot.x, self.robot.x + dx], [self.robot.y, self.robot.y + dy])
        self.direction_line2.set_data([self.robot.x, self.robot.x + dx], [self.robot.y, self.robot.y + dy])
        
        # 更新探索区域
        if hasattr(self.robot, 'visited_cells'):
            for cell in self.robot.visited_cells:
                x, y = cell
                if 0 <= x < self.grid_env.x_range and 0 <= y < self.grid_env.y_range:
                    self.unexplored[y][x] = 0
            
            # 更新阴影图
            self.shadow.set_data(self.unexplored.T)
        
        # 更新目标路径
        if hasattr(self.robot, 'goal_path') and self.robot.goal_path:
            # 绘制新路径
            path_x = [p[0] for p in self.robot.goal_path]
            path_y = [p[1] for p in self.robot.goal_path]
            self.path_line1.set_data(path_x, path_y)
            self.path_line2.set_data(path_x, path_y)
        
        # 更新信息文本
        if hasattr(self.robot, 'exploration_progress'):
            info_text = f"探索进度: {self.robot.exploration_progress:.2f}%"
            self.info_text.set_text(info_text)
        
        # 刷新画布
        self.fig.canvas.draw_idle() 