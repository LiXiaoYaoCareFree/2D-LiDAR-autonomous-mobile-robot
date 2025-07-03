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
        # 清除之前的绘图
        self.ax1.clear()
        self.ax2.clear()
        
        # 设置坐标轴范围
        self.ax1.set_xlim(0, self.grid_env.x_range)
        self.ax1.set_ylim(0, self.grid_env.y_range)
        self.ax2.set_xlim(0, self.grid_env.x_range)
        self.ax2.set_ylim(0, self.grid_env.y_range)
        
        # 绘制障碍物
        for obs in self.grid_env.obstacles:
            self.ax1.add_patch(Rectangle((obs[0] - 0.5, obs[1] - 0.5), 1, 1, color='black'))
            self.ax2.add_patch(Rectangle((obs[0] - 0.5, obs[1] - 0.5), 1, 1, color='black'))
        
        # 绘制起点
        self.ax1.plot(self.start_pos[0], self.start_pos[1], 'go', markersize=10)  # 绿色起点
        self.ax2.plot(self.start_pos[0], self.start_pos[1], 'go', markersize=10)  # 绿色起点
        
        # 只有在目标被发现后才绘制终点
        if hasattr(self, 'controller') and hasattr(self.controller, 'goal_found') and self.controller.goal_found:
            self.ax1.plot(self.goal_pos[0], self.goal_pos[1], 'ro', markersize=10)  # 红色终点
            self.ax2.plot(self.goal_pos[0], self.goal_pos[1], 'ro', markersize=10)  # 红色终点
        
        # 绘制机器人位置
        self.ax1.plot(self.robot.x, self.robot.y, 'bo', markersize=8)  # 蓝色机器人
        self.ax2.plot(self.robot.x, self.robot.y, 'bo', markersize=8)  # 蓝色机器人
        
        # 绘制机器人朝向
        direction_length = 1.0
        dx = direction_length * math.cos(self.robot.theta)
        dy = direction_length * math.sin(self.robot.theta)
        self.ax1.arrow(self.robot.x, self.robot.y, dx, dy, head_width=0.3, head_length=0.3, fc='blue', ec='blue')
        
        # 绘制激光传感器数据
        if hasattr(self.robot, 'sensor_data') and self.robot.sensor_data:
            for point in self.robot.sensor_data:
                self.ax1.plot([self.robot.x, point[0]], [self.robot.y, point[1]], 'y-', alpha=0.3)  # 黄色激光线
                self.ax1.plot(point[0], point[1], 'rx', markersize=3)  # 红色激光点
        
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
            self.ax2.plot(path_x, path_y, 'g-', linewidth=2)  # 绿色路径线
        
        # 添加标题
        self.ax1.set_title('机器人视角')
        if hasattr(self, 'controller') and hasattr(self.controller, 'current_state'):
            if self.controller.current_state == "exploration":
                self.ax2.set_title(f'迷宫地图 - 探索阶段 ({self.robot.exploration_progress:.1f}%)')
            elif self.controller.current_state == "search_goal":
                self.ax2.set_title('迷宫地图 - 搜索目标点阶段')
            elif self.controller.current_state == "navigate_to_goal":
                self.ax2.set_title('迷宫地图 - 导航到目标点阶段')
            elif self.controller.current_state == "path_planning":
                self.ax2.set_title('迷宫地图 - 规划返回路径阶段')
            elif self.controller.current_state == "navigation":
                self.ax2.set_title('迷宫地图 - 返回起点阶段')
            elif self.controller.current_state == "completed":
                self.ax2.set_title('迷宫地图 - 探索完成')
        else:
            self.ax2.set_title('迷宫地图')
        
        # 添加网格
        self.ax1.grid(True)
        self.ax2.grid(True)
        
        # 更新画布
        self.fig.canvas.draw()
        
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
        
        # 更新探索地图 - 使用try-except处理可能的错误
        try:
            if hasattr(self, 'explored_scatter'):
                self.explored_scatter.remove()
        except:
            pass  # 如果无法移除，就忽略错误
        
        if hasattr(self.robot, 'visited_cells') and self.robot.visited_cells:
            explored_x, explored_y = zip(*self.robot.visited_cells)
            self.explored_scatter = self.ax2.scatter(explored_x, explored_y, c='lightblue', s=5, alpha=0.8)
        
        # 更新迷宫格子状态 - 使用try-except处理可能的错误
        try:
            if hasattr(self, 'unknown_cells_scatter'):
                self.unknown_cells_scatter.remove()
        except:
            pass  # 如果无法移除，就忽略错误
            
        try:
            if hasattr(self, 'path_cells_scatter'):
                self.path_cells_scatter.remove()
        except:
            pass  # 如果无法移除，就忽略错误
            
        # 显示未知区域（灰色）和通路（浅蓝色）
        if hasattr(self.robot, 'maze_cells'):
            try:
                # 显示未知区域（灰色）
                unknown_cells = [(x, y) for (x, y), status in self.robot.maze_cells.items() if status == 0]
                if unknown_cells:
                    unknown_x, unknown_y = zip(*unknown_cells)
                    self.unknown_cells_scatter = self.ax2.scatter(unknown_x, unknown_y, c='lightgray', s=5, alpha=0.5)
                
                # 显示通路（浅蓝色）
                path_cells = [(x, y) for (x, y), status in self.robot.maze_cells.items() if status == 1]
                if path_cells:
                    path_x, path_y = zip(*path_cells)
                    self.path_cells_scatter = self.ax2.scatter(path_x, path_y, c='lightblue', s=5, alpha=0.8)
            except Exception as e:
                print(f"处理maze_cells时出错: {e}")
        
        # 更新最优路径
        if hasattr(self.robot, 'goal_path') and self.robot.goal_path:
            path_x, path_y = zip(*self.robot.goal_path)
            self.optimal_path_line1.set_data(path_x, path_y)
            self.optimal_path_line2.set_data(path_x, path_y)
        
        # 更新阴影
        if hasattr(self.robot, 'visited_cells'):
            for pos in self.robot.visited_cells:
                if 0 <= pos[0] < self.grid_env.x_range and 0 <= pos[1] < self.grid_env.y_range:
                    self.unexplored[pos[0], pos[1]] = 0
        
        self.shadow.set_data(self.unexplored.T)
        
        # 计算探索进度
        total_cells = (self.grid_env.x_range - 2) * (self.grid_env.y_range - 2) - len(self.grid_env.obstacles)
        if hasattr(self.robot, 'visited_cells'):
            explored_cells = len(self.robot.visited_cells)
        else:
            explored_cells = 0
        explored_percent = explored_cells / total_cells * 100 if total_cells > 0 else 0
        
        # 计算到目标的距离
        dist_to_goal = math.sqrt((self.robot.x - self.goal_pos[0])**2 + (self.robot.y - self.goal_pos[1])**2)
        
        # 更新信息显示
        info_str = f"位置: ({self.robot.x:.1f}, {self.robot.y:.1f})\n到终点距离: {dist_to_goal:.1f}\n探索进度: {explored_percent:.1f}%"
        self.info_text.set_text(info_str)
        
        # 更新状态显示
        if self.controller.current_state == "exploration":
            status = "探索中"
        elif self.controller.current_state == "path_planning":
            status = "路径规划中"
        elif self.controller.current_state == "navigation":
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