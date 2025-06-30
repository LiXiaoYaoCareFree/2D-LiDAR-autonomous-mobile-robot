#!/usr/bin/env python3
'''
enhanced_visualization.py - 增强的可视化模块

该模块实现了美观的GUI界面，包括地图可视化、导航可视化、状态显示等功能。
'''

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
import tkinter as tk
from tkinter import ttk
import time
import threading
from PIL import Image, ImageTk
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.patches import Polygon, Circle, Arrow

class EnhancedMapVisualizer:
    """增强的地图可视化类，提供美观的地图显示和导航可视化"""
    
    def __init__(self, map_size_pixels=800, map_size_meters=32):
        """
        初始化增强的地图可视化器
        
        参数:
            map_size_pixels: 地图大小(像素)
            map_size_meters: 地图大小(米)
        """
        self.map_size_pixels = map_size_pixels
        self.map_size_meters = map_size_meters
        self.cell_size = map_size_meters / map_size_pixels
        
        # 创建自定义的地图颜色映射
        self.cmap = self._create_custom_colormap()
        
        # 可视化参数
        self.robot_marker_size = 15
        self.robot_trail_length = 100
        self.robot_trail = []
        
        # 路径和边界点可视化
        self.current_path = []
        self.frontiers = []
        self.selected_frontier = None
        self.exit_position = None
        self.start_position = None
        
        # 状态信息
        self.current_state = "准备中"
        self.performance_stats = {}
    
    def _create_custom_colormap(self):
        """创建自定义的地图颜色映射"""
        # 定义颜色映射：未知区域(灰色)、自由空间(白色)、障碍物(黑色)
        colors = [(0.7, 0.7, 0.7),    # 灰色 (未知区域)
                  (1.0, 1.0, 1.0),    # 白色 (自由空间)
                  (0.0, 0.0, 0.0)]    # 黑色 (障碍物)
        
        # 创建颜色映射
        cmap = LinearSegmentedColormap.from_list("custom_map", colors, N=256)
        
        return cmap
    
    def update_visualization_data(self, occupancy_grid, robot_pose, current_path=None, 
                                frontiers=None, selected_frontier=None, exit_position=None,
                                start_position=None, current_state=None, performance_stats=None):
        """
        更新可视化数据
        
        参数:
            occupancy_grid: 占用栅格地图，numpy数组
            robot_pose: 机器人位置，(x, y, theta)，单位为米和度
            current_path: 当前规划的路径，路径点列表
            frontiers: 检测到的边界点列表
            selected_frontier: 选择的边界点
            exit_position: 出口位置，(x, y)，单位为米
            start_position: 起始位置，(x, y)，单位为米
            current_state: 当前状态描述
            performance_stats: 性能统计信息
        """
        self.occupancy_grid = occupancy_grid
        self.robot_pose = robot_pose
        
        # 更新机器人轨迹
        if robot_pose is not None:
            self.robot_trail.append((robot_pose[0], robot_pose[1]))
            # 保持轨迹长度限制
            if len(self.robot_trail) > self.robot_trail_length:
                self.robot_trail = self.robot_trail[-self.robot_trail_length:]
        
        # 更新路径和边界点
        if current_path is not None:
            self.current_path = current_path
        
        if frontiers is not None:
            self.frontiers = frontiers
        
        if selected_frontier is not None:
            self.selected_frontier = selected_frontier
        
        if exit_position is not None:
            self.exit_position = exit_position
        
        if start_position is not None:
            self.start_position = start_position
        
        # 更新状态信息
        if current_state is not None:
            self.current_state = current_state
        
        if performance_stats is not None:
            self.performance_stats = performance_stats
    
    def create_map_image(self):
        """
        创建地图图像
        
        返回:
            map_img: 地图图像，PIL图像对象
        """
        # 创建图像
        fig = Figure(figsize=(8, 8), dpi=100)
        ax = fig.add_subplot(111)
        
        # 显示占用栅格地图
        # 将0-255范围的栅格值归一化到0-1范围
        normalized_grid = self.occupancy_grid / 255.0
        ax.imshow(normalized_grid, cmap=self.cmap, origin='lower')
        
        # 绘制机器人轨迹
        if self.robot_trail:
            trail_x = [p[0] / self.cell_size for p in self.robot_trail]
            trail_y = [p[1] / self.cell_size for p in self.robot_trail]
            ax.plot(trail_x, trail_y, 'b-', linewidth=1, alpha=0.5)
        
        # 绘制当前路径
        if self.current_path:
            path_x = [p[0] / self.cell_size for p in self.current_path]
            path_y = [p[1] / self.cell_size for p in self.current_path]
            ax.plot(path_x, path_y, 'g-', linewidth=2)
        
        # 绘制边界点
        if self.frontiers:
            frontier_x = [f[0] / self.cell_size for f in self.frontiers]
            frontier_y = [f[1] / self.cell_size for f in self.frontiers]
            ax.scatter(frontier_x, frontier_y, c='orange', s=30, alpha=0.7)
        
        # 绘制选择的边界点
        if self.selected_frontier:
            sel_x = self.selected_frontier[0] / self.cell_size
            sel_y = self.selected_frontier[1] / self.cell_size
            ax.scatter([sel_x], [sel_y], c='red', s=50, marker='*')
        
        # 绘制出口位置
        if self.exit_position:
            exit_x = self.exit_position[0] / self.cell_size
            exit_y = self.exit_position[1] / self.cell_size
            ax.add_patch(Circle((exit_x, exit_y), radius=5, color='green', alpha=0.7))
            ax.text(exit_x + 5, exit_y + 5, "出口", fontsize=12, color='green')
        
        # 绘制起始位置
        if self.start_position:
            start_x = self.start_position[0] / self.cell_size
            start_y = self.start_position[1] / self.cell_size
            ax.add_patch(Circle((start_x, start_y), radius=5, color='blue', alpha=0.7))
            ax.text(start_x + 5, start_y + 5, "起点", fontsize=12, color='blue')
        
        # 绘制机器人位置和朝向
        if self.robot_pose:
            robot_x = self.robot_pose[0] / self.cell_size
            robot_y = self.robot_pose[1] / self.cell_size
            robot_theta = self.robot_pose[2] * np.pi / 180  # 转换为弧度
            
            # 绘制机器人标记
            ax.add_patch(Circle((robot_x, robot_y), radius=3, color='red'))
            
            # 绘制朝向箭头
            arrow_length = 10
            dx = arrow_length * np.cos(robot_theta)
            dy = arrow_length * np.sin(robot_theta)
            ax.arrow(robot_x, robot_y, dx, dy, head_width=5, head_length=5, fc='red', ec='red')
        
        # 设置坐标轴
        ax.set_xlim(0, self.map_size_pixels)
        ax.set_ylim(0, self.map_size_pixels)
        ax.set_title("机器人导航地图")
        ax.set_xlabel("X (像素)")
        ax.set_ylabel("Y (像素)")
        
        # 转换为PIL图像
        fig.canvas.draw()
        map_img = Image.frombytes('RGB', fig.canvas.get_width_height(), fig.canvas.tostring_rgb())
        
        plt.close(fig)  # 关闭图像，避免内存泄漏
        
        return map_img
    
    def create_stats_image(self, width=400, height=300):
        """
        创建状态信息图像
        
        参数:
            width: 图像宽度
            height: 图像高度
            
        返回:
            stats_img: 状态信息图像，PIL图像对象
        """
        # 创建图像
        fig = Figure(figsize=(width/100, height/100), dpi=100)
        ax = fig.add_subplot(111)
        
        # 禁用坐标轴
        ax.axis('off')
        
        # 添加状态信息文本
        text = f"当前状态: {self.current_state}\n\n"
        
        if self.performance_stats:
            if 'exploration_time' in self.performance_stats and self.performance_stats['exploration_time'] is not None:
                text += f"探索时间: {self.performance_stats['exploration_time']:.2f}秒\n"
            
            if 'exit_navigation_time' in self.performance_stats and self.performance_stats['exit_navigation_time'] is not None:
                text += f"到达出口时间: {self.performance_stats['exit_navigation_time']:.2f}秒\n"
            
            if 'return_time' in self.performance_stats and self.performance_stats['return_time'] is not None:
                text += f"返回时间: {self.performance_stats['return_time']:.2f}秒\n"
            
            if 'total_time' in self.performance_stats and self.performance_stats['total_time'] is not None:
                text += f"总时间: {self.performance_stats['total_time']:.2f}秒\n"
        
        # 添加机器人位置信息
        if self.robot_pose:
            text += f"\n机器人位置:\nX: {self.robot_pose[0]:.2f}米\nY: {self.robot_pose[1]:.2f}米\n方向: {self.robot_pose[2]:.2f}度"
        
        ax.text(0.05, 0.95, text, transform=ax.transAxes, fontsize=12, verticalalignment='top')
        
        # 转换为PIL图像
        fig.canvas.draw()
        stats_img = Image.frombytes('RGB', fig.canvas.get_width_height(), fig.canvas.tostring_rgb())
        
        plt.close(fig)  # 关闭图像，避免内存泄漏
        
        return stats_img

class RobotGUI:
    """机器人控制GUI类，提供美观的用户界面"""
    
    def __init__(self, window_title="2D激光雷达自主移动机器人控制系统"):
        """
        初始化GUI
        
        参数:
            window_title: 窗口标题
        """
        # 创建主窗口
        self.root = tk.Tk()
        self.root.title(window_title)
        self.root.geometry("1200x800")
        self.root.configure(bg="#f0f0f0")
        
        # 创建样式
        self.style = ttk.Style()
        self.style.theme_use('clam')  # 使用clam主题
        self.style.configure("TFrame", background="#f0f0f0")
        self.style.configure("TButton", background="#4CAF50", foreground="black", font=('Arial', 10, 'bold'))
        self.style.configure("TLabel", background="#f0f0f0", font=('Arial', 10))
        self.style.configure("Header.TLabel", font=('Arial', 14, 'bold'))
        
        # 创建可视化器
        self.visualizer = EnhancedMapVisualizer()
        
        # 创建GUI布局
        self._create_layout()
        
        # 初始化更新标志
        self.update_flag = False
        self.running = True
        
        # 创建更新线程
        self.update_thread = threading.Thread(target=self._update_loop)
        self.update_thread.daemon = True
        self.update_thread.start()
    
    def _create_layout(self):
        """创建GUI布局"""
        # 主框架
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # 标题标签
        title_label = ttk.Label(main_frame, text="2D激光雷达自主移动机器人控制系统", style="Header.TLabel")
        title_label.pack(pady=10)
        
        # 创建左右分割框架
        content_frame = ttk.Frame(main_frame)
        content_frame.pack(fill=tk.BOTH, expand=True)
        
        # 左侧地图显示区域
        self.map_frame = ttk.Frame(content_frame, width=800, height=600)
        self.map_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # 地图画布
        self.map_canvas = tk.Canvas(self.map_frame, bg="white", width=800, height=600)
        self.map_canvas.pack(fill=tk.BOTH, expand=True)
        
        # 右侧控制和状态区域
        right_frame = ttk.Frame(content_frame, width=400)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, padx=5, pady=5)
        
        # 状态信息区域
        stats_frame = ttk.LabelFrame(right_frame, text="状态信息")
        stats_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        # 状态画布
        self.stats_canvas = tk.Canvas(stats_frame, bg="white", width=380, height=300)
        self.stats_canvas.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # 控制按钮区域
        control_frame = ttk.LabelFrame(right_frame, text="控制面板")
        control_frame.pack(fill=tk.X, pady=5)
        
        # 添加控制按钮
        btn_frame = ttk.Frame(control_frame)
        btn_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # 启动按钮
        self.start_btn = ttk.Button(btn_frame, text="启动", command=self._on_start_clicked)
        self.start_btn.grid(row=0, column=0, padx=5, pady=5, sticky="ew")
        
        # 停止按钮
        self.stop_btn = ttk.Button(btn_frame, text="停止", command=self._on_stop_clicked)
        self.stop_btn.grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        
        # 重置按钮
        self.reset_btn = ttk.Button(btn_frame, text="重置", command=self._on_reset_clicked)
        self.reset_btn.grid(row=0, column=2, padx=5, pady=5, sticky="ew")
        
        # 保存地图按钮
        self.save_map_btn = ttk.Button(btn_frame, text="保存地图", command=self._on_save_map_clicked)
        self.save_map_btn.grid(row=1, column=0, padx=5, pady=5, sticky="ew")
        
        # 加载地图按钮
        self.load_map_btn = ttk.Button(btn_frame, text="加载地图", command=self._on_load_map_clicked)
        self.load_map_btn.grid(row=1, column=1, padx=5, pady=5, sticky="ew")
        
        # 设置按钮
        self.settings_btn = ttk.Button(btn_frame, text="设置", command=self._on_settings_clicked)
        self.settings_btn.grid(row=1, column=2, padx=5, pady=5, sticky="ew")
        
        # 日志区域
        log_frame = ttk.LabelFrame(right_frame, text="系统日志")
        log_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        # 日志文本框
        self.log_text = tk.Text(log_frame, height=10, width=40)
        self.log_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # 日志滚动条
        log_scrollbar = ttk.Scrollbar(self.log_text, command=self.log_text.yview)
        log_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.log_text.config(yscrollcommand=log_scrollbar.set)
        
        # 状态栏
        self.status_bar = ttk.Label(main_frame, text="系统就绪", relief=tk.SUNKEN, anchor=tk.W)
        self.status_bar.pack(fill=tk.X, side=tk.BOTTOM, pady=5)
        
        # 初始化地图和状态图像
        self._update_map_image()
        self._update_stats_image()
    
    def _update_map_image(self):
        """更新地图图像"""
        try:
            # 获取地图图像
            map_img = self.visualizer.create_map_image()
            
            # 调整图像大小以适应画布
            canvas_width = self.map_canvas.winfo_width()
            canvas_height = self.map_canvas.winfo_height()
            
            if canvas_width > 1 and canvas_height > 1:  # 确保画布已经渲染
                map_img = map_img.resize((canvas_width, canvas_height), Image.LANCZOS)
            
            # 转换为Tkinter图像
            self.map_tk_img = ImageTk.PhotoImage(map_img)
            
            # 清除画布并显示新图像
            self.map_canvas.delete("all")
            self.map_canvas.create_image(0, 0, anchor=tk.NW, image=self.map_tk_img)
        except Exception as e:
            self.log(f"地图更新错误: {str(e)}")
    
    def _update_stats_image(self):
        """更新状态信息图像"""
        try:
            # 获取状态信息图像
            stats_img = self.visualizer.create_stats_image()
            
            # 调整图像大小以适应画布
            canvas_width = self.stats_canvas.winfo_width()
            canvas_height = self.stats_canvas.winfo_height()
            
            if canvas_width > 1 and canvas_height > 1:  # 确保画布已经渲染
                stats_img = stats_img.resize((canvas_width, canvas_height), Image.LANCZOS)
            
            # 转换为Tkinter图像
            self.stats_tk_img = ImageTk.PhotoImage(stats_img)
            
            # 清除画布并显示新图像
            self.stats_canvas.delete("all")
            self.stats_canvas.create_image(0, 0, anchor=tk.NW, image=self.stats_tk_img)
        except Exception as e:
            self.log(f"状态信息更新错误: {str(e)}")
    
    def _update_loop(self):
        """更新循环，在单独的线程中运行"""
        while self.running:
            if self.update_flag:
                # 在主线程中更新GUI
                self.root.after(0, self._update_gui)
            time.sleep(0.1)  # 降低CPU使用率
    
    def _update_gui(self):
        """更新GUI，在主线程中调用"""
        self._update_map_image()
        self._update_stats_image()
    
    def update(self, occupancy_grid, robot_pose, current_path=None, 
              frontiers=None, selected_frontier=None, exit_position=None,
              start_position=None, current_state=None, performance_stats=None):
        """
        更新GUI显示
        
        参数:
            occupancy_grid: 占用栅格地图，numpy数组
            robot_pose: 机器人位置，(x, y, theta)，单位为米和度
            current_path: 当前规划的路径，路径点列表
            frontiers: 检测到的边界点列表
            selected_frontier: 选择的边界点
            exit_position: 出口位置，(x, y)，单位为米
            start_position: 起始位置，(x, y)，单位为米
            current_state: 当前状态描述
            performance_stats: 性能统计信息
        """
        # 更新可视化器数据
        self.visualizer.update_visualization_data(
            occupancy_grid, robot_pose, current_path, frontiers, 
            selected_frontier, exit_position, start_position, 
            current_state, performance_stats
        )
        
        # 设置更新标志
        self.update_flag = True
        
        # 更新状态栏
        if current_state:
            self.status_bar.config(text=f"当前状态: {current_state}")
    
    def log(self, message):
        """
        添加日志消息
        
        参数:
            message: 日志消息
        """
        timestamp = time.strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}\n"
        
        # 在主线程中更新日志
        self.root.after(0, lambda: self._append_log(log_entry))
    
    def _append_log(self, log_entry):
        """在主线程中添加日志条目"""
        self.log_text.insert(tk.END, log_entry)
        self.log_text.see(tk.END)  # 滚动到最新日志
    
    def _on_start_clicked(self):
        """启动按钮点击处理"""
        self.log("系统启动")
        # 这里添加启动逻辑
    
    def _on_stop_clicked(self):
        """停止按钮点击处理"""
        self.log("系统停止")
        # 这里添加停止逻辑
    
    def _on_reset_clicked(self):
        """重置按钮点击处理"""
        self.log("系统重置")
        # 这里添加重置逻辑
    
    def _on_save_map_clicked(self):
        """保存地图按钮点击处理"""
        self.log("保存地图")
        # 这里添加保存地图逻辑
    
    def _on_load_map_clicked(self):
        """加载地图按钮点击处理"""
        self.log("加载地图")
        # 这里添加加载地图逻辑
    
    def _on_settings_clicked(self):
        """设置按钮点击处理"""
        self.log("打开设置")
        # 这里添加设置逻辑
    
    def run(self):
        """运行GUI主循环"""
        self.root.mainloop()
    
    def close(self):
        """关闭GUI"""
        self.running = False
        if self.update_thread.is_alive():
            self.update_thread.join(timeout=1.0)
        self.root.quit()
        self.root.destroy()

# 测试代码
if __name__ == "__main__":
    # 创建测试地图
    map_size_pixels = 500
    test_map = np.ones((map_size_pixels, map_size_pixels), dtype=np.uint8) * 128  # 未知区域
    
    # 添加一些已知区域
    center = map_size_pixels // 2
    radius = 100
    y, x = np.ogrid[-center:map_size_pixels-center, -center:map_size_pixels-center]
    mask = x*x + y*y <= radius*radius
    test_map[mask] = 0  # 自由空间
    
    # 添加一些障碍物
    for i in range(20):
        x = np.random.randint(0, map_size_pixels)
        y = np.random.randint(0, map_size_pixels)
        r = np.random.randint(5, 20)
        y_grid, x_grid = np.ogrid[-r:r+1, -r:r+1]
        mask = x_grid*x_grid + y_grid*y_grid <= r*r
        
        # 确保坐标有效
        y_min = max(0, y - r)
        y_max = min(map_size_pixels, y + r + 1)
        x_min = max(0, x - r)
        x_max = min(map_size_pixels, x + r + 1)
        
        # 调整掩码大小
        mask_height = y_max - y_min
        mask_width = x_max - x_min
        mask_adjusted = mask[:mask_height, :mask_width]
        
        # 设置障碍物
        test_map[y_min:y_max, x_min:x_max][mask_adjusted] = 255
    
    # 创建测试路径
    test_path = []
    for i in range(20):
        x = center + int(radius * 0.8 * np.cos(i * np.pi / 10))
        y = center + int(radius * 0.8 * np.sin(i * np.pi / 10))
        test_path.append((x, y))
    
    # 创建GUI并运行
    gui = RobotGUI()
    
    # 模拟机器人位置
    robot_pose = (center, center, 0)
    
    # 更新GUI
    gui.update(
        occupancy_grid=test_map,
        robot_pose=robot_pose,
        current_path=test_path,
        frontiers=[(center + radius, center), (center, center + radius)],
        selected_frontier=(center + radius, center),
        exit_position=(center + radius * 1.5, center),
        start_position=(center - radius * 1.5, center),
        current_state="测试模式",
        performance_stats={
            'exploration_time': 10.5,
            'exit_navigation_time': 5.2,
            'return_time': 8.7,
            'total_time': 24.4
        }
    )
    
    # 添加一些日志
    gui.log("系统初始化完成")
    gui.log("开始测试模式")
    gui.log("检测到边界点: 2个")
    gui.log("规划路径: 20个点")
    
    # 运行GUI
    gui.run() 