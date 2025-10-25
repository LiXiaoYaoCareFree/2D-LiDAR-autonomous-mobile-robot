#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
迷宫SLAM仿真程序的图形用户界面(GUI)启动器
"""

import tkinter as tk
from tkinter import ttk
from tkinter import font as tk_font
from tkinter import messagebox
import json
import glob
import subprocess
import sys
import platform

class LauncherApp:
    """GUI启动器主程序"""

    def __init__(self, root):
        self.root = root
        self.setup_window()
        
        self.map_files = self.find_map_files()
        if not self.map_files:
            self.show_error_and_exit("未找到任何地图文件 (*.json)！")
            return
            
        self.selected_map = tk.StringVar()
        
        self.setup_styles()
        self.create_widgets()
        
        # 初始加载第一个地图的预览
        self.selected_map.set(self.map_files[0])
        self.on_map_select()

    def setup_window(self):
        """设置主窗口属性"""
        self.root.title("迷宫SLAM仿真启动器")
        self.root.geometry("500x650")
        self.root.resizable(False, False)
        
        # 窗口居中
        screen_width = self.root.winfo_screenwidth()
        screen_height = self.root.winfo_screenheight()
        x = (screen_width / 2) - (500 / 2)
        y = (screen_height / 2) - (650 / 2)
        self.root.geometry(f'+{int(x)}+{int(y)}')

    def setup_styles(self):
        """配置ttk控件的样式"""
        style = ttk.Style()
        style.theme_use('clam') # 使用一个更现代的主题
        
        style.configure("TFrame", background="#f0f0f0")
        style.configure("TLabel", background="#f0f0f0", font=("Microsoft YaHei", 12))
        style.configure("Title.TLabel", font=("Microsoft YaHei", 18, "bold"))
        style.configure("TButton", font=("Microsoft YaHei", 12), padding=10)
        style.configure("Start.TButton", foreground="white", background="#4CAF50") # 绿色
        style.configure("Exit.TButton", foreground="white", background="#f44336")  # 红色
        style.map("Start.TButton", background=[('active', '#45a049')])
        style.map("Exit.TButton", background=[('active', '#da190b')])
        
    def find_map_files(self):
        """查找当前目录下的所有.json地图文件"""
        # 优先寻找数字命名的文件并排序
        numeric_maps = sorted([f for f in glob.glob('[0-9]*.json')])
        other_maps = sorted([f for f in glob.glob('*.json') if f not in numeric_maps])
        return numeric_maps + other_maps

    def create_widgets(self):
        """创建并布局所有UI控件"""
        main_frame = ttk.Frame(self.root, padding="20")
        main_frame.pack(expand=True, fill=tk.BOTH)

        # 标题
        title_label = ttk.Label(main_frame, text="迷宫 SLAM 仿真", style="Title.TLabel")
        title_label.pack(pady=(0, 20))

        # 地图选择
        map_frame = ttk.Frame(main_frame, padding=10)
        map_frame.pack(fill=tk.X)
        
        map_label = ttk.Label(map_frame, text="选择一个地图:")
        map_label.pack(side=tk.LEFT, padx=(0, 10))
        
        self.map_selector = ttk.Combobox(
            map_frame, 
            textvariable=self.selected_map,
            values=self.map_files,
            state='readonly',
            font=("Microsoft YaHei", 11)
        )
        self.map_selector.pack(side=tk.LEFT, expand=True, fill=tk.X)
        self.map_selector.bind("<<ComboboxSelected>>", self.on_map_select)

        # 地图预览画布
        self.preview_canvas = tk.Canvas(main_frame, width=400, height=400, bg="white", highlightthickness=1, highlightbackground="#ccc")
        self.preview_canvas.pack(pady=20)

        # 按钮
        button_frame = ttk.Frame(main_frame)
        button_frame.pack(fill=tk.X, pady=(10, 0))
        button_frame.columnconfigure(0, weight=1)
        button_frame.columnconfigure(1, weight=1)

        start_button = ttk.Button(button_frame, text="开始仿真", command=self.launch_simulation, style="Start.TButton")
        start_button.grid(row=0, column=0, padx=5, sticky=tk.EW)

        exit_button = ttk.Button(button_frame, text="退出", command=self.root.quit, style="Exit.TButton")
        exit_button.grid(row=0, column=1, padx=5, sticky=tk.EW)

    def on_map_select(self, event=None):
        """当地图选择变化时，更新预览"""
        self.preview_map(self.selected_map.get())

    def preview_map(self, map_file):
        """在画布上绘制地图预览"""
        self.preview_canvas.delete("all")
        
        try:
            with open(map_file, 'r') as f:
                map_data = json.load(f)
        except Exception as e:
            self.preview_canvas.create_text(200, 200, text=f"无法加载地图:\n{e}", fill="red", font=("Microsoft YaHei", 10))
            return

        segments = map_data.get("segments", [])
        start_point = map_data.get("start_point")

        if not segments:
            self.preview_canvas.create_text(200, 200, text="地图文件中没有墙体信息", font=("Microsoft YaHei", 10))
            return
            
        # 计算地图边界和缩放比例
        all_points = [p for seg in segments for p in (seg["start"], seg["end"])]
        if not all_points:
            min_x, max_x, min_y, max_y = 0, 10, 0, 10
        else:
            min_x = min(p[0] for p in all_points)
            max_x = max(p[0] for p in all_points)
            min_y = min(p[1] for p in all_points)
            max_y = max(p[1] for p in all_points)

        map_width = max_x - min_x
        map_height = max_y - min_y
        
        padding = 20 # 画布边距
        canvas_width = self.preview_canvas.winfo_width()
        canvas_height = self.preview_canvas.winfo_height()
        
        if map_width == 0 or map_height == 0:
            scale = 1
        else:
            scale_x = (canvas_width - 2 * padding) / map_width
            scale_y = (canvas_height - 2 * padding) / map_height
            scale = min(scale_x, scale_y)

        # 转换坐标函数
        def transform(x, y):
            # 将(min_x, min_y)作为原点
            tx = (x - min_x) * scale + padding
            # Y轴翻转，因为tkinter的y轴向下
            ty = (max_y - y) * scale + padding
            return tx, ty

        # 绘制墙体
        for seg in segments:
            x1, y1 = transform(seg["start"][0], seg["start"][1])
            x2, y2 = transform(seg["end"][0], seg["end"][1])
            self.preview_canvas.create_line(x1, y1, x2, y2, fill="black", width=2)
            
        # 绘制起点
        if start_point:
            sx, sy = transform(start_point[0], start_point[1])
            radius = 5
            self.preview_canvas.create_oval(sx-radius, sy-radius, sx+radius, sy+radius, fill="green", outline="darkgreen")
            self.preview_canvas.create_text(sx, sy+15, text="起点", fill="green", font=("Microsoft YaHei", 9))
    
    def launch_simulation(self):
        """启动主仿真程序"""
        map_file = self.selected_map.get()
        if not map_file:
            messagebox.showwarning("提示", "请先选择一个地图！")
            return
            
        print(f"正在使用地图 '{map_file}' 启动仿真...")
        
        # 使用subprocess来运行，这样更灵活
        # 根据操作系统选择python可执行文件
        python_executable = sys.executable
        command = [python_executable, "maze_slam_simulation.py", "--map", map_file]
        
        try:
            # Popen是非阻塞的, 启动后GUI可以立即关闭
            subprocess.Popen(command)
            self.root.quit() # 关闭启动器
        except FileNotFoundError:
            self.show_error_and_exit(f"错误: 无法找到 'maze_slam_simulation.py'。\n请确保它和启动器在同一个目录下。")
        except Exception as e:
            self.show_error_and_exit(f"启动仿真时发生未知错误: {e}")

    def show_error_and_exit(self, message):
        """显示错误消息并退出程序"""
        messagebox.showerror("严重错误", message)
        self.root.quit()

def set_font():
    """根据操作系统设置合适的UI字体"""
    os_system = platform.system()
    if os_system == 'Windows':
        font_name = 'Microsoft YaHei UI'
    elif os_system == 'Darwin': # macOS
        font_name = 'PingFang SC'
    else: # Linux
        font_name = 'WenQuanYi Zen Hei'
    
    try:
        # 更新默认字体
        default_font = tk_font.nametofont("TkDefaultFont")
        default_font.configure(family=font_name, size=10)
        
        # 更新Text和Entry等控件的字体
        text_font = tk_font.nametofont("TkTextFont")
        text_font.configure(family=font_name, size=10)

        fixed_font = tk_font.nametofont("TkFixedFont")
        fixed_font.configure(family=font_name, size=10)
        
    except tk.TclError:
        print(f"警告: 未能加载推荐字体 '{font_name}'。将使用系统默认字体。")

if __name__ == "__main__":
    root = tk.Tk()
    set_font()
    app = LauncherApp(root)
    root.mainloop()