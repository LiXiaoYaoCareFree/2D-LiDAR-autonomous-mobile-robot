#!/usr/bin/env python3
"""
完全匹配硬件代码的蓝牙控制程序
基于hardware.c的数据格式和bluetooth2.py的显示方式
"""

import re
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import matplotlib
import threading
import serial
import time
import sys
import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext

# 基础配置：解决中文显示
matplotlib.rcParams["font.family"] = ["SimHei", "WenQuanYi Micro Hei", "Heiti TC"]
matplotlib.use('TkAgg')  # 确保GUI后端兼容

class HardwareMatchedBluetooth:
    def __init__(self):
        # 全局变量定义 - 完全匹配hardware.c
        self.radar_data = deque(maxlen=700)  # 雷达数据缓存
        self.axis_angles = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}  # 小车三轴角度
        self.data_buffer = ""  # 串口断帧数据拼接缓冲区
        
        # 机器人状态 - 匹配hardware.c的状态变量
        self.robot_pose = [0.0, 0.0, 0.0]  # x, y, theta
        self.motor_data = {'rpmA': 0.0, 'rpmB': 0.0}
        self.imu_data = {
            'accel': [0.0, 0.0, 0.0],
            'gyro': [0.0, 0.0, 0.0]
        }
        
        # 控制状态
        self.connected = False
        self.serial_port = None
        self.running = False
        
        # 控制命令 - 完全匹配hardware.c的命令
        self.commands = {
            'STOP': '0',      # DIR_STOP
            'FORWARD': '1',   # DIR_FORWARD  
            'BACKWARD': '2',  # DIR_BACKWARD
            'LEFT': '3',      # DIR_LEFT
            'RIGHT': '4',     # DIR_RIGHT
            'UTURN': '5'      # DIR_UTURN
        }
        
        # 串口配置 - 匹配hardware.c的UART1设置
        self.COM_PORT = "COM3"  # 默认串口
        self.BAUDRATE = 115200  # 匹配hardware.c的波特率
        
        # 创建GUI
        self.setup_gui()
        
    def setup_gui(self):
        """设置图形界面"""
        self.root = tk.Tk()
        self.root.title("Hardware Matched Bluetooth Control - Robot System")
        self.root.geometry("1400x900")
        
        # 主框架
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill='both', expand=True, padx=10, pady=5)
        
        # 左侧控制面板
        left_panel = ttk.Frame(main_frame)
        left_panel.pack(side='left', fill='y', padx=(0, 10))
        
        # 连接控制
        self.setup_connection_panel(left_panel)
        
        # 手动控制
        self.setup_control_panel(left_panel)
        
        # 数据显示
        self.setup_data_panel(left_panel)
        
        # 右侧雷达显示
        right_panel = ttk.Frame(main_frame)
        right_panel.pack(side='right', fill='both', expand=True)
        
        self.setup_radar_panel(right_panel)
        
        # 键盘绑定
        self.root.bind('<KeyPress>', self.on_key_press)
        self.root.bind('<KeyRelease>', self.on_key_release)
        self.root.focus_set()
        
    def setup_connection_panel(self, parent):
        """连接控制面板"""
        conn_frame = ttk.LabelFrame(parent, text="Bluetooth Connection")
        conn_frame.pack(fill='x', pady=5)
        
        # 串口设置
        port_frame = ttk.Frame(conn_frame)
        port_frame.pack(fill='x', padx=5, pady=5)
        
        ttk.Label(port_frame, text="Port:").pack(side='left')
        self.port_var = tk.StringVar(value=self.COM_PORT)
        port_entry = ttk.Entry(port_frame, textvariable=self.port_var, width=10)
        port_entry.pack(side='left', padx=5)
        
        ttk.Label(port_frame, text="Baud:").pack(side='left', padx=(20,0))
        self.baud_var = tk.StringVar(value=str(self.BAUDRATE))
        baud_entry = ttk.Entry(port_frame, textvariable=self.baud_var, width=10)
        baud_entry.pack(side='left', padx=5)
        
        # 连接按钮
        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.pack(pady=5)
        
        self.status_label = ttk.Label(conn_frame, text="Disconnected", foreground="red")
        self.status_label.pack()
        
    def setup_control_panel(self, parent):
        """控制面板"""
        control_frame = ttk.LabelFrame(parent, text="Robot Control")
        control_frame.pack(fill='x', pady=5)
        
        # 方向控制按钮
        btn_frame = ttk.Frame(control_frame)
        btn_frame.pack(expand=True, padx=5, pady=5)
        
        # 按钮布局 - 匹配hardware.c的控制命令
        ttk.Button(btn_frame, text="Forward", command=lambda: self.send_command('FORWARD')).grid(row=0, column=1, padx=2, pady=2)
        ttk.Button(btn_frame, text="Left", command=lambda: self.send_command('LEFT')).grid(row=1, column=0, padx=2, pady=2)
        ttk.Button(btn_frame, text="Stop", command=lambda: self.send_command('STOP')).grid(row=1, column=1, padx=2, pady=2)
        ttk.Button(btn_frame, text="Right", command=lambda: self.send_command('RIGHT')).grid(row=1, column=2, padx=2, pady=2)
        ttk.Button(btn_frame, text="Backward", command=lambda: self.send_command('BACKWARD')).grid(row=2, column=1, padx=2, pady=2)
        ttk.Button(btn_frame, text="U-Turn", command=lambda: self.send_command('UTURN')).grid(row=3, column=1, padx=2, pady=2)
        
        # 键盘提示
        ttk.Label(control_frame, text="Keyboard: Arrow Keys + Space", font=('Arial', 8)).pack(pady=2)
        
    def setup_data_panel(self, parent):
        """数据显示面板"""
        data_frame = ttk.LabelFrame(parent, text="Sensor Data")
        data_frame.pack(fill='both', expand=True, pady=5)
        
        # 数据显示文本框
        self.data_text = scrolledtext.ScrolledText(data_frame, height=20, width=45)
        self.data_text.pack(fill='both', expand=True, padx=5, pady=5)
        
    def setup_radar_panel(self, parent):
        """雷达显示面板"""
        radar_frame = ttk.LabelFrame(parent, text="Real-time Radar Scan")
        radar_frame.pack(fill='both', expand=True)
        
        # 创建极坐标图表 - 借鉴bluetooth2.py
        self.fig = plt.figure(figsize=(10, 8))
        self.ax = plt.subplot(111, polar=True)
        
        # 图表基础配置
        self.ax.set_theta_zero_location('N')  # 0°方向朝北（上）
        self.ax.set_theta_direction(-1)  # 角度顺时针递增
        self.ax.set_ylim(0, 10)  # 初始距离范围（米）
        self.ax.set_ylabel('距离 (米)', labelpad=20, fontsize=10)
        self.ax.set_title(
            "小车雷达实时扫描（50ms/帧 | 鼠标滚轮缩放）",
            pad=20, fontsize=12, fontweight="bold"
        )
        
        # 初始化绘图元素
        self.radar_line, = self.ax.plot([], [], 'bo', markersize=3)  # 雷达扫描点（蓝色）
        self.angle_text = plt.text(
            1.25, 0.5, "等待数据...",
            transform=self.ax.transAxes,
            fontsize=11,
            bbox=dict(facecolor="lightblue", alpha=0.9, boxstyle="round,pad=0.5")
        )
        
        # 鼠标滚轮缩放功能
        self.manual_zoom_flag = [False]
        self.ZOOM_RATIO = 0.8
        self.MIN_DIST = 0.1
        self.MAX_DIST = 50.0
        
        def on_scroll(event):
            if event.inaxes != self.ax:
                return
            current_max = self.ax.get_ylim()[1]
            new_max = current_max * self.ZOOM_RATIO if event.step > 0 else current_max / self.ZOOM_RATIO
            new_max = max(self.MIN_DIST, min(new_max, self.MAX_DIST))
            self.ax.set_ylim(0, new_max)
            self.ax.figure.canvas.draw()
            self.manual_zoom_flag[0] = True
            self.log_data(f"[缩放] 距离范围：0 ~ {new_max:.2f}米")
        
        self.fig.canvas.mpl_connect('scroll_event', on_scroll)
        
        # 启动动画
        self.ani = FuncAnimation(
            self.fig,
            self.update_radar_plot,
            fargs=(self.ax, self.radar_line, self.angle_text, self.manual_zoom_flag),
            interval=50,  # 每50ms更新一帧
            blit=True,
            cache_frame_data=False
        )
        
    def parse_buffer_data(self):
        """解析缓冲区数据 - 完全匹配hardware.c的数据格式"""
        global data_buffer, axis_angles, radar_data
        parsed_flag = False
        
        # 1. 解析三轴角度数据（格式：roll:12.34, pitch:-5.67, yaw:90.0）
        angle_matches = re.findall(r'(roll|pitch|yaw):([-+]?\d*\.\d+)', self.data_buffer)
        if angle_matches:
            temp_angles = self.axis_angles.copy()
            for axis_name, value_str in angle_matches:
                try:
                    temp_angles[axis_name] = float(value_str)
                    self.log_data(f"[角度解析] {axis_name}: {temp_angles[axis_name]:.2f}°")
                    parsed_flag = True
                except Exception as e:
                    self.log_data(f"[角度错误] {axis_name}:{value_str} | 错误：{e}")
            self.axis_angles.update(temp_angles)
            # 从缓冲区移除已解析的角度数据
            self.data_buffer = re.sub(r'(roll|pitch|yaw):[-+]?\d*\.\d+', "", self.data_buffer)
        
        # 2. 解析雷达数据（格式：A:123.45, D:0.678, Q:100） - 匹配hardware.c的SendLidarToBluetooth格式
        radar_matches = re.findall(r'A:([-+]?\d*\.\d+),\s*D:([-+]?\d*\.\d+),\s*Q:(\d+)', self.data_buffer)
        
        if radar_matches:
            new_points = []
            for angle_str, dist_str, quality_str in radar_matches:
                try:
                    angle_deg = float(angle_str)
                    distance_m = float(dist_str)
                    quality = int(quality_str)
                    
                    # 质量过滤 - 匹配hardware.c的质量要求
                    if quality > 0 and 50 <= distance_m <= 12000:
                        new_points.append((np.radians(angle_deg), distance_m))
                        self.log_data(f"[雷达解析] 角度：{angle_deg:.1f}° | 距离：{distance_m:.3f}m | 质量：{quality}")
                        parsed_flag = True
                except Exception as e:
                    self.log_data(f"[雷达错误] {angle_str},{dist_str},{quality_str} | 错误：{e}")
            
            # 合并历史数据与新数据
            combined = sorted(deque(self.radar_data) + deque(new_points), key=lambda x: x[0])
            
            # 异常值检测
            filtered = []
            window_size = 3
            max_diff_threshold = 1
            std_factor = 1.0
            
            for i, (angle, dist) in enumerate(combined):
                neighbors = []
                for j in range(max(0, i - window_size), i):
                    neighbors.append(combined[j][1])
                for j in range(i + 1, min(len(combined), i + 1 + window_size)):
                    neighbors.append(combined[j][1])
                
                is_abnormal = False
                if len(neighbors) >= 2:
                    neighbor_mean = np.mean(neighbors)
                    neighbor_std = np.std(neighbors)
                    diff_from_mean = abs(dist - neighbor_mean)
                    all_neighbors_exceed = all(abs(dist - n) > max_diff_threshold for n in neighbors)
                    
                    if diff_from_mean > std_factor * neighbor_std and all_neighbors_exceed:
                        is_abnormal = True
                        self.log_data(f"[异常过滤] 角度：{np.degrees(angle):.1f}° | 距离：{dist:.3f}m")
                
                if not is_abnormal:
                    filtered.append((angle, dist))
            
            # 更新雷达数据
            angle_dict = {angle: dist for angle, dist in filtered}
            self.radar_data.clear()
            self.radar_data.extend(sorted(angle_dict.items(), key=lambda x: x[0]))
            
            # 清理缓冲区
            self.data_buffer = re.sub(r'A:[-+]?\d*\.\d+,\s*D:[-+]?\d*\.\d+,\s*Q:\d+', "", self.data_buffer)
        
        # 3. 解析电机数据（格式：MotorA:123.45,RPMB:67.89） - 匹配hardware.c的电机数据格式
        motor_matches = re.findall(r'MotorA:([-+]?\d*\.\d+),RPMB:([-+]?\d*\.\d+)', self.data_buffer)
        if motor_matches:
            for rpmA_str, rpmB_str in motor_matches:
                try:
                    rpmA = float(rpmA_str)
                    rpmB = float(rpmB_str)
                    self.motor_data = {'rpmA': rpmA, 'rpmB': rpmB}
                    self.log_data(f"[电机数据] A={rpmA:.1f}, B={rpmB:.1f}")
                    parsed_flag = True
                except Exception as e:
                    self.log_data(f"[电机错误] {rpmA_str},{rpmB_str} | 错误：{e}")
        
        # 4. 解析IMU数据（格式：AccX:1.23,AccY:4.56,AccZ:7.89,GyroX:0.12,GyroY:0.34,GyroZ:0.56） - 匹配hardware.c的IMU数据格式
        imu_matches = re.findall(r'AccX:([-+]?\d*\.\d+),AccY:([-+]?\d*\.\d+),AccZ:([-+]?\d*\.\d+),GyroX:([-+]?\d*\.\d+),GyroY:([-+]?\d*\.\d+),GyroZ:([-+]?\d*\.\d+)', self.data_buffer)
        if imu_matches:
            for accX, accY, accZ, gyroX, gyroY, gyroZ in imu_matches:
                try:
                    accel = [float(accX), float(accY), float(accZ)]
                    gyro = [float(gyroX), float(gyroY), float(gyroZ)]
                    self.imu_data = {'accel': accel, 'gyro': gyro}
                    self.log_data(f"[IMU数据] Acc={accel}, Gyro={gyro}")
                    parsed_flag = True
                except Exception as e:
                    self.log_data(f"[IMU错误] {accX},{accY},{accZ},{gyroX},{gyroY},{gyroZ} | 错误：{e}")
        
        # 缓冲区过长时清理
        if not parsed_flag and len(self.data_buffer) > 600:
            self.log_data(f"[缓冲区清理] 丢弃前500字符")
            self.data_buffer = self.data_buffer[-300:]
        
        return self.data_buffer, self.radar_data
    
    def serial_read_thread(self):
        """串口读取线程 - 匹配hardware.c的UART1接收方式"""
        try:
            # 打开串口
            self.serial_port = serial.Serial(
                port=self.COM_PORT,
                baudrate=self.BAUDRATE,
                timeout=0.1,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
            self.log_data(f"✅ 成功打开串口：{self.COM_PORT}（波特率：{self.BAUDRATE}）")
            self.connected = True
            self.status_label.config(text="Connected", foreground="green")
            self.connect_btn.config(text="Disconnect")
            
            # 持续读取数据
            while self.running and self.connected:
                if self.serial_port.in_waiting > 0:
                    # 读取一行数据（按换行符分割）
                    line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.data_buffer += line
                        self.log_data(f"[串口数据] 收到：{line[:50]}")
                        self.parse_buffer_data()  # 解析数据
                time.sleep(0.01)  # 降低CPU占用
                
        except serial.SerialException as e:
            self.log_data(f"❌ 串口异常：{e}")
            self.connected = False
            self.status_label.config(text="Disconnected", foreground="red")
            self.connect_btn.config(text="Connect")
        except Exception as e:
            self.log_data(f"❌ 数据读取异常：{e}")
        finally:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
                self.log_data("📌 串口已安全关闭")
    
    def update_radar_plot(self, frame, ax, radar_line, angle_text, manual_zoom_flag):
        """更新极坐标图和角度文本 - 借鉴bluetooth2.py的更新方式"""
        # 1. 雷达数据更新
        if self.radar_data:
            angles_rad, distances_m = zip(*self.radar_data)
            radar_line.set_data(angles_rad, distances_m)
            
            # 仅当未手动缩放时，自动适配距离范围
            if not manual_zoom_flag[0]:
                max_dist = max(distances_m) if max(distances_m) > 0 else 10
                ax.set_ylim(0, max_dist * 1.1)  # 预留10%余量
        
        # 2. 角度文本更新
        angle_text.set_text(
            "小车三轴角度（实时更新）\n"
            f"横滚角(roll)：{self.axis_angles['roll']:.2f}°\n"
            f"俯仰角(pitch)：{self.axis_angles['pitch']:.2f}°\n"
            f"偏航角(yaw)：{self.axis_angles['yaw']:.2f}°"
        )
        
        return radar_line, angle_text
    
    def toggle_connection(self):
        """切换连接状态"""
        if not self.connected:
            self.connect()
        else:
            self.disconnect()
    
    def connect(self):
        """连接蓝牙"""
        try:
            self.COM_PORT = self.port_var.get()
            self.BAUDRATE = int(self.baud_var.get())
            
            # 串口预检查
            test_ser = serial.Serial(self.COM_PORT, self.BAUDRATE, timeout=0.5)
            test_ser.close()
            
            self.running = True
            self.connected = False
            
            # 启动串口读取线程
            self.serial_thread = threading.Thread(target=self.serial_read_thread, daemon=True)
            self.serial_thread.start()
            
        except serial.SerialException as e:
            messagebox.showerror("Connection Error", f"Cannot connect to port: {e}")
        except Exception as e:
            messagebox.showerror("Connection Error", f"Error: {e}")
    
    def disconnect(self):
        """断开连接"""
        self.running = False
        self.connected = False
        
        if self.serial_port:
            self.serial_port.close()
            self.serial_port = None
            
        self.connect_btn.config(text="Connect")
        self.status_label.config(text="Disconnected", foreground="red")
        self.log_data("Disconnected")
    
    def send_command(self, command):
        """发送控制命令 - 完全匹配hardware.c的命令格式"""
        if self.connected and self.serial_port:
            try:
                cmd = self.commands[command]
                self.serial_port.write(cmd.encode())
                self.log_data(f"Sent: {command} -> {cmd}")
            except Exception as e:
                self.log_data(f"Send command failed: {e}")
    
    def log_data(self, message):
        """记录数据"""
        timestamp = time.strftime("%H:%M:%S")
        self.data_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.data_text.see(tk.END)
        
        # 限制显示行数
        lines = self.data_text.get("1.0", tk.END).split('\n')
        if len(lines) > 200:
            self.data_text.delete("1.0", "100.0")
    
    def on_key_press(self, event):
        """键盘按下事件"""
        key = event.keysym
        if key == 'Up':
            self.send_command('FORWARD')
        elif key == 'Down':
            self.send_command('BACKWARD')
        elif key == 'Left':
            self.send_command('LEFT')
        elif key == 'Right':
            self.send_command('RIGHT')
        elif key == 'space':
            self.send_command('STOP')
    
    def on_key_release(self, event):
        """键盘释放事件"""
        key = event.keysym
        if key in ['Up', 'Down', 'Left', 'Right']:
            self.send_command('STOP')
    
    def run(self):
        """运行主程序"""
        self.root.mainloop()
    
    def __del__(self):
        """析构函数"""
        if self.connected:
            self.disconnect()

if __name__ == "__main__":
    app = HardwareMatchedBluetooth()
    app.run()
