#!/usr/bin/env python3
'''
simulator.py - 激光雷达和机器人模拟器

该模块实现了激光雷达数据模拟和随机迷宫生成功能，用于测试和演示。
'''

import os
import time
import math
import random
import threading
import numpy as np
from queue import Queue
import matplotlib.pyplot as plt
from scipy.ndimage import binary_dilation, generate_binary_structure

class LidarSimulator:
    """激光雷达模拟器类，模拟激光雷达数据和随机迷宫环境"""
    
    def __init__(self, map_size_meters=32, map_size_pixels=800):
        """
        初始化激光雷达模拟器
        
        参数:
            map_size_meters: 地图大小(米)
            map_size_pixels: 地图大小(像素)
        """
        self.map_size_meters = map_size_meters
        self.map_size_pixels = map_size_pixels
        self.cell_size = map_size_meters / map_size_pixels
        
        # 创建地图 (0=自由空间, 255=障碍物)
        self.map = np.zeros((map_size_pixels, map_size_pixels), dtype=np.uint8)
        
        # 激光雷达参数
        self.scan_size = 360  # 每次扫描的点数
        self.max_range_meters = 10.0  # 最大测量距离(米)
        
        # 机器人参数
        self.robot_radius_meters = 0.2  # 机器人半径(米)
        self.robot_speed_meters_per_sec = 0.5  # 机器人速度(米/秒)
        self.robot_turn_speed_degrees_per_sec = 45  # 机器人转向速度(度/秒)
        
        # 迷宫参数
        self.wall_thickness_pixels = 10  # 墙壁厚度(像素)
        self.min_corridor_width_pixels = 30  # 最小走廊宽度(像素)
        
        # 噪声参数
        self.distance_noise_stddev = 0.05  # 距离测量噪声标准差(米)
        self.angle_noise_stddev = 0.5  # 角度测量噪声标准差(度)
    
    def generate_random_maze(self):
        """生成随机迷宫"""
        # 清空地图
        self.map.fill(0)
        
        # 添加外墙
        border_width = self.wall_thickness_pixels
        self.map[:border_width, :] = 255  # 上边界
        self.map[-border_width:, :] = 255  # 下边界
        self.map[:, :border_width] = 255  # 左边界
        self.map[:, -border_width:] = 255  # 右边界
        
        # 随机生成迷宫
        self._generate_recursive_division_maze()
        
        # 添加一个出口
        self._add_exit()
        
        # 确保机器人起始位置周围是空的
        center = self.map_size_pixels // 2
        radius = int(self.robot_radius_meters / self.cell_size) * 2
        y, x = np.ogrid[-radius:radius+1, -radius:radius+1]
        mask = x*x + y*y <= radius*radius
        self.map[center-radius:center+radius+1, center-radius:center+radius+1][mask] = 0
        
        # 膨胀障碍物，确保机器人不会碰撞
        robot_radius_pixels = int(self.robot_radius_meters / self.cell_size)
        structure = generate_binary_structure(2, 2)
        obstacle_map = self.map > 0
        dilated_obstacles = binary_dilation(obstacle_map, structure=structure, iterations=robot_radius_pixels)
        self.map = np.where(dilated_obstacles, 255, 0).astype(np.uint8)
    
    def _generate_recursive_division_maze(self):
        """使用递归分割算法生成迷宫"""
        # 初始化为空地图
        width = self.map_size_pixels
        height = self.map_size_pixels
        
        # 递归分割
        self._divide(0, 0, width, height)
    
    def _divide(self, x, y, width, height):
        """
        递归分割算法
        
        参数:
            x: 区域左上角x坐标
            y: 区域左上角y坐标
            width: 区域宽度
            height: 区域高度
        """
        # 如果区域太小，不再分割
        min_size = self.min_corridor_width_pixels * 2 + self.wall_thickness_pixels
        if width < min_size or height < min_size:
            return
        
        # 决定水平或垂直分割
        horizontal = np.random.random() > 0.5 if width > height else False
        
        if horizontal:
            # 水平分割
            wall_y = y + np.random.randint(self.min_corridor_width_pixels, height - self.min_corridor_width_pixels)
            self.map[wall_y:wall_y+self.wall_thickness_pixels, x:x+width] = 255
            
            # 在墙上开一个门
            door_x = x + np.random.randint(0, width - self.min_corridor_width_pixels)
            self.map[wall_y:wall_y+self.wall_thickness_pixels, door_x:door_x+self.min_corridor_width_pixels] = 0
            
            # 递归分割上下两个区域
            self._divide(x, y, width, wall_y - y)
            self._divide(x, wall_y + self.wall_thickness_pixels, width, height - (wall_y - y) - self.wall_thickness_pixels)
        else:
            # 垂直分割
            wall_x = x + np.random.randint(self.min_corridor_width_pixels, width - self.min_corridor_width_pixels)
            self.map[y:y+height, wall_x:wall_x+self.wall_thickness_pixels] = 255
            
            # 在墙上开一个门
            door_y = y + np.random.randint(0, height - self.min_corridor_width_pixels)
            self.map[door_y:door_y+self.min_corridor_width_pixels, wall_x:wall_x+self.wall_thickness_pixels] = 0
            
            # 递归分割左右两个区域
            self._divide(x, y, wall_x - x, height)
            self._divide(wall_x + self.wall_thickness_pixels, y, width - (wall_x - x) - self.wall_thickness_pixels, height)
    
    def _add_exit(self):
        """在迷宫边界添加一个出口"""
        border_width = self.wall_thickness_pixels
        exit_size = self.min_corridor_width_pixels
        
        # 随机选择一个边
        side = np.random.randint(0, 4)
        
        if side == 0:  # 上边
            x = np.random.randint(exit_size, self.map_size_pixels - exit_size * 2)
            self.map[:border_width, x:x+exit_size] = 0
        elif side == 1:  # 右边
            y = np.random.randint(exit_size, self.map_size_pixels - exit_size * 2)
            self.map[y:y+exit_size, -border_width:] = 0
        elif side == 2:  # 下边
            x = np.random.randint(exit_size, self.map_size_pixels - exit_size * 2)
            self.map[-border_width:, x:x+exit_size] = 0
        else:  # 左边
            y = np.random.randint(exit_size, self.map_size_pixels - exit_size * 2)
            self.map[y:y+exit_size, :border_width] = 0
    
    def get_scan_data(self, pose):
        """
        获取模拟的激光扫描数据
        
        参数:
            pose: 机器人位置，(x, y, theta)，单位为米和度
            
        返回:
            scan_data: 激光扫描数据，包含距离和角度
        """
        # 提取位姿
        x_meters, y_meters, theta_degrees = pose
        
        # 转换为像素坐标
        x_pixels = int(x_meters / self.cell_size)
        y_pixels = int(y_meters / self.cell_size)
        
        # 确保坐标在地图范围内
        if x_pixels < 0 or x_pixels >= self.map_size_pixels or y_pixels < 0 or y_pixels >= self.map_size_pixels:
            return None
        
        # 计算每个角度的激光测量
        angles = np.linspace(0, 359, self.scan_size)
        distances = []
        
        for angle in angles:
            # 计算激光方向（全局坐标系）
            global_angle = (theta_degrees + angle) % 360
            rad_angle = global_angle * np.pi / 180
            
            # 计算激光线上的点
            max_range_pixels = int(self.max_range_meters / self.cell_size)
            
            # 使用Bresenham算法计算激光线上的点
            line_points = self._bresenham_line(x_pixels, y_pixels, 
                                              x_pixels + int(max_range_pixels * np.cos(rad_angle)),
                                              y_pixels + int(max_range_pixels * np.sin(rad_angle)))
            
            # 检查激光线上的点是否为障碍物
            hit_distance = self.max_range_meters
            for px, py in line_points:
                # 检查点是否在地图范围内
                if px < 0 or px >= self.map_size_pixels or py < 0 or py >= self.map_size_pixels:
                    break
                
                # 检查点是否为障碍物
                if self.map[py, px] > 0:
                    # 计算距离
                    dx = (px - x_pixels) * self.cell_size
                    dy = (py - y_pixels) * self.cell_size
                    hit_distance = np.sqrt(dx*dx + dy*dy)
                    break
            
            # 添加噪声
            if hit_distance < self.max_range_meters:
                hit_distance += np.random.normal(0, self.distance_noise_stddev)
                hit_distance = max(0, hit_distance)  # 确保距离非负
            
            distances.append(hit_distance)
        
        # 添加角度噪声
        noisy_angles = angles + np.random.normal(0, self.angle_noise_stddev, self.scan_size)
        noisy_angles = noisy_angles % 360
        
        return {
            'distances': distances,
            'angles': noisy_angles.tolist()
        }
    
    def _bresenham_line(self, x0, y0, x1, y1):
        """
        Bresenham直线算法，用于计算激光线上的点
        
        参数:
            x0, y0: 起点坐标
            x1, y1: 终点坐标
            
        返回:
            points: 线上的点列表
        """
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        while True:
            points.append((x0, y0))
            
            if x0 == x1 and y0 == y1:
                break
                
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
        
        return points
    
    def move_to(self, current_pose, target_pose):
        """
        模拟机器人移动
        
        参数:
            current_pose: 当前位置，(x, y, theta)，单位为米和度
            target_pose: 目标位置，(x, y, theta)，单位为米和度
            
        返回:
            new_pose: 新位置，(x, y, theta)，单位为米和度
        """
        # 提取当前位姿和目标位姿
        x, y, theta = current_pose
        target_x, target_y, target_theta = target_pose
        
        # 计算距离和角度差
        dx = target_x - x
        dy = target_y - y
        distance = np.sqrt(dx*dx + dy*dy)
        
        # 计算朝向目标的角度
        target_angle = np.arctan2(dy, dx) * 180 / np.pi
        
        # 计算角度差
        angle_diff = (target_angle - theta + 180) % 360 - 180
        
        # 模拟转向
        if abs(angle_diff) > 1:
            # 计算转向时间
            turn_time = abs(angle_diff) / self.robot_turn_speed_degrees_per_sec
            
            # 模拟时间流逝
            time.sleep(min(turn_time, 0.1))
            
            # 更新角度
            if angle_diff > 0:
                theta += min(angle_diff, self.robot_turn_speed_degrees_per_sec * 0.1)
            else:
                theta -= min(abs(angle_diff), self.robot_turn_speed_degrees_per_sec * 0.1)
            
            # 规范化角度
            theta = theta % 360
            
            return (x, y, theta)
        
        # 模拟前进
        if distance > 0.01:
            # 计算移动时间
            move_time = distance / self.robot_speed_meters_per_sec
            
            # 模拟时间流逝
            time.sleep(min(move_time, 0.1))
            
            # 计算移动距离
            move_distance = min(distance, self.robot_speed_meters_per_sec * 0.1)
            
            # 更新位置
            rad_theta = theta * np.pi / 180
            x += move_distance * np.cos(rad_theta)
            y += move_distance * np.sin(rad_theta)
            
            return (x, y, theta)
        
        # 如果已经到达目标位置，调整朝向
        if abs(target_theta - theta) > 1:
            # 计算角度差
            angle_diff = (target_theta - theta + 180) % 360 - 180
            
            # 计算转向时间
            turn_time = abs(angle_diff) / self.robot_turn_speed_degrees_per_sec
            
            # 模拟时间流逝
            time.sleep(min(turn_time, 0.1))
            
            # 更新角度
            if angle_diff > 0:
                theta += min(angle_diff, self.robot_turn_speed_degrees_per_sec * 0.1)
            else:
                theta -= min(abs(angle_diff), self.robot_turn_speed_degrees_per_sec * 0.1)
            
            # 规范化角度
            theta = theta % 360
        
        return (x, y, theta)
    
    def visualize_map(self):
        """可视化地图"""
        plt.figure(figsize=(10, 10))
        plt.imshow(self.map, cmap='gray')
        plt.title("模拟迷宫地图")
        plt.colorbar(label='占用状态')
        plt.show()
    
    def visualize_scan(self, pose, scan_data=None):
        """
        可视化激光扫描
        
        参数:
            pose: 机器人位置，(x, y, theta)，单位为米和度
            scan_data: 激光扫描数据，如果为None则重新获取
        """
        if scan_data is None:
            scan_data = self.get_scan_data(pose)
        
        if scan_data is None:
            print("无法获取扫描数据")
            return
        
        # 提取位姿
        x_meters, y_meters, theta_degrees = pose
        
        # 转换为像素坐标
        x_pixels = int(x_meters / self.cell_size)
        y_pixels = int(y_meters / self.cell_size)
        
        # 创建图像
        plt.figure(figsize=(10, 10))
        
        # 显示地图
        plt.imshow(self.map, cmap='gray')
        
        # 绘制机器人位置
        plt.plot(x_pixels, y_pixels, 'ro', markersize=10)
        
        # 绘制机器人朝向
        rad_theta = theta_degrees * np.pi / 180
        dx = 20 * np.cos(rad_theta)
        dy = 20 * np.sin(rad_theta)
        plt.arrow(x_pixels, y_pixels, dx, dy, head_width=10, head_length=10, fc='r', ec='r')
        
        # 绘制激光扫描点
        distances = scan_data['distances']
        angles = scan_data['angles']
        
        for i in range(len(distances)):
            if distances[i] < self.max_range_meters:
                # 计算扫描点的全局坐标
                global_angle = (theta_degrees + angles[i]) % 360
                rad_angle = global_angle * np.pi / 180
                
                # 计算扫描点的像素坐标
                scan_x = x_pixels + int(distances[i] / self.cell_size * np.cos(rad_angle))
                scan_y = y_pixels + int(distances[i] / self.cell_size * np.sin(rad_angle))
                
                # 绘制扫描线
                plt.plot([x_pixels, scan_x], [y_pixels, scan_y], 'y-', alpha=0.3)
                
                # 绘制扫描点
                plt.plot(scan_x, scan_y, 'yo', markersize=3)
        
        plt.title("激光扫描可视化")
        plt.xlabel("X (像素)")
        plt.ylabel("Y (像素)")
        plt.show()

class RobotSimulator:
    """机器人模拟器类，生成模拟的激光雷达和里程计数据"""
    
    def __init__(self, map_file=None, sensor_queue=None, command_queue=None):
        """
        初始化机器人模拟器
        
        参数:
            map_file: 地图文件，如果为None，则使用默认地图
            sensor_queue: 传感器数据队列
            command_queue: 命令队列
        """
        self.sensor_queue = sensor_queue if sensor_queue else Queue()
        self.command_queue = command_queue if command_queue else Queue()
        
        # 加载地图
        self.map = self._load_map(map_file)
        
        # 地图参数
        self.map_size_pixels = self.map.shape[0]
        self.map_size_meters = 32.0
        self.cell_size = self.map_size_meters / self.map_size_pixels
        
        # 机器人参数
        self.position = (self.map_size_meters / 2, self.map_size_meters / 2, 0)  # (x, y, theta)
        self.velocity = (0, 0)  # (linear, angular)
        
        # 激光雷达参数
        self.lidar_range = 5.0  # 米
        self.lidar_angles = np.linspace(0, 2*np.pi, 360, endpoint=False)  # 360个点
        
        # 运行标志
        self.running = False
        
        # 命令处理锁
        self.lock = threading.Lock()
        
        # 当前目标位置
        self.target_position = None
    
    def start(self):
        """启动模拟器"""
        self.running = True
        
        # 创建传感器数据生成线程
        sensor_thread = threading.Thread(target=self._sensor_loop)
        sensor_thread.daemon = True
        sensor_thread.start()
        
        # 创建命令处理线程
        command_thread = threading.Thread(target=self._command_loop)
        command_thread.daemon = True
        command_thread.start()
        
        print("模拟器已启动")
    
    def stop(self):
        """停止模拟器"""
        self.running = False
        print("模拟器已停止")
    
    def _load_map(self, map_file):
        """
        加载地图
        
        参数:
            map_file: 地图文件
            
        返回:
            map: 地图数据，numpy数组
        """
        if map_file and os.path.exists(map_file):
            # 加载指定地图文件
            if map_file.endswith('.npy'):
                return np.load(map_file)
            else:
                # 尝试加载图像文件
                from PIL import Image
                img = Image.open(map_file).convert('L')
                return np.array(img)
        else:
            # 创建默认地图（简单的迷宫）
            map_size = 800
            map_data = np.ones((map_size, map_size), dtype=np.uint8) * 128  # 未知区域
            
            # 创建边界墙
            map_data[0:10, :] = 255  # 上墙
            map_data[-10:, :] = 255  # 下墙
            map_data[:, 0:10] = 255  # 左墙
            map_data[:, -10:] = 255  # 右墙
            
            # 创建一些内部墙
            map_data[200:220, 100:600] = 255
            map_data[400:700, 300:320] = 255
            map_data[600:620, 100:300] = 255
            
            # 创建一些自由空间
            map_data[50:150, 50:150] = 0
            map_data[300:500, 100:200] = 0
            map_data[600:700, 600:700] = 0
            
            return map_data
    
    def _sensor_loop(self):
        """传感器数据生成循环"""
        last_time = time.time()
        
        while self.running:
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            
            # 更新机器人位置
            self._update_position(dt)
            
            # 生成激光雷达数据
            lidar_data = self._generate_lidar_data()
            
            # 生成里程计数据
            odometry_data = self._generate_odometry_data()
            
            # 组合数据
            combined_data = {
                'type': 'combined',
                'timestamp': int(current_time * 1000) % 65536,  # 模拟时间戳
                'position': self.position,
                'lidar': lidar_data,
                'odometry': self.position
            }
            
            # 发送数据
            self.sensor_queue.put(combined_data)
            
            # 控制频率
            time.sleep(0.1)
    
    def _command_loop(self):
        """命令处理循环"""
        while self.running:
            try:
                # 获取命令
                command = self.command_queue.get(timeout=0.1)
                
                # 处理命令
                self._process_command(command)
                
            except Exception:
                pass
            
            # 控制频率
            time.sleep(0.01)
    
    def _update_position(self, dt):
        """
        更新机器人位置
        
        参数:
            dt: 时间间隔，秒
        """
        with self.lock:
            x, y, theta = self.position
            linear_vel, angular_vel = self.velocity
            
            # 更新位置
            theta_new = theta + angular_vel * dt
            theta_new = theta_new % (2 * np.pi)  # 规范化角度
            
            x_new = x + linear_vel * np.cos(theta) * dt
            y_new = y + linear_vel * np.sin(theta) * dt
            
            # 检查碰撞
            if not self._check_collision(x_new, y_new):
                self.position = (x_new, y_new, theta_new)
            else:
                # 碰撞，停止移动
                self.velocity = (0, 0)
            
            # 如果有目标位置，更新速度
            if self.target_position:
                self._update_velocity()
    
    def _check_collision(self, x, y):
        """
        检查位置是否碰撞
        
        参数:
            x: x坐标，米
            y: y坐标，米
            
        返回:
            collision: 是否碰撞
        """
        # 转换为栅格坐标
        cell_x = int(x / self.cell_size)
        cell_y = int(y / self.cell_size)
        
        # 检查是否超出地图范围
        if cell_x < 0 or cell_x >= self.map_size_pixels or cell_y < 0 or cell_y >= self.map_size_pixels:
            return True
        
        # 检查是否碰到障碍物（值大于200的认为是障碍物）
        return self.map[cell_y, cell_x] > 200
    
    def _generate_lidar_data(self):
        """
        生成激光雷达数据
        
        返回:
            lidar_data: 激光雷达数据，距离值列表，单位为毫米
        """
        x, y, theta = self.position
        
        lidar_data = []
        for angle in self.lidar_angles:
            # 计算全局角度
            global_angle = theta + angle
            
            # 光线追踪
            distance = self._ray_cast(x, y, global_angle)
            
            # 转换为毫米
            distance_mm = int(distance * 1000)
            
            # 添加噪声
            noise = random.gauss(0, 10)  # 10毫米标准差
            distance_mm = max(0, int(distance_mm + noise))
            
            lidar_data.append(distance_mm)
        
        return lidar_data
    
    def _ray_cast(self, x, y, angle):
        """
        光线追踪，计算从(x, y)位置沿angle方向的距离
        
        参数:
            x: x坐标，米
            y: y坐标，米
            angle: 角度，弧度
            
        返回:
            distance: 距离，米
        """
        # 光线步长
        step_size = self.cell_size / 2
        
        # 最大距离
        max_distance = self.lidar_range
        
        # 初始位置
        current_x = x
        current_y = y
        
        # 方向向量
        dx = np.cos(angle)
        dy = np.sin(angle)
        
        # 累计距离
        distance = 0
        
        while distance < max_distance:
            # 更新位置
            current_x += dx * step_size
            current_y += dy * step_size
            distance += step_size
            
            # 转换为栅格坐标
            cell_x = int(current_x / self.cell_size)
            cell_y = int(current_y / self.cell_size)
            
            # 检查是否超出地图范围
            if cell_x < 0 or cell_x >= self.map_size_pixels or cell_y < 0 or cell_y >= self.map_size_pixels:
                return distance
            
            # 检查是否碰到障碍物（值大于200的认为是障碍物）
            if self.map[cell_y, cell_x] > 200:
                return distance
        
        return max_distance
    
    def _generate_odometry_data(self):
        """
        生成里程计数据
        
        返回:
            odometry_data: 里程计数据，(x, y, theta)，单位为米和弧度
        """
        x, y, theta = self.position
        
        # 添加噪声
        noise_x = random.gauss(0, 0.01)  # 1厘米标准差
        noise_y = random.gauss(0, 0.01)  # 1厘米标准差
        noise_theta = random.gauss(0, 0.01)  # 0.01弧度标准差
        
        noisy_position = (x + noise_x, y + noise_y, theta + noise_theta)
        
        return noisy_position
    
    def _process_command(self, command):
        """
        处理命令
        
        参数:
            command: 命令字典
        """
        if command['type'] == 'navigate':
            self._process_navigate_command(command)
        elif command['type'] == 'stop':
            self._process_stop_command()
        elif command['type'] == 'reset':
            self._process_reset_command()
    
    def _process_navigate_command(self, command):
        """
        处理导航命令
        
        参数:
            command: 导航命令字典
        """
        position = command['position']
        
        with self.lock:
            # 设置目标位置
            if len(position) >= 3:
                self.target_position = position
            else:
                # 如果只有(x, y)，保持当前朝向
                x, y = position
                _, _, theta = self.position
                self.target_position = (x, y, theta)
            
            # 更新速度
            self._update_velocity()
    
    def _process_stop_command(self):
        """处理停止命令"""
        with self.lock:
            self.velocity = (0, 0)
            self.target_position = None
    
    def _process_reset_command(self):
        """处理重置命令"""
        with self.lock:
            self.position = (self.map_size_meters / 2, self.map_size_meters / 2, 0)
            self.velocity = (0, 0)
            self.target_position = None
    
    def _update_velocity(self):
        """根据目标位置更新速度"""
        if not self.target_position:
            return
        
        # 当前位置
        x, y, theta = self.position
        
        # 目标位置
        target_x, target_y, target_theta = self.target_position
        
        # 计算距离和角度
        dx = target_x - x
        dy = target_y - y
        distance = np.sqrt(dx*dx + dy*dy)
        target_angle = np.arctan2(dy, dx)
        
        # 计算角度差
        angle_diff = target_angle - theta
        # 规范化到[-pi, pi]
        while angle_diff > np.pi:
            angle_diff -= 2 * np.pi
        while angle_diff < -np.pi:
            angle_diff += 2 * np.pi
        
        # 如果距离目标很近，停止
        if distance < 0.1:
            self.velocity = (0, 0)
            self.target_position = None
            return
        
        # 设置角速度（简单的P控制器）
        angular_vel = 0.5 * angle_diff
        
        # 如果角度差较大，先调整方向
        if abs(angle_diff) > 0.1:
            self.velocity = (0, angular_vel)
        else:
            # 角度差较小，同时调整位置
            linear_vel = 0.2 * distance  # 线速度与距离成正比
            linear_vel = min(linear_vel, 0.5)  # 限制最大线速度
            self.velocity = (linear_vel, angular_vel)

def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description='机器人模拟器')
    parser.add_argument('--map', type=str, help='地图文件')
    
    args = parser.parse_args()
    
    # 创建数据队列
    sensor_queue = Queue()
    command_queue = Queue()
    
    # 创建模拟器
    simulator = RobotSimulator(
        map_file=args.map,
        sensor_queue=sensor_queue,
        command_queue=command_queue
    )
    
    # 启动模拟器
    simulator.start()
    
    try:
        # 等待用户输入命令
        print("模拟器已启动，按Ctrl+C退出")
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\n接收到退出信号")
    finally:
        simulator.stop()

if __name__ == "__main__":
    main() 