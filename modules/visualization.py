#!/usr/bin/env python3
'''
visualization.py - 可视化模块，使用PyRoboViz显示地图和机器人位置

该模块负责实时显示SLAM构建的地图、机器人位置、激光雷达点云和路径规划。
'''

import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as colormap
from queue import Empty

# 使用我们修复的PyRoboViz版本
from roboviz_fix import MapVisualizer

class MapVisualization:
    """地图可视化类，使用PyRoboViz显示地图和机器人位置"""
    
    def __init__(self, map_size_pixels=800, map_size_meters=32, update_queue=None, title="SLAM地图"):
        """
        初始化地图可视化
        
        参数:
            map_size_pixels: 地图大小(像素)
            map_size_meters: 地图大小(米)
            update_queue: 更新队列，用于接收地图更新
            title: 窗口标题
        """
        self.map_size_pixels = map_size_pixels
        self.map_size_meters = map_size_meters
        self.update_queue = update_queue
        self.title = title
        
        # 初始化地图可视化器
        self.visualizer = MapVisualizer(map_size_pixels, map_size_meters, title, show_trajectory=True)
        
        # 初始化地图数据
        self.mapbytes = bytearray(map_size_pixels * map_size_pixels)
        
        # 当前位置
        self.current_pose = (0, 0, 0)  # (x, y, theta)
        
        # 运行标志
        self.running = False
        
        # 路径点列表，用于显示规划路径
        self.path_points = []
    
    def run(self):
        """运行可视化循环"""
        self.running = True
        print("启动地图可视化...")
        
        try:
            while self.running:
                # 检查是否有更新
                if self.update_queue:
                    try:
                        # 非阻塞获取更新
                        update = self.update_queue.get_nowait()
                        
                        # 更新地图和位置
                        if 'map' in update:
                            # 将numpy数组转换为字节数组
                            self.mapbytes = bytearray(update['map'].flatten())
                        
                        if 'pose' in update:
                            self.current_pose = update['pose']
                        
                        if 'path' in update:
                            self.path_points = update['path']
                        
                    except Empty:
                        pass
                
                # 显示地图和位置
                x, y, theta = self.current_pose
                if not self.visualizer.display(x, y, theta, self.mapbytes):
                    print("可视化窗口已关闭")
                    self.running = False
                    break
                
                # 如果有路径点，显示路径
                if self.path_points:
                    self.display_path()
                
                # 小延迟以避免CPU占用过高
                time.sleep(0.05)
                
        except Exception as e:
            print(f"可视化错误: {e}")
        finally:
            print("停止地图可视化")
    
    def stop(self):
        """停止可视化循环"""
        self.running = False
    
    def display_path(self):
        """显示规划路径"""
        # 获取当前坐标轴
        ax = plt.gcf().gca()
        
        # 清除之前的路径
        for artist in ax.get_children():
            if hasattr(artist, 'get_label') and artist.get_label() == 'path':
                artist.remove()
        
        # 转换路径点为像素坐标
        path_pixels = []
        for x, y in self.path_points:
            px = x * self.map_size_pixels / self.map_size_meters
            py = y * self.map_size_pixels / self.map_size_meters
            path_pixels.append((px, py))
        
        # 绘制路径
        if len(path_pixels) > 1:
            x_vals = [p[0] for p in path_pixels]
            y_vals = [p[1] for p in path_pixels]
            ax.plot(x_vals, y_vals, 'g-', linewidth=2, label='path')
        
        # 刷新显示
        plt.draw()
    
    def save_map_image(self, filename):
        """
        保存当前地图为图像文件
        
        参数:
            filename: 文件名
        """
        try:
            # 创建图像
            plt.figure(figsize=(10, 10))
            
            # 显示地图
            map_array = np.reshape(np.frombuffer(self.mapbytes, dtype=np.uint8), 
                                  (self.map_size_pixels, self.map_size_pixels))
            plt.imshow(map_array, cmap=colormap.gray)
            
            # 标记当前位置
            x, y, theta = self.current_pose
            px = x * self.map_size_pixels / self.map_size_meters
            py = y * self.map_size_pixels / self.map_size_meters
            plt.plot(px, py, 'ro', markersize=10)
            
            # 绘制方向箭头
            arrow_length = 20
            dx = arrow_length * np.cos(np.radians(theta))
            dy = arrow_length * np.sin(np.radians(theta))
            plt.arrow(px, py, dx, dy, head_width=10, head_length=10, fc='r', ec='r')
            
            # 保存图像
            plt.savefig(filename)
            plt.close()
            
            print(f"地图已保存为 {filename}")
            
        except Exception as e:
            print(f"保存地图图像失败: {e}") 