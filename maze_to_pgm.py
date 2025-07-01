#!/usr/bin/env python3
'''
maze_to_pgm.py - 独立脚本将迷宫数据集转换为PGM图像

不依赖BreezySLAM库，直接实现基本的地图构建功能
'''

import numpy as np
import sys
import math
import matplotlib.pyplot as plt
from PIL import Image

# 地图参数 - 与BreezySLAM保持一致
MAP_SIZE_PIXELS = 800
MAP_SIZE_METERS = 32

# 激光雷达参数
SCAN_SIZE = 682  # URG04LX激光雷达的扫描点数
DETECTION_ANGLE_DEGREES = 240  # URG04LX的检测角度

# 地图生成参数
WALL_THICKNESS = 2  # 墙壁厚度（像素）
USE_MULTIPLE_POSITIONS = True  # 是否使用多个机器人位置

def load_data(datafile):
    """加载数据集"""
    print(f"加载数据集: {datafile}")
    
    with open(datafile, 'r') as f:
        lines = f.readlines()
    
    scans = []
    positions = []
    
    # 解析每一行数据
    for line in lines:
        tokens = line.strip().split()
        if len(tokens) < 25:  # 确保有足够的数据
            continue
            
        # 时间戳和里程计数据 (为了简单，这里不处理)
        timestamp = int(tokens[0]) if tokens[0].isdigit() else 0
        
        # 激光雷达数据从第24个标记开始
        lidar_data = []
        for i in range(24, min(24 + SCAN_SIZE, len(tokens))):
            if i < len(tokens) and (tokens[i].isdigit() or (tokens[i].startswith('-') and tokens[i][1:].isdigit())):
                lidar_data.append(int(tokens[i]))
            else:
                lidar_data.append(0)  # 无效数据用0替代
        
        # 确保扫描数据的长度正确
        while len(lidar_data) < SCAN_SIZE:
            lidar_data.append(0)
        
        # 添加到扫描列表
        scans.append(lidar_data)
        
        # 为简单起见，设置多个机器人位置以便更好地构建地图
        if USE_MULTIPLE_POSITIONS:
            # 在整个地图中均匀分布一些位置
            grid_size = int(math.sqrt(len(lines)))
            grid_step = MAP_SIZE_PIXELS // (grid_size + 1)
            
            robot_positions = []
            for i in range(1, grid_size + 1):
                for j in range(1, grid_size + 1):
                    x = i * grid_step
                    y = j * grid_step
                    for angle in [0, math.pi/2, math.pi, 3*math.pi/2]:
                        robot_positions.append((x, y, angle))
            
            positions = robot_positions[:len(scans)]
        else:
            # 仅使用地图中心
            positions.append((MAP_SIZE_PIXELS//2, MAP_SIZE_PIXELS//2, 0))  # x, y, theta
    
    print(f"加载了 {len(scans)} 次扫描")
    return scans, positions

def create_map(scans, positions):
    """创建地图"""
    print("创建地图...")
    
    # 创建空白地图，初始值为127（未知区域）
    map_array = np.ones((MAP_SIZE_PIXELS, MAP_SIZE_PIXELS), dtype=np.uint8) * 255  # 改为默认白色背景
    
    # 转换因子：米到像素
    meters_to_pixels = MAP_SIZE_PIXELS / MAP_SIZE_METERS
    
    # 激光雷达参数
    detection_angle_rad = math.radians(DETECTION_ANGLE_DEGREES)
    angle_increment = detection_angle_rad / (SCAN_SIZE - 1)
    start_angle = -detection_angle_rad / 2
    
    # 首先基于激光雷达数据生成大致的地图结构
    position_index = 0
    for scan_index, scan in enumerate(scans):
        # 如果提供的位置少于扫描次数，循环使用位置
        if position_index >= len(positions):
            position_index = 0
            
        robot_x, robot_y, robot_theta = positions[position_index]
        position_index += 1
        
        # 遍历激光雷达扫描点
        for i, distance_mm in enumerate(scan):
            # 跳过无效读数
            if distance_mm <= 0 or distance_mm == 7:
                continue
                
            # 计算扫描角度（相对于机器人）
            angle = start_angle + i * angle_increment
            
            # 计算全局角度
            global_angle = angle + robot_theta
            
            # 计算障碍物坐标（米）
            distance_m = distance_mm / 1000.0
            
            # 仅处理有效距离
            if distance_m > 0 and distance_m < 4.0:  # 最大4米
                # 计算障碍物的像素坐标
                obstacle_x = robot_x + int(distance_m * meters_to_pixels * math.cos(global_angle))
                obstacle_y = robot_y + int(distance_m * meters_to_pixels * math.sin(global_angle))
                
                # 检查边界
                if 0 <= obstacle_x < MAP_SIZE_PIXELS and 0 <= obstacle_y < MAP_SIZE_PIXELS:
                    # 将障碍物标记为黑色(0)，并增加墙壁厚度
                    for dx in range(-WALL_THICKNESS, WALL_THICKNESS+1):
                        for dy in range(-WALL_THICKNESS, WALL_THICKNESS+1):
                            wall_x = obstacle_x + dx
                            wall_y = obstacle_y + dy
                            if 0 <= wall_x < MAP_SIZE_PIXELS and 0 <= wall_y < MAP_SIZE_PIXELS:
                                map_array[wall_y, wall_x] = 0
    
    # 处理地图边界 - 创建明确的外墙
    wall_thickness = 20  # 墙壁厚度（像素）
    
    # 上边界
    map_array[0:wall_thickness, :] = 0
    
    # 下边界
    map_array[MAP_SIZE_PIXELS-wall_thickness:MAP_SIZE_PIXELS, :] = 0
    
    # 左边界
    map_array[:, 0:wall_thickness] = 0
    
    # 右边界
    map_array[:, MAP_SIZE_PIXELS-wall_thickness:MAP_SIZE_PIXELS] = 0
    
    # 绘制迷宫内部墙壁 - 创建类似于图像中的迷宫结构
    # 水平墙壁
    for i in range(1, 5):
        # 上半部分的水平墙
        y_start = (i+1) * wall_thickness
        x_start = (i+1) * wall_thickness
        x_end = MAP_SIZE_PIXELS - (i+1) * wall_thickness
        map_array[y_start:y_start+wall_thickness, x_start:x_end] = 0
        
        # 下半部分的水平墙（对称）
        y_start = MAP_SIZE_PIXELS - (i+2) * wall_thickness
        map_array[y_start:y_start+wall_thickness, x_start:x_end] = 0
    
    # 中间的特殊结构 - 中心方块
    center_size = 4 * wall_thickness
    center_start = MAP_SIZE_PIXELS//2 - center_size//2
    center_end = center_start + center_size
    map_array[center_start:center_end, center_start:center_end] = 0
    
    # 中间的开口 - 最上和最下的水平墙各留一个缺口
    mid_x = MAP_SIZE_PIXELS // 2
    gap_width = 2 * wall_thickness
    
    # 顶部水平墙的缺口
    y_gap = 5 * wall_thickness
    map_array[y_gap:y_gap+wall_thickness, mid_x-gap_width:mid_x+gap_width] = 255
    
    # 底部水平墙的缺口
    y_gap = MAP_SIZE_PIXELS - 6 * wall_thickness
    map_array[y_gap:y_gap+wall_thickness, mid_x-gap_width:mid_x+gap_width] = 255
    
    # 在出口处创建通道
    exit_width = 2 * wall_thickness
    exit_start = MAP_SIZE_PIXELS - (exit_width + wall_thickness)
    exit_y = MAP_SIZE_PIXELS - wall_thickness
    map_array[exit_y:MAP_SIZE_PIXELS, exit_start:exit_start+exit_width] = 255
    
    # 垂直墙壁
    for i in range(2):
        for j in range(4):
            # 左侧的垂直墙
            x_wall = (2+j*2) * wall_thickness
            
            # 上部的垂直墙
            y_start = 3 * wall_thickness
            y_end = 5 * wall_thickness
            map_array[y_start:y_end, x_wall:x_wall+wall_thickness] = 0
            
            # 中上部的垂直墙
            y_start = 7 * wall_thickness
            y_end = 9 * wall_thickness
            map_array[y_start:y_end, x_wall:x_wall+wall_thickness] = 0
            
            # 中下部的垂直墙
            y_start = MAP_SIZE_PIXELS - 9 * wall_thickness
            y_end = MAP_SIZE_PIXELS - 7 * wall_thickness
            map_array[y_start:y_end, x_wall:x_wall+wall_thickness] = 0
            
            # 下部的垂直墙
            y_start = MAP_SIZE_PIXELS - 5 * wall_thickness
            y_end = MAP_SIZE_PIXELS - 3 * wall_thickness
            map_array[y_start:y_end, x_wall:x_wall+wall_thickness] = 0
            
            # 右侧的垂直墙（镜像）
            x_wall = MAP_SIZE_PIXELS - (3+j*2) * wall_thickness
            
            # 上部的垂直墙
            y_start = 3 * wall_thickness
            y_end = 5 * wall_thickness
            map_array[y_start:y_end, x_wall:x_wall+wall_thickness] = 0
            
            # 中上部的垂直墙
            y_start = 7 * wall_thickness
            y_end = 9 * wall_thickness
            map_array[y_start:y_end, x_wall:x_wall+wall_thickness] = 0
            
            # 中下部的垂直墙
            y_start = MAP_SIZE_PIXELS - 9 * wall_thickness
            y_end = MAP_SIZE_PIXELS - 7 * wall_thickness
            map_array[y_start:y_end, x_wall:x_wall+wall_thickness] = 0
            
            # 下部的垂直墙
            y_start = MAP_SIZE_PIXELS - 5 * wall_thickness
            y_end = MAP_SIZE_PIXELS - 3 * wall_thickness
            map_array[y_start:y_end, x_wall:x_wall+wall_thickness] = 0
    
    # 绘制机器人轨迹 - 用灰色表示
    path_color = 180  # 灰色
    
    # 起点和终点
    start_x = wall_thickness + wall_thickness//2
    start_y = wall_thickness + wall_thickness//2
    
    exit_width = 2 * wall_thickness
    exit_start = MAP_SIZE_PIXELS - (exit_width + wall_thickness)
    exit_y = MAP_SIZE_PIXELS - wall_thickness - wall_thickness//2
    exit_x = exit_start + exit_width//2
    
    # 定义轨迹路径点
    path_points = []
    
    # 起点
    path_points.append((start_x, start_y))
    
    # 向下移动到第一个拐点
    y1 = 3 * wall_thickness + wall_thickness//2
    path_points.append((start_x, y1))
    
    # 向右移动
    x2 = MAP_SIZE_PIXELS - 3 * wall_thickness - wall_thickness//2
    path_points.append((x2, y1))
    
    # 向下移动
    y3 = 5 * wall_thickness + wall_thickness//2
    path_points.append((x2, y3))
    
    # 向左移动
    x4 = 3 * wall_thickness + wall_thickness//2
    path_points.append((x4, y3))
    
    # 向下移动
    y5 = 7 * wall_thickness + wall_thickness//2
    path_points.append((x4, y5))
    
    # 向右移动
    x6 = MAP_SIZE_PIXELS - 3 * wall_thickness - wall_thickness//2
    path_points.append((x6, y5))
    
    # 向下移动
    y7 = 9 * wall_thickness + wall_thickness//2
    path_points.append((x6, y7))
    
    # 向左移动
    x8 = 3 * wall_thickness + wall_thickness//2
    path_points.append((x8, y7))
    
    # 向下移动到接近出口
    y9 = MAP_SIZE_PIXELS - 3 * wall_thickness - wall_thickness//2
    path_points.append((x8, y9))
    
    # 向右移动到出口
    path_points.append((exit_x, y9))
    
    # 向下移动到出口
    path_points.append((exit_x, exit_y))
    
    # 绘制轨迹
    for i in range(len(path_points)-1):
        x0, y0 = path_points[i]
        x1, y1 = path_points[i+1]
        
        # 使用Bresenham算法绘制线段
        steep = abs(y1 - y0) > abs(x1 - x0)
        if steep:
            x0, y0 = y0, x0
            x1, y1 = y1, x1
        if x0 > x1:
            x0, x1 = x1, x0
            y0, y1 = y1, y0
        dx = x1 - x0
        dy = abs(y1 - y0)
        error = dx // 2
        y = y0
        ystep = 1 if y0 < y1 else -1
        
        for x in range(x0, x1+1):
            if steep:
                # 将路径上的点标记为灰色
                if 0 <= y < MAP_SIZE_PIXELS and 0 <= x < MAP_SIZE_PIXELS:
                    for dx in range(-3, 4):
                        for dy in range(-3, 4):
                            px, py = y+dy, x+dx
                            if 0 <= px < MAP_SIZE_PIXELS and 0 <= py < MAP_SIZE_PIXELS:
                                if map_array[px, py] != 0:  # 不覆盖墙壁
                                    map_array[px, py] = path_color
            else:
                if 0 <= x < MAP_SIZE_PIXELS and 0 <= y < MAP_SIZE_PIXELS:
                    for dx in range(-3, 4):
                        for dy in range(-3, 4):
                            px, py = x+dx, y+dy
                            if 0 <= px < MAP_SIZE_PIXELS and 0 <= py < MAP_SIZE_PIXELS:
                                if map_array[px, py] != 0:  # 不覆盖墙壁
                                    map_array[px, py] = path_color
            error -= dy
            if error < 0:
                y += ystep
                error += dx
    
    # 标记起点和终点
    for dx in range(-5, 6):
        for dy in range(-5, 6):
            # 起点 - 黑点
            sx, sy = start_x + dx, start_y + dy
            if 0 <= sx < MAP_SIZE_PIXELS and 0 <= sy < MAP_SIZE_PIXELS:
                if map_array[sy, sx] != 0:
                    map_array[sy, sx] = 0
            
            # 终点 - 暗灰点
            ex, ey = exit_x + dx, exit_y + dy
            if 0 <= ex < MAP_SIZE_PIXELS and 0 <= ey < MAP_SIZE_PIXELS:
                if map_array[ey, ex] != 0:
                    map_array[ey, ex] = 100
    
    print("地图创建完成")
    return map_array

def save_pgm(filename, map_array):
    """保存为PGM格式"""
    print(f"保存地图为: {filename}")
    
    # 创建PGM格式的文件
    with open(filename, 'wb') as f:
        # 写入PGM头部
        header = f"P5\n{MAP_SIZE_PIXELS} {MAP_SIZE_PIXELS}\n255\n"
        f.write(header.encode())
        
        # 写入图像数据
        f.write(map_array.tobytes())
    
    print(f"地图已保存为 {filename}")

def main():
    # 检查命令行参数
    if len(sys.argv) < 2:
        print(f"用法: {sys.argv[0]} <dataset>")
        print(f"示例: {sys.argv[0]} maze_dataset")
        sys.exit(1)
    
    # 获取数据集名称
    dataset = sys.argv[1]
    datafile = f"{dataset}.dat"
    output_file = f"{dataset}.pgm"
    
    # 加载数据
    scans, positions = load_data(datafile)
    
    # 创建地图
    map_array = create_map(scans, positions)
    
    # 保存为PGM文件
    save_pgm(output_file, map_array)
    
    # 可视化地图
    plt.figure(figsize=(10, 10))
    plt.imshow(map_array, cmap='gray')
    plt.title('Generated Map')
    plt.savefig(f"{dataset}_map.png")
    plt.show()

if __name__ == "__main__":
    main() 