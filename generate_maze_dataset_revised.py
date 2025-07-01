#!/usr/bin/env python3
'''
generate_maze_dataset_revised.py - 生成迷宫数据集

该脚本生成模拟的激光雷达和里程计数据，用于BreezySLAM构建迷宫地图。
参考了exp1.dat和exp2.dat的格式，生成完全兼容的数据。
'''

import numpy as np
import os
import math
import time
import matplotlib.pyplot as plt
from PIL import Image, ImageDraw

# 迷宫参数
MAP_SIZE_METERS = 32.0  # 迷宫实际大小(米)，与BreezySLAM默认值匹配
WALL_THICKNESS_METERS = 0.5  # 墙壁厚度(米)

# 数据集参数
NUM_SCANS = 800  # 扫描数量
SCAN_SIZE = 682  # 每次扫描的点数 (与BreezySLAM的MinesLaser兼容)
DETECTION_ANGLE_DEGREES = 240  # URG04LX的检测角度
MAX_RANGE_METERS = 4.0  # 最大测量距离(米)，URG04LX为4米

# 模拟参数
ROBOT_SPEED_METERS_PER_SEC = 0.5  # 机器人速度(米/秒)
SCAN_RATE_HZ = 10  # 扫描频率(赫兹)，URG04LX为10Hz
TIMESTEP_SEC = 1.0 / SCAN_RATE_HZ  # 时间步长(秒)

def create_maze_image(maze_size_pixels=800, wall_thickness_pixels=20):
    """创建迷宫图像"""
    # 创建白色背景
    img = Image.new('L', (maze_size_pixels, maze_size_pixels), color=255)
    draw = ImageDraw.Draw(img)
    
    # 定义迷宫墙壁
    walls = []
    
    # 外墙
    walls.append([(0, 0), (maze_size_pixels, 0), 
                 (maze_size_pixels, wall_thickness_pixels), 
                 (0, wall_thickness_pixels)])
    walls.append([(0, 0), (wall_thickness_pixels, 0), 
                 (wall_thickness_pixels, maze_size_pixels), 
                 (0, maze_size_pixels)])
    walls.append([(0, maze_size_pixels-wall_thickness_pixels), 
                 (maze_size_pixels-wall_thickness_pixels, maze_size_pixels-wall_thickness_pixels), 
                 (maze_size_pixels-wall_thickness_pixels, maze_size_pixels), 
                 (0, maze_size_pixels)])
    walls.append([(maze_size_pixels-wall_thickness_pixels, 0), 
                 (maze_size_pixels, 0), 
                 (maze_size_pixels, maze_size_pixels), 
                 (maze_size_pixels-wall_thickness_pixels, maze_size_pixels)])
    
    # 迷宫内部墙壁 - 根据图片创建类似的迷宫结构
    # 水平墙壁
    # 第一行
    h1_start = 2*wall_thickness_pixels
    h1_end = maze_size_pixels - 2*wall_thickness_pixels
    walls.append([(h1_start, 2*wall_thickness_pixels), 
                 (h1_end, 2*wall_thickness_pixels), 
                 (h1_end, 3*wall_thickness_pixels), 
                 (h1_start, 3*wall_thickness_pixels)])
    
    # 第二行
    h2_start = 3*wall_thickness_pixels
    h2_end = maze_size_pixels - 3*wall_thickness_pixels
    walls.append([(h2_start, 4*wall_thickness_pixels), 
                 (h2_end, 4*wall_thickness_pixels), 
                 (h2_end, 5*wall_thickness_pixels), 
                 (h2_start, 5*wall_thickness_pixels)])
    
    # 第三行
    h3_start = 4*wall_thickness_pixels
    h3_end = maze_size_pixels - 4*wall_thickness_pixels
    walls.append([(h3_start, 6*wall_thickness_pixels), 
                 (h3_end, 6*wall_thickness_pixels), 
                 (h3_end, 7*wall_thickness_pixels), 
                 (h3_start, 7*wall_thickness_pixels)])
    
    # 第四行
    h4_start = 5*wall_thickness_pixels
    h4_end = maze_size_pixels - 5*wall_thickness_pixels
    walls.append([(h4_start, 8*wall_thickness_pixels), 
                 (h4_end, 8*wall_thickness_pixels), 
                 (h4_end, 9*wall_thickness_pixels), 
                 (h4_start, 9*wall_thickness_pixels)])
    
    # 第五行
    h5_start = 6*wall_thickness_pixels
    h5_mid = maze_size_pixels // 2
    h5_end = maze_size_pixels - 6*wall_thickness_pixels
    walls.append([(h5_start, 10*wall_thickness_pixels), 
                 (h5_mid-wall_thickness_pixels, 10*wall_thickness_pixels), 
                 (h5_mid-wall_thickness_pixels, 11*wall_thickness_pixels), 
                 (h5_start, 11*wall_thickness_pixels)])
    walls.append([(h5_mid+wall_thickness_pixels, 10*wall_thickness_pixels), 
                 (h5_end, 10*wall_thickness_pixels), 
                 (h5_end, 11*wall_thickness_pixels), 
                 (h5_mid+wall_thickness_pixels, 11*wall_thickness_pixels)])
    
    # 第六行 - 中心方块
    center_size = 4 * wall_thickness_pixels
    center_start = maze_size_pixels//2 - center_size//2
    center_end = center_start + center_size
    walls.append([(center_start, center_start), 
                 (center_end, center_start), 
                 (center_end, center_end), 
                 (center_start, center_end)])
    
    # 对称添加下半部分的水平墙
    for i in range(5):
        y_pos = maze_size_pixels - (2+i)*2*wall_thickness_pixels
        start_x = (i+2)*wall_thickness_pixels
        end_x = maze_size_pixels - (i+2)*wall_thickness_pixels
        
        if i == 4:  # 如果是第五行，添加中间的间隙
            mid_x = maze_size_pixels // 2
            walls.append([(start_x, y_pos), 
                         (mid_x-wall_thickness_pixels, y_pos), 
                         (mid_x-wall_thickness_pixels, y_pos+wall_thickness_pixels), 
                         (start_x, y_pos+wall_thickness_pixels)])
            walls.append([(mid_x+wall_thickness_pixels, y_pos), 
                         (end_x, y_pos), 
                         (end_x, y_pos+wall_thickness_pixels), 
                         (mid_x+wall_thickness_pixels, y_pos+wall_thickness_pixels)])
        else:
            walls.append([(start_x, y_pos), 
                         (end_x, y_pos), 
                         (end_x, y_pos+wall_thickness_pixels), 
                         (start_x, y_pos+wall_thickness_pixels)])
    
    # 添加垂直墙壁以完成迷宫
    for i in range(4):
        for j in range(2):
            x_pos = 2*wall_thickness_pixels + i*4*wall_thickness_pixels
            if j == 1:
                x_pos = maze_size_pixels - x_pos
            
            y_start = 3*wall_thickness_pixels
            y_end = 5*wall_thickness_pixels
            
            walls.append([(x_pos, y_start), 
                         (x_pos+wall_thickness_pixels, y_start), 
                         (x_pos+wall_thickness_pixels, y_end), 
                         (x_pos, y_end)])
            
            y_start = 7*wall_thickness_pixels
            y_end = 9*wall_thickness_pixels
            
            walls.append([(x_pos, y_start), 
                         (x_pos+wall_thickness_pixels, y_start), 
                         (x_pos+wall_thickness_pixels, y_end), 
                         (x_pos, y_end)])
            
            # 对称添加下半部分的垂直墙
            y_start = maze_size_pixels - 9*wall_thickness_pixels
            y_end = maze_size_pixels - 7*wall_thickness_pixels
            
            walls.append([(x_pos, y_start), 
                         (x_pos+wall_thickness_pixels, y_start), 
                         (x_pos+wall_thickness_pixels, y_end), 
                         (x_pos, y_end)])
            
            y_start = maze_size_pixels - 5*wall_thickness_pixels
            y_end = maze_size_pixels - 3*wall_thickness_pixels
            
            walls.append([(x_pos, y_start), 
                         (x_pos+wall_thickness_pixels, y_start), 
                         (x_pos+wall_thickness_pixels, y_end), 
                         (x_pos, y_end)])
    
    # 创建出口
    exit_width = 2*wall_thickness_pixels
    exit_start = maze_size_pixels - (exit_width + wall_thickness_pixels)
    exit_y = maze_size_pixels - wall_thickness_pixels
    
    # 清除出口区域
    draw.rectangle([exit_start, exit_y, exit_start+exit_width, maze_size_pixels], fill=255)
    
    # 绘制所有墙壁
    for wall in walls:
        draw.polygon(wall, fill=0)
    
    # 保存迷宫图像
    img.save("maze_layout.png")
    
    return img, np.array(img)

def ray_cast(maze_array, start_pos, angle, max_range_pixels):
    """
    从指定位置沿指定角度进行光线投射，找到第一个障碍物
    
    参数:
        maze_array: 迷宫数组，0表示障碍物，255表示空白区域
        start_pos: 起始位置，(x, y)像素坐标
        angle: 角度（弧度）
        max_range_pixels: 最大探测距离（像素）
        
    返回:
        distance: 到障碍物的距离（像素）
    """
    dx = math.cos(angle)
    dy = math.sin(angle)
    
    x, y = start_pos
    
    for i in range(max_range_pixels):
        # 更新位置
        x += dx
        y += dy
        
        # 检查边界
        ix, iy = int(x), int(y)
        if ix < 0 or ix >= maze_array.shape[1] or iy < 0 or iy >= maze_array.shape[0]:
            return i
        
        # 检查障碍物
        if maze_array[iy, ix] == 0:  # 黑色为墙壁
            return i
    
    return max_range_pixels

def generate_path(maze_array, start_pos, exit_pos, maze_size_pixels, wall_thickness_pixels):
    """
    生成从起点到终点的路径
    
    参数:
        maze_array: 迷宫数组，0表示障碍物，255表示空白区域
        start_pos: 起始位置，(x, y)像素坐标
        exit_pos: 终点位置，(x, y)像素坐标
        
    返回:
        path: 路径点列表，每个点为(x, y, theta)
    """
    # 简化的路径生成，沿着迷宫的中心线移动
    path = []
    
    # 起点
    current_x, current_y = start_pos
    path.append((current_x, current_y, 0))
    
    # 移动到第一个拐点
    target_y = 3*wall_thickness_pixels + wall_thickness_pixels//2
    while current_y < target_y:
        current_y += 1
        path.append((current_x, current_y, math.pi/2))
    
    # 向右移动到第一个通道
    target_x = maze_size_pixels - 3*wall_thickness_pixels - wall_thickness_pixels//2
    while current_x < target_x:
        current_x += 1
        path.append((current_x, current_y, 0))
    
    # 向下移动
    target_y = 5*wall_thickness_pixels + wall_thickness_pixels//2
    while current_y < target_y:
        current_y += 1
        path.append((current_x, current_y, math.pi/2))
    
    # 向左移动
    target_x = 3*wall_thickness_pixels + wall_thickness_pixels//2
    while current_x > target_x:
        current_x -= 1
        path.append((current_x, current_y, math.pi))
    
    # 向下移动
    target_y = 7*wall_thickness_pixels + wall_thickness_pixels//2
    while current_y < target_y:
        current_y += 1
        path.append((current_x, current_y, math.pi/2))
    
    # 向右移动
    target_x = maze_size_pixels - 3*wall_thickness_pixels - wall_thickness_pixels//2
    while current_x < target_x:
        current_x += 1
        path.append((current_x, current_y, 0))
    
    # 向下移动
    target_y = 9*wall_thickness_pixels + wall_thickness_pixels//2
    while current_y < target_y:
        current_y += 1
        path.append((current_x, current_y, math.pi/2))
    
    # 向左移动
    target_x = 3*wall_thickness_pixels + wall_thickness_pixels//2
    while current_x > target_x:
        current_x -= 1
        path.append((current_x, current_y, math.pi))
    
    # 继续添加路径点直到接近出口
    # 向下移动
    target_y = maze_size_pixels - 3*wall_thickness_pixels - wall_thickness_pixels//2
    while current_y < target_y:
        current_y += 1
        path.append((current_x, current_y, math.pi/2))
    
    # 向右移动到出口
    while current_x < exit_pos[0]:
        current_x += 1
        path.append((current_x, current_y, 0))
    
    return path

def sample_path(path, num_samples):
    """从路径中均匀采样点"""
    if len(path) <= num_samples:
        return path
    
    indices = np.linspace(0, len(path)-1, num_samples, dtype=int)
    return [path[i] for i in indices]

def generate_dataset(output_file, maze_size_pixels=800, wall_thickness_pixels=20):
    """
    生成数据集
    
    参数:
        output_file: 输出文件名
        maze_size_pixels: 迷宫大小(像素)
        wall_thickness_pixels: 墙壁厚度(像素)
    """
    print(f"生成迷宫图像...")
    maze_img, maze_array = create_maze_image(maze_size_pixels, wall_thickness_pixels)
    
    # 定义起点和终点
    start_pos = (wall_thickness_pixels + wall_thickness_pixels//2, 
                wall_thickness_pixels + wall_thickness_pixels//2)
    
    exit_width = 2*wall_thickness_pixels
    exit_start = maze_size_pixels - (exit_width + wall_thickness_pixels)
    exit_pos = (exit_start + exit_width//2, 
               maze_size_pixels - wall_thickness_pixels//2)
    
    print(f"生成路径...")
    path = generate_path(maze_array, start_pos, exit_pos, maze_size_pixels, wall_thickness_pixels)
    sampled_path = sample_path(path, NUM_SCANS)
    
    # 缩放因子：从像素坐标转换为米
    scale_factor = MAP_SIZE_METERS / maze_size_pixels
    
    print(f"生成激光雷达和里程计数据...")
    # 时间戳（微秒）
    timestamps = [int(i * (1000000 / SCAN_RATE_HZ)) for i in range(len(sampled_path))]
    
    # 里程计数据（时间戳，左轮编码器，右轮编码器）
    odometries = []
    
    # 激光雷达数据
    lidars = []
    
    max_range_pixels = int(MAX_RANGE_METERS / scale_factor)
    
    # URG04LX激光雷达的角度范围（240度）
    # 转换为弧度
    detection_angle_rad = math.radians(DETECTION_ANGLE_DEGREES)
    
    # 每次扫描的角度增量
    angle_increment = detection_angle_rad / (SCAN_SIZE - 1)
    
    # 起始角度 (-120度, 转换为弧度)
    start_angle = -detection_angle_rad / 2
    
    for i, (x, y, theta) in enumerate(sampled_path):
        # 生成里程计数据
        if i > 0:
            prev_x, prev_y, prev_theta = sampled_path[i-1]
            dx = x - prev_x
            dy = y - prev_y
            dtheta = theta - prev_theta
            
            # 简化的里程计模型
            distance = math.sqrt(dx*dx + dy*dy)
            left_encoder = int(2000 * (distance - dtheta))
            right_encoder = int(2000 * (distance + dtheta))
            
            odometries.append((timestamps[i], left_encoder, right_encoder))
        else:
            odometries.append((timestamps[i], 0, 0))
        
        # 生成激光雷达数据
        lidar_scan = []
        
        for j in range(SCAN_SIZE):
            # 计算实际扫描角度 (相对于机器人前方)
            scan_angle = start_angle + j * angle_increment
            
            # 转换为全局角度
            global_angle = scan_angle + theta
            
            # 光线投射
            distance_pixels = ray_cast(maze_array, (x, y), global_angle, max_range_pixels)
            
            # 转换为毫米
            distance_mm = int(distance_pixels * scale_factor * 1000)
            
            # 添加噪声
            noise = np.random.normal(0, 10)  # 10mm标准差
            distance_mm = max(0, int(distance_mm + noise))
            
            # 如果超出范围，设置为最大值或使用7表示无效读数（与原始数据集一致）
            if distance_mm >= MAX_RANGE_METERS * 1000:
                # 有10%的概率使用7表示无效读数
                if np.random.random() < 0.1:
                    distance_mm = 7
                else:
                    distance_mm = 4000  # URG04LX的默认最大值
                
            lidar_scan.append(distance_mm)
        
        lidars.append(lidar_scan)
    
    # 写入数据文件
    print(f"写入数据文件: {output_file}")
    with open(output_file, 'w') as f:
        for i in range(len(sampled_path)):
            # 时间戳
            f.write(f"{timestamps[i]} ")
            
            # 位置（固定值）
            f.write("0 ")
            
            # 里程计
            f.write(f"{odometries[i][1]} {odometries[i][2]} ")
            
            # 添加固定的传感器数据（与原始数据集保持一致）
            f.write("0 0 0 170.5 9800 12750 8700 12750 475 255 0 0 255 0 0 178 60 125 255 0 0 ")
            
            # 激光雷达数据
            for j in range(len(lidars[i])):
                f.write(f"{lidars[i][j]} ")
                
                # 每20个数据换行（提高可读性）
                if (j + 1) % 20 == 0:
                    f.write("\n")
            
            f.write("\n")
    
    print(f"数据集生成完成: {output_file}")
    
    # 保存路径可视化
    plt.figure(figsize=(10, 10))
    plt.imshow(maze_array, cmap='gray')
    
    path_x = [p[0] for p in sampled_path]
    path_y = [p[1] for p in sampled_path]
    
    plt.plot(path_x, path_y, 'b-', linewidth=2)
    plt.plot(start_pos[0], start_pos[1], 'ro', markersize=10)
    plt.plot(exit_pos[0], exit_pos[1], 'go', markersize=10)
    
    plt.title('Maze with Path')
    plt.axis('equal')
    plt.savefig('maze_with_path.png')
    
    return maze_img, sampled_path

if __name__ == "__main__":
    # 生成数据集
    output_file = "maze_dataset.dat"
    maze_img, path = generate_dataset(output_file)
    
    print(f"现在您可以使用以下命令生成迷宫地图:")
    print(f"python BreezySLAM/examples/log2pgm.py maze_dataset 1 9999") 