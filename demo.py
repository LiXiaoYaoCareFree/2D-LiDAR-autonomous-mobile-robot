#!/usr/bin/env python3
'''
demo.py - 演示脚本，用于演示2D激光雷达自主移动机器人系统的功能

该脚本可以使用模拟器生成模拟的激光雷达和里程计数据，或者加载BreezySLAM项目中的示例数据。
'''

import time
import threading
import argparse
import numpy as np
import os
import sys
from queue import Queue

# 添加BreezySLAM示例目录到Python路径
current_dir = os.path.dirname(os.path.abspath(__file__))
examples_dir = os.path.join(current_dir, 'BreezySLAM', 'examples')
sys.path.append(examples_dir)

# 导入BreezySLAM示例模块
from breezyslam.algorithms import Deterministic_SLAM, RMHC_SLAM
from mines import MinesLaser, Rover, load_data
from pgm_utils import pgm_save
from roboviz_fix import MapVisualizer

# 导入自定义模块
from simulator import RobotSimulator

# 地图大小和比例
MAP_SIZE_PIXELS = 800
MAP_SIZE_METERS = 32

def demo_breezyslam_dataset(dataset_name, use_odometry=True, random_seed=0, 
                            map_size_pixels=MAP_SIZE_PIXELS, 
                            map_size_meters=MAP_SIZE_METERS):
    """运行BreezySLAM示例数据集演示"""
    
    print(f"\n=== 开始BreezySLAM示例演示：{dataset_name} ===\n")
    
    # 加载数据
    examples_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'BreezySLAM', 'examples')
    dataset_path = examples_dir
    
    print(f"从{dataset_path}加载数据集{dataset_name}...")
    timestamps, lidars, odometries = load_data(dataset_path, dataset_name)
    
    # 建立机器人模型（如果使用里程计）
    robot = Rover() if use_odometry else None
    
    # 创建SLAM对象
    slam = RMHC_SLAM(MinesLaser(), map_size_pixels, map_size_meters, random_seed=random_seed) \
           if random_seed \
           else Deterministic_SLAM(MinesLaser(), map_size_pixels, map_size_meters)
    
    # 创建可视化对象
    viz = MapVisualizer(map_size_pixels, map_size_meters, f'BreezySLAM Demo: {dataset_name}', 
                        show_trajectory=True)
    
    # 初始化地图数据
    mapbytes = bytearray(map_size_pixels * map_size_pixels)
    
    # 开始计时
    start_time = time.time()
    
    # 初始化轨迹
    trajectory = []
    
    # 处理数据
    nscans = len(lidars)
    print(f"处理{nscans}次扫描数据 {'有' if use_odometry else '无'}里程计 / {'有' if random_seed else '无'}粒子滤波...")
    
    try:
        for scanno in range(nscans):
            if use_odometry:
                # 转换里程计为位姿变化
                velocities = robot.computePoseChange(odometries[scanno])
                
                # 使用激光雷达和速度更新SLAM
                slam.update(lidars[scanno], velocities)
            else:
                # 仅使用激光雷达更新SLAM
                slam.update(lidars[scanno])
            
            # 获取当前位置
            x_mm, y_mm, theta_degrees = slam.getpos()
            
            # 添加到轨迹
            trajectory.append((x_mm, y_mm))
            
            # 获取地图
            slam.getmap(mapbytes)
            
            # 更新可视化
            if not viz.display(x_mm/1000., y_mm/1000., theta_degrees, mapbytes):
                break
            
            # 延迟以便观察
            time.sleep(0.01)
            
            # 显示进度
            if scanno % 20 == 0:
                progress = (scanno + 1) / nscans * 100
                print(f"进度: {progress:.1f}% ({scanno+1}/{nscans})", end='\r')
    
    except KeyboardInterrupt:
        print("\n用户中断演示")
    
    # 计算耗时
    elapsed_sec = time.time() - start_time
    print(f"\n处理{nscans}次扫描完成，耗时{elapsed_sec:.2f}秒，速度{nscans/elapsed_sec:.2f}次/秒")
    
    # 获取最终地图
    slam.getmap(mapbytes)
    
    # 添加轨迹到地图
    for coords in trajectory:
        x_mm, y_mm = coords
        x_pix = int(x_mm / (map_size_meters * 1000. / map_size_pixels))
        y_pix = int(y_mm / (map_size_meters * 1000. / map_size_pixels))
        
        # 确保坐标在地图范围内
        if 0 <= x_pix < map_size_pixels and 0 <= y_pix < map_size_pixels:
            mapbytes[y_pix * map_size_pixels + x_pix] = 0
    
    # 保存地图为PGM文件
    output_filename = f"{dataset_name}_output.pgm"
    pgm_save(output_filename, mapbytes, (map_size_pixels, map_size_pixels))
    print(f"地图已保存为 {output_filename}")
    
    # 保持显示窗口直到用户关闭
    print("\n地图生成完成，请关闭窗口继续...")
    while True:
        try:
            # 最后一次显示
            if not viz.display(x_mm/1000., y_mm/1000., theta_degrees, mapbytes):
                break
            time.sleep(0.1)
        except KeyboardInterrupt:
            break
    
    print("\n=== BreezySLAM示例演示结束 ===\n")

def demo_simulator(map_file=None, log_dir='./logs', map_size=MAP_SIZE_PIXELS, map_scale=MAP_SIZE_METERS):
    """运行模拟器演示"""
    
    print("\n=== 开始模拟器演示 ===\n")
    
    # 创建数据队列
    sensor_queue = Queue()
    command_queue = Queue()
    
    # 创建模拟器
    print("正在初始化模拟器...")
    simulator = RobotSimulator(
        map_file=map_file,
        sensor_queue=sensor_queue,
        command_queue=command_queue
    )
    
    # 创建配置
    config = {
        'serial_port': None,  # 不使用实际串口
        'baudrate': 115200,
        'map_size_pixels': map_size,
        'map_size_meters': map_scale,
        'log_directory': log_dir
    }
    
    # 创建控制系统
    print("正在初始化控制系统...")
    from robot_control_system import RobotControlSystem
    control_system = RobotControlSystem(config)
    
    # 替换控制系统的通信队列
    control_system.sensor_data_queue = sensor_queue
    control_system.command_queue = command_queue
    
    # 启动模拟器和控制系统
    print("启动模拟器...")
    simulator.start()
    
    print("启动控制系统...")
    control_system.start()
    
    try:
        # 等待系统初始化
        print("等待系统初始化...")
        time.sleep(3)
        
        # 开始演示
        print("\n=== 开始探索环境 ===\n")
        
        # 演示1：探索环境
        print("系统将自动探索环境并构建地图...")
        
        # 等待一段时间，让系统探索环境
        exploration_time = 30  # 30秒
        for i in range(exploration_time):
            print(f"探索中... {i+1}/{exploration_time} 秒", end='\r')
            time.sleep(1)
        print("\n探索完成！")
        
        # 演示2：导航到指定位置
        print("\n=== 导航到指定位置 ===\n")
        target_x = 25.0
        target_y = 25.0
        print(f"导航到位置 ({target_x}, {target_y})...")
        
        # 发送导航命令
        navigation_command = {
            'type': 'navigate',
            'position': (target_x, target_y)
        }
        command_queue.put(navigation_command)
        
        # 等待导航完成
        navigation_time = 20  # 20秒
        for i in range(navigation_time):
            print(f"导航中... {i+1}/{navigation_time} 秒", end='\r')
            time.sleep(1)
        print("\n导航完成！")
        
        # 演示3：返回起始位置
        print("\n=== 返回起始位置 ===\n")
        print("系统将导航回起始位置...")
        
        # 发送导航命令
        navigation_command = {
            'type': 'navigate',
            'position': (16.0, 16.0)  # 假设起始位置在地图中心
        }
        command_queue.put(navigation_command)
        
        # 等待导航完成
        navigation_time = 20  # 20秒
        for i in range(navigation_time):
            print(f"返回中... {i+1}/{navigation_time} 秒", end='\r')
            time.sleep(1)
        print("\n返回完成！")
        
        print("\n=== 模拟器演示结束 ===\n")
        
    except KeyboardInterrupt:
        print("\n接收到退出信号")
    finally:
        # 停止控制系统和模拟器
        print("停止控制系统...")
        control_system.stop()
        
        print("停止模拟器...")
        simulator.stop()
        
        print("演示结束")

def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='2D激光雷达自主移动机器人系统演示')
    parser.add_argument('--breezyslam', action='store_true', help='运行BreezySLAM示例')
    parser.add_argument('--dataset', type=str, default='exp1', help='BreezySLAM数据集名称 (exp1或exp2)')
    parser.add_argument('--odometry', type=int, default=1, help='是否使用里程计 (1=是, 0=否)')
    parser.add_argument('--seed', type=int, default=0, help='随机种子 (0=禁用粒子滤波)')
    parser.add_argument('--map', type=str, help='地图文件 (用于模拟器)')
    parser.add_argument('--log-dir', type=str, default='./logs', help='日志目录')
    parser.add_argument('--map-size', type=int, default=MAP_SIZE_PIXELS, help='地图大小(像素)')
    parser.add_argument('--map-scale', type=float, default=MAP_SIZE_METERS, help='地图比例(米)')
    
    args = parser.parse_args()
    
    if args.breezyslam:
        # 运行BreezySLAM示例
        demo_breezyslam_dataset(args.dataset, args.odometry == 1, args.seed, args.map_size, args.map_scale)
    else:
        # 运行模拟器演示
        demo_simulator(args.map, args.log_dir, args.map_size, args.map_scale)

if __name__ == "__main__":
    main() 