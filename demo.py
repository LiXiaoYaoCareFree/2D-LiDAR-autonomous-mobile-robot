#!/usr/bin/env python3
'''
demo.py - 演示脚本，用于演示2D激光雷达自主移动机器人系统的功能

该脚本使用模拟器生成模拟的激光雷达和里程计数据，并使用机器人控制系统进行SLAM和导航。
'''

import time
import threading
import argparse
from queue import Queue

from simulator import RobotSimulator
from robot_control_system import RobotControlSystem

def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='2D激光雷达自主移动机器人系统演示')
    parser.add_argument('--map', type=str, help='地图文件')
    parser.add_argument('--log-dir', type=str, default='./logs', help='日志目录')
    parser.add_argument('--map-size', type=int, default=800, help='地图大小(像素)')
    parser.add_argument('--map-scale', type=float, default=32.0, help='地图比例(米)')
    
    args = parser.parse_args()
    
    # 创建数据队列
    sensor_queue = Queue()
    command_queue = Queue()
    
    # 创建模拟器
    print("正在初始化模拟器...")
    simulator = RobotSimulator(
        map_file=args.map,
        sensor_queue=sensor_queue,
        command_queue=command_queue
    )
    
    # 创建配置
    config = {
        'serial_port': None,  # 不使用实际串口
        'baudrate': 115200,
        'map_size_pixels': args.map_size,
        'map_size_meters': args.map_scale,
        'log_directory': args.log_dir
    }
    
    # 创建控制系统
    print("正在初始化控制系统...")
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
        print("\n=== 开始演示 ===\n")
        
        # 演示1：探索环境
        print("演示1：探索环境")
        print("系统将自动探索环境并构建地图...")
        
        # 等待一段时间，让系统探索环境
        exploration_time = 30  # 30秒
        for i in range(exploration_time):
            print(f"探索中... {i+1}/{exploration_time} 秒", end='\r')
            time.sleep(1)
        print("\n探索完成！")
        
        # 演示2：导航到指定位置
        print("\n演示2：导航到指定位置")
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
        print("\n演示3：返回起始位置")
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
        
        print("\n=== 演示结束 ===\n")
        
    except KeyboardInterrupt:
        print("\n接收到退出信号")
    finally:
        # 停止控制系统和模拟器
        print("停止控制系统...")
        control_system.stop()
        
        print("停止模拟器...")
        simulator.stop()
        
        print("演示结束")

if __name__ == "__main__":
    main() 