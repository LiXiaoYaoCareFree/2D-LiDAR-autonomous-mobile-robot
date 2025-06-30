#!/usr/bin/env python3
'''
robot_control_system.py - 2D激光雷达自主移动机器人主控系统

该系统作为机器人的高层控制系统，实现智能决策、环境感知、导航规划。
处理来自机器人固件的传入传感器数据，特别是2D激光雷达扫描和里程计数据，
以执行关键任务，如自主导航、SLAM、实时地图和机器人姿态可视化、路径规划，
以及向机器人固件生成导航路点命令（目标位置）。

作者: [您的名字]
日期: [当前日期]
'''

import time
import argparse
import threading
import numpy as np
from queue import Queue

# 导入自定义模块
from modules.slam_module import SLAMSystem
from modules.visualization import MapVisualization
from modules.navigation import FrontierExploration, PathPlanner
from modules.communication import RobotCommunication
from modules.data_logger import DataLogger

class RobotControlSystem:
    """机器人主控系统类，协调各模块工作"""
    
    def __init__(self, config):
        """
        初始化机器人控制系统
        
        参数:
            config: 配置参数字典
        """
        self.config = config
        self.running = False
        self.exploration_complete = False
        self.exit_found = False
        self.exit_position = None
        self.start_position = None
        
        # 创建数据队列用于模块间通信
        self.sensor_data_queue = Queue()
        self.command_queue = Queue()
        self.map_update_queue = Queue()
        
        # 初始化各模块
        print("初始化SLAM系统...")
        self.slam = SLAMSystem(
            map_size_pixels=config['map_size_pixels'],
            map_size_meters=config['map_size_meters']
        )
        
        print("初始化可视化模块...")
        self.visualization = MapVisualization(
            map_size_pixels=config['map_size_pixels'],
            map_size_meters=config['map_size_meters'],
            update_queue=self.map_update_queue
        )
        
        print("初始化导航模块...")
        self.frontier_explorer = FrontierExploration(
            map_size_pixels=config['map_size_pixels'],
            map_size_meters=config['map_size_meters']
        )
        
        self.path_planner = PathPlanner(
            map_size_pixels=config['map_size_pixels'],
            map_size_meters=config['map_size_meters']
        )
        
        print("初始化通信模块...")
        self.communication = RobotCommunication(
            port=config['serial_port'],
            baudrate=config['baudrate'],
            sensor_queue=self.sensor_data_queue,
            command_queue=self.command_queue
        )
        
        print("初始化数据记录器...")
        self.logger = DataLogger(
            log_dir=config['log_directory']
        )
    
    def start(self):
        """启动系统的所有线程和处理"""
        if self.running:
            print("系统已经在运行中")
            return
            
        self.running = True
        print("启动机器人控制系统...")
        
        # 启动通信线程
        self.comm_thread = threading.Thread(target=self.communication.run)
        self.comm_thread.daemon = True
        self.comm_thread.start()
        
        # 启动可视化线程
        self.vis_thread = threading.Thread(target=self.visualization.run)
        self.vis_thread.daemon = True
        self.vis_thread.start()
        
        # 启动主处理线程
        self.process_thread = threading.Thread(target=self.main_process_loop)
        self.process_thread.daemon = True
        self.process_thread.start()
        
        # 记录起始位置
        print("等待初始位置数据...")
        time.sleep(2)  # 等待一些数据到达
        if not self.sensor_data_queue.empty():
            initial_data = self.sensor_data_queue.get()
            if 'position' in initial_data:
                self.start_position = initial_data['position']
                print(f"记录起始位置: {self.start_position}")
    
    def stop(self):
        """停止系统的所有线程和处理"""
        if not self.running:
            return
            
        self.running = False
        print("停止机器人控制系统...")
        
        # 等待线程结束
        if hasattr(self, 'comm_thread') and self.comm_thread.is_alive():
            self.comm_thread.join(timeout=1.0)
        
        if hasattr(self, 'vis_thread') and self.vis_thread.is_alive():
            self.vis_thread.join(timeout=1.0)
            
        if hasattr(self, 'process_thread') and self.process_thread.is_alive():
            self.process_thread.join(timeout=1.0)
            
        # 关闭通信
        self.communication.close()
        
        # 保存最终地图
        self.save_final_map()
        
        print("系统已停止")
    
    def main_process_loop(self):
        """主处理循环，处理传感器数据并执行导航决策"""
        print("开始主处理循环...")
        
        while self.running:
            # 处理传感器数据
            if not self.sensor_data_queue.empty():
                sensor_data = self.sensor_data_queue.get()
                self.process_sensor_data(sensor_data)
            
            # 如果探索完成且找到出口，但还没有返回起点
            if self.exploration_complete and self.exit_found and not self.at_start_position():
                self.navigate_to_start()
            
            # 如果探索未完成，继续探索
            elif not self.exploration_complete:
                self.explore_environment()
            
            time.sleep(0.01)  # 小延迟以避免CPU占用过高
    
    def process_sensor_data(self, sensor_data):
        """
        处理来自机器人的传感器数据
        
        参数:
            sensor_data: 包含激光雷达和里程计数据的字典
        """
        # 记录数据
        self.logger.log_sensor_data(sensor_data)
        
        # 更新SLAM系统
        if 'lidar' in sensor_data and 'odometry' in sensor_data:
            # 更新SLAM
            self.slam.update(sensor_data['lidar'], sensor_data['odometry'])
            
            # 获取当前位置和地图
            current_pose = self.slam.get_position()
            current_map = self.slam.get_map()
            
            # 更新可视化
            map_update = {
                'map': current_map,
                'pose': current_pose,
                'lidar': sensor_data['lidar']
            }
            self.map_update_queue.put(map_update)
            
            # 检查是否找到出口
            if not self.exit_found:
                exit_found, exit_position = self.check_for_exit(current_map, current_pose)
                if exit_found:
                    self.exit_found = True
                    self.exit_position = exit_position
                    print(f"找到出口点: {exit_position}")
    
    def explore_environment(self):
        """实现边界探索算法来探索未知环境"""
        # 获取当前地图和位置
        current_map = self.slam.get_map()
        current_pose = self.slam.get_position()
        
        # 找到边界点
        frontiers = self.frontier_explorer.detect_frontiers(current_map)
        
        if not frontiers:
            print("没有找到更多边界点，探索完成")
            self.exploration_complete = True
            return
        
        # 选择最佳边界点
        best_frontier = self.frontier_explorer.select_best_frontier(frontiers, current_pose)
        
        # 规划到边界点的路径
        path = self.path_planner.plan_path(current_map, current_pose, best_frontier)
        
        if path:
            # 发送导航命令
            self.send_navigation_commands(path)
        else:
            print("无法规划到边界点的路径，尝试其他边界点")
    
    def navigate_to_start(self):
        """规划并导航回起始位置的路径"""
        current_map = self.slam.get_map()
        current_pose = self.slam.get_position()
        
        # 规划回起始位置的路径
        path = self.path_planner.plan_path(current_map, current_pose, self.start_position)
        
        if path:
            print("规划回起始位置的路径成功，开始导航")
            self.send_navigation_commands(path)
        else:
            print("无法规划回起始位置的路径，重试...")
    
    def check_for_exit(self, current_map, current_pose):
        """
        检查地图中是否存在出口
        
        参数:
            current_map: 当前地图
            current_pose: 当前位置
            
        返回:
            (exit_found, exit_position): 是否找到出口及其位置
        """
        # 这里需要实现出口检测算法
        # 简单示例: 检查地图边缘是否有开放区域
        
        # 此处为简化示例，实际应根据迷宫特性开发更复杂的出口检测算法
        map_size = current_map.shape[0]
        border_width = 10  # 检查边缘的宽度
        
        # 检查地图四个边缘
        for i in range(map_size):
            # 检查上边缘
            if i < border_width or i >= map_size - border_width:
                for j in range(map_size):
                    # 如果边缘有开放区域（值小于某个阈值）
                    if current_map[i, j] < 50:  # 假设<50为开放区域
                        # 转换为世界坐标
                        exit_x = j * self.config['map_size_meters'] / self.config['map_size_pixels']
                        exit_y = i * self.config['map_size_meters'] / self.config['map_size_pixels']
                        return True, (exit_x, exit_y)
            
            # 检查左右边缘
            if i >= border_width and i < map_size - border_width:
                # 左边缘
                if current_map[i, 0:border_width].min() < 50:
                    exit_x = 0
                    exit_y = i * self.config['map_size_meters'] / self.config['map_size_pixels']
                    return True, (exit_x, exit_y)
                
                # 右边缘
                if current_map[i, -border_width:].min() < 50:
                    exit_x = self.config['map_size_meters']
                    exit_y = i * self.config['map_size_meters'] / self.config['map_size_pixels']
                    return True, (exit_x, exit_y)
        
        return False, None
    
    def at_start_position(self):
        """检查机器人是否回到了起始位置"""
        if not self.start_position:
            return False
            
        current_pose = self.slam.get_position()
        distance = np.sqrt((current_pose[0] - self.start_position[0])**2 + 
                          (current_pose[1] - self.start_position[1])**2)
        
        # 如果距离小于阈值，认为回到了起始位置
        return distance < 0.5  # 0.5米阈值
    
    def send_navigation_commands(self, path):
        """
        发送导航命令到机器人
        
        参数:
            path: 路径点列表
        """
        if not path:
            return
            
        # 记录导航命令
        self.logger.log_navigation_commands(path)
        
        # 将路径点转换为导航命令并发送
        for waypoint in path:
            command = {
                'type': 'navigate',
                'position': waypoint
            }
            self.command_queue.put(command)
            
            # 等待一段时间，确保命令被处理
            time.sleep(0.1)
    
    def save_final_map(self):
        """保存最终生成的地图"""
        final_map = self.slam.get_map()
        if final_map is not None:
            self.logger.save_map(final_map)
            print("最终地图已保存")

def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='2D激光雷达自主移动机器人控制系统')
    parser.add_argument('--port', type=str, default='/dev/ttyUSB0', help='串口设备')
    parser.add_argument('--baudrate', type=int, default=115200, help='波特率')
    parser.add_argument('--map-size', type=int, default=800, help='地图大小(像素)')
    parser.add_argument('--map-scale', type=float, default=32.0, help='地图比例(米)')
    parser.add_argument('--log-dir', type=str, default='./logs', help='日志目录')
    
    args = parser.parse_args()
    
    # 创建配置
    config = {
        'serial_port': args.port,
        'baudrate': args.baudrate,
        'map_size_pixels': args.map_size,
        'map_size_meters': args.map_scale,
        'log_directory': args.log_dir
    }
    
    # 创建并启动控制系统
    control_system = RobotControlSystem(config)
    
    try:
        control_system.start()
        
        # 主循环，保持程序运行
        print("按Ctrl+C退出...")
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\n接收到退出信号")
    finally:
        control_system.stop()

if __name__ == "__main__":
    main() 