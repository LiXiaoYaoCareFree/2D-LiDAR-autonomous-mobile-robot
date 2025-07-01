#!/usr/bin/env python3
'''
demo_maze_solver.py - 迷宫求解器演示程序

该程序演示了2D激光雷达自主移动机器人系统的高级功能，包括：
1. 随机迷宫解决方案：机器人能够主动探索未知区域，实时构建地图，找到出口并返回起点
2. 交互界面：美观的GUI界面，实时展示地图构建、路径规划和导航过程
3. SLAM技术：使用BreezySLAM实现同步定位与地图构建
4. 路径规划：集成A*算法和动态窗口法(DWA)进行全局路径规划和局部避障
5. 数据记录与分析：记录传感器数据、地图和导航命令，支持离线分析
6. BreezySLAM示例：支持加载和可视化BreezySLAM项目中的示例数据

作者: [您的名字]
日期: [当前日期]
'''

import numpy as np
import time
import threading
import argparse
import os
import sys
from queue import Queue, Empty
from datetime import datetime

# 添加BreezySLAM示例目录到Python路径
current_dir = os.path.dirname(os.path.abspath(__file__))
examples_dir = os.path.join(current_dir, 'BreezySLAM', 'examples')
sys.path.append(examples_dir)

# 导入BreezySLAM相关模块
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import Laser
from mines import MinesLaser, Rover, load_data
from pgm_utils import pgm_save
from roboviz_fix import MapVisualizer

# 导入自定义模块
from modules.maze_solver import MazeSolver
from modules.enhanced_visualization import RobotGUI
from modules.communication import BluetoothCommunication
from modules.data_logger import DataLogger
from modules.advanced_navigation import InfoGainFrontierExploration, OptimalPathPlanner
from simulator import LidarSimulator, RobotSimulator

class BreezySLAMDemo:
    """BreezySLAM示例演示类，用于展示BreezySLAM示例数据"""
    
    def __init__(self, dataset_name, use_odometry=True, random_seed=0, 
                 map_size_pixels=800, map_size_meters=32, 
                 visualization_mode='standard'):
        """
        初始化BreezySLAM示例演示
        
        参数:
            dataset_name: 数据集名称，如'exp1'或'exp2'
            use_odometry: 是否使用里程计数据
            random_seed: 随机种子，为0时不使用粒子滤波
            map_size_pixels: 地图大小(像素)
            map_size_meters: 地图大小(米)
            visualization_mode: 可视化模式，'standard'使用原始可视化，'enhanced'使用增强可视化
        """
        self.dataset_name = dataset_name
        self.use_odometry = use_odometry
        self.random_seed = random_seed
        self.map_size_pixels = map_size_pixels
        self.map_size_meters = map_size_meters
        self.visualization_mode = visualization_mode
        
        # 加载数据
        examples_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'BreezySLAM', 'examples')
        self.dataset_path = examples_dir
        
        print(f"从{self.dataset_path}加载数据集{dataset_name}...")
        self.timestamps, self.lidars, self.odometries = load_data(self.dataset_path, dataset_name)
        
        # 建立机器人模型（如果使用里程计）
        self.robot = Rover() if use_odometry else None
        
        # 创建SLAM对象
        self.slam = RMHC_SLAM(MinesLaser(), map_size_pixels, map_size_meters, random_seed=random_seed) \
                   if random_seed \
                   else None  # 将在run()中创建
        
        # 初始化地图数据
        self.mapbytes = bytearray(map_size_pixels * map_size_pixels)
        
        # 初始化轨迹
        self.trajectory = []
        
        # 初始化可视化
        if visualization_mode == 'standard':
            self.viz = MapVisualizer(map_size_pixels, map_size_meters, 
                                    f'BreezySLAM Demo: {dataset_name}', 
                                    show_trajectory=True)
        else:
            # 增强可视化将在run()中创建
            self.viz = None
            self.gui = RobotGUI()
    
    def run(self):
        """运行BreezySLAM示例演示"""
        print(f"\n=== 开始BreezySLAM示例演示：{self.dataset_name} ===\n")
        
        # 如果没有初始化SLAM，现在初始化
        if self.slam is None:
            from breezyslam.algorithms import Deterministic_SLAM
            self.slam = Deterministic_SLAM(MinesLaser(), self.map_size_pixels, self.map_size_meters)
        
        # 如果使用增强可视化但未初始化，现在初始化
        if self.visualization_mode == 'enhanced' and self.viz is None:
            self.gui.log(f"加载数据集：{self.dataset_name}")
            self.gui.log(f"{'使用' if self.use_odometry else '不使用'}里程计")
            self.gui.log(f"{'使用粒子滤波' if self.random_seed else '使用确定性SLAM'}")
        
        # 开始计时
        start_time = time.time()
        
        # 处理数据
        nscans = len(self.lidars)
        print(f"处理{nscans}次扫描数据 {'有' if self.use_odometry else '无'}里程计 / " + 
              f"{'有' if self.random_seed else '无'}粒子滤波...")
        
        try:
            for scanno in range(nscans):
                if self.use_odometry:
                    # 转换里程计为位姿变化
                    velocities = self.robot.computePoseChange(self.odometries[scanno])
                    
                    # 使用激光雷达和速度更新SLAM
                    self.slam.update(self.lidars[scanno], velocities)
                else:
                    # 仅使用激光雷达更新SLAM
                    self.slam.update(self.lidars[scanno])
                
                # 获取当前位置
                x_mm, y_mm, theta_degrees = self.slam.getpos()
                
                # 添加到轨迹
                self.trajectory.append((x_mm, y_mm))
                
                # 获取地图
                self.slam.getmap(self.mapbytes)
                
                # 更新可视化
                if self.visualization_mode == 'standard':
                    if not self.viz.display(x_mm/1000., y_mm/1000., theta_degrees, self.mapbytes):
                        break
                else:
                    # 使用增强可视化
                    occupancy_map = np.frombuffer(self.mapbytes, dtype=np.uint8).reshape(
                        (self.map_size_pixels, self.map_size_pixels))
                    
                    # 更新GUI
                    self.gui.update(
                        occupancy_grid=occupancy_map,
                        robot_pose=(x_mm/1000., y_mm/1000., theta_degrees),
                        start_position=(self.map_size_meters/2, self.map_size_meters/2),
                        current_state=f"处理扫描 {scanno+1}/{nscans}",
                        performance_stats={
                            'progress': (scanno + 1) / nscans * 100,
                            'elapsed_time': time.time() - start_time
                        }
                    )
                
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
        self.slam.getmap(self.mapbytes)
        
        # 添加轨迹到地图
        for coords in self.trajectory:
            x_mm, y_mm = coords
            x_pix = int(x_mm / (self.map_size_meters * 1000. / self.map_size_pixels))
            y_pix = int(y_mm / (self.map_size_meters * 1000. / self.map_size_pixels))
            
            # 确保坐标在地图范围内
            if 0 <= x_pix < self.map_size_pixels and 0 <= y_pix < self.map_size_pixels:
                self.mapbytes[y_pix * self.map_size_pixels + x_pix] = 0
        
        # 保存地图为PGM文件
        output_filename = f"{self.dataset_name}_output.pgm"
        pgm_save(output_filename, self.mapbytes, (self.map_size_pixels, self.map_size_pixels))
        print(f"地图已保存为 {output_filename}")
        
        # 保持显示窗口直到用户关闭
        print("\n地图生成完成，请关闭窗口继续...")
        
        if self.visualization_mode == 'standard':
            while True:
                try:
                    # 最后一次显示
                    if not self.viz.display(x_mm/1000., y_mm/1000., theta_degrees, self.mapbytes):
                        break
                    time.sleep(0.1)
                except KeyboardInterrupt:
                    break
        else:
            self.gui.log("演示完成，请手动关闭窗口")
        
        print("\n=== BreezySLAM示例演示结束 ===\n")

class MazeSolverDemo:
    """迷宫求解器演示类，展示随机迷宫解决方案和交互界面"""
    
    def __init__(self, use_simulator=True, map_size_pixels=800, map_size_meters=32, 
                 log_dir='logs', bluetooth_port=None):
        """
        初始化迷宫求解器演示
        
        参数:
            use_simulator: 是否使用模拟器
            map_size_pixels: 地图大小(像素)
            map_size_meters: 地图大小(米)
            log_dir: 日志目录
            bluetooth_port: 蓝牙串口
        """
        self.map_size_pixels = map_size_pixels
        self.map_size_meters = map_size_meters
        self.use_simulator = use_simulator
        self.bluetooth_port = bluetooth_port
        
        # 创建数据队列
        self.sensor_queue = Queue()
        self.command_queue = Queue()
        self.visualization_queue = Queue()
        
        # 初始化SLAM
        print("初始化SLAM系统...")
        laser_params = {'scan_size': 360, 'scan_rate_hz': 10,
                       'detection_angle_degrees': 360, 'distance_no_detection_mm': 5000,
                       'detection_margin': 70, 'offset_mm': 0}
        self.laser = Laser(**laser_params)
        
        # 初始化SLAM算法
        self.slam = RMHC_SLAM(self.laser, map_size_pixels, map_size_meters)
        
        # 初始化地图
        self.map = np.zeros((map_size_pixels, map_size_pixels), dtype=np.uint8)
        self.map.fill(128)  # 初始值为未知区域
        
        # 初始化迷宫求解器
        print("初始化迷宫求解器...")
        self.maze_solver = MazeSolver(map_size_pixels, map_size_meters)
        
        # 初始化GUI
        print("初始化GUI界面...")
        self.gui = RobotGUI()
        
        # 初始化数据记录器
        print("初始化数据记录器...")
        self.logger = DataLogger(log_dir=log_dir)
        
        # 如果使用模拟器，初始化模拟器
        if use_simulator:
            print("初始化模拟器...")
            self.simulator = RobotSimulator(
                sensor_queue=self.sensor_queue,
                command_queue=self.command_queue
            )
        else:
            self.simulator = None
            # 初始化通信模块
            print("初始化蓝牙通信...")
            self.comm = BluetoothCommunication(
                port=bluetooth_port,
                sensor_queue=self.sensor_queue,
                command_queue=self.command_queue
            )
        
        # 初始化位姿
        self.pose = (map_size_meters/2, map_size_meters/2, 0)  # (x, y, theta) 单位为米和度
        
        # 运行标志
        self.running = False
        self.paused = False
        
        # 初始化线程
        self.slam_thread = None
        self.visualization_thread = None
        
        # 性能统计
        self.stats = {
            'start_time': None,
            'exploration_time': None,
            'exit_found_time': None,
            'return_time': None,
            'completion_time': None,
            'frontiers_detected': 0,
            'path_length': 0
        }
    
    def initialize(self):
        """初始化演示"""
        # 记录日志
        self.gui.log("初始化迷宫求解器演示...")
        
        # 记录会话信息
        session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.gui.log(f"会话ID: {session_id}")
        
        # 初始化迷宫求解器
        self.maze_solver.initialize(self.pose)
        self.gui.log(f"迷宫求解器初始化，起始位置: ({self.pose[0]:.2f}, {self.pose[1]:.2f})")
        
        # 更新GUI
        self.gui.update(
            occupancy_grid=self.map,
            robot_pose=self.pose,
            start_position=(self.pose[0], self.pose[1]),
            current_state="初始化",
            performance_stats=None
        )
        
        # 如果使用模拟器，生成随机迷宫
        if self.use_simulator:
            self.gui.log("生成随机迷宫...")
            self.simulator.start()
        else:
            # 启动蓝牙通信
            self.gui.log("启动蓝牙通信...")
            self.comm.start()
        
        # 记录日志
        self.gui.log("初始化完成")
    
    def start(self):
        """启动演示"""
        if self.running:
            return
        
        self.running = True
        self.paused = False
        
        # 记录日志
        self.gui.log("启动迷宫求解器...")
        
        # 记录开始时间
        self.stats['start_time'] = time.time()
        
        # 创建并启动SLAM线程
        self.slam_thread = threading.Thread(target=self._slam_loop)
        self.slam_thread.daemon = True
        self.slam_thread.start()
        
        # 创建并启动可视化线程
        self.visualization_thread = threading.Thread(target=self._visualization_loop)
        self.visualization_thread.daemon = True
        self.visualization_thread.start()
        
        # 记录日志
        self.gui.log("迷宫求解器已启动")
    
    def stop(self):
        """停止演示"""
        if not self.running:
            return
            
        self.running = False
        
        # 等待线程结束
        if self.slam_thread and self.slam_thread.is_alive():
            self.slam_thread.join(timeout=1.0)
        
        if self.visualization_thread and self.visualization_thread.is_alive():
            self.visualization_thread.join(timeout=1.0)
        
        # 停止模拟器或通信
        if self.use_simulator and self.simulator:
            self.simulator.stop()
        elif hasattr(self, 'comm'):
            self.comm.stop()
        
        # 保存最终地图
        self._save_final_results()
        
        # 记录日志
        self.gui.log("迷宫求解器已停止")
    
    def pause(self):
        """暂停演示"""
        self.paused = True
        self.gui.log("迷宫求解器已暂停")
    
    def resume(self):
        """恢复演示"""
        self.paused = False
        self.gui.log("迷宫求解器已恢复")
    
    def reset(self):
        """重置演示"""
        # 停止当前运行
        self.stop()
        
        # 重置地图
        self.map = np.zeros((self.map_size_pixels, self.map_size_pixels), dtype=np.uint8)
        self.map.fill(128)  # 初始值为未知区域
        
        # 重置SLAM
        self.slam = RMHC_SLAM(self.laser, self.map_size_pixels, self.map_size_meters)
        
        # 重置位姿
        self.pose = (self.map_size_meters/2, self.map_size_meters/2, 0)
        
        # 重置迷宫求解器
        self.maze_solver = MazeSolver(self.map_size_pixels, self.map_size_meters)
        
        # 重置统计信息
        self.stats = {
            'start_time': None,
            'exploration_time': None,
            'exit_found_time': None,
            'return_time': None,
            'completion_time': None,
            'frontiers_detected': 0,
            'path_length': 0
        }
        
        # 如果使用模拟器，重置模拟器
        if self.use_simulator and self.simulator:
            # 发送重置命令
            reset_cmd = {'type': 'reset'}
            self.command_queue.put(reset_cmd)
        
        # 重新初始化
        self.initialize()
        
        # 记录日志
        self.gui.log("迷宫求解器已重置")
    
    def _slam_loop(self):
        """SLAM主循环，在单独的线程中运行"""
        self.gui.log("开始SLAM处理循环...")
        
        # 记录上次处理时间
        last_process_time = time.time()
        
        # 主循环
        while self.running:
            if self.paused:
                time.sleep(0.1)
                continue
            
            try:
                # 获取传感器数据
                sensor_data = self.sensor_queue.get(timeout=0.1)
                
                # 处理传感器数据
                self._process_sensor_data(sensor_data)
                
                # 更新迷宫求解器
                current_time = time.time()
                if current_time - last_process_time > 0.2:  # 每200ms更新一次
                    self._update_maze_solver()
                    last_process_time = current_time
                
            except Empty:
                # 队列为空，继续等待
                pass
            except Exception as e:
                self.gui.log(f"SLAM处理错误: {str(e)}")
            
            # 控制循环速率
            time.sleep(0.01)
    
    def _visualization_loop(self):
        """可视化更新循环，在单独的线程中运行"""
        last_update_time = time.time()
        
        while self.running:
            current_time = time.time()
            
            # 每100ms更新一次GUI
            if current_time - last_update_time > 0.1:
                self._update_gui()
                last_update_time = current_time
            
            # 控制循环速率
            time.sleep(0.01)
    
    def _process_sensor_data(self, sensor_data):
        """
        处理来自模拟器或实际设备的传感器数据
        
        参数:
            sensor_data: 传感器数据字典
        """
        # 记录数据
        self.logger.log_sensor_data(sensor_data)
        
        if 'lidar' in sensor_data and 'position' in sensor_data:
            # 提取激光雷达数据
            lidar_data = sensor_data['lidar']
            
            # 提取位置数据
            position = sensor_data['position']
            
            # 更新SLAM
            self._update_slam(lidar_data, position)
    
    def _update_slam(self, lidar_data, position):
        """
        更新SLAM
        
        参数:
            lidar_data: 激光雷达数据
            position: 位置数据
        """
        # 提取距离数据（单位：毫米）
        distances_mm = np.array(lidar_data, dtype=np.int32)
        
        # 提取位置（单位：米和度）
        x_m, y_m, theta_deg = position
        
        # 转换为SLAM所需格式（单位：毫米和度）
        x_mm = int(x_m * 1000)
        y_mm = int(y_m * 1000)
        
        # 更新SLAM
        self.slam.update(distances_mm)
        
        # 获取更新后的位置
        x_mm, y_mm, theta_deg = self.slam.getpos()
        
        # 更新位置（转换回米）
        self.pose = (x_mm / 1000, y_mm / 1000, theta_deg)
        
        # 获取地图
        self.slam.getmap(self.map)
    
    def _update_maze_solver(self):
        """更新迷宫求解器"""
        # 更新迷宫求解器
        target_pose, is_complete = self.maze_solver.update(self.map, self.pose)
        
        # 记录探索统计信息
        self.stats['frontiers_detected'] = len(self.maze_solver.debug_info.get('frontiers_detected', []))
        
        # 如果有路径，更新路径长度
        if self.maze_solver.current_path:
            path_length = 0
            for i in range(1, len(self.maze_solver.current_path)):
                x1, y1 = self.maze_solver.current_path[i-1]
                x2, y2 = self.maze_solver.current_path[i]
                path_length += np.sqrt((x2-x1)**2 + (y2-y1)**2)
            self.stats['path_length'] = path_length
        
        # 更新状态时间
        current_state = self.maze_solver.state.name
        current_time = time.time()
        
        if current_state == 'EXIT_FOUND' and not self.stats['exit_found_time']:
            self.stats['exit_found_time'] = current_time
            self.stats['exploration_time'] = current_time - self.stats['start_time']
            self.gui.log(f"出口已找到! 探索耗时: {self.stats['exploration_time']:.2f}秒")
        
        elif current_state == 'RETURNING' and not self.stats['return_time']:
            self.stats['return_time'] = current_time
            self.gui.log("开始返回起点...")
        
        elif current_state == 'COMPLETED' and not self.stats['completion_time']:
            self.stats['completion_time'] = current_time
            total_time = current_time - self.stats['start_time']
            self.gui.log(f"迷宫任务完成! 总耗时: {total_time:.2f}秒")
        
        # 如果任务完成
        if is_complete:
            self.gui.log("迷宫任务完成!")
            self.pause()
            return
        
        # 如果有目标位姿，发送导航命令
        if target_pose:
            # 记录导航命令
            self.logger.log_command({
                'type': 'navigate',
                'target': target_pose
            })
            
            if self.use_simulator:
                # 发送导航命令到模拟器
                nav_cmd = {
                    'type': 'navigate',
                    'position': target_pose
                }
                self.command_queue.put(nav_cmd)
            else:
                # 发送导航命令到实际设备
                nav_cmd = {
                    'type': 'navigate',
                    'position': target_pose
                }
                self.command_queue.put(nav_cmd)
    
    def _update_gui(self):
        """更新GUI"""
        # 获取迷宫求解器状态和性能统计
        state_name = self.maze_solver.get_state_name()
        
        # 准备性能统计信息
        performance_stats = {
            'state': state_name,
            'frontiers_detected': self.stats['frontiers_detected'],
            'path_length': self.stats['path_length'],
            'exploration_time': self.stats['exploration_time'],
            'exit_navigation_time': (self.stats['return_time'] - self.stats['exit_found_time']) 
                                    if self.stats['return_time'] and self.stats['exit_found_time'] else None,
            'return_time': (self.stats['completion_time'] - self.stats['return_time']) 
                          if self.stats['completion_time'] and self.stats['return_time'] else None,
            'total_time': (time.time() - self.stats['start_time']) if self.stats['start_time'] else None
        }
        
        # 获取迷宫求解器调试信息
        debug_info = self.maze_solver.debug_info
        
        # 更新GUI
        self.gui.update(
            occupancy_grid=self.map,
            robot_pose=self.pose,
            current_path=self.maze_solver.current_path,
            frontiers=debug_info.get('frontiers_detected', []),
            selected_frontier=self.maze_solver.current_target,
            exit_position=self.maze_solver.exit_position,
            start_position=self.maze_solver.start_position,
            current_state=state_name,
            performance_stats=performance_stats
        )
    
    def _save_final_results(self):
        """保存最终结果"""
        try:
            # 保存最终地图
            self.logger.save_map(self.map, "final_map")
            self.gui.log("最终地图已保存")
            
            # 保存性能统计
            self.logger.log_event("performance", str(self.stats))
            self.gui.log("性能统计已保存")
            
            # 保存路径
            if self.maze_solver.current_path:
                self.logger.log_navigation_commands(self.maze_solver.current_path)
                self.gui.log("最终路径已保存")
                
        except Exception as e:
            self.gui.log(f"保存结果时出错: {str(e)}")
    
    def run(self):
        """运行演示"""
        # 初始化
        self.initialize()
        
        # 设置GUI回调
        self.gui._on_start_clicked = self.start
        self.gui._on_stop_clicked = self.stop
        self.gui._on_reset_clicked = self.reset
        self.gui._on_save_map_clicked = lambda: self.logger.save_map(self.map, "saved_map")
        
        # 自动开始演示
        self.start()
        
        # 手动更新一次GUI确保初始化显示
        self._update_gui()
        
        # 运行GUI
        try:
            self.gui.run()
        finally:
            # 确保在退出时停止所有线程
            self.stop()
            self.gui.close()

def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='2D激光雷达自主移动机器人系统高级演示')
    
    # 添加子命令解析器
    subparsers = parser.add_subparsers(dest='command', help='选择运行模式')
    
    # 迷宫求解器模式
    maze_parser = subparsers.add_parser('maze', help='运行迷宫求解器演示')
    maze_parser.add_argument('--simulator', action='store_true', default=True, help='使用模拟器')
    maze_parser.add_argument('--port', type=str, help='蓝牙串口，例如 COM3 或 /dev/rfcomm0')
    maze_parser.add_argument('--map-size', type=int, default=800, help='地图大小(像素)')
    maze_parser.add_argument('--map-scale', type=float, default=32.0, help='地图大小(米)')
    maze_parser.add_argument('--log-dir', type=str, default='logs', help='日志目录')
    
    # BreezySLAM示例模式
    breezy_parser = subparsers.add_parser('breezyslam', help='运行BreezySLAM示例')
    breezy_parser.add_argument('--dataset', type=str, default='exp1', help='数据集名称 (exp1或exp2)')
    breezy_parser.add_argument('--odometry', action='store_true', default=True, help='使用里程计数据')
    breezy_parser.add_argument('--no-odometry', action='store_false', dest='odometry', help='不使用里程计数据')
    breezy_parser.add_argument('--seed', type=int, default=0, help='随机种子 (0=禁用粒子滤波)')
    breezy_parser.add_argument('--map-size', type=int, default=800, help='地图大小(像素)')
    breezy_parser.add_argument('--map-scale', type=float, default=32.0, help='地图大小(米)')
    breezy_parser.add_argument('--enhanced-viz', action='store_true', help='使用增强可视化')
    
    args = parser.parse_args()
    
    # 默认为迷宫求解器模式
    if not args.command:
        args.command = 'maze'
        # 设置默认参数
        args.simulator = True
        args.port = None
        args.map_size = 800
        args.map_scale = 32.0
        args.log_dir = 'logs'
    
    if args.command == 'maze':
        # 确保日志目录存在
        os.makedirs(args.log_dir, exist_ok=True)
        
        # 运行迷宫求解器演示
        demo = MazeSolverDemo(
            use_simulator=args.simulator,
            map_size_pixels=args.map_size,
            map_size_meters=args.map_scale,
            log_dir=args.log_dir,
            bluetooth_port=args.port
        )
        
        # 运行演示
        demo.run()
        
    elif args.command == 'breezyslam':
        # 运行BreezySLAM示例演示
        visualization_mode = 'enhanced' if args.enhanced_viz else 'standard'
        demo = BreezySLAMDemo(
            dataset_name=args.dataset,
            use_odometry=args.odometry,
            random_seed=args.seed,
            map_size_pixels=args.map_size,
            map_size_meters=args.map_scale,
            visualization_mode=visualization_mode
        )
        
        # 运行演示
        demo.run()

if __name__ == "__main__":
    main() 