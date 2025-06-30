#!/usr/bin/env python3
'''
demo_maze_solver.py - 迷宫求解器演示程序

该程序演示了随机迷宫解决方案和交互界面功能，包括地图构建、边界探索、路径规划等。
'''

import numpy as np
import time
import threading
import argparse
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import Laser
from modules.maze_solver import MazeSolver
from modules.enhanced_visualization import RobotGUI
from modules.communication import BluetoothCommunication
from modules.data_logger import DataLogger
from simulator import LidarSimulator

class MazeSolverDemo:
    """迷宫求解器演示类，展示随机迷宫解决方案和交互界面"""
    
    def __init__(self, use_simulator=True, map_size_pixels=800, map_size_meters=32):
        """
        初始化迷宫求解器演示
        
        参数:
            use_simulator: 是否使用模拟器
            map_size_pixels: 地图大小(像素)
            map_size_meters: 地图大小(米)
        """
        self.map_size_pixels = map_size_pixels
        self.map_size_meters = map_size_meters
        self.use_simulator = use_simulator
        
        # 初始化SLAM
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
        self.maze_solver = MazeSolver(map_size_pixels, map_size_meters)
        
        # 初始化GUI
        self.gui = RobotGUI()
        
        # 初始化通信模块
        self.comm = BluetoothCommunication()
        
        # 初始化数据记录器
        self.logger = DataLogger()
        
        # 如果使用模拟器，初始化模拟器
        if use_simulator:
            self.simulator = LidarSimulator(map_size_meters, map_size_pixels)
            self.simulator.generate_random_maze()
        else:
            self.simulator = None
        
        # 初始化位姿
        self.pose = (map_size_meters/2, map_size_meters/2, 0)  # (x, y, theta) 单位为米和度
        
        # 运行标志
        self.running = False
        self.paused = False
        
        # 初始化线程
        self.slam_thread = None
    
    def initialize(self):
        """初始化演示"""
        # 记录日志
        self.gui.log("初始化迷宫求解器演示...")
        
        # 初始化迷宫求解器
        self.maze_solver.initialize(self.pose)
        
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
            self.simulator.generate_random_maze()
        
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
        
        # 创建并启动SLAM线程
        self.slam_thread = threading.Thread(target=self._slam_loop)
        self.slam_thread.daemon = True
        self.slam_thread.start()
        
        # 记录日志
        self.gui.log("迷宫求解器已启动")
    
    def stop(self):
        """停止演示"""
        self.running = False
        
        # 等待线程结束
        if self.slam_thread and self.slam_thread.is_alive():
            self.slam_thread.join(timeout=1.0)
        
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
        
        # 重置位姿
        self.pose = (self.map_size_meters/2, self.map_size_meters/2, 0)
        
        # 重置迷宫求解器
        self.maze_solver = MazeSolver(self.map_size_pixels, self.map_size_meters)
        
        # 如果使用模拟器，重置模拟器
        if self.use_simulator:
            self.simulator.generate_random_maze()
        
        # 重新初始化
        self.initialize()
        
        # 记录日志
        self.gui.log("迷宫求解器已重置")
    
    def save_map(self, filename="maze_map.png"):
        """
        保存地图
        
        参数:
            filename: 文件名
        """
        import matplotlib.pyplot as plt
        
        plt.figure(figsize=(10, 10))
        plt.imshow(self.map, cmap='gray')
        plt.title("迷宫地图")
        plt.colorbar(label='占用概率')
        plt.savefig(filename)
        plt.close()
        
        self.gui.log(f"地图已保存至 {filename}")
    
    def _slam_loop(self):
        """SLAM主循环，在单独的线程中运行"""
        # 记录起始时间
        start_time = time.time()
        last_update_time = start_time
        
        # 主循环
        while self.running:
            if self.paused:
                time.sleep(0.1)
                continue
            
            # 获取激光数据
            if self.use_simulator:
                # 从模拟器获取激光数据
                scan_data = self.simulator.get_scan_data(self.pose)
            else:
                # 从实际设备获取激光数据
                scan_data = self.comm.get_lidar_data()
            
            if scan_data is None:
                time.sleep(0.01)
                continue
            
            # 更新SLAM
            self._update_slam(scan_data)
            
            # 更新迷宫求解器
            self._update_maze_solver()
            
            # 每100ms更新一次GUI
            current_time = time.time()
            if current_time - last_update_time > 0.1:
                self._update_gui()
                last_update_time = current_time
            
            # 控制循环速率
            time.sleep(0.01)
    
    def _update_slam(self, scan_data):
        """
        更新SLAM
        
        参数:
            scan_data: 激光扫描数据
        """
        # 提取距离数据和角度
        distances = scan_data['distances']
        angles = scan_data['angles']
        
        # 转换为SLAM所需格式
        distances_mm = [int(d * 1000) for d in distances]  # 转换为毫米
        
        # 提取当前位姿
        x_mm = int(self.pose[0] * 1000)  # 转换为毫米
        y_mm = int(self.pose[1] * 1000)  # 转换为毫米
        theta_degrees = self.pose[2]     # 度
        
        # 更新SLAM
        self.slam.update(distances_mm, scan_angles_degrees=angles)
        
        # 获取更新后的位姿
        x_mm, y_mm, theta_degrees = self.slam.getpos()
        
        # 更新位姿（转换回米）
        self.pose = (x_mm / 1000, y_mm / 1000, theta_degrees)
        
        # 获取地图
        self.slam.getmap(self.map)
    
    def _update_maze_solver(self):
        """更新迷宫求解器"""
        # 更新迷宫求解器
        target_pose, is_complete = self.maze_solver.update(self.map, self.pose)
        
        # 如果任务完成
        if is_complete:
            self.gui.log("迷宫任务完成!")
            self.pause()
            return
        
        # 如果有目标位姿，发送导航命令
        if target_pose:
            if self.use_simulator:
                # 在模拟器中移动
                self.simulator.move_to(self.pose, target_pose)
                # 更新位姿
                self.pose = target_pose
            else:
                # 发送导航命令到实际设备
                self.comm.send_navigation_command(target_pose)
    
    def _update_gui(self):
        """更新GUI"""
        # 获取迷宫求解器状态和性能统计
        state_name = self.maze_solver.get_state_name()
        stats = self.maze_solver.get_performance_stats()
        
        # 获取迷宫求解器调试信息
        debug_info = self.maze_solver.debug_info
        
        # 更新GUI
        self.gui.update(
            occupancy_grid=self.map,
            robot_pose=self.pose,
            current_path=self.maze_solver.current_path,
            frontiers=debug_info['frontiers_detected'] if 'frontiers_detected' in debug_info else None,
            selected_frontier=self.maze_solver.current_target,
            exit_position=self.maze_solver.exit_position,
            start_position=self.maze_solver.start_position,
            current_state=state_name,
            performance_stats=stats
        )
    
    def run(self):
        """运行演示"""
        # 初始化
        self.initialize()
        
        # 设置GUI回调
        self.gui._on_start_clicked = self.start
        self.gui._on_stop_clicked = self.stop
        self.gui._on_reset_clicked = self.reset
        self.gui._on_save_map_clicked = self.save_map
        
        # 运行GUI
        try:
            self.gui.run()
        finally:
            # 确保在退出时停止所有线程
            self.stop()
            self.gui.close()

def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='迷宫求解器演示程序')
    parser.add_argument('--simulator', action='store_true', help='使用模拟器')
    parser.add_argument('--map-size-pixels', type=int, default=800, help='地图大小(像素)')
    parser.add_argument('--map-size-meters', type=int, default=32, help='地图大小(米)')
    
    args = parser.parse_args()
    
    # 创建演示对象
    demo = MazeSolverDemo(
        use_simulator=args.simulator,
        map_size_pixels=args.map_size_pixels,
        map_size_meters=args.map_size_meters
    )
    
    # 运行演示
    demo.run()

if __name__ == "__main__":
    main() 