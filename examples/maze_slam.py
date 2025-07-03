#!/usr/bin/env python3
'''
maze_slam.py - 迷宫探索SLAM可视化
结合迷宫探索与BreezySLAM风格的可视化
'''

import os
import sys
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import argparse

# 添加BreezySLAM路径
sys.path.append(os.path.join(os.path.dirname(__file__), 'BreezySLAM/python'))
sys.path.append(os.path.join(os.path.dirname(__file__), 'BreezySLAM/examples'))

# 导入BreezySLAM相关模块
try:
    from breezyslam.algorithms import Deterministic_SLAM, RMHC_SLAM
    from roboviz_fix import MapVisualizer
except ImportError:
    print("错误: 无法导入BreezySLAM模块")
    print("请确保BreezySLAM已正确安装，或者BreezySLAM目录在当前目录下")
    sys.exit(1)

# 导入迷宫探索相关模块
from maze_env import MazeEnvironment
from maze_robot import Robot
from maze_visualization import MazeVisualization
from maze_data_recorder import MazeDataRecorder
from threading import Thread

# 地图大小和比例
MAP_SIZE_PIXELS = 500
MAP_SIZE_METERS = 30

class MazeLaser:
    """迷宫激光传感器，适配BreezySLAM接口"""
    
    def __init__(self, num_rays=36, detection_angle_degrees=360, distance_no_detection_mm=5000):
        """初始化激光传感器
        
        参数:
            num_rays: 激光射线数量
            detection_angle_degrees: 检测角度范围（度）
            distance_no_detection_mm: 无检测时的距离（毫米）
        """
        self.num_rays = num_rays
        self.detection_angle_degrees = detection_angle_degrees
        self.distance_no_detection_mm = distance_no_detection_mm
        
        # 计算角度分辨率
        self.angle_resolution = detection_angle_degrees / num_rays
        
        # 设置BreezySLAM所需的参数
        self.scan_size = num_rays
        self.scan_rate_hz = 10
        self.detection_angle_degrees = detection_angle_degrees
        self.distance_no_detection_mm = distance_no_detection_mm
        self.detection_margin = 70
        self.offset_mm = 0
        
class MazeRover:
    """迷宫机器人，适配BreezySLAM接口"""
    
    def __init__(self):
        """初始化机器人"""
        # 设置轮距（毫米）
        self.wheel_radius_mm = 77
        self.half_axle_length_mm = 165
        
        # 设置每圈的计数
        self.ticks_per_cycle = 2000
        
    def computePoseChange(self, odometry):
        """计算位姿变化
        
        参数:
            odometry: 里程计数据 (timestamp, left_wheel, right_wheel)
            
        返回:
            位姿变化 (dxy_mm, dtheta_degrees, dt_seconds)
        """
        # 解析里程计数据
        timestamp, left_wheel, right_wheel = odometry
        
        # 将轮子计数转换为角度
        left_degrees = self._ticks_to_degrees(left_wheel)
        right_degrees = self._ticks_to_degrees(right_wheel)
        
        # 计算位姿变化
        dxy_mm, dtheta_degrees = self._compute_pose_change(left_degrees, right_degrees)
        
        # 添加时间差（秒）- 使用固定值，因为我们不跟踪上一个时间戳
        dt_seconds = 0.1  # 假设每次更新间隔为100毫秒
        
        return dxy_mm, dtheta_degrees, dt_seconds
        
    def _ticks_to_degrees(self, ticks):
        """将轮子计数转换为角度
        
        参数:
            ticks: 轮子计数
            
        返回:
            角度（度）
        """
        return ticks * (180. / self.ticks_per_cycle)
        
    def _compute_pose_change(self, left_degrees, right_degrees):
        """计算位姿变化
        
        参数:
            left_degrees: 左轮角度变化（度）
            right_degrees: 右轮角度变化（度）
            
        返回:
            位姿变化 (dxy_mm, dtheta_degrees)
        """
        # 计算轮子转动的弧长
        left_radians = np.radians(left_degrees)
        right_radians = np.radians(right_degrees)
        
        left_arc_mm = self.wheel_radius_mm * left_radians
        right_arc_mm = self.wheel_radius_mm * right_radians
        
        # 计算位姿变化
        dxy_mm = (left_arc_mm + right_arc_mm) / 2
        dtheta_degrees = (right_arc_mm - left_arc_mm) / (2 * self.half_axle_length_mm) * 180 / np.pi
        
        return dxy_mm, dtheta_degrees

class MazeSLAM:
    """迷宫SLAM可视化器"""
    
    def __init__(self, maze_width=15, maze_height=15, json_file=None, maze_id=None, use_odometry=True, random_seed=0):
        """初始化SLAM可视化器
        
        参数:
            maze_width: 迷宫宽度
            maze_height: 迷宫高度
            json_file: JSON文件路径
            maze_id: 迷宫ID
            use_odometry: 是否使用里程计
            random_seed: 随机种子
        """
        self.maze_width = maze_width
        self.maze_height = maze_height
        self.json_file = json_file
        self.maze_id = maze_id
        self.use_odometry = use_odometry
        self.random_seed = random_seed
        
        # 创建迷宫环境
        self.maze_env = MazeEnvironment(width=maze_width, height=maze_height, json_file=json_file)
        
        # 如果使用了预定义迷宫ID，设置maze_id属性
        if maze_id:
            self.maze_env.maze_id = maze_id
            
        # 确保goal属性与goal_pos一致
        self.maze_env.goal = self.maze_env.goal_pos
        print(f"目标位置设置为: ({self.maze_env.goal[0]}, {self.maze_env.goal[1]})")
            
        # 创建机器人
        self.robot = Robot(self.maze_env.start_pos, self.maze_env)
        
        # 创建激光传感器
        self.laser = MazeLaser()
        
        # 创建机器人模型（用于里程计）
        self.rover = MazeRover() if use_odometry else None
        
        # 创建SLAM算法
        self.slam = RMHC_SLAM(self.laser, MAP_SIZE_PIXELS, MAP_SIZE_METERS, random_seed=random_seed) \
                   if random_seed else \
                   Deterministic_SLAM(self.laser, MAP_SIZE_PIXELS, MAP_SIZE_METERS)
                   
        # 创建地图可视化器
        title = f"迷宫SLAM可视化 - {maze_width}x{maze_height}"
        if maze_id:
            title += f" (迷宫ID: {maze_id})"
        self.viz = MapVisualizer(MAP_SIZE_PIXELS, MAP_SIZE_METERS, title)
        
        # 创建数据记录器
        filename = f"maze_slam_{maze_width}x{maze_height}"
        if maze_id:
            filename += f"_maze{maze_id}"
        self.data_recorder = MazeDataRecorder(filename=filename, save_dir="../data")
        
        # 分配地图字节数组
        self.mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)
        
        # 机器人位姿
        self.pose = [0, 0, 0]  # x_mm, y_mm, theta_degrees
        
        # 上次更新时间
        self.last_update_time = 0
        self.update_interval = 0.1  # 100ms
        
        # 记录间隔
        self.record_interval = 5  # 每5帧记录一次
        self.frame_count = 0
        
        # 探索状态
        self.state = "exploring"  # exploring, planning_to_goal, moving_to_goal, planning_to_start, moving_to_start, finished
        
        # 打印初始信息
        print(f"迷宫大小: {maze_width}x{maze_height}")
        print(f"机器人初始位置: ({self.robot.x}, {self.robot.y})")
        print(f"使用里程计: {use_odometry}")
        print(f"随机种子: {random_seed}")
        
    def update(self):
        """更新一帧"""
        # 检查是否需要更新（限制更新频率）
        current_time = time.time()
        if current_time - self.last_update_time < self.update_interval:
            return True
            
        self.last_update_time = current_time
        self.frame_count += 1
        
        # 更新机器人的传感器数据
        self.robot.update_sensor_data()
        
        # 获取激光扫描数据
        scan_data = self.robot.get_scan_data()
        
        # 获取里程计数据
        if self.use_odometry:
            # 使用机器人位置作为模拟里程计
            odometry = (
                int(current_time * 1000000),  # 时间戳（微秒）
                int(self.robot.x * 100),      # 左轮（简单地将x坐标乘以100）
                int(self.robot.y * 100)       # 右轮（简单地将y坐标乘以100）
            )
            
            # 计算位姿变化
            velocities = self.rover.computePoseChange(odometry)
            
            # 更新SLAM
            self.slam.update(scan_data, velocities)
        else:
            # 只使用激光更新SLAM
            self.slam.update(scan_data)
            
        # 获取新位置
        self.pose[0], self.pose[1], self.pose[2] = self.slam.getpos()
        
        # 获取新地图
        self.slam.getmap(self.mapbytes)
        
        # 显示地图和机器人位姿
        if not self.viz.display(self.pose[0]/1000., self.pose[1]/1000., self.pose[2], self.mapbytes):
            print("可视化窗口已关闭")
            return False
            
        # 记录数据
        if self.frame_count % self.record_interval == 0:
            self.data_recorder.record(self.robot, self.maze_env.grid_env)
            
        # 检查机器人是否到达终点
        current_pos = (int(round(self.robot.x)), int(round(self.robot.y)))
        goal_pos = (int(round(self.maze_env.goal[0])), int(round(self.maze_env.goal[1])))
        
        if current_pos == goal_pos and self.state != "planning_to_start" and self.state != "moving_to_start" and self.state != "finished":
            print(f"机器人到达终点 {goal_pos}！")
            self.state = "planning_to_start"
            
        # 根据当前状态执行不同的操作
        if self.state == "exploring":
            # 探索阶段：机器人探索迷宫，寻找起点和终点
            if not self.robot.explore_maze():
                print("探索完成，进入规划阶段")
                self.state = "planning_to_goal"
                
        elif self.state == "planning_to_goal":
            # 规划阶段：规划从当前位置到终点的路径
            if self.maze_env.goal is not None:
                print(f"规划到终点 {self.maze_env.goal} 的路径")
                goal_path = self.robot.find_path_to_goal(self.maze_env.goal)
                
                if goal_path:
                    print(f"找到到终点的路径，长度: {len(goal_path)}")
                    self.robot.goal_path = goal_path[1:]  # 跳过起点
                    self.state = "moving_to_goal"
                else:
                    print("无法找到到终点的路径，尝试继续探索")
                    self.state = "exploring"
            else:
                print("终点未知，继续探索")
                self.state = "exploring"
                
        elif self.state == "moving_to_goal":
            # 移动阶段：机器人沿着规划的路径移动到终点
            if self.robot.goal_path:
                # 获取路径上的下一个点
                next_point = self.robot.goal_path[0]
                self.robot.goal_path.pop(0)
                
                print(f"移动到终点路径的下一个点 {next_point}")
                
                # 更新位置
                self.robot.update_position((next_point[0], next_point[1], self.robot.theta))
            else:
                # 到达终点
                current_pos = (int(round(self.robot.x)), int(round(self.robot.y)))
                goal_pos = (int(round(self.maze_env.goal[0])), int(round(self.maze_env.goal[1])))
                
                if current_pos == goal_pos:
                    print("已到达终点！")
                    self.state = "planning_to_start"
                else:
                    print("路径执行完毕但未到达终点，重新规划")
                    self.state = "planning_to_goal"
                    
        elif self.state == "planning_to_start":
            # 规划阶段：规划从终点回到起点的路径
            if self.maze_env.start_pos is not None:
                print(f"规划回到起点 {self.maze_env.start_pos} 的路径")
                
                # 使用A*算法直接规划从当前位置到起点的最优路径
                current_pos = (int(round(self.robot.x)), int(round(self.robot.y)))
                start_pos = (int(round(self.maze_env.start_pos[0])), int(round(self.maze_env.start_pos[1])))
                
                # 使用已知的地图信息规划最短路径
                start_path = self.robot.plan_path(current_pos, start_pos)
                
                if start_path:
                    print(f"找到回到起点的路径，长度: {len(start_path)}")
                    self.robot.goal_path = start_path[1:]  # 跳过起点
                    self.state = "moving_to_start"
                else:
                    print("无法找到回到起点的路径")
                    # 尝试随机移动
                    self.robot.random_move_count = 3
                    self.state = "exploring"  # 回到探索状态
            else:
                print("起点未知，继续探索")
                self.state = "exploring"
                
        elif self.state == "moving_to_start":
            # 移动阶段：机器人沿着规划的路径移动回起点
            if self.robot.goal_path:
                # 获取路径上的下一个点
                next_point = self.robot.goal_path[0]
                self.robot.goal_path.pop(0)
                
                print(f"移动到起点路径的下一个点 {next_point}")
                
                # 更新位置
                self.robot.update_position((next_point[0], next_point[1], self.robot.theta))
            else:
                # 到达起点
                current_pos = (int(round(self.robot.x)), int(round(self.robot.y)))
                start_pos = (int(round(self.maze_env.start_pos[0])), int(round(self.maze_env.start_pos[1])))
                
                if current_pos == start_pos:
                    print("已回到起点！任务完成！")
                    self.state = "finished"
                else:
                    print("路径执行完毕但未回到起点，重新规划")
                    self.state = "planning_to_start"
                    
        elif self.state == "finished":
            # 完成状态：任务完成
            print("迷宫探索任务完成！")
            
            # 保存记录的数据
            filepath = self.data_recorder.save()
            print(f"探索数据已保存到 {filepath}")
            
            # 提示用户如何查看数据
            print("\n要查看探索过程，请运行以下命令:")
            print(f"python maze_visualizer.py {filepath}.pkl")
            
            return False
            
        # 更新探索进度
        exploration_progress = self.robot.exploration_progress
        print(f"探索进度: {exploration_progress:.2f}%")
        
        return True
        
    def run(self):
        """运行SLAM可视化"""
        print("开始迷宫SLAM可视化...")
        
        try:
            # 添加最大迭代次数，防止无限循环
            max_iterations = 1000
            iteration_count = 0
            
            while self.update() and iteration_count < max_iterations:
                iteration_count += 1
                
            if iteration_count >= max_iterations:
                print("\n达到最大迭代次数，可能陷入循环。保存数据...")
                # 保存记录的数据
                filepath = self.data_recorder.save()
                print(f"探索数据已保存到 {filepath}")
                
                # 提示用户如何查看数据
                print("\n要查看探索过程，请运行以下命令:")
                print(f"python maze_visualizer.py {filepath}.pkl")
        except KeyboardInterrupt:
            print("\n用户中断，保存数据...")
            
            # 保存记录的数据
            filepath = self.data_recorder.save()
            print(f"探索数据已保存到 {filepath}")
            
            # 提示用户如何查看数据
            print("\n要查看探索过程，请运行以下命令:")
            print(f"python maze_visualizer.py {filepath}.pkl")
            
        print("SLAM可视化结束")
        
def main():
    """主函数"""
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='迷宫SLAM可视化')
    parser.add_argument('--width', type=int, default=15, help='迷宫宽度，默认15')
    parser.add_argument('--height', type=int, default=15, help='迷宫高度，默认15')
    parser.add_argument('--maze-id', type=int, help='使用预定义的迷宫ID (1, 2, 或 3)')
    parser.add_argument('--json-file', help='使用JSON文件定义的迷宫')
    parser.add_argument('--no-odometry', action='store_true', help='不使用里程计')
    parser.add_argument('--seed', type=int, default=0, help='随机种子，0表示使用确定性SLAM')
    args = parser.parse_args()
    
    # 确定JSON文件路径
    json_file = None
    if args.maze_id:
        json_file = os.path.join('json_data', f"{args.maze_id}.json")
    elif args.json_file:
        json_file = args.json_file
        
    # 创建SLAM可视化器
    slam = MazeSLAM(
        maze_width=args.width,
        maze_height=args.height,
        json_file=json_file,
        maze_id=args.maze_id,
        use_odometry=not args.no_odometry,
        random_seed=args.seed
    )
    
    # 运行SLAM可视化
    slam.run()
    
if __name__ == "__main__":
    import argparse
    import os
    sys.path.append(os.path.join(os.path.dirname(__file__), 'best'))
    sys.path.append(os.path.join(os.path.dirname(__file__), '../best'))
    try:
        from maze_slam_visual_new2 import GlobalMazeSLAMSystem
    except ImportError as e:
        print("无法导入GlobalMazeSLAMSystem，请确认examples/best/maze_slam_visual_new2.py存在且无语法错误。")
        raise e

    parser = argparse.ArgumentParser(description="迷宫SLAM探索与可视化 (集成新版SLAM系统)")
    parser.add_argument('--map', type=str, default="json_data/1.json", help="json地图文件路径，默认json_data/1.json")
    args = parser.parse_args()
    map_file = os.path.join(os.path.dirname(__file__), args.map)
    if not os.path.exists(map_file):
        print(f"地图文件不存在: {map_file}")
        exit(1)
    print(f"使用地图文件: {map_file}")
    slam_system = GlobalMazeSLAMSystem(map_file)
    slam_system.run_exploration()  # 会自动弹出matplotlib窗口实时显示 