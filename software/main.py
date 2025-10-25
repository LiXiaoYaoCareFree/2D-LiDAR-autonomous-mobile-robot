import sys
import time
import logging
from pathlib import Path

sys.path.append(str(Path(__file__).parent))

from modules.communication import RobotInterface
from modules.sensors.data_structures import Pose, Scan, SensorData
from modules.mapping import SLAMProcessor, MapManager
from modules.navigation import AStarPlanner, GoalManager, DWALocalPlanner, DWAConfig, SpeedCompensator
from modules.visualization import Dashboard
from modules.utils import ConfigManager, DataLogger

import matplotlib.pyplot as plt
import numpy as np

logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] %(levelname)s: %(message)s',
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler('robot_system.log')
    ]
)
logger = logging.getLogger(__name__)


class RobotNavigationSystem:
    
    def __init__(self, config_file: str = "config.json"):
        self.config_manager = ConfigManager(config_file)
        self.config = self.config_manager.get_config()
        
        self._init_components()
        
        self.is_running = False
        self.current_mode = "IDLE"
        
        logger.info("机器人导航系统已初始化")
    
    def _init_components(self):
        """初始化所有系统组件"""
        
        # 1. 通信接口 (LDR3 协议)
        self.robot_interface = RobotInterface(
            port=self.config.communication.port,
            baudrate=self.config.communication.baudrate,
            wheel_base=self.config.sensor.wheel_base,
            init_pose=(0.0, 0.0, 0.0),
            use_ldr3_protocol=True
        )
        
        # 2. SLAM 处理器 (集成 ICP + 位姿融合 + OGM)
        self.slam_processor = SLAMProcessor(
            map_bounds=self.config.mapping.map_bounds,
            map_resolution=self.config.mapping.map_resolution,
            enable_icp=True,
            enable_fusion=True
        )
        
        # 3. 地图管理器
        self.map_manager = MapManager()
        
        # 4. 全局路径规划器 (使用 PythonRobotics A*)
        self.path_planner = AStarPlanner(
            grid_resolution=self.config.mapping.map_resolution,
            robot_radius=self.config.navigation.robot_radius
        )
        
        # 5. DWA 局部控制器
        dwa_config = DWAConfig(
            v_max_mps=self.config.navigation.max_linear_vel,
            w_max_rps=self.config.navigation.max_angular_vel,
            a_max_mps2=0.8,
            aw_max_rps2=5.0,
            v_resolution=0.05,
            w_resolution=0.2,
            sim_time_s=1.0,
            dt_s=0.1,
            weight_heading=2.5,
            weight_clearance=1.0,
            weight_velocity=0.5,
            robot_radius_m=self.config.navigation.robot_radius,
            mcu_wheel_min_mps=0.13,
            front_safety_margin_m=0.20,
            debug=False
        )
        
        self.dwa_controller = DWALocalPlanner(
            config=dwa_config,
            wheel_base_m=self.config.sensor.wheel_base
        )
        logger.info("使用 DWA 局部控制器")
        
        # 6. 速度补偿器
        self.speed_compensator = SpeedCompensator(
            alpha=0.2,
            min_gain=0.80,
            max_gain=1.50,
            update_threshold=0.08
        )
        
        # 7. 目标管理器
        self.goal_manager = GoalManager(
            default_tolerance=self.config.navigation.goal_tolerance
        )
        
        # 8. 可视化
        self.dashboard = Dashboard(
            figsize=self.config.visualization.figsize,
            layout=self.config.visualization.layout
        )
        
        # 9. 数据记录
        if self.config.data_logging:
            self.data_logger = DataLogger(
                log_dir=self.config.log_dir,
                max_file_size=self.config.max_log_size
            )
        else:
            self.data_logger = None
    
    def start(self):
        try:
            self.is_running = True
            self.current_mode = "AUTO"
            
            logger.info("系统启动中...")
            
            self._main_loop()
            
        except KeyboardInterrupt:
            logger.info("用户中断，正在关闭系统...")
        except Exception as e:
            logger.error(f"系统运行错误: {e}")
        finally:
            self.stop()
    
    def _main_loop(self):
        """
        主循环：SENSE → ICP → FUSE → MAP → PLAN → ACT
        """
        logger.info("进入主循环 (SENSE→ICP→FUSE→MAP→PLAN→ACT)")
        
        loop_count = 0
        
        while self.is_running:
            try:
                loop_count += 1
                
                # === SENSE: 获取传感器数据 ===
                sensor_data = self._get_sensor_data()
                
                if not sensor_data.is_valid():
                    time.sleep(0.1)
                    continue
                
                # === ICP + FUSE + MAP: SLAM 更新 ===
                # (包含 ICP 定位、位姿融合、OGM 建图)
                slam_result = self.slam_processor.update(sensor_data)
                
                # 获取融合后的位姿
                fused_pose = self.slam_processor.get_current_pose()
                
                # === PLAN + ACT: 导航控制 ===
                navigation_result = self._update_navigation_dwa(fused_pose, sensor_data.scan)
                
                # === 可视化 ===
                self._update_visualization(sensor_data, slam_result, navigation_result)
                
                # === 数据记录 ===
                if self.data_logger:
                    self._log_data(sensor_data, slam_result, navigation_result, loop_count)
                
                time.sleep(self.config.visualization.update_interval)
                
            except Exception as e:
                logger.error(f"主循环错误: {e}")
                import traceback
                traceback.print_exc()
                time.sleep(0.1)
    
    def _get_sensor_data(self):
        """
        SENSE: 获取传感器数据
        
        Returns:
            SensorData 对象
        """
        # 从 LDR3 接口获取位姿（已包含里程计+IMU融合）
        pose_tuple = self.robot_interface.get_pose()
        pose = Pose(pose_tuple[0], pose_tuple[1], pose_tuple[2])
        
        # 获取激光扫描
        scan_angles, scan_ranges = self.robot_interface.get_lidar_scan()
        scan = Scan(angles=scan_angles, ranges=scan_ranges)
        
        # 构建传感器数据对象
        sensor_data = SensorData(pose=pose, scan=scan)
        
        return sensor_data
    
    def _update_navigation_dwa(self, current_pose: Pose, scan: Scan):
        """
        PLAN + ACT: 使用 DWA 进行导航控制
        
        Args:
            current_pose: 当前位姿（融合后）
            scan: 激光扫描数据
            
        Returns:
            导航结果字典
        """
        # 检查是否有目标
        if not self.goal_manager.get_current_goal():
            self.robot_interface.send_wheel_speeds(0.0, 0.0)
            return {
                'state': 'IDLE',
                'vL': 0.0,
                'vR': 0.0,
                'path': []
            }
        
        # 更新导航状态
        nav_state = self.goal_manager.update_navigation(current_pose)
        
        if nav_state['state'] == 'NAVIGATING':
            current_goal = nav_state['current_goal']
            
            # PLAN: 全局路径规划 (使用 PythonRobotics A*)
            start = (current_pose.x, current_pose.y)
            goal = (current_goal.x, current_goal.y)
            
            # 从 OGM 获取障碍物
            occupancy_map = self.slam_processor.get_occupancy_map()
            obstacles = occupancy_map.get_obstacles()
            
            # 转换障碍物格式（栅格坐标 → 世界坐标）
            obstacle_coords = []
            for gx, gy in obstacles:
                wx, wy = occupancy_map.grid_to_world(gx, gy)
                obstacle_coords.append((wx, wy))
            
            self.path_planner.set_obstacles(obstacle_coords)
            path = self.path_planner.plan(start, goal)
            
            # ACT: DWA 局部控制
            self.dwa_controller.set_goal(current_goal.x, current_goal.y)
            
            # 获取当前速度（用于动态窗口）
            v_left, v_right = self.robot_interface.get_velocity()
            v_current = 0.5 * (v_left + v_right)
            w_current = (v_right - v_left) / self.config.sensor.wheel_base
            self.dwa_controller.update_velocity(v_current, w_current)
            
            # 计算 DWA 控制输出（轮速）
            vL, vR = self.dwa_controller.compute(current_pose, scan)
            
            # 速度补偿
            vL_comp, vR_comp = self.speed_compensator.apply(vL, vR)
            
            # 发送控制命令
            self.robot_interface.send_wheel_speeds(vL_comp, vR_comp)
            
            # 反馈更新速度补偿器
            v_fb_left, v_fb_right = self.robot_interface.get_velocity()
            self.speed_compensator.update(vL_comp, vR_comp, v_fb_left, v_fb_right)
            
            return {
                'state': nav_state['state'],
                'vL': vL,
                'vR': vR,
                'vL_comp': vL_comp,
                'vR_comp': vR_comp,
                'path': path,
                'current_goal': current_goal
            }
        
        # 非导航状态：停止
        self.robot_interface.send_wheel_speeds(0.0, 0.0)
        return {
            'state': nav_state['state'],
            'vL': 0.0,
            'vR': 0.0,
            'path': []
        }
    
    
    def _update_visualization(self, sensor_data, slam_result, navigation_result):
        """
        更新可视化显示
        
        Args:
            sensor_data: 传感器数据
            slam_result: SLAM 更新结果
            navigation_result: 导航结果
        """
        occupancy_map = self.slam_processor.get_occupancy_map()
        
        # 使用融合后的位姿
        current_pose = self.slam_processor.get_current_pose()
        
        path = navigation_result.get('path', [])
        
        # SLAM 地图可视化
        self.dashboard.update_slam_visualization(
            occupancy_map=occupancy_map,
            robot_pose=current_pose,
            scan_data=sensor_data.scan
        )
        
        # 路径可视化
        if path and len(path) > 0:
            goal = navigation_result.get('current_goal')
            goal_pos = (goal.x, goal.y) if goal else None
            
            # 从地图获取障碍物
            obstacles = occupancy_map.get_obstacles()
            obstacle_coords = []
            for gx, gy in obstacles[:1000]:  # 限制显示数量
                wx, wy = occupancy_map.grid_to_world(gx, gy)
                obstacle_coords.append((wx, wy))
            
            self.dashboard.update_path_visualization(
                path=path,
                obstacles=obstacle_coords,
                start=(current_pose.x, current_pose.y),
                goal=goal_pos
            )
        
        # 数据面板
        vL = navigation_result.get('vL', 0.0)
        vR = navigation_result.get('vR', 0.0)
        v = 0.5 * (vL + vR)
        w = (vR - vL) / self.config.sensor.wheel_base if vL != 0 or vR != 0 else 0
        
        self.dashboard.update_data_panel({
            'pose': current_pose,
            'velocity': v,
            'angular_velocity': w,
            'vL': vL,
            'vR': vR,
            'navigation_state': navigation_result.get('state', 'IDLE'),
            'icp_enabled': slam_result.get('icp_result') is not None,
            'fusion_applied': slam_result.get('fusion_info', {}).get('fusion_applied', False)
        })
        
        self.dashboard.show_visualization()
    
    def _log_data(self, sensor_data, slam_result, navigation_result, loop_count):
        """
        记录数据到日志文件
        
        Args:
            sensor_data: 传感器数据
            slam_result: SLAM 结果
            navigation_result: 导航结果
            loop_count: 循环计数
        """
        if not self.data_logger:
            return
        
        # 获取融合后的位姿
        final_pose = self.slam_processor.get_current_pose()
        
        # 构建日志数据
        log_data = {
            'step': loop_count,
            'timestamp': time.time(),
            
            # 位姿数据
            'pose_x': final_pose.x,
            'pose_y': final_pose.y,
            'pose_theta': final_pose.theta,
            
            # ICP 结果
            'icp_success': slam_result.get('icp_result') is not None,
            'icp_delta_x': slam_result.get('icp_result').delta_x if slam_result.get('icp_result') else 0.0,
            'icp_delta_y': slam_result.get('icp_result').delta_y if slam_result.get('icp_result') else 0.0,
            'icp_rmse': slam_result.get('icp_result').rmse if slam_result.get('icp_result') else 0.0,
            
            # 融合状态
            'fusion_applied': slam_result.get('fusion_info', {}).get('fusion_applied', False),
            
            # 控制输出
            'vL': navigation_result.get('vL', 0.0),
            'vR': navigation_result.get('vR', 0.0),
            'vL_comp': navigation_result.get('vL_comp', 0.0),
            'vR_comp': navigation_result.get('vR_comp', 0.0),
            
            # 导航状态
            'nav_state': navigation_result.get('state', 'IDLE'),
            'path_length': len(navigation_result.get('path', []))
        }
        
        self.data_logger.log_navigation_data(
            pose=final_pose,
            scan=sensor_data.scan,
            control_output=log_data,
            navigation_state=navigation_result
        )
    
    def add_goal(self, x: float, y: float, theta: float = None):
        self.goal_manager.add_goal(x, y, theta)
        logger.info(f"添加目标: ({x}, {y})")
    
    def add_goals(self, goals: list):
        self.goal_manager.add_goals(goals)
        logger.info(f"批量添加目标: {len(goals)} 个")
    
    def set_mode(self, mode: str):
        self.current_mode = mode
        logger.info(f"运行模式设置为: {mode}")
    
    def stop(self):
        self.is_running = False
        
        self.robot_interface.stop()
        
        self.dashboard.close()
        
        logger.info("系统已停止")
    
    def save_map(self, map_name: str):
        self.map_manager.current_map = self.slam_processor.get_occupancy_map()
        self.map_manager.save_map(map_name)
        logger.info(f"地图已保存: {map_name}")
    
    def load_map(self, map_name: str):
        map_obj = self.map_manager.load_map(map_name)
        if map_obj:
            self.slam_processor.occupancy_map = map_obj
            logger.info(f"地图已加载: {map_name}")


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='智能小车导航系统 - 使用BreezySLAM, PyRoboViz, PythonRobotics')
    parser.add_argument('--config', '-c', 
                      default='config.json',
                      help='配置文件路径 (默认: config.json)')
    parser.add_argument('--mode', '-m',
                      choices=['default', 'debug', 'production'],
                      help='预设配置模式')
    parser.add_argument('--goals', '-g',
                      nargs='+',
                      help='目标点坐标 (格式: x1,y1 x2,y2 ...)')
    parser.add_argument('--test', '-t',
                      action='store_true',
                      help='运行测试模式，展示库集成效果')
    
    args = parser.parse_args()
    
    if args.mode:
        config_file = f"configs/{args.mode}.json"
    else:
        config_file = args.config
    
    
    system = RobotNavigationSystem(config_file)
    
    if args.goals:
        goals = []
        for goal_str in args.goals:
            try:
                x, y = map(float, goal_str.split(','))
                goals.append((x, y))
            except ValueError:
                print(f"无效的目标点格式: {goal_str}")
                continue
        system.add_goals(goals)
    else:
        system.add_goals([
            (1.0, 1.0),
            (2.0, 2.0),
            (3.0, 1.0)
        ])
    
    try:
        system.start()
    except KeyboardInterrupt:
        logger.info("用户中断，正在关闭系统...")
    finally:
        system.stop()
        logger.info("系统已关闭")


if __name__ == "__main__":
    main()
