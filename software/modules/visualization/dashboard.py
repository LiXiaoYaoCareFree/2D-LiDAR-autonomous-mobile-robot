"""
仪表板模块
使用PyRoboViz提供多面板实时显示界面
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.gridspec import GridSpec
import numpy as np
from typing import Dict, Any, Optional, List, Tuple
import logging

# 使用PyRoboViz库
from pyroboviz import RobotViz, MapViz, PathViz
from pyroboviz.utils import ColorMap

from .map_visualizer import MapVisualizer
from ..sensors.data_structures import Pose, Scan
from ..mapping.occupancy_grid import OccupancyGridMap

logger = logging.getLogger(__name__)


class Dashboard:
    """使用PyRoboViz的多面板仪表板"""
    
    def __init__(self, figsize: Tuple[float, float] = (16, 12),
                 layout: str = "2x2"):
        """
        初始化仪表板
        
        Args:
            figsize: 图像尺寸
            layout: 布局方式 ("2x2", "1x3", "3x1")
        """
        self.figsize = figsize
        self.layout = layout
        
        # 创建图形
        self.fig = plt.figure(figsize=figsize)
        
        # 初始化PyRoboViz组件
        try:
            self.robot_viz = RobotViz()
            self.map_viz = MapViz()
            self.path_viz = PathViz()
            self.color_map = ColorMap()
        except:
            # 如果PyRoboViz不可用，使用matplotlib作为备选
            self.robot_viz = None
            self.map_viz = None
            self.path_viz = None
            self.color_map = None
        
        # 设置网格布局
        if layout == "2x2":
            self.gs = GridSpec(2, 2, figure=self.fig)
            self.ax_map = self.fig.add_subplot(self.gs[0, 0])
            self.ax_ogm = self.fig.add_subplot(self.gs[0, 1])
            self.ax_path = self.fig.add_subplot(self.gs[1, 0])
            self.ax_data = self.fig.add_subplot(self.gs[1, 1])
        elif layout == "1x3":
            self.gs = GridSpec(1, 3, figure=self.fig)
            self.ax_map = self.fig.add_subplot(self.gs[0])
            self.ax_ogm = self.fig.add_subplot(self.gs[1])
            self.ax_data = self.fig.add_subplot(self.gs[2])
            self.ax_path = None
        elif layout == "3x1":
            self.gs = GridSpec(3, 1, figure=self.fig)
            self.ax_map = self.fig.add_subplot(self.gs[0])
            self.ax_ogm = self.fig.add_subplot(self.gs[1])
            self.ax_data = self.fig.add_subplot(self.gs[2])
            self.ax_path = None
        else:
            raise ValueError(f"不支持的布局: {layout}")
        
        # 初始化各面板
        self._setup_panels()
        
        # 数据存储
        self.pose_history = []
        self.velocity_history = []
        self.error_history = []
        
        logger.info(f"仪表板已初始化: {layout} 布局")
    
    def _setup_panels(self):
        """设置各面板"""
        # 地图面板
        self.ax_map.set_title("实时地图")
        self.ax_map.set_xlabel("X (米)")
        self.ax_map.set_ylabel("Y (米)")
        self.ax_map.grid(True, alpha=0.3)
        self.ax_map.set_aspect('equal')
        
        # 占用栅格地图面板
        self.ax_ogm.set_title("占用栅格地图")
        self.ax_ogm.set_xlabel("X (米)")
        self.ax_ogm.set_ylabel("Y (米)")
        self.ax_ogm.grid(True, alpha=0.3)
        self.ax_ogm.set_aspect('equal')
        
        # 路径规划面板
        if self.ax_path is not None:
            self.ax_path.set_title("路径规划")
            self.ax_path.set_xlabel("X (米)")
            self.ax_path.set_ylabel("Y (米)")
            self.ax_path.grid(True, alpha=0.3)
            self.ax_path.set_aspect('equal')
        
        # 数据面板
        self.ax_data.set_title("系统数据")
        self.ax_data.set_xlabel("时间")
        self.ax_data.set_ylabel("值")
        self.ax_data.grid(True, alpha=0.3)
    
    def update_map_panel(self, robot_pose: Pose, scan: Scan, 
                        path: Optional[List[Tuple[float, float]]] = None,
                        obstacles: Optional[List[Tuple[float, float]]] = None):
        """更新地图面板"""
        self.ax_map.clear()
        self._setup_panels()
        
        # 绘制机器人
        self.ax_map.plot(robot_pose.x, robot_pose.y, 'bo', markersize=8, label='机器人')
        
        # 绘制机器人方向
        arrow_length = 0.1
        dx = arrow_length * np.cos(robot_pose.theta)
        dy = arrow_length * np.sin(robot_pose.theta)
        self.ax_map.arrow(robot_pose.x, robot_pose.y, dx, dy,
                          head_width=0.03, head_length=0.02,
                          fc='blue', ec='blue', alpha=0.8)
        
        # 绘制激光雷达数据
        if scan.angles and scan.ranges:
            world_points = []
            for angle, range_val in zip(scan.angles, scan.ranges):
                if range_val > 0:
                    local_x = range_val * np.cos(angle)
                    local_y = range_val * np.sin(angle)
                    
                    cos_theta = np.cos(robot_pose.theta)
                    sin_theta = np.sin(robot_pose.theta)
                    
                    world_x = robot_pose.x + local_x * cos_theta - local_y * sin_theta
                    world_y = robot_pose.y + local_x * sin_theta + local_y * cos_theta
                    
                    world_points.append((world_x, world_y))
            
            if world_points:
                x_coords = [p[0] for p in world_points]
                y_coords = [p[1] for p in world_points]
                self.ax_map.scatter(x_coords, y_coords, c='red', s=1, alpha=0.6, label='激光雷达')
        
        # 绘制路径
        if path:
            x_coords = [p[0] for p in path]
            y_coords = [p[1] for p in path]
            self.ax_map.plot(x_coords, y_coords, 'g--', linewidth=2, alpha=0.8, label='路径')
        
        # 绘制障碍物
        if obstacles:
            x_coords = [obs[0] for obs in obstacles]
            y_coords = [obs[1] for obs in obstacles]
            self.ax_map.scatter(x_coords, y_coords, c='black', s=5, alpha=0.7, label='障碍物')
        
        self.ax_map.legend()
        self.ax_map.set_title("实时地图")
    
    def update_ogm_panel(self, occupancy_map: OccupancyGridMap, robot_pose: Pose):
        """更新占用栅格地图面板"""
        self.ax_ogm.clear()
        self._setup_panels()
        
        # 绘制占用栅格地图
        occ_map = occupancy_map.get_occupancy_map()
        bounds = occupancy_map.get_map_bounds()
        
        extent = [bounds[0], bounds[2], bounds[1], bounds[3]]
        im = self.ax_ogm.imshow(occ_map, extent=extent, origin='lower',
                               cmap='gray', vmin=0, vmax=1)
        
        # 绘制机器人位置
        self.ax_ogm.plot(robot_pose.x, robot_pose.y, 'bo', markersize=6, label='机器人')
        
        # 绘制机器人方向
        arrow_length = 0.1
        dx = arrow_length * np.cos(robot_pose.theta)
        dy = arrow_length * np.sin(robot_pose.theta)
        self.ax_ogm.arrow(robot_pose.x, robot_pose.y, dx, dy,
                          head_width=0.03, head_length=0.02,
                          fc='blue', ec='blue', alpha=0.8)
        
        self.ax_ogm.legend()
        self.ax_ogm.set_title("占用栅格地图")
    
    def update_path_panel(self, path: List[Tuple[float, float]], 
                         obstacles: List[Tuple[float, float]],
                         robot_pose: Pose):
        """更新路径规划面板"""
        if self.ax_path is None:
            return
        
        self.ax_path.clear()
        self._setup_panels()
        
        # 绘制障碍物
        if obstacles:
            x_coords = [obs[0] for obs in obstacles]
            y_coords = [obs[1] for obs in obstacles]
            self.ax_path.scatter(x_coords, y_coords, c='black', s=5, alpha=0.7, label='障碍物')
        
        # 绘制路径
        if path:
            x_coords = [p[0] for p in path]
            y_coords = [p[1] for p in path]
            self.ax_path.plot(x_coords, y_coords, 'g-', linewidth=2, alpha=0.8, label='路径')
            
            # 标记起点和终点
            if len(path) > 0:
                self.ax_path.plot(path[0][0], path[0][1], 'go', markersize=8, label='起点')
            if len(path) > 1:
                self.ax_path.plot(path[-1][0], path[-1][1], 'ro', markersize=8, label='终点')
        
        # 绘制机器人
        self.ax_path.plot(robot_pose.x, robot_pose.y, 'bo', markersize=8, label='机器人')
        
        self.ax_path.legend()
        self.ax_path.set_title("路径规划")
    
    def update_data_panel(self, data: Dict[str, Any]):
        """更新数据面板"""
        self.ax_data.clear()
        self._setup_panels()
        
        # 更新历史数据
        current_time = len(self.pose_history)
        
        if 'pose' in data:
            pose = data['pose']
            self.pose_history.append((pose.x, pose.y, pose.theta))
        
        if 'velocity' in data:
            velocity = data['velocity']
            self.ax_data.plot(current_time, velocity, 'b-', label='速度')
        
        if 'error' in data:
            error = data['error']
            self.ax_data.plot(current_time, error, 'r-', label='误差')
        
        # 限制历史长度
        max_history = 100
        if len(self.pose_history) > max_history:
            self.pose_history = self.pose_history[-max_history:]
        
        # 绘制位姿历史
        if len(self.pose_history) > 1:
            x_coords = [p[0] for p in self.pose_history]
            y_coords = [p[1] for p in self.pose_history]
            self.ax_data.plot(x_coords, y_coords, 'g-', alpha=0.7, label='轨迹')
        
        self.ax_data.legend()
        self.ax_data.set_title("系统数据")
    
    def update_all(self, robot_pose: Pose, scan: Scan, 
                  occupancy_map: Optional[OccupancyGridMap] = None,
                  path: Optional[List[Tuple[float, float]]] = None,
                  obstacles: Optional[List[Tuple[float, float]]] = None,
                  data: Optional[Dict[str, Any]] = None):
        """更新所有面板"""
        # 更新地图面板
        self.update_map_panel(robot_pose, scan, path, obstacles)
        
        # 更新占用栅格地图面板
        if occupancy_map is not None:
            self.update_ogm_panel(occupancy_map, robot_pose)
        
        # 更新路径规划面板
        if path is not None:
            self.update_path_panel(path, obstacles or [], robot_pose)
        
        # 更新数据面板
        if data is not None:
            self.update_data_panel(data)
        
        # 刷新显示
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
    
    def save_dashboard(self, filename: str, dpi: int = 300):
        """保存仪表板"""
        self.fig.savefig(filename, dpi=dpi, bbox_inches='tight')
        logger.info(f"仪表板已保存: {filename}")
    
    def show(self, block: bool = False):
        """显示仪表板"""
        plt.show(block=block)
    
    def close(self):
        """关闭仪表板"""
        plt.close(self.fig)
        logger.info("仪表板已关闭")
    
    def update_slam_visualization(self, occupancy_map, robot_pose, scan_data=None):
        """
        更新SLAM地图可视化
        
        Args:
            occupancy_map: 占用栅格地图
            robot_pose: 机器人位姿
            scan_data: 激光雷达扫描数据
        """
        # 清除当前地图
        self.ax_map.clear()
        
        # 绘制占用栅格地图
        if occupancy_map and hasattr(occupancy_map, 'grid'):
            # 显示占用栅格地图
            grid = occupancy_map.grid
            extent = [occupancy_map.min_x, occupancy_map.max_x, 
                     occupancy_map.min_y, occupancy_map.max_y]
            
            # 使用imshow显示地图
            im = self.ax_map.imshow(grid, extent=extent, origin='lower', 
                                  cmap='gray_r', alpha=0.8)
        
        # 绘制机器人位置
        if robot_pose:
            robot_x, robot_y = robot_pose.x, robot_pose.y
            robot_theta = robot_pose.theta
            
            # 绘制机器人位置
            self.ax_map.scatter(robot_x, robot_y, c='red', s=100, 
                               marker='o', label='机器人位置')
            
            # 绘制机器人方向
            arrow_length = 0.3
            dx = arrow_length * np.cos(robot_theta)
            dy = arrow_length * np.sin(robot_theta)
            self.ax_map.arrow(robot_x, robot_y, dx, dy, 
                             head_width=0.1, head_length=0.1, 
                             fc='red', ec='red')
        
        # 绘制激光雷达扫描数据
        if scan_data and hasattr(scan_data, 'angles') and hasattr(scan_data, 'ranges'):
            scan_points = []
            for angle, distance in zip(scan_data.angles, scan_data.ranges):
                if distance > 0:
                    x = robot_x + distance * np.cos(angle + robot_theta)
                    y = robot_y + distance * np.sin(angle + robot_theta)
                    scan_points.append([x, y])
            
            if scan_points:
                scan_points = np.array(scan_points)
                self.ax_map.scatter(scan_points[:, 0], scan_points[:, 1], 
                                 c='blue', s=10, alpha=0.6, label='激光雷达点')
        
        # 设置地图显示
        self.ax_map.set_xlabel('X坐标 (m)')
        self.ax_map.set_ylabel('Y坐标 (m)')
        self.ax_map.set_title('SLAM地图')
        self.ax_map.legend()
        self.ax_map.grid(True, alpha=0.3)
        self.ax_map.set_aspect('equal')
    
    def update_path_visualization(self, path, obstacles=None, start=None, goal=None):
        """
        更新路径规划可视化
        
        Args:
            path: 规划路径点列表
            obstacles: 障碍物列表
            start: 起点
            goal: 终点
        """
        if self.ax_path is None:
            return
            
        # 清除当前路径
        self.ax_path.clear()
        
        # 绘制障碍物
        if obstacles:
            for obs in obstacles:
                circle = plt.Circle(obs, 0.2, color='red', alpha=0.7)
                self.ax_path.add_patch(circle)
        
        # 绘制起点
        if start:
            self.ax_path.scatter(start[0], start[1], c='green', s=100, 
                               marker='o', label='起点')
        
        # 绘制终点
        if goal:
            self.ax_path.scatter(goal[0], goal[1], c='red', s=100, 
                               marker='s', label='终点')
        
        # 绘制规划路径
        if path and len(path) > 1:
            path_x = [p[0] for p in path]
            path_y = [p[1] for p in path]
            self.ax_path.plot(path_x, path_y, 'b-', linewidth=2, 
                            marker='o', markersize=4, label='规划路径')
        
        # 设置路径显示
        self.ax_path.set_xlabel('X坐标 (m)')
        self.ax_path.set_ylabel('Y坐标 (m)')
        self.ax_path.set_title('路径规划')
        self.ax_path.legend()
        self.ax_path.grid(True, alpha=0.3)
        self.ax_path.set_aspect('equal')
    
    def show_visualization(self):
        """显示可视化界面"""
        plt.tight_layout()
        plt.show(block=False)
        plt.pause(0.1)  # 短暂暂停以更新显示
