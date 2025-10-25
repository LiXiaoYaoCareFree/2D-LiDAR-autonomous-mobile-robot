"""
地图可视化模块
负责地图、路径、机器人状态的显示
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
from typing import List, Tuple, Optional, Dict, Any
import logging

from ..sensors.data_structures import Pose, Scan
from ..mapping.occupancy_grid import OccupancyGridMap

logger = logging.getLogger(__name__)


class MapVisualizer:
    """地图可视化器"""
    
    def __init__(self, figsize: Tuple[float, float] = (12, 10), 
                 dpi: int = 100):
        """
        初始化地图可视化器
        
        Args:
            figsize: 图像尺寸
            dpi: 分辨率
        """
        self.figsize = figsize
        self.dpi = dpi
        
        # 创建图形和轴
        self.fig, self.ax = plt.subplots(figsize=figsize, dpi=dpi)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        
        # 可视化元素
        self.robot_arrow = None
        self.path_line = None
        self.scan_points = None
        self.obstacle_points = None
        
        logger.info("地图可视化器已初始化")
    
    def plot_occupancy_map(self, occupancy_map: OccupancyGridMap, 
                          title: str = "占用栅格地图"):
        """
        绘制占用栅格地图
        
        Args:
            occupancy_map: 占用栅格地图
            title: 图像标题
        """
        # 获取地图数据
        occ_map = occupancy_map.get_occupancy_map()
        bounds = occupancy_map.get_map_bounds()
        
        # 显示地图
        extent = [bounds[0], bounds[2], bounds[1], bounds[3]]
        im = self.ax.imshow(occ_map, extent=extent, origin='lower', 
                           cmap='gray', vmin=0, vmax=1)
        
        # 设置标题和标签
        self.ax.set_title(title)
        self.ax.set_xlabel('X (米)')
        self.ax.set_ylabel('Y (米)')
        
        # 添加颜色条
        plt.colorbar(im, ax=self.ax, label='占用概率')
        
        logger.debug("占用栅格地图已绘制")
    
    def plot_robot_pose(self, pose: Pose, color: str = 'blue', 
                       size: float = 0.1, label: str = "机器人"):
        """
        绘制机器人位姿
        
        Args:
            pose: 机器人位姿
            color: 颜色
            size: 箭头大小
            label: 标签
        """
        # 移除旧的机器人箭头
        if self.robot_arrow is not None:
            self.robot_arrow.remove()
        
        # 绘制机器人位置
        self.ax.plot(pose.x, pose.y, 'o', color=color, markersize=8, label=label)
        
        # 绘制机器人方向箭头
        arrow_length = size
        dx = arrow_length * np.cos(pose.theta)
        dy = arrow_length * np.sin(pose.theta)
        
        self.robot_arrow = self.ax.arrow(pose.x, pose.y, dx, dy,
                                       head_width=size*0.3, head_length=size*0.2,
                                       fc=color, ec=color, alpha=0.8)
        
        logger.debug(f"机器人位姿已绘制: ({pose.x:.2f}, {pose.y:.2f}, {pose.theta:.2f})")
    
    def plot_path(self, path: List[Tuple[float, float]], 
                 color: str = 'green', linewidth: float = 2,
                 label: str = "路径"):
        """
        绘制路径
        
        Args:
            path: 路径点列表
            color: 颜色
            linewidth: 线宽
            label: 标签
        """
        if not path:
            return
        
        # 移除旧路径
        if self.path_line is not None:
            self.path_line.remove()
        
        # 绘制路径
        x_coords = [point[0] for point in path]
        y_coords = [point[1] for point in path]
        
        self.path_line, = self.ax.plot(x_coords, y_coords, 
                                     color=color, linewidth=linewidth,
                                     linestyle='--', alpha=0.8, label=label)
        
        # 标记起点和终点
        if len(path) > 0:
            self.ax.plot(path[0][0], path[0][1], 'go', markersize=8, label='起点')
        if len(path) > 1:
            self.ax.plot(path[-1][0], path[-1][1], 'ro', markersize=8, label='终点')
        
        logger.debug(f"路径已绘制: {len(path)} 个点")
    
    def plot_scan_data(self, scan: Scan, robot_pose: Pose,
                      color: str = 'red', alpha: float = 0.6,
                      label: str = "激光雷达"):
        """
        绘制激光雷达扫描数据
        
        Args:
            scan: 扫描数据
            robot_pose: 机器人位姿
            color: 颜色
            alpha: 透明度
            label: 标签
        """
        # 移除旧扫描点
        if self.scan_points is not None:
            for point in self.scan_points:
                point.remove()
            self.scan_points = None
        
        if not scan.angles or not scan.ranges:
            return
        
        # 转换扫描数据到世界坐标系
        world_points = []
        for angle, range_val in zip(scan.angles, scan.ranges):
            if range_val > 0:
                # 局部坐标
                local_x = range_val * np.cos(angle)
                local_y = range_val * np.sin(angle)
                
                # 转换到世界坐标
                cos_theta = np.cos(robot_pose.theta)
                sin_theta = np.sin(robot_pose.theta)
                
                world_x = robot_pose.x + local_x * cos_theta - local_y * sin_theta
                world_y = robot_pose.y + local_x * sin_theta + local_y * cos_theta
                
                world_points.append((world_x, world_y))
        
        if world_points:
            # 绘制扫描点
            x_coords = [point[0] for point in world_points]
            y_coords = [point[1] for point in world_points]
            
            self.scan_points = self.ax.scatter(x_coords, y_coords, 
                                            c=color, s=1, alpha=alpha, label=label)
        
        logger.debug(f"激光雷达数据已绘制: {len(world_points)} 个点")
    
    def plot_obstacles(self, obstacles: List[Tuple[float, float]],
                     color: str = 'black', size: float = 0.05,
                     label: str = "障碍物"):
        """
        绘制障碍物
        
        Args:
            obstacles: 障碍物点列表
            color: 颜色
            size: 点大小
            label: 标签
        """
        # 移除旧障碍物
        if self.obstacle_points is not None:
            self.obstacle_points.remove()
        
        if obstacles:
            x_coords = [obs[0] for obs in obstacles]
            y_coords = [obs[1] for obs in obstacles]
            
            self.obstacle_points = self.ax.scatter(x_coords, y_coords,
                                                c=color, s=size*100, 
                                                alpha=0.7, label=label)
        
        logger.debug(f"障碍物已绘制: {len(obstacles)} 个点")
    
    def plot_goals(self, goals: List[Tuple[float, float]],
                  color: str = 'orange', size: float = 0.1,
                  label: str = "目标"):
        """
        绘制目标点
        
        Args:
            goals: 目标点列表
            color: 颜色
            size: 点大小
            label: 标签
        """
        if goals:
            x_coords = [goal[0] for goal in goals]
            y_coords = [goal[1] for goal in goals]
            
            self.ax.scatter(x_coords, y_coords, c=color, s=size*100,
                          marker='*', alpha=0.8, label=label)
        
        logger.debug(f"目标点已绘制: {len(goals)} 个点")
    
    def set_limits(self, xlim: Tuple[float, float], ylim: Tuple[float, float]):
        """设置坐标轴范围"""
        self.ax.set_xlim(xlim)
        self.ax.set_ylim(ylim)
        logger.debug(f"坐标轴范围已设置: x={xlim}, y={ylim}")
    
    def add_legend(self):
        """添加图例"""
        self.ax.legend(loc='upper right')
    
    def clear(self):
        """清空图像"""
        self.ax.clear()
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        
        # 重置可视化元素
        self.robot_arrow = None
        self.path_line = None
        self.scan_points = None
        self.obstacle_points = None
        
        logger.debug("图像已清空")
    
    def save_figure(self, filename: str, dpi: int = 300):
        """保存图像"""
        self.fig.savefig(filename, dpi=dpi, bbox_inches='tight')
        logger.info(f"图像已保存: {filename}")
    
    def show(self, block: bool = False):
        """显示图像"""
        plt.show(block=block)
    
    def update_display(self):
        """更新显示"""
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
    
    def close(self):
        """关闭图像"""
        plt.close(self.fig)
        logger.info("地图可视化器已关闭")
