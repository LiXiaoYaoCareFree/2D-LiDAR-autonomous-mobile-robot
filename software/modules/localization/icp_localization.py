"""
ICP (Iterative Closest Point) 定位模块
用于 scan-to-scan 匹配和位姿估计
"""

import numpy as np
import math
import logging
from dataclasses import dataclass
from typing import Tuple, Optional, List

from ..communication.ldr3_protocol import ProtocolConstants

logger = logging.getLogger(__name__)


@dataclass
class ICPResult:
    """ICP 匹配结果"""
    success: bool                    # 是否成功
    delta_x: float                   # x 方向位移 (米)
    delta_y: float                   # y 方向位移 (米)
    delta_yaw: float                 # 航向角变化 (弧度)
    rmse: float                      # 均方根误差 (米)
    num_correspondences: int         # 对应点对数量
    iterations: int                  # 迭代次数
    
    def get_transform_matrix(self) -> np.ndarray:
        """获取 3x3 变换矩阵"""
        cos_yaw = math.cos(self.delta_yaw)
        sin_yaw = math.sin(self.delta_yaw)
        
        T = np.eye(3)
        T[0:2, 0:2] = [[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]]
        T[0, 2] = self.delta_x
        T[1, 2] = self.delta_y
        
        return T


class ICPLocalizer:
    """ICP 定位器"""
    
    def __init__(self,
                 max_iterations: int = 30,
                 convergence_threshold: float = 1e-5,
                 max_correspondence_distance: float = 0.5,
                 min_points: int = 10):
        """
        初始化 ICP 定位器
        
        Args:
            max_iterations: 最大迭代次数
            convergence_threshold: 收敛阈值 (米)
            max_correspondence_distance: 最大对应距离 (米)
            min_points: 最少点数要求
        """
        self.max_iterations = max_iterations
        self.convergence_threshold = convergence_threshold
        self.max_correspondence_distance = max_correspondence_distance
        self.min_points = min_points
        
        # 雷达偏心补偿参数
        self.lidar_offset_x = ProtocolConstants.LIDAR_OFFSET_X
        self.lidar_offset_y = ProtocolConstants.LIDAR_OFFSET_Y
    
    def scan_to_points(self, angles: List[float], ranges: List[float],
                      robot_x: float, robot_y: float, robot_theta: float,
                      max_range: float = 3.0) -> np.ndarray:
        """
        将激光扫描转换为世界坐标系点云（应用雷达偏心补偿）
        
        Args:
            angles: 角度列表 (弧度)
            ranges: 距离列表 (米)
            robot_x: 机器人 x 坐标 (米)
            robot_y: 机器人 y 坐标 (米)
            robot_theta: 机器人航向角 (弧度)
            max_range: 最大有效距离 (米)
            
        Returns:
            Nx2 点云数组
        """
        if len(angles) == 0 or len(ranges) == 0:
            return np.zeros((0, 2), dtype=float)
        
        cos_theta = math.cos(robot_theta)
        sin_theta = math.sin(robot_theta)
        
        # 雷达在机器人坐标系中的位置
        lidar_x_robot = self.lidar_offset_x
        lidar_y_robot = self.lidar_offset_y
        
        # 雷达在世界坐标系中的位置
        lidar_x_world = robot_x + cos_theta * lidar_x_robot - sin_theta * lidar_y_robot
        lidar_y_world = robot_y + sin_theta * lidar_x_robot + cos_theta * lidar_y_robot
        
        points = []
        for angle, distance in zip(angles, ranges):
            # 过滤无效点
            if distance <= 0.0 or not math.isfinite(distance) or distance >= max_range - 1e-6:
                continue
            
            # 扫描点相对雷达的位置
            dx_sensor = distance * math.cos(angle)
            dy_sensor = distance * math.sin(angle)
            
            # 旋转到世界坐标系
            dx_world = cos_theta * dx_sensor - sin_theta * dy_sensor
            dy_world = sin_theta * dx_sensor + cos_theta * dy_sensor
            
            # 最终世界坐标
            x_world = lidar_x_world + dx_world
            y_world = lidar_y_world + dy_world
            
            points.append([x_world, y_world])
        
        if not points:
            return np.zeros((0, 2), dtype=float)
        
        return np.array(points, dtype=float)
    
    def find_nearest_neighbors(self, source_points: np.ndarray, 
                               target_points: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        为源点云中的每个点找到目标点云中的最近邻
        
        Args:
            source_points: Nx2 源点云
            target_points: Mx2 目标点云
            
        Returns:
            (对应的目标点, 对应的距离)
        """
        if source_points.shape[0] == 0 or target_points.shape[0] == 0:
            return np.zeros((0, 2)), np.zeros(0)
        
        # 计算距离矩阵 (N x M)
        # 使用广播避免循环
        diff = source_points[:, np.newaxis, :] - target_points[np.newaxis, :, :]
        distances_sq = np.sum(diff ** 2, axis=2)
        
        # 找到最近邻索引
        nearest_indices = np.argmin(distances_sq, axis=1)
        nearest_distances = np.sqrt(distances_sq[np.arange(len(source_points)), nearest_indices])
        
        # 对应的目标点
        corresponding_points = target_points[nearest_indices]
        
        return corresponding_points, nearest_distances
    
    def compute_transformation(self, source_points: np.ndarray,
                              target_points: np.ndarray) -> Tuple[float, float, float]:
        """
        计算从源点云到目标点云的刚体变换 (使用 SVD)
        
        Args:
            source_points: Nx2 源点云
            target_points: Nx2 目标点云 (对应点)
            
        Returns:
            (delta_yaw, delta_x, delta_y)
        """
        # 计算质心
        source_centroid = np.mean(source_points, axis=0)
        target_centroid = np.mean(target_points, axis=0)
        
        # 去中心化
        source_centered = source_points - source_centroid
        target_centered = target_points - target_centroid
        
        # 计算协方差矩阵 H = source^T * target
        H = source_centered.T @ target_centered
        
        # SVD 分解
        U, S, Vt = np.linalg.svd(H)
        
        # 计算旋转矩阵
        R = Vt.T @ U.T
        
        # 处理反射情况 (确保 det(R) = 1)
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T
        
        # 提取旋转角
        delta_yaw = math.atan2(R[1, 0], R[0, 0])
        
        # 计算平移
        translation = target_centroid - R @ source_centroid
        delta_x = translation[0]
        delta_y = translation[1]
        
        return delta_yaw, delta_x, delta_y
    
    def apply_transform(self, points: np.ndarray, 
                       delta_yaw: float, delta_x: float, delta_y: float) -> np.ndarray:
        """
        应用变换到点云
        
        Args:
            points: Nx2 点云
            delta_yaw: 旋转角 (弧度)
            delta_x: x 平移 (米)
            delta_y: y 平移 (米)
            
        Returns:
            变换后的点云
        """
        if points.shape[0] == 0:
            return points
        
        cos_yaw = math.cos(delta_yaw)
        sin_yaw = math.sin(delta_yaw)
        
        R = np.array([[cos_yaw, -sin_yaw],
                     [sin_yaw, cos_yaw]])
        t = np.array([delta_x, delta_y])
        
        transformed = (R @ points.T).T + t
        
        return transformed
    
    def compute_rmse(self, source_points: np.ndarray, 
                    target_points: np.ndarray) -> float:
        """
        计算对应点对的均方根误差
        
        Args:
            source_points: Nx2 源点云
            target_points: Nx2 目标点云 (对应点)
            
        Returns:
            RMSE (米)
        """
        if source_points.shape[0] == 0:
            return float('inf')
        
        squared_errors = np.sum((source_points - target_points) ** 2, axis=1)
        rmse = math.sqrt(np.mean(squared_errors))
        
        return rmse
    
    def match(self, source_points: np.ndarray, 
             target_points: np.ndarray) -> ICPResult:
        """
        执行 ICP 匹配
        
        Args:
            source_points: Nx2 源点云 (上一帧)
            target_points: Mx2 目标点云 (当前帧)
            
        Returns:
            ICP 匹配结果
        """
        # 检查输入
        if source_points.shape[0] < self.min_points or target_points.shape[0] < self.min_points:
            return ICPResult(
                success=False,
                delta_x=0.0, delta_y=0.0, delta_yaw=0.0,
                rmse=float('inf'),
                num_correspondences=0,
                iterations=0
            )
        
        # 初始化
        current_source = source_points.copy()
        best_delta_yaw = 0.0
        best_delta_x = 0.0
        best_delta_y = 0.0
        best_rmse = float('inf')
        
        # ICP 迭代
        for iteration in range(self.max_iterations):
            # 1. 找最近邻
            corresponding_targets, distances = self.find_nearest_neighbors(
                current_source, target_points
            )
            
            # 2. 过滤距离过大的对应点
            valid_mask = distances < self.max_correspondence_distance
            valid_source = current_source[valid_mask]
            valid_targets = corresponding_targets[valid_mask]
            
            if valid_source.shape[0] < self.min_points:
                # 有效对应点太少
                break
            
            # 3. 计算变换
            delta_yaw, delta_x, delta_y = self.compute_transformation(
                valid_source, valid_targets
            )
            
            # 4. 应用变换
            transformed_source = self.apply_transform(
                current_source, delta_yaw, delta_x, delta_y
            )
            
            # 5. 计算 RMSE
            rmse = self.compute_rmse(
                transformed_source[valid_mask], 
                valid_targets
            )
            
            # 6. 累积变换
            # 先旋转再平移
            cos_prev = math.cos(best_delta_yaw)
            sin_prev = math.sin(best_delta_yaw)
            
            dx_rotated = cos_prev * delta_x - sin_prev * delta_y
            dy_rotated = sin_prev * delta_x + cos_prev * delta_y
            
            best_delta_x += dx_rotated
            best_delta_y += dy_rotated
            best_delta_yaw += delta_yaw
            best_rmse = rmse
            
            # 7. 更新源点云
            current_source = transformed_source
            
            # 8. 检查收敛
            translation_change = math.sqrt(delta_x**2 + delta_y**2)
            if translation_change < self.convergence_threshold and abs(delta_yaw) < math.radians(0.5):
                logger.debug(f"ICP 收敛于第 {iteration + 1} 次迭代")
                break
        
        # 归一化角度
        best_delta_yaw = (best_delta_yaw + math.pi) % (2 * math.pi) - math.pi
        
        return ICPResult(
            success=True,
            delta_x=best_delta_x,
            delta_y=best_delta_y,
            delta_yaw=best_delta_yaw,
            rmse=best_rmse,
            num_correspondences=int(np.sum(valid_mask)),
            iterations=iteration + 1
        )

