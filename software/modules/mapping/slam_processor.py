"""
SLAM 处理器模块
整合 ICP 定位、位姿融合和 OGM 建图
"""

import numpy as np
import math
import sys
from pathlib import Path
import logging
from typing import Optional, Tuple, Dict, Any
from collections import deque

# 添加 env 到 Python 路径
_env_path = Path(__file__).parent.parent.parent / "env"
if str(_env_path) not in sys.path:
    sys.path.insert(0, str(_env_path))

# 从 env 中导入 PythonRobotics 的 ICP
from SLAM.ICPMatching.icp_matching import icp_matching as pr_icp_matching

from .occupancy_grid import OccupancyGridMap
from ..sensors.data_structures import Pose, Scan, SensorData
from ..localization import PoseFusion, FusionConfig, ICPResult
from ..communication.ldr3_protocol import ProtocolConstants

logger = logging.getLogger(__name__)


class SLAMProcessor:
    """SLAM 处理器"""
    
    def __init__(self, 
                 map_bounds: Tuple[float, float, float, float] = (-5, -5, 5, 5),
                 map_resolution: float = 0.03,
                 enable_icp: bool = True,
                 enable_fusion: bool = True):
        """
        初始化 SLAM 处理器
        
        Args:
            map_bounds: 地图边界 (minx, miny, maxx, maxy)
            map_resolution: 地图分辨率 (米/栅格)
            enable_icp: 是否启用 ICP 定位
            enable_fusion: 是否启用位姿融合
        """
        minx, miny, maxx, maxy = map_bounds
        
        # 初始化占用栅格地图
        self.occupancy_map = OccupancyGridMap(
            minx, miny, maxx, maxy,
            resolution=map_resolution,
            l_free=-0.4,
            l_occ=0.85,
            l_min=-4.0,
            l_max=4.0,
            hit_margin=0.001
        )
        
        # ICP 定位器（使用 PythonRobotics 实现）
        self.enable_icp = enable_icp
        self.min_icp_points = 50  # ICP 最少点数要求
        
        # 位姿融合器
        self.enable_fusion = enable_fusion
        if self.enable_fusion:
            fusion_config = FusionConfig(
                enabled=True,
                alpha=0.1,
                max_rmse_m=0.05,
                min_correspondences=50,
                max_translation_m=0.20,
                max_rotation_deg=20.0,
                snap_translation_m=0.02,
                snap_rotation_deg=2.0
            )
            self.pose_fusion = PoseFusion(fusion_config)
        else:
            self.pose_fusion = None
        
        # 位姿历史（用于 ICP）
        self.current_pose = Pose(0, 0, 0)
        self.pose_history = deque(maxlen=100)
        
        # 扫描历史（用于 ICP）
        self.prev_scan_angles = None
        self.prev_scan_ranges = None
        self.prev_pose = None
        
        # 统计信息
        self._update_count = 0
        self._icp_success_count = 0
        self._fusion_accept_count = 0
        
        logger.info(f"SLAM 处理器已初始化: ICP={enable_icp}, 融合={enable_fusion}")
    
    def update(self, sensor_data: SensorData) -> Dict[str, Any]:
        """
        更新 SLAM 状态
        
        Args:
            sensor_data: 传感器数据
            
        Returns:
            更新结果字典
        """
        self._update_count += 1
        
        result = {
            'update_id': self._update_count,
            'icp_result': None,
            'fusion_info': None,
            'pose_updated': False,
            'map_updated': False
        }
        
        if not sensor_data.is_valid():
            logger.warning("传感器数据无效")
            return result
        
        # 1. 获取里程计位姿
        odometry_pose = sensor_data.pose
        
        # 2. ICP 定位（如果启用且有历史数据）
        icp_result = None
        if self.enable_icp and self.prev_scan_angles is not None and self.prev_pose is not None:
            icp_result = self._perform_icp(
                sensor_data.scan.angles,
                sensor_data.scan.ranges,
                odometry_pose
            )
            result['icp_result'] = icp_result
            
            if icp_result and icp_result.success:
                self._icp_success_count += 1
        
        # 3. 位姿融合（如果启用且 ICP 成功）
        fused_pose = None
        fusion_info = None
        
        if (self.enable_fusion and 
            icp_result is not None and 
            self.prev_pose is not None):
            
            fused_pose, fusion_info = self.pose_fusion.fuse_poses(
                odometry_pose,
                icp_result,
                self.prev_pose
            )
            
            result['fusion_info'] = fusion_info
            
            if fused_pose is not None:
                self._fusion_accept_count += 1
        
        # 4. 确定最终使用的位姿
        final_pose = fused_pose if fused_pose is not None else odometry_pose
        
        self.current_pose = final_pose
        self.pose_history.append(final_pose)
        result['pose_updated'] = True
        result['final_pose'] = (final_pose.x, final_pose.y, final_pose.theta)
        
        # 5. 更新地图
        if sensor_data.scan and len(sensor_data.scan.angles) > 0:
            self.occupancy_map.integrate_scan(
                sensor_data.scan.angles,
                sensor_data.scan.ranges,
                (final_pose.x, final_pose.y, final_pose.theta)
            )
            result['map_updated'] = True
        
        # 6. 更新历史（为下一次 ICP 做准备）
        self.prev_scan_angles = sensor_data.scan.angles
        self.prev_scan_ranges = sensor_data.scan.ranges
        self.prev_pose = final_pose
        
        return result
    
    def _scan_to_points(self, angles: list, ranges: list,
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
            2xN 点云数组 (PythonRobotics ICP 格式)
        """
        if len(angles) == 0 or len(ranges) == 0:
            return np.zeros((2, 0), dtype=float)
        
        cos_theta = math.cos(robot_theta)
        sin_theta = math.sin(robot_theta)
        
        # 雷达偏心补偿
        lidar_offset_x = ProtocolConstants.LIDAR_OFFSET_X
        lidar_offset_y = ProtocolConstants.LIDAR_OFFSET_Y
        
        # 雷达在世界坐标系中的位置
        lidar_x_world = robot_x + cos_theta * lidar_offset_x - sin_theta * lidar_offset_y
        lidar_y_world = robot_y + sin_theta * lidar_offset_x + cos_theta * lidar_offset_y
        
        points_x = []
        points_y = []
        
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
            
            points_x.append(x_world)
            points_y.append(y_world)
        
        if not points_x:
            return np.zeros((2, 0), dtype=float)
        
        # PythonRobotics ICP 格式：2xN 数组
        return np.array([points_x, points_y], dtype=float)
    
    def _perform_icp(self, 
                    current_angles: list, 
                    current_ranges: list,
                    odometry_pose: Pose) -> Optional[ICPResult]:
        """
        执行 ICP 定位（使用 PythonRobotics 实现）
        
        Args:
            current_angles: 当前扫描角度
            current_ranges: 当前扫描距离
            odometry_pose: 当前里程计位姿
            
        Returns:
            ICP 结果，失败返回 None
        """
        try:
            # 转换上一帧扫描到点云 (2xN 格式)
            prev_points = self._scan_to_points(
                self.prev_scan_angles,
                self.prev_scan_ranges,
                self.prev_pose.x,
                self.prev_pose.y,
                self.prev_pose.theta
            )
            
            # 转换当前帧扫描到点云
            curr_points = self._scan_to_points(
                current_angles,
                current_ranges,
                odometry_pose.x,
                odometry_pose.y,
                odometry_pose.theta
            )
            
            # 检查点数
            if prev_points.shape[1] < self.min_icp_points or curr_points.shape[1] < self.min_icp_points:
                logger.debug(f"ICP 点数不足: prev={prev_points.shape[1]}, curr={curr_points.shape[1]}")
                return None
            
            # 关闭 PythonRobotics ICP 的动画显示
            import SLAM.ICPMatching.icp_matching as icp_module
            original_show = icp_module.show_animation
            icp_module.show_animation = False
            
            try:
                # 调用 PythonRobotics ICP
                # 返回: R (2x2 旋转矩阵), T (2x1 平移向量)
                R, T = pr_icp_matching(prev_points, curr_points)
            finally:
                # 恢复原设置
                icp_module.show_animation = original_show
            
            # 提取变换参数
            delta_yaw = math.atan2(R[1, 0], R[0, 0])
            delta_x = float(T[0])
            delta_y = float(T[1])
            
            # 计算 RMSE（用于门控检查）
            # 变换后的点云
            prev_transformed = (R @ prev_points) + T[:, np.newaxis]
            
            # 计算最近邻距离
            if curr_points.shape[1] > 0 and prev_transformed.shape[1] > 0:
                # 计算每个变换后的点到当前点云的最近距离
                diffs = prev_transformed[:, :, np.newaxis] - curr_points[:, np.newaxis, :]
                distances = np.sqrt(np.sum(diffs ** 2, axis=0))
                min_distances = np.min(distances, axis=1)
                rmse = float(np.sqrt(np.mean(min_distances ** 2)))
                num_correspondences = prev_transformed.shape[1]
            else:
                rmse = float('inf')
                num_correspondences = 0
            
            # 构建 ICPResult
            icp_result = ICPResult(
                success=True,
                delta_x=delta_x,
                delta_y=delta_y,
                delta_yaw=delta_yaw,
                rmse=rmse,
                num_correspondences=num_correspondences,
                iterations=0  # PythonRobotics ICP 不返回迭代次数
            )
            
            logger.debug(
                f"ICP 成功 (PythonRobotics): Δx={delta_x:.4f}m, "
                f"Δy={delta_y:.4f}m, "
                f"Δθ={math.degrees(delta_yaw):.2f}°, "
                f"RMSE={rmse:.4f}m, "
                f"对应点={num_correspondences}"
            )
            
            return icp_result
            
        except Exception as e:
            logger.error(f"ICP 执行错误: {e}")
            import traceback
            traceback.print_exc()
            return None
    
    def get_current_pose(self) -> Pose:
        """获取当前位姿"""
        return self.current_pose
    
    def get_occupancy_map(self) -> OccupancyGridMap:
        """获取占用栅格地图"""
        return self.occupancy_map
    
    def get_pose_history(self) -> list:
        """获取位姿历史"""
        return list(self.pose_history)
    
    def get_statistics(self) -> dict:
        """获取 SLAM 统计信息"""
        stats = {
            'total_updates': self._update_count,
            'icp_success_count': self._icp_success_count,
            'fusion_accept_count': self._fusion_accept_count
        }
        
        if self._update_count > 0:
            stats['icp_success_rate'] = self._icp_success_count / self._update_count
            stats['fusion_accept_rate'] = self._fusion_accept_count / self._update_count
        
        if self.pose_fusion:
            stats['fusion_stats'] = self.pose_fusion.get_statistics()
        
        return stats
    
    def reset(self):
        """重置 SLAM 状态"""
        # 重置地图
        bounds = self.occupancy_map.get_map_bounds()
        resolution = self.occupancy_map.resolution
        
        self.occupancy_map = OccupancyGridMap(
            bounds[0], bounds[1], bounds[2], bounds[3],
            resolution=resolution,
            l_free=-0.4,
            l_occ=0.85,
            l_min=-4.0,
            l_max=4.0,
            hit_margin=0.001
        )
        
        # 重置位姿
        self.current_pose = Pose(0, 0, 0)
        self.pose_history.clear()
        
        # 重置历史
        self.prev_scan_angles = None
        self.prev_scan_ranges = None
        self.prev_pose = None
        
        # 重置统计
        self._update_count = 0
        self._icp_success_count = 0
        self._fusion_accept_count = 0
        
        if self.pose_fusion:
            self.pose_fusion.reset_statistics()
        
        logger.info("SLAM 状态已重置")
