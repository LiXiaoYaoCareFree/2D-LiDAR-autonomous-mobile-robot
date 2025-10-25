"""
位姿融合模块
结合里程计位姿和 ICP 位姿估计，提供更鲁棒的定位
"""

import math
import logging
from dataclasses import dataclass
from typing import Tuple, Optional

from .icp_localization import ICPResult
from ..sensors.data_structures import Pose

logger = logging.getLogger(__name__)


@dataclass
class FusionConfig:
    """位姿融合配置参数"""
    
    # 是否启用融合
    enabled: bool = True
    
    # 融合权重 (0-1, 越大越信任 ICP)
    alpha: float = 0.1
    
    # ICP 门控阈值
    max_rmse_m: float = 0.05           # 最大均方根误差 (米)
    min_correspondences: int = 50       # 最少对应点数
    max_translation_m: float = 0.20     # 最大平移 (米)
    max_rotation_deg: float = 20.0      # 最大旋转 (度)
    
    # 快速对齐阈值 (小于此值直接采用 ICP)
    snap_translation_m: float = 0.02    # 平移对齐阈值 (米)
    snap_rotation_deg: float = 2.0      # 旋转对齐阈值 (度)


class PoseFusion:
    """位姿融合器"""
    
    def __init__(self, config: Optional[FusionConfig] = None):
        """
        初始化位姿融合器
        
        Args:
            config: 融合配置，如果为 None 则使用默认配置
        """
        self.config = config if config is not None else FusionConfig()
        
        # 统计信息
        self._total_fusions = 0
        self._accepted_fusions = 0
        self._rejected_fusions = 0
        self._snap_fusions = 0
    
    def normalize_angle(self, angle: float) -> float:
        """
        将角度归一化到 [-π, π]
        
        Args:
            angle: 输入角度 (弧度)
            
        Returns:
            归一化后的角度 (弧度)
        """
        return (angle + math.pi) % (2 * math.pi) - math.pi
    
    def angle_difference(self, angle1: float, angle2: float) -> float:
        """
        计算两个角度之间的差值（最短角度）
        
        Args:
            angle1: 角度1 (弧度)
            angle2: 角度2 (弧度)
            
        Returns:
            角度差 (弧度, 范围 [-π, π])
        """
        diff = angle1 - angle2
        return self.normalize_angle(diff)
    
    def check_icp_validity(self, icp_result: ICPResult, 
                          odometry_pose: Pose, 
                          prev_pose: Pose) -> Tuple[bool, str]:
        """
        检查 ICP 结果是否有效（门控检查）
        
        Args:
            icp_result: ICP 匹配结果
            odometry_pose: 里程计位姿
            prev_pose: 上一帧位姿
            
        Returns:
            (是否有效, 拒绝原因)
        """
        # 检查 ICP 是否成功
        if not icp_result.success:
            return False, "ICP 匹配失败"
        
        # 检查对应点数
        if icp_result.num_correspondences < self.config.min_correspondences:
            return False, f"对应点数不足 ({icp_result.num_correspondences} < {self.config.min_correspondences})"
        
        # 检查 RMSE
        if icp_result.rmse > self.config.max_rmse_m:
            return False, f"RMSE 过大 ({icp_result.rmse:.4f} > {self.config.max_rmse_m})"
        
        # 计算 ICP 估计的位姿
        icp_x = prev_pose.x + icp_result.delta_x
        icp_y = prev_pose.y + icp_result.delta_y
        icp_theta = prev_pose.theta + icp_result.delta_yaw
        
        # 检查与里程计的差异
        translation_diff = math.sqrt(
            (icp_x - odometry_pose.x) ** 2 + 
            (icp_y - odometry_pose.y) ** 2
        )
        
        if translation_diff > self.config.max_translation_m:
            return False, f"平移差异过大 ({translation_diff:.4f} > {self.config.max_translation_m})"
        
        rotation_diff_rad = abs(self.angle_difference(icp_theta, odometry_pose.theta))
        rotation_diff_deg = math.degrees(rotation_diff_rad)
        
        if rotation_diff_deg > self.config.max_rotation_deg:
            return False, f"旋转差异过大 ({rotation_diff_deg:.2f}° > {self.config.max_rotation_deg}°)"
        
        return True, "通过"
    
    def fuse_poses(self, 
                   odometry_pose: Pose,
                   icp_result: ICPResult,
                   prev_pose: Pose) -> Tuple[Optional[Pose], dict]:
        """
        融合里程计位姿和 ICP 估计
        
        Args:
            odometry_pose: 当前里程计位姿
            icp_result: ICP 匹配结果
            prev_pose: 上一帧位姿
            
        Returns:
            (融合后的位姿, 融合信息字典)
        """
        self._total_fusions += 1
        
        info = {
            'fusion_applied': False,
            'fusion_type': 'none',
            'reason': ''
        }
        
        # 如果未启用融合，直接返回里程计位姿
        if not self.config.enabled:
            info['reason'] = '融合未启用'
            return None, info
        
        # 检查 ICP 有效性
        is_valid, reason = self.check_icp_validity(icp_result, odometry_pose, prev_pose)
        
        if not is_valid:
            self._rejected_fusions += 1
            info['reason'] = reason
            logger.debug(f"ICP 融合被拒绝: {reason}")
            return None, info
        
        # 计算 ICP 估计的位姿
        icp_x = prev_pose.x + icp_result.delta_x
        icp_y = prev_pose.y + icp_result.delta_y
        icp_theta = self.normalize_angle(prev_pose.theta + icp_result.delta_yaw)
        
        # 计算与里程计的差异
        translation_diff = math.sqrt(
            (icp_x - odometry_pose.x) ** 2 + 
            (icp_y - odometry_pose.y) ** 2
        )
        rotation_diff_rad = abs(self.angle_difference(icp_theta, odometry_pose.theta))
        rotation_diff_deg = math.degrees(rotation_diff_rad)
        
        # 快速对齐模式：差异很小，直接采用 ICP
        if (translation_diff < self.config.snap_translation_m and 
            rotation_diff_deg < self.config.snap_rotation_deg):
            
            self._snap_fusions += 1
            self._accepted_fusions += 1
            
            fused_pose = Pose(icp_x, icp_y, icp_theta)
            
            info['fusion_applied'] = True
            info['fusion_type'] = 'snap'
            info['reason'] = f'快速对齐 (Δd={translation_diff:.4f}m, Δθ={rotation_diff_deg:.2f}°)'
            info['icp_pose'] = (icp_x, icp_y, icp_theta)
            info['translation_diff'] = translation_diff
            info['rotation_diff_deg'] = rotation_diff_deg
            
            logger.debug(f"ICP 快速对齐: {info['reason']}")
            
            return fused_pose, info
        
        # 加权融合模式
        alpha = self.config.alpha
        
        # 线性插值位置
        fused_x = (1 - alpha) * odometry_pose.x + alpha * icp_x
        fused_y = (1 - alpha) * odometry_pose.y + alpha * icp_y
        
        # 角度插值（使用最短路径）
        angle_diff = self.angle_difference(icp_theta, odometry_pose.theta)
        fused_theta = self.normalize_angle(odometry_pose.theta + alpha * angle_diff)
        
        self._accepted_fusions += 1
        
        fused_pose = Pose(fused_x, fused_y, fused_theta)
        
        info['fusion_applied'] = True
        info['fusion_type'] = 'weighted'
        info['reason'] = f'加权融合 (α={alpha}, Δd={translation_diff:.4f}m, Δθ={rotation_diff_deg:.2f}°)'
        info['icp_pose'] = (icp_x, icp_y, icp_theta)
        info['translation_diff'] = translation_diff
        info['rotation_diff_deg'] = rotation_diff_deg
        info['alpha'] = alpha
        
        logger.debug(f"ICP 加权融合: {info['reason']}")
        
        return fused_pose, info
    
    def get_statistics(self) -> dict:
        """
        获取融合统计信息
        
        Returns:
            统计信息字典
        """
        if self._total_fusions == 0:
            acceptance_rate = 0.0
            snap_rate = 0.0
        else:
            acceptance_rate = self._accepted_fusions / self._total_fusions
            snap_rate = self._snap_fusions / self._total_fusions
        
        return {
            'total_fusions': self._total_fusions,
            'accepted_fusions': self._accepted_fusions,
            'rejected_fusions': self._rejected_fusions,
            'snap_fusions': self._snap_fusions,
            'acceptance_rate': acceptance_rate,
            'snap_rate': snap_rate
        }
    
    def reset_statistics(self):
        """重置统计信息"""
        self._total_fusions = 0
        self._accepted_fusions = 0
        self._rejected_fusions = 0
        self._snap_fusions = 0

