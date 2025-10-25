"""
速度补偿器模块
使用 EWMA 算法自适应补偿轮速偏差
"""

import logging
from typing import Tuple

logger = logging.getLogger(__name__)


class SpeedCompensator:
    """
    速度自适应补偿器
    
    通过反馈速度和命令速度的比较，自适应调整轮速增益，
    补偿电机特性差异和摩擦力不对称等因素。
    """
    
    def __init__(self,
                 alpha: float = 0.2,
                 min_gain: float = 0.80,
                 max_gain: float = 1.50,
                 update_threshold: float = 0.08):
        """
        初始化速度补偿器
        
        Args:
            alpha: EWMA 融合系数 (0-1)，越大越快适应
            min_gain: 最小增益限制
            max_gain: 最大增益限制
            update_threshold: 更新门槛（命令速度绝对值需大于此值）
        """
        self.alpha = alpha
        self.min_gain = min_gain
        self.max_gain = max_gain
        self.update_threshold = update_threshold
        
        # 左右轮增益
        self.gain_left = 1.0
        self.gain_right = 1.0
        
        # 上次下发的命令（用于快速反馈）
        self._last_cmd_left = 0.0
        self._last_cmd_right = 0.0
        
        # 统计信息
        self._update_count = 0
        self._total_adjustments = 0
        
        logger.info(
            f"速度补偿器已初始化: α={alpha}, "
            f"增益范围=[{min_gain}, {max_gain}], "
            f"更新门槛={update_threshold} m/s"
        )
    
    def apply(self, v_left_cmd: float, v_right_cmd: float) -> Tuple[float, float]:
        """
        应用增益补偿到命令速度
        
        Args:
            v_left_cmd: 左轮命令速度 (m/s)
            v_right_cmd: 右轮命令速度 (m/s)
            
        Returns:
            (v_left_compensated, v_right_compensated) 补偿后的速度 (m/s)
        """
        # 保存命令（用于快速反馈）
        self._last_cmd_left = float(v_left_cmd)
        self._last_cmd_right = float(v_right_cmd)
        
        # 应用增益
        v_left_comp = v_left_cmd * self.gain_left
        v_right_comp = v_right_cmd * self.gain_right
        
        return v_left_comp, v_right_comp
    
    def update(self, 
               v_left_cmd_applied: float, 
               v_right_cmd_applied: float,
               v_left_feedback: float, 
               v_right_feedback: float):
        """
        根据反馈速度更新增益
        
        Args:
            v_left_cmd_applied: 实际下发的左轮命令速度（已应用增益）
            v_right_cmd_applied: 实际下发的右轮命令速度（已应用增益）
            v_left_feedback: 左轮反馈速度 (m/s)
            v_right_feedback: 右轮反馈速度 (m/s)
        """
        self._update_count += 1
        
        eps = 1e-3
        
        # 更新左轮增益
        if abs(v_left_cmd_applied) > self.update_threshold and abs(v_left_feedback) > eps:
            # 计算目标增益：命令速度 / 反馈速度
            target_gain_left = abs(v_left_cmd_applied) / max(eps, abs(v_left_feedback))
            
            # EWMA 融合
            self.gain_left = self._blend_gain(self.gain_left, target_gain_left)
            
            self._total_adjustments += 1
        
        # 更新右轮增益
        if abs(v_right_cmd_applied) > self.update_threshold and abs(v_right_feedback) > eps:
            target_gain_right = abs(v_right_cmd_applied) / max(eps, abs(v_right_feedback))
            
            self.gain_right = self._blend_gain(self.gain_right, target_gain_right)
            
            self._total_adjustments += 1
        
        # 定期日志
        if self._update_count % 50 == 0:
            logger.debug(
                f"速度补偿器状态: gain_L={self.gain_left:.3f}, "
                f"gain_R={self.gain_right:.3f}, "
                f"调整次数={self._total_adjustments}"
            )
    
    def feed_fast(self, v_left_feedback: float, v_right_feedback: float):
        """
        快速反馈入口（高频更新）
        
        使用最近一次下发的命令和快速反馈速度更新增益
        
        Args:
            v_left_feedback: 左轮反馈速度 (m/s)
            v_right_feedback: 右轮反馈速度 (m/s)
        """
        # 使用最近一次命令（已应用增益）
        v_left_applied = self._last_cmd_left * self.gain_left
        v_right_applied = self._last_cmd_right * self.gain_right
        
        self.update(v_left_applied, v_right_applied, v_left_feedback, v_right_feedback)
    
    def _blend_gain(self, current_gain: float, target_gain: float) -> float:
        """
        使用 EWMA 融合增益
        
        Args:
            current_gain: 当前增益
            target_gain: 目标增益
            
        Returns:
            融合后的增益
        """
        # EWMA: new = (1-α) * old + α * target
        blended = (1.0 - self.alpha) * current_gain + self.alpha * target_gain
        
        # 限幅
        blended = max(self.min_gain, min(self.max_gain, blended))
        
        return blended
    
    def get_gains(self) -> Tuple[float, float]:
        """
        获取当前增益
        
        Returns:
            (gain_left, gain_right)
        """
        return self.gain_left, self.gain_right
    
    def set_gains(self, gain_left: float, gain_right: float):
        """
        手动设置增益（用于初始化或校准）
        
        Args:
            gain_left: 左轮增益
            gain_right: 右轮增益
        """
        self.gain_left = max(self.min_gain, min(self.max_gain, gain_left))
        self.gain_right = max(self.min_gain, min(self.max_gain, gain_right))
        
        logger.info(f"增益已手动设置: L={self.gain_left:.3f}, R={self.gain_right:.3f}")
    
    def reset(self):
        """重置补偿器状态"""
        self.gain_left = 1.0
        self.gain_right = 1.0
        self._last_cmd_left = 0.0
        self._last_cmd_right = 0.0
        self._update_count = 0
        self._total_adjustments = 0
        
        logger.info("速度补偿器已重置")
    
    def get_statistics(self) -> dict:
        """
        获取统计信息
        
        Returns:
            统计信息字典
        """
        return {
            'gain_left': self.gain_left,
            'gain_right': self.gain_right,
            'update_count': self._update_count,
            'total_adjustments': self._total_adjustments,
            'gain_imbalance': abs(self.gain_left - self.gain_right)
        }

