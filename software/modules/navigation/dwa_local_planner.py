"""
DWA (Dynamic Window Approach) 局部路径规划器
基于动态窗口法的局部避障和速度规划
"""

import math
import numpy as np
import logging
from typing import Tuple, List, Optional, Dict, Any
from dataclasses import dataclass

from ..sensors.data_structures import Pose, Scan
from ..communication.ldr3_protocol import ProtocolConstants

logger = logging.getLogger(__name__)


@dataclass
class DWAConfig:
    """DWA 配置参数"""
    
    # 速度限制
    v_max_mps: float = 0.30           # 最大线速度 (m/s)
    w_max_rps: float = 0.75           # 最大角速度 (rad/s)
    
    # 加速度限制
    a_max_mps2: float = 0.8           # 最大线加速度 (m/s²)
    aw_max_rps2: float = 5.0          # 最大角加速度 (rad/s²)
    
    # 采样分辨率
    v_resolution: float = 0.05        # 线速度采样分辨率 (m/s)
    w_resolution: float = 0.2         # 角速度采样分辨率 (rad/s)
    
    # 轨迹仿真参数
    sim_time_s: float = 1.0           # 前瞻时间 (秒)
    dt_s: float = 0.1                 # 仿真时间步长 (秒)
    
    # 代价函数权重
    weight_heading: float = 2.5       # 朝向目标权重
    weight_clearance: float = 1.0    # 障碍物清距权重
    weight_velocity: float = 0.5     # 速度权重
    
    # 机器人参数
    robot_radius_m: float = 0.1       # 机器人半径 (米)
    
    # 目标相关参数
    goal_reached_radius_m: float = 0.22   # 到达目标判定半径 (米)
    slow_radius_m: float = 0.90           # 减速半径 (米)
    near_goal_w_max_rps: float = 0.8      # 近目标最大角速度 (rad/s)
    near_goal_vel_w_scale: float = 1.6    # 近目标速度权重缩放
    near_goal_heading_w_scale: float = 0.7  # 近目标朝向权重缩放
    
    # 安全参数
    mcu_wheel_min_mps: float = 0.13       # MCU 轮速起步阈值 (m/s)
    front_safety_margin_m: float = 0.20   # 前向安全余量 (米)
    
    # 调试
    debug: bool = False


class DWALocalPlanner:
    """DWA 局部路径规划器"""
    
    def __init__(self, config: Optional[DWAConfig] = None, wheel_base_m: float = 0.165):
        """
        初始化 DWA 规划器
        
        Args:
            config: DWA 配置
            wheel_base_m: 轮距 (米)
        """
        self.config = config if config is not None else DWAConfig()
        self.wheel_base_m = wheel_base_m
        
        # 当前速度状态（用于动态窗口计算）
        self._current_v = 0.0
        self._current_w = 0.0
        
        # 目标点
        self._goal_x = 0.0
        self._goal_y = 0.0
        
        # 恢复状态
        self._recovery_mode = False
        self._recovery_ticks = 0
        
        # 雷达偏心
        self.lidar_offset_x = ProtocolConstants.LIDAR_OFFSET_X
        self.lidar_offset_y = ProtocolConstants.LIDAR_OFFSET_Y
        
        logger.info("DWA 局部规划器已初始化")
    
    def set_goal(self, goal_x: float, goal_y: float):
        """
        设置目标点
        
        Args:
            goal_x: 目标 x 坐标 (米)
            goal_y: 目标 y 坐标 (米)
        """
        self._goal_x = goal_x
        self._goal_y = goal_y
    
    def update_velocity(self, v: float, w: float):
        """
        更新当前速度状态
        
        Args:
            v: 线速度 (m/s)
            w: 角速度 (rad/s)
        """
        self._current_v = v
        self._current_w = w
    
    def compute(self, pose: Pose, scan: Scan) -> Tuple[float, float]:
        """
        计算控制命令（轮速）
        
        Args:
            pose: 当前位姿
            scan: 激光扫描数据
            
        Returns:
            (vL_mps, vR_mps) 左右轮速度 (m/s)
        """
        c = self.config
        
        # 计算到目标的距离
        dist_to_goal = math.hypot(self._goal_x - pose.x, self._goal_y - pose.y)
        
        # 到达目标：停止
        if dist_to_goal <= c.goal_reached_radius_m:
            if c.debug:
                logger.info(f"[DWA] 到达目标: d={dist_to_goal:.3f}m")
            return 0.0, 0.0
        
        # 近目标调整
        local_w_max = c.w_max_rps
        weight_heading = c.weight_heading
        weight_vel = c.weight_velocity
        
        if dist_to_goal <= c.slow_radius_m:
            local_w_max = min(local_w_max, c.near_goal_w_max_rps)
            weight_heading *= c.near_goal_heading_w_scale
            weight_vel *= c.near_goal_vel_w_scale
        
        # 计算动态窗口
        v_min = max(0.0, self._current_v - c.a_max_mps2 * c.dt_s)
        v_max = min(c.v_max_mps, self._current_v + c.a_max_mps2 * c.dt_s)
        w_min = max(-local_w_max, self._current_w - c.aw_max_rps2 * c.dt_s)
        w_max = min(+local_w_max, self._current_w + c.aw_max_rps2 * c.dt_s)
        
        # 快速前方/后方清距估计
        front_clear, rear_clear = self._estimate_clearance(scan)
        
        # E-brake：前方清距不足
        strict_margin = c.robot_radius_m + c.front_safety_margin_m
        if math.isfinite(front_clear) and front_clear < strict_margin:
            # 后退
            v_back = -max(c.mcu_wheel_min_mps, min(0.22, c.v_max_mps * 0.75))
            
            # 如果后方也很近，减小后退速度
            if math.isfinite(rear_clear) and rear_clear < (c.robot_radius_m + 0.12):
                v_back = -max(0.10, c.mcu_wheel_min_mps)
            
            vL = v_back
            vR = v_back
            
            if c.debug:
                logger.info(
                    f"[DWA] E-brake: front_clear={front_clear:.3f} < {strict_margin:.3f}, "
                    f"后退 vL={vL:.3f}, vR={vR:.3f}"
                )
            
            self._recovery_mode = True
            self._recovery_ticks += 1
            
            return vL, vR
        
        # 重置恢复模式
        if self._recovery_mode:
            self._recovery_mode = False
            self._recovery_ticks = 0
        
        # 将扫描转换为世界坐标系障碍点
        obstacle_points = self._scan_to_world_points(pose, scan)
        
        # DWA 评分
        best_v = 0.0
        best_w = 0.0
        best_score = -1e9
        best_heading = 0.0
        best_clear = 0.0
        best_vel = 0.0
        
        collision_margin = c.robot_radius_m + 0.05
        
        # 遍历动态窗口
        v = v_min
        while v <= v_max + 1e-9:
            w = w_min
            while w <= w_max + 1e-9:
                # 检查轨迹是否安全
                is_safe = self._check_trajectory_safe(
                    pose, float(v), float(w), 
                    c.sim_time_s, c.dt_s,
                    obstacle_points, collision_margin
                )
                
                if not is_safe:
                    w += c.w_resolution
                    continue
                
                # 计算轨迹终点
                end_pose = self._simulate_trajectory(
                    pose, float(v), float(w), c.sim_time_s, c.dt_s
                )
                
                # 评分
                heading_cost = -self._heading_cost(end_pose, self._goal_x, self._goal_y)
                clearance_cost = self._clearance_cost(end_pose, obstacle_points) - c.robot_radius_m
                velocity_cost = float(v)
                
                # 边界检查
                border_margin = self._border_margin_rollout(
                    pose, float(v), float(w), c.sim_time_s, c.dt_s
                )
                clearance_cost = min(clearance_cost, border_margin)
                
                # 拒绝会越界的前进运动
                if border_margin < -0.02 and v > 1e-3:
                    w += c.w_resolution
                    continue
                
                # 总评分
                score = (weight_heading * heading_cost + 
                        c.weight_clearance * clearance_cost + 
                        weight_vel * velocity_cost)
                
                if score > best_score:
                    best_score = score
                    best_v = float(v)
                    best_w = float(w)
                    best_heading = heading_cost
                    best_clear = clearance_cost
                    best_vel = velocity_cost
                
                w += c.w_resolution
            v += c.v_resolution
        
        # 前向速度限制
        if math.isfinite(front_clear):
            max_safe_v = max(0.0, 0.4 * (front_clear - c.robot_radius_m))
            if best_v > max_safe_v:
                best_v = max_safe_v
        
        # 如果清距太小，避免前进
        if best_clear < 0.03 and best_v > 0.0:
            best_v = 0.0
        
        # 后备行为：前方阻塞时原地旋转
        if not math.isfinite(front_clear) or front_clear < c.robot_radius_m * 1.2:
            # 选择更开阔的一侧旋转
            left_sum = sum(d for a, d in zip(scan.angles, scan.ranges) if 0.0 < a < math.pi)
            right_sum = sum(d for a, d in zip(scan.angles, scan.ranges) if a > math.pi or a < 0.0)
            
            best_v = 0.0
            best_w = (local_w_max if left_sum >= right_sum else -local_w_max) * 0.7
        
        # 转换为轮速
        vL = best_v - 0.5 * best_w * self.wheel_base_m
        vR = best_v + 0.5 * best_w * self.wheel_base_m
        
        # MCU 起步阈值适配
        if abs(vL) < c.mcu_wheel_min_mps or abs(vR) < c.mcu_wheel_min_mps:
            avg = 0.5 * (vL + vR)
            diff = 0.5 * (vR - vL)
            
            # 前方宽敞：提升平均速度满足阈值
            if math.isfinite(front_clear) and front_clear > (c.robot_radius_m + c.front_safety_margin_m):
                desired_avg = c.mcu_wheel_min_mps + abs(diff)
                avg = max(0.0, desired_avg)
            else:
                # 前方近障：提升差速满足阈值（原地旋转）
                sign = 1.0 if diff >= 0.0 else -1.0
                diff = sign * max(abs(diff), c.mcu_wheel_min_mps)
                
                # 限制角速度
                max_diff_by_w = 0.5 * self.wheel_base_m * local_w_max
                if abs(diff) > max_diff_by_w:
                    diff = sign * max_diff_by_w
            
            vL = avg - diff
            vR = avg + diff
        
        # 限幅
        vL = max(-c.v_max_mps, min(c.v_max_mps, vL))
        vR = max(-c.v_max_mps, min(c.v_max_mps, vR))
        
        if c.debug:
            logger.info(
                f"[DWA] d={dist_to_goal:.3f} v*={best_v:.3f} w*={best_w:.3f} | "
                f"vL={vL:.3f} vR={vR:.3f} | score={best_score:.3f} "
                f"(head={best_heading:.3f}, clr={best_clear:.3f}, vel={best_vel:.3f})"
            )
        
        return vL, vR
    
    def _estimate_clearance(self, scan: Scan) -> Tuple[float, float]:
        """
        快速估计前方和后方清距
        
        Args:
            scan: 激光扫描
            
        Returns:
            (front_clear, rear_clear) 前方和后方最小距离 (米)
        """
        def wrap_angle(a: float) -> float:
            return (a + math.pi) % (2 * math.pi) - math.pi
        
        front_angles = [d for a, d in zip(scan.angles, scan.ranges) 
                       if abs(wrap_angle(a)) < math.radians(20)]
        rear_angles = [d for a, d in zip(scan.angles, scan.ranges) 
                      if abs(wrap_angle(a)) > (math.pi - math.radians(25))]
        
        front_clear = min(front_angles) if front_angles else float('inf')
        rear_clear = min(rear_angles) if rear_angles else float('inf')
        
        return front_clear, rear_clear
    
    def _scan_to_world_points(self, pose: Pose, scan: Scan) -> np.ndarray:
        """
        将扫描转换为世界坐标系点云（应用雷达偏心）
        
        Args:
            pose: 机器人位姿
            scan: 激光扫描
            
        Returns:
            Nx2 点云数组
        """
        if len(scan.angles) == 0:
            return np.zeros((0, 2), dtype=float)
        
        cos_theta = math.cos(pose.theta)
        sin_theta = math.sin(pose.theta)
        
        # 雷达在世界坐标系中的位置
        lidar_x = pose.x + cos_theta * self.lidar_offset_x - sin_theta * self.lidar_offset_y
        lidar_y = pose.y + sin_theta * self.lidar_offset_x + cos_theta * self.lidar_offset_y
        
        points = []
        for angle, distance in zip(scan.angles, scan.ranges):
            if distance <= 0.0 or not math.isfinite(distance):
                continue
            
            dx_sensor = distance * math.cos(angle)
            dy_sensor = distance * math.sin(angle)
            
            dx_world = cos_theta * dx_sensor - sin_theta * dy_sensor
            dy_world = sin_theta * dx_sensor + cos_theta * dy_sensor
            
            x_world = lidar_x + dx_world
            y_world = lidar_y + dy_world
            
            points.append([x_world, y_world])
        
        if not points:
            return np.zeros((0, 2), dtype=float)
        
        return np.array(points, dtype=float)
    
    def _simulate_trajectory(self, pose: Pose, v: float, w: float, 
                            T: float, dt: float) -> Pose:
        """
        仿真轨迹并返回终点位姿
        
        Args:
            pose: 起始位姿
            v: 线速度 (m/s)
            w: 角速度 (rad/s)
            T: 仿真总时间 (秒)
            dt: 时间步长 (秒)
            
        Returns:
            终点位姿
        """
        x, y, theta = pose.x, pose.y, pose.theta
        t = 0.0
        
        while t < T:
            x += v * math.cos(theta) * dt
            y += v * math.sin(theta) * dt
            theta = (theta + w * dt) % (2 * math.pi)
            t += dt
        
        return Pose(x, y, theta)
    
    def _check_trajectory_safe(self, pose: Pose, v: float, w: float,
                               T: float, dt: float,
                               obstacle_points: np.ndarray, 
                               safety_margin: float) -> bool:
        """
        检查整条轨迹是否安全
        
        Args:
            pose: 起始位姿
            v: 线速度 (m/s)
            w: 角速度 (rad/s)
            T: 仿真总时间 (秒)
            dt: 时间步长 (秒)
            obstacle_points: 障碍点云
            safety_margin: 安全距离 (米)
            
        Returns:
            True=安全，False=碰撞
        """
        if obstacle_points.shape[0] == 0:
            return True
        
        x, y, theta = pose.x, pose.y, pose.theta
        t = 0.0
        
        while t < T:
            x += v * math.cos(theta) * dt
            y += v * math.sin(theta) * dt
            theta = (theta + w * dt) % (2 * math.pi)
            t += dt
            
            # 检查当前点到障碍的最小距离
            dx = obstacle_points[:, 0] - x
            dy = obstacle_points[:, 1] - y
            min_dist = float(np.min(np.hypot(dx, dy)))
            
            if min_dist < safety_margin:
                return False
        
        return True
    
    def _heading_cost(self, pose: Pose, goal_x: float, goal_y: float) -> float:
        """
        计算朝向代价（朝向目标的角度差）
        
        Args:
            pose: 位姿
            goal_x: 目标 x
            goal_y: 目标 y
            
        Returns:
            角度差 (弧度)
        """
        angle_to_goal = math.atan2(goal_y - pose.y, goal_x - pose.x)
        angle_diff = (angle_to_goal - pose.theta + math.pi) % (2 * math.pi) - math.pi
        return abs(angle_diff)
    
    def _clearance_cost(self, pose: Pose, obstacle_points: np.ndarray) -> float:
        """
        计算清距代价（到障碍的最小距离）
        
        Args:
            pose: 位姿
            obstacle_points: 障碍点云
            
        Returns:
            最小距离 (米)
        """
        if obstacle_points.shape[0] == 0:
            return float('inf')
        
        dx = obstacle_points[:, 0] - pose.x
        dy = obstacle_points[:, 1] - pose.y
        min_dist = float(np.min(np.hypot(dx, dy)))
        
        return min_dist
    
    def _border_margin_rollout(self, pose: Pose, v: float, w: float, 
                               T: float, dt: float) -> float:
        """
        计算轨迹的边界余量（防止越界）
        
        假设世界边界为 [0, size_m] x [0, size_m]
        
        Args:
            pose: 起始位姿
            v: 线速度
            w: 角速度
            T: 时间
            dt: 时间步长
            
        Returns:
            最小边界余量 (米)，负值表示越界
        """
        # TODO: 从配置获取世界大小，这里暂时假设 2.8m
        size_m = 2.8
        
        x, y, theta = pose.x, pose.y, pose.theta
        
        min_margin = self._border_margin(x, y, size_m)
        t = 0.0
        
        while t < T:
            x += v * math.cos(theta) * dt
            y += v * math.sin(theta) * dt
            theta = (theta + w * dt) % (2 * math.pi)
            t += dt
            
            margin = self._border_margin(x, y, size_m)
            if margin < min_margin:
                min_margin = margin
        
        return min_margin
    
    def _border_margin(self, x: float, y: float, size_m: float) -> float:
        """
        计算点到边界的余量
        
        Args:
            x: x 坐标
            y: y 坐标
            size_m: 世界大小
            
        Returns:
            边界余量 (米)，负值表示越界
        """
        margin_to_edges = min(x, y, size_m - x, size_m - y)
        return margin_to_edges - self.config.robot_radius_m

