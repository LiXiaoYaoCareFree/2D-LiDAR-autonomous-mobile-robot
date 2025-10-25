import math
import numpy as np
from typing import List, Tuple, Optional, Dict, Any
import logging

try:
    from python_robotics.dwa import DWA
    from python_robotics.dynamic_window import DynamicWindow
    PYTHON_ROBOTICS_AVAILABLE = True
except ImportError:
    PYTHON_ROBOTICS_AVAILABLE = False

from ..sensors.data_structures import Pose, Scan

logger = logging.getLogger(__name__)


class NavigationController:
    
    def __init__(self, robot_radius: float = 0.15):
        self.robot_radius = robot_radius
        self.current_path: List[Tuple[float, float]] = []
        self.path_index = 0
        
    def set_path(self, path: List[Tuple[float, float]]):
        self.current_path = path
        self.path_index = 0
        logger.info(f"设置导航路径，路径长度: {len(path)}")
    
    def update(self, current_pose: Pose, scan: Scan) -> Dict[str, Any]:
        raise NotImplementedError("子类必须实现update方法")
    
    def is_goal_reached(self, current_pose: Pose, goal: Tuple[float, float], 
                       tolerance: float = 0.1) -> bool:
        distance = math.sqrt((current_pose.x - goal[0])**2 + (current_pose.y - goal[1])**2)
        return distance <= tolerance


class SimpleController(NavigationController):
    
    def __init__(self, robot_radius: float = 0.15, 
                 lookahead_distance: float = 0.3,
                 max_linear_vel: float = 0.3,
                 max_angular_vel: float = 2.0,
                 k_angular: float = 1.8,
                 k_avoid: float = 1.2,
                 avoid_radius: float = 0.35):
        super().__init__(robot_radius)
        
        self.lookahead_distance = lookahead_distance
        self.max_linear_vel = max_linear_vel
        self.max_angular_vel = max_angular_vel
        self.k_angular = k_angular
        self.k_avoid = k_avoid
        self.avoid_radius = avoid_radius
        
    def update(self, current_pose: Pose, scan: Scan) -> Dict[str, Any]:
        if not self.current_path:
            return {
                'linear_vel': 0.0,
                'angular_vel': 0.0,
                'target_reached': True,
                'path_progress': 0.0
            }
        
        target_point = self._get_lookahead_target(current_pose)
        
        avoid_force = self._calculate_avoidance_force(current_pose, scan)
        
        linear_vel, angular_vel = self._calculate_control_output(
            current_pose, target_point, avoid_force
        )
        
        goal_reached = self.is_goal_reached(current_pose, self.current_path[-1])
        
        path_progress = self._calculate_path_progress(current_pose)
        
        return {
            'linear_vel': linear_vel,
            'angular_vel': angular_vel,
            'target_reached': goal_reached,
            'path_progress': path_progress,
            'target_point': target_point,
            'avoid_force': avoid_force
        }
    
    def _get_lookahead_target(self, current_pose: Pose) -> Tuple[float, float]:
        if not self.current_path:
            return (current_pose.x, current_pose.y)
        
        min_distance = float('inf')
        closest_index = 0
        
        for i, point in enumerate(self.current_path):
            distance = math.sqrt(
                (point[0] - current_pose.x)**2 + (point[1] - current_pose.y)**2
            )
            if distance < min_distance:
                min_distance = distance
                closest_index = i
        
        lookahead_distance = 0.0
        target_index = closest_index
        
        for i in range(closest_index, len(self.current_path) - 1):
            current_point = self.current_path[i]
            next_point = self.current_path[i + 1]
            
            segment_distance = math.sqrt(
                (next_point[0] - current_point[0])**2 + 
                (next_point[1] - current_point[1])**2
            )
            
            if lookahead_distance + segment_distance >= self.lookahead_distance:
                ratio = (self.lookahead_distance - lookahead_distance) / segment_distance
                target_x = current_point[0] + ratio * (next_point[0] - current_point[0])
                target_y = current_point[1] + ratio * (next_point[1] - current_point[1])
                return (target_x, target_y)
            
            lookahead_distance += segment_distance
            target_index = i + 1
        
        return self.current_path[-1]
    
    def _calculate_avoidance_force(self, current_pose: Pose, scan: Scan) -> Tuple[float, float]:
        avoid_force_x = 0.0
        avoid_force_y = 0.0
        
        for angle, range_val in zip(scan.angles, scan.ranges):
            if range_val <= 0 or range_val > self.avoid_radius:
                continue
            
            force_magnitude = self.k_avoid * (1.0 / range_val - 1.0 / self.avoid_radius)
            
            avoid_force_x -= force_magnitude * math.cos(angle)
            avoid_force_y -= force_magnitude * math.sin(angle)
        
        return (avoid_force_x, avoid_force_y)
    
    def _calculate_control_output(self, current_pose: Pose, target_point: Tuple[float, float],
                                 avoid_force: Tuple[float, float]) -> Tuple[float, float]:
        target_x, target_y = target_point
        dx = target_x - current_pose.x
        dy = target_y - current_pose.y
        
        target_angle = math.atan2(dy, dx)
        
        angle_error = target_angle - current_pose.theta
        angle_error = self._normalize_angle(angle_error)
        
        distance = math.sqrt(dx**2 + dy**2)
        
        linear_vel = min(self.max_linear_vel, distance * 2.0)
        angular_vel = self.k_angular * angle_error
        
        avoid_force_x, avoid_force_y = avoid_force
        angular_vel += self.k_avoid * (avoid_force_x * math.sin(current_pose.theta) - 
                                      avoid_force_y * math.cos(current_pose.theta))
        
        angular_vel = max(-self.max_angular_vel, min(self.max_angular_vel, angular_vel))
        
        return linear_vel, angular_vel
    
    def _calculate_path_progress(self, current_pose: Pose) -> float:
        if not self.current_path:
            return 0.0
        
        min_distance = float('inf')
        closest_index = 0
        
        for i, point in enumerate(self.current_path):
            distance = math.sqrt(
                (point[0] - current_pose.x)**2 + (point[1] - current_pose.y)**2
            )
            if distance < min_distance:
                min_distance = distance
                closest_index = i
        
        return closest_index / len(self.current_path)
    
    def _normalize_angle(self, angle: float) -> float:
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


class PIDController(NavigationController):
    
    def __init__(self, robot_radius: float = 0.15,
                 kp: float = 1.0, ki: float = 0.1, kd: float = 0.05,
                 max_linear_vel: float = 0.3, max_angular_vel: float = 2.0):
        super().__init__(robot_radius)
        
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_linear_vel = max_linear_vel
        self.max_angular_vel = max_angular_vel
        
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = None
    
    def update(self, current_pose: Pose, scan: Scan) -> Dict[str, Any]:
        if not self.current_path:
            return {
                'linear_vel': 0.0,
                'angular_vel': 0.0,
                'target_reached': True,
                'path_progress': 0.0
            }
        
        target_point = self._get_closest_target(current_pose)
        
        error = self._calculate_error(current_pose, target_point)
        
        angular_vel = self._pid_control(error)
        
        distance = math.sqrt(
            (target_point[0] - current_pose.x)**2 + 
            (target_point[1] - current_pose.y)**2
        )
        linear_vel = min(self.max_linear_vel, distance * 1.0)
        
        goal_reached = self.is_goal_reached(current_pose, self.current_path[-1])
        
        return {
            'linear_vel': linear_vel,
            'angular_vel': angular_vel,
            'target_reached': goal_reached,
            'path_progress': self._calculate_path_progress(current_pose),
            'error': error
        }
    
    def _get_closest_target(self, current_pose: Pose) -> Tuple[float, float]:
        if not self.current_path:
            return (current_pose.x, current_pose.y)
        
        min_distance = float('inf')
        closest_point = self.current_path[0]
        
        for point in self.current_path:
            distance = math.sqrt(
                (point[0] - current_pose.x)**2 + (point[1] - current_pose.y)**2
            )
            if distance < min_distance:
                min_distance = distance
                closest_point = point
        
        return closest_point
    
    def _calculate_error(self, current_pose: Pose, target_point: Tuple[float, float]) -> float:
        target_angle = math.atan2(
            target_point[1] - current_pose.y,
            target_point[0] - current_pose.x
        )
        
        error = target_angle - current_pose.theta
        return self._normalize_angle(error)
    
    def _pid_control(self, error: float) -> float:
        import time
        current_time = time.time()
        
        if self.last_time is None:
            self.last_time = current_time
            dt = 0.1
        else:
            dt = current_time - self.last_time
            self.last_time = current_time
        
        self.integral += error * dt
        
        derivative = (error - self.last_error) / dt if dt > 0 else 0
        
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        
        self.last_error = error
        
        return max(-self.max_angular_vel, min(self.max_angular_vel, output))
    
    def _calculate_path_progress(self, current_pose: Pose) -> float:
        if not self.current_path:
            return 0.0
        
        min_distance = float('inf')
        closest_index = 0
        
        for i, point in enumerate(self.current_path):
            distance = math.sqrt(
                (point[0] - current_pose.x)**2 + (point[1] - current_pose.y)**2
            )
            if distance < min_distance:
                min_distance = distance
                closest_index = i
        
        return closest_index / len(self.current_path)
    
    def _normalize_angle(self, angle: float) -> float:
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
