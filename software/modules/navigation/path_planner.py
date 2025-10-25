import numpy as np
import math
import sys
from pathlib import Path
from typing import List, Tuple, Optional, Dict, Any
from collections import deque
import logging

# 添加 env 到 Python 路径
_env_path = Path(__file__).parent.parent.parent / "env"
if str(_env_path) not in sys.path:
    sys.path.insert(0, str(_env_path))

# 从 env 中导入 PythonRobotics 的 A*
from PathPlanning.AStar.a_star import AStarPlanner as PR_AStarPlanner

logger = logging.getLogger(__name__)


class PathPlanner:
    
    def __init__(self, grid_resolution: float = 0.03, robot_radius: float = 0.15):
        self.grid_resolution = grid_resolution
        self.robot_radius = robot_radius
        self.obstacles: List[Tuple[float, float]] = []
        self.grid_map: Optional[np.ndarray] = None
        
    def set_obstacles(self, obstacles: List[Tuple[float, float]]):
        self.obstacles = obstacles
        self._build_grid_map()
    
    def _build_grid_map(self):
        if not self.obstacles:
            return
        
        if self.obstacles:
            x_coords = [obs[0] for obs in self.obstacles]
            y_coords = [obs[1] for obs in self.obstacles]
            
            min_x, max_x = min(x_coords), max(x_coords)
            min_y, max_y = min(y_coords), max(y_coords)
            
            margin = self.robot_radius * 2
            min_x -= margin
            max_x += margin
            min_y -= margin
            max_y += margin
        else:
            min_x = min_y = -5
            max_x = max_y = 5
        
        self.width = int((max_x - min_x) / self.grid_resolution)
        self.height = int((max_y - min_y) / self.grid_resolution)
        
        self.grid_map = np.zeros((self.height, self.width), dtype=bool)
        self.min_x = min_x
        self.min_y = min_y
        
        for obs_x, obs_y in self.obstacles:
            gx = int((obs_x - min_x) / self.grid_resolution)
            gy = int((obs_y - min_y) / self.grid_resolution)
            
            if 0 <= gx < self.width and 0 <= gy < self.height:
                self._inflate_obstacle(gx, gy)
    
    def _inflate_obstacle(self, center_x: int, center_y: int):
        radius_cells = int(self.robot_radius / self.grid_resolution)
        
        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                if dx*dx + dy*dy <= radius_cells*radius_cells:
                    gx = center_x + dx
                    gy = center_y + dy
                    
                    if 0 <= gx < self.width and 0 <= gy < self.height:
                        self.grid_map[gy, gx] = True
    
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        gx = int((x - self.min_x) / self.grid_resolution)
        gy = int((y - self.min_y) / self.grid_resolution)
        return gx, gy
    
    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        x = gx * self.grid_resolution + self.min_x
        y = gy * self.grid_resolution + self.min_y
        return x, y
    
    def is_valid_grid(self, gx: int, gy: int) -> bool:
        if not (0 <= gx < self.width and 0 <= gy < self.height):
            return False
        return not self.grid_map[gy, gx]
    
    def plan(self, start: Tuple[float, float], goal: Tuple[float, float]) -> List[Tuple[float, float]]:
        raise NotImplementedError("子类必须实现plan方法")


class AStarPlanner(PathPlanner):
    """
    A* 路径规划器（使用 PythonRobotics 实现）
    """
    
    def __init__(self, grid_resolution: float = 0.03, robot_radius: float = 0.15):
        super().__init__(grid_resolution, robot_radius)
        self.heuristic_weight = 1.0
        self._pr_planner = None  # PythonRobotics A* 规划器实例
    
    def plan(self, start: Tuple[float, float], goal: Tuple[float, float]) -> List[Tuple[float, float]]:
        """
        使用 PythonRobotics A* 进行路径规划
        
        Args:
            start: 起点 (x, y) 米
            goal: 终点 (x, y) 米
            
        Returns:
            路径点列表 [(x, y), ...]
        """
        if not self.obstacles:
            logger.warning("没有设置障碍物，无法规划")
            return [start, goal]
        
        try:
            # 提取障碍物坐标
            ox = [obs[0] for obs in self.obstacles]
            oy = [obs[1] for obs in self.obstacles]
            
            # 创建 PythonRobotics A* 规划器
            # 参数: (ox, oy, resolution, robot_radius)
            self._pr_planner = PR_AStarPlanner(ox, oy, self.grid_resolution, self.robot_radius)
            
            # 执行规划
            # 参数: (sx, sy, gx, gy)
            # 返回: (rx, ry) - 路径点的 x 和 y 列表
            rx, ry = self._pr_planner.planning(start[0], start[1], goal[0], goal[1])
            
            if rx is None or ry is None or len(rx) == 0:
                logger.warning("A* 未找到有效路径")
                return []
            
            # 转换为 [(x, y), ...] 格式
            path = list(zip(rx, ry))
            
            # 路径平滑
            smoothed_path = self._smooth_path(path)
            
            logger.info(f"A* 路径规划完成，路径长度: {len(smoothed_path)}")
            return smoothed_path
            
        except Exception as e:
            logger.error(f"A* 规划失败: {e}")
            import traceback
            traceback.print_exc()
            return []
    
    
    def _smooth_path(self, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        if len(path) <= 2:
            return path
        
        smoothed = [path[0]]
        
        for i in range(1, len(path) - 1):
            prev_point = path[i-1]
            curr_point = path[i]
            next_point = path[i+1]
            
            smooth_x = (prev_point[0] + curr_point[0] + next_point[0]) / 3
            smooth_y = (prev_point[1] + curr_point[1] + next_point[1]) / 3
            
            smoothed.append((smooth_x, smooth_y))
        
        smoothed.append(path[-1])
        return smoothed


class RRTPlanner(PathPlanner):
    
    def __init__(self, grid_resolution: float = 0.03, robot_radius: float = 0.15,
                 max_iterations: int = 1000, step_size: float = 0.5):
        super().__init__(grid_resolution, robot_radius)
        self.max_iterations = max_iterations
        self.step_size = step_size
    
    def plan(self, start: Tuple[float, float], goal: Tuple[float, float]) -> List[Tuple[float, float]]:
        return [start, goal]
