import numpy as np
import math
from typing import Tuple, List, Optional
import logging

logger = logging.getLogger(__name__)


class OccupancyGridMap:
    
    def __init__(self, minx: float, miny: float, maxx: float, maxy: float,
                 resolution: float = 0.03, 
                 l_free: float = -0.4,
                 l_occ: float = 0.85,
                 l_min: float = -4.0,
                 l_max: float = 4.0,
                 hit_margin: float = 1e-3):
        self.minx, self.miny, self.maxx, self.maxy = minx, miny, maxx, maxy
        self.resolution = resolution
        self.l_free = l_free
        self.l_occ = l_occ
        self.l_min = l_min
        self.l_max = l_max
        self.hit_margin = hit_margin
        
        # 计算栅格尺寸
        self.width = int((maxx - minx) / resolution)
        self.height = int((maxy - miny) / resolution)
        
        # 初始化栅格
        self.grid = np.zeros((self.height, self.width), dtype=np.float32)
        
        logger.info(f"初始化占用栅格地图: {self.width}x{self.height}, 分辨率: {resolution}m")
    
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        gx = int((x - self.minx) / self.resolution)
        gy = int((y - self.miny) / self.resolution)
        return gx, gy
    
    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        x = gx * self.resolution + self.minx
        y = gy * self.resolution + self.miny
        return x, y
    
    def is_valid_grid(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.width and 0 <= gy < self.height
    
    def integrate_scan(self, scan_angles: List[float], scan_ranges: List[float], 
                      robot_pose: Tuple[float, float, float],
                      lidar_offset_x: float = -0.05,
                      lidar_offset_y: float = 0.0):
        """
        将激光扫描数据整合到占用栅格地图
        
        Args:
            scan_angles: 扫描角度列表 (弧度)
            scan_ranges: 扫描距离列表 (米)
            robot_pose: 机器人位姿 (x, y, theta)
            lidar_offset_x: 雷达 x 偏移 (米, 相对机器人中心)
            lidar_offset_y: 雷达 y 偏移 (米, 相对机器人中心)
        """
        robot_x, robot_y, robot_theta = robot_pose
        
        # 应用雷达偏心补偿：雷达在车身中心后方 5cm（车头朝 +x）
        cos_theta = math.cos(robot_theta)
        sin_theta = math.sin(robot_theta)
        
        lidar_x = robot_x + cos_theta * lidar_offset_x - sin_theta * lidar_offset_y
        lidar_y = robot_y + sin_theta * lidar_offset_x + cos_theta * lidar_offset_y
        
        lidar_gx, lidar_gy = self.world_to_grid(lidar_x, lidar_y)
        
        if not self.is_valid_grid(lidar_gx, lidar_gy):
            logger.warning(f"雷达位置超出地图范围: ({lidar_x:.3f}, {lidar_y:.3f})")
            return
        
        # 获取最大距离（用于判断是否命中）
        max_range = max(scan_ranges) if scan_ranges else 0.0
        
        for angle, range_val in zip(scan_angles, scan_ranges):
            if range_val <= 0:
                continue
            
            # 限制范围
            rng = min(range_val, max_range)
            
            # 计算激光束终点（世界坐标系）
            end_x = lidar_x + rng * math.cos(robot_theta + angle)
            end_y = lidar_y + rng * math.sin(robot_theta + angle)
            
            end_gx, end_gy = self.world_to_grid(end_x, end_y)
            
            # 判断是否命中障碍物
            hit_occupied = (range_val < max_range - self.hit_margin)
            
            self._update_beam(lidar_gx, lidar_gy, end_gx, end_gy, hit_occupied)
    
    def _update_beam(self, start_gx: int, start_gy: int, 
                    end_gx: int, end_gy: int, hit_occupied: bool):
        """
        使用 Bresenham 算法更新一条激光束路径上的栅格
        
        Args:
            start_gx: 起始栅格 x 坐标
            start_gy: 起始栅格 y 坐标
            end_gx: 终止栅格 x 坐标
            end_gy: 终止栅格 y 坐标
            hit_occupied: 是否命中障碍物
        """
        # Bresenham 算法
        dx = abs(end_gx - start_gx)
        dy = -abs(end_gy - start_gy)
        sx = 1 if start_gx < end_gx else -1
        sy = 1 if start_gy < end_gy else -1
        err = dx + dy
        
        x, y = start_gx, start_gy
        cells = []
        
        # 光线投射：收集路径上的所有栅格
        while True:
            cells.append((x, y))
            if x == end_gx and y == end_gy:
                break
            
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x += sx
            if e2 <= dx:
                err += dx
                y += sy
        
        if not cells:
            return
        
        # 更新自由空间（光线路径上的栅格，除了终点）
        update_cells = cells[:-1] if hit_occupied else cells
        
        for gx, gy in update_cells:
            if self.is_valid_grid(gx, gy):
                # log-odds 更新：自由空间
                self.grid[gy, gx] = np.clip(
                    self.grid[gy, gx] + self.l_free, 
                    self.l_min, 
                    self.l_max
                )
        
        # 如果命中障碍物，更新终点为占用
        if hit_occupied and cells:
            ex, ey = cells[-1]
            if self.is_valid_grid(ex, ey):
                # log-odds 更新：占用空间
                self.grid[ey, ex] = np.clip(
                    self.grid[ey, ex] + self.l_occ,
                    self.l_min,
                    self.l_max
                )
    
    def get_probability_map(self) -> np.ndarray:
        prob_map = 1.0 / (1.0 + np.exp(-self.grid))
        return prob_map
    
    def get_occupancy_map(self) -> np.ndarray:
        prob_map = self.get_probability_map()
        
        occ_map = np.full_like(prob_map, 0.5, dtype=np.float32)
        occ_map[prob_map <= 0.35] = 1.0
        occ_map[prob_map >= 0.65] = 0.0
        
        return occ_map
    
    def get_obstacles(self, threshold: float = 0.65) -> List[Tuple[int, int]]:
        prob_map = self.get_probability_map()
        obstacles = []
        
        for gy in range(self.height):
            for gx in range(self.width):
                if prob_map[gy, gx] >= threshold:
                    obstacles.append((gx, gy))
        
        return obstacles
    
    def get_free_space(self, threshold: float = 0.35) -> List[Tuple[int, int]]:
        prob_map = self.get_probability_map()
        free_space = []
        
        for gy in range(self.height):
            for gx in range(self.width):
                if prob_map[gy, gx] <= threshold:
                    free_space.append((gx, gy))
        
        return free_space
    
    def is_occupied(self, x: float, y: float, threshold: float = 0.65) -> bool:
        gx, gy = self.world_to_grid(x, y)
        
        if not self.is_valid_grid(gx, gy):
            return True
        
        prob_map = self.get_probability_map()
        return prob_map[gy, gx] >= threshold
    
    def get_map_bounds(self) -> Tuple[float, float, float, float]:
        return (self.minx, self.miny, self.maxx, self.maxy)
    
    def resize_map(self, new_minx: float, new_miny: float, 
                   new_maxx: float, new_maxy: float):
        logger.info(f"调整地图大小: ({new_minx}, {new_miny}) -> ({new_maxx}, {new_maxy})")
        
        new_map = OccupancyGridMap(
            new_minx, new_miny, new_maxx, new_maxy,
            self.resolution, self.l_free, self.l_occ,
            self.l_min, self.l_max, self.hit_margin
        )
                
        old_minx, old_miny, old_maxx, old_maxy = self.get_map_bounds()
        
        for gy in range(self.height):
            for gx in range(self.width):
                old_x, old_y = self.grid_to_world(gx, gy)
                if (new_minx <= old_x <= new_maxx and 
                    new_miny <= old_y <= new_maxy):
                    new_gx, new_gy = new_map.world_to_grid(old_x, old_y)
                    if new_map.is_valid_grid(new_gx, new_gy):
                        new_map.grid[new_gy, new_gx] = self.grid[gy, gx]
        
        # 更新当前地图
        self.__dict__.update(new_map.__dict__)
    
    def save_map(self, filename: str):
        """保存地图到文件"""
        np.save(filename, {
            'grid': self.grid,
            'bounds': (self.minx, self.miny, self.maxx, self.maxy),
            'resolution': self.resolution,
            'params': (self.l_free, self.l_occ, self.l_min, self.l_max, self.hit_margin)
        })
        logger.info(f"地图已保存到: {filename}")
    
    def load_map(self, filename: str):
        """从文件加载地图"""
        data = np.load(filename, allow_pickle=True).item()
        
        self.grid = data['grid']
        self.minx, self.miny, self.maxx, self.maxy = data['bounds']
        self.resolution = data['resolution']
        self.l_free, self.l_occ, self.l_min, self.l_max, self.hit_margin = data['params']
        
        self.width = self.grid.shape[1]
        self.height = self.grid.shape[0]
        
        logger.info(f"地图已从文件加载: {filename}")
