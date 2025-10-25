#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
迷宫SLAM仿真程序 - 使用位姿图优化(Pose-Graph SLAM)
模拟小车在迷宫中缓慢移动并实时重建地图
"""

import matplotlib.pyplot as plt
import numpy as np
import math
import time
import json
import matplotlib.patches as patches
import queue
from scipy.ndimage import distance_transform_edt
from new import PoseGraphSLAM
from collections import deque
from datetime import datetime
import os

# 加速绘图模式配置
FAST_PLOT = True  # True: 更快绘图, False: 更精致绘制
FAST_PLOT_MAX_MAP_POINTS = 5000      # SLAM点云最大绘制点数
FAST_PLOT_MAX_TRAJ_POINTS = 2000     # 轨迹最大绘制点数
FAST_PLOT_LIDAR_STRIDE = 2           # 雷达散点步长

# --- Matplotlib 中文显示设置 ---
import matplotlib
import platform
try:
    # 新增：根据操作系统选择字体
    os_system = platform.system()
    if os_system == 'Darwin':  # macOS
        font_name = 'Arial Unicode MS'
    elif os_system == 'Windows':  # Windows
        font_name = 'SimHei'
    elif os_system == 'Linux':  # Linux
        font_name = 'WenQuanYi Zen Hei'  # 一个常用的开源中文字体
    else:
        font_name = None
    
    if font_name:
        matplotlib.rcParams['font.sans-serif'] = [font_name]

    matplotlib.rcParams['font.family']='sans-serif'
    # 解决负号'-'显示为方块的问题
    matplotlib.rcParams['axes.unicode_minus'] = False
except Exception:
    print("未能动态设置中文字体, 部分标签可能显示为方块。")

# --- 新增: 全局仿真运行状态标志 ---
simulation_running = True

def on_window_close(event):
    """Matplotlib窗口关闭事件处理函数"""
    global simulation_running
    print("检测到窗口关闭... 正在停止仿真...")
    simulation_running = False

# --- ICP算法核心函数 (用于回环检测) ---
def _icp_svd_motion_estimation(previous_points, current_points):
    """通过SVD分解计算运动估计"""
    pm = np.mean(previous_points, axis=1)
    cm = np.mean(current_points, axis=1)
    p_shift = previous_points - pm[:, np.newaxis]
    c_shift = current_points - cm[:, np.newaxis]
    W = c_shift @ p_shift.T
    u, _, vh = np.linalg.svd(W)
    R = (u @ vh).T
    t = pm - (R @ cm)
    return R, t

def _icp_nearest_neighbor_association(previous_points, current_points):
    """最近邻关联, 并计算误差"""
    d = np.linalg.norm(
        np.repeat(current_points, previous_points.shape[1], axis=1) - 
        np.tile(previous_points, (1, current_points.shape[1])), 
        axis=0
    )
    d_matrix = d.reshape(current_points.shape[1], previous_points.shape[1])
    indexes = np.argmin(d_matrix, axis=1)
    error = np.sum(np.min(d_matrix, axis=1))
    return indexes, error

def _icp_matching(previous_points, current_points, max_iter=20, eps=0.001):
    """执行ICP匹配, 返回 (R, t, error)"""
    H = np.identity(3)
    current_points_copy = np.copy(current_points)
    prev_error = float('inf')

    if previous_points.shape[1] < 5 or current_points.shape[1] < 5:
        return np.identity(2), np.zeros(2), float('inf')

    for i in range(max_iter):
        indexes, error = _icp_nearest_neighbor_association(previous_points, current_points_copy)
        Rt, Tt = _icp_svd_motion_estimation(previous_points[:, indexes], current_points_copy)
        
        current_points_copy = (Rt @ current_points_copy) + Tt[:, np.newaxis]
        
        H_delta = np.identity(3)
        H_delta[0:2, 0:2] = Rt
        H_delta[0:2, 2] = Tt
        H = H_delta @ H
        
        if abs(prev_error - error) < eps:
            break
        prev_error = error
    
    R = H[0:2, 0:2]
    T = H[0:2, 2]
    return R, T, error

def scan_to_points(scan_distances):
    """将激光雷达扫描(毫米)转换为机器人坐标系下的点云(米)"""
    points = []
    angles = np.linspace(0, 2*np.pi, len(scan_distances), endpoint=False)
    for angle, dist_mm in zip(angles, scan_distances):
        if 10 < dist_mm < 3990:
             dist_m = dist_mm / 1000.0
             x = dist_m * np.cos(angle)
             y = dist_m * np.sin(angle)
             points.append([x, y])
    if not points:
         return np.array([[],[]])
    return np.array(points).T

class MazeEnvironment:
    """迷宫环境类 - 处理地图加载和激光雷达仿真"""
    
    def __init__(self, map_file='map_information.json'):
        with open(map_file, 'r') as f:
            self.map_data = json.load(f)
        
        self.segments = self.map_data['segments']
        
        # 动态计算迷宫边界并添加虚拟墙壁
        if self.segments:
            all_points = np.array([p for seg in self.segments for p in (seg["start"], seg["end"])])
            max_x, max_y = np.max(all_points, axis=0)
        else:
            max_x, max_y = 15, 15 # 默认值，以防万一

        self.max_x, self.max_y = max_x, max_y # 保存最大边界

        self.start_point = self.map_data['start_point']
        
        # 创建网格地图
        self.grid_size = 30
        self.resolution = 0.05  # 网格分辨率
        grid_dim = int(self.grid_size / self.resolution)
        self.grid = np.zeros((grid_dim, grid_dim))
        
        self._create_grid_map()
        
        # 新增：对障碍物进行膨胀
        self.robot_radius = 0.15 # 机器人半径 (m)
        self._inflate_obstacles()
        '''print(self.grid)
        np.savetxt('grid.txt', self.grid, fmt='%d')
        exit()'''
    
    def _inflate_obstacles(self):
        """对地图中的障碍物进行膨胀，以考虑机器人体积"""
        print("正在膨胀障碍物以考虑机器人体积...")
        inflation_radius_cells = int(self.robot_radius / self.resolution)
        if inflation_radius_cells == 0:
            return # 如果半径小于一个单元格，则不膨胀

        inflated_grid = np.copy(self.grid)
        rows, cols = self.grid.shape
        
        # 找出所有障碍物格子的坐标
        obstacle_indices = np.argwhere(self.grid == 1)
        
        for r, c in obstacle_indices:
            # 对于每个障碍物格子，将其周围的区域标记为障碍
            for dr in range(-inflation_radius_cells, inflation_radius_cells + 1):
                for dc in range(-inflation_radius_cells, inflation_radius_cells + 1):
                    # 使用圆形半径进行检查
                    if dr**2 + dc**2 <= inflation_radius_cells**2:
                        nr, nc = r + dr, c + dc
                        if 0 <= nr < rows and 0 <= nc < cols:
                            inflated_grid[nr, nc] = 1
        
        self.grid = inflated_grid
        print("障碍物膨胀完成。")
    
    def _create_grid_map(self):
        """将线段转换为网格地图"""
        for seg in self.segments:
            x1, y1 = seg["start"]
            x2, y2 = seg["end"]
            self._draw_line_in_grid(x1, y1, x2, y2)
    
    def _draw_line_in_grid(self, x1, y1, x2, y2):
        """在网格中绘制线段"""
        # 简单的线段栅格化
        num_points = int(max(abs(x2-x1), abs(y2-y1)) / self.resolution)
        if num_points == 0:
            num_points = 1
        
        x_coords = np.linspace(x1, x2, num_points)
        y_coords = np.linspace(y1, y2, num_points)

        for x, y in zip(x_coords, y_coords):
            grid_x = int(x / self.resolution)
            grid_y = int(y / self.resolution)
            
            if 0 <= grid_x < self.grid.shape[1] and 0 <= grid_y < self.grid.shape[0]:
                self.grid[grid_y, grid_x] = 1

    def is_occupied(self, x, y):
        """检查世界坐标中的一个点是否在墙内或界外"""
        grid_x = int(x / self.resolution)
        grid_y = int(y / self.resolution)
        
        # 边界检查
        if (grid_x < 0 or grid_x >= self.grid.shape[1] or 
            grid_y < 0 or grid_y >= self.grid.shape[0]):
            return True # 界外视为障碍物
        
        # 碰撞检查
        return self.grid[grid_y, grid_x] == 1

    def simulate_lidar(self, robot_x, robot_y, robot_theta, num_rays=360, max_range=4.0):
        """模拟激光雷达扫描"""
        angles = np.linspace(0, 2*np.pi, num_rays, endpoint=False)
        scan_distances = []
        
        for angle in angles:
            ray_angle = robot_theta + angle
            distance = self._cast_ray(robot_x, robot_y, ray_angle, max_range)
            scan_distances.append(distance * 1000)  # 转换为毫米
        
        return scan_distances
    
    def _cast_ray(self, start_x, start_y, angle, max_range):
        """射线投射"""
        step_size = 0.02
        distance = 0.0
        
        while distance < max_range:
            x = start_x + distance * math.cos(angle)
            y = start_y + distance * math.sin(angle)
            
            grid_x = int(x / self.resolution)
            grid_y = int(y / self.resolution)
            
            # 边界检查 - 修复：当超出边界时返回max_range
            if (grid_x < 0 or grid_x >= self.grid.shape[1] or 
                grid_y < 0 or grid_y >= self.grid.shape[0]):
                return max_range  # 射线射出地图边界，返回最大距离
            
            # 碰撞检查
            if self.grid[grid_y, grid_x] == 1:
                break
            
            distance += step_size
        
        return min(distance, max_range)

class RobotController:
    """机器人控制器 - 处理运动和导航"""
    
    def __init__(self, start_x, start_y, start_theta=0.0, env=None):
        self.x = start_x
        self.y = start_y  
        self.theta = start_theta
        self.env = env # 引用环境以进行碰撞检测
        
        # 运动参数 - 优化后的参数，提高速度但保持安全性
        self.linear_speed = 1.2  # 线速度 (m/s) - 提高速度
        self.angular_speed = 0.75   # 角速度 (rad/s) - 提高转向速度
         
        
        # 轨迹
        self.trajectory_x = [start_x]
        self.trajectory_y = [start_y]
        
        # --- 新增：最近的雷达扫描数据 ---
        self.recent_scans = deque(maxlen=10)

        # --- 新增：占据栅格地图 ---
        self.map_resolution = 0.1  # 地图分辨率 (m/cell)
        self.map_size_m = 30    # 地图大小 (meters)
        self.map_dim = int(self.map_size_m / self.map_resolution) # 地图维度 (cells)
        
        # 使用对数概率表示地图, 初始为0 (未知)
        self.log_odds_map = np.zeros((self.map_dim, self.map_dim))
        
        # 更新参数
        self.log_odds_occ = np.log(0.9 / 0.1)  # 占用概率 0.9
        self.log_odds_free = np.log(0.3 / 0.7) # 空闲概率 0.3
        self.log_odds_min = -10 # 最小log-odds值
        self.log_odds_max = 10  # 最大log-odds值

        # --- 高级任务状态机 ---
        self.mission_phase = "EXPLORING_MAZE" # EXPLORING_MAZE, RETURNING_TO_START, GOING_TO_EXIT, MISSION_COMPLETE

        # --- 精细探索状态 ---
        self.exploration_state = "FIND_TARGET" # "FIND_TARGET", "FOLLOW_PATH", "CONFIRMING_EXIT", "RETURNING_TO_MAZE", "FINISHED"
        self.current_path = []
        self.current_target = None
        self.path_step = 0
        self.exploration_percentage = 0.0

        # --- 新增: 扫描无效率 ---
        self.scan_inefficiency = 0.0
        self.inefficiency_history = []

        # --- 新增: 出口检测 ---
        self.exit_pose = None # 存储(x, y, theta)
        self.exit_confirmation_target = None # 存储世界坐标(x,y)
        self.last_frontier_discovery = time.time()  # 记录最后发现前沿点的时间
        self.last_movement_time = time.time()  # 记录最后移动时间
        
        # --- 新增: 避免重复访问机制 ---
        self.visited_positions = set()  # 记录已访问的网格位置
        self.visited_frontiers = set()  # 记录已访问的前沿点
        self.exploration_history = deque(maxlen=50)  # 探索历史记录（减少内存占用）

    def _bresenham_line(self, x0, y0, x1, y1):
        """Bresenham's line algorithm. 返回线段上的所有点。"""
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        points = []
        while True:
            points.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy
        return points

    def update_occupancy_grid(self, pose, scan_distances):
        """根据当前位姿和原始激光扫描更新占据栅格地图"""
        robot_x, robot_y, robot_theta = pose
        robot_x_grid = int(robot_x / self.map_resolution)
        robot_y_grid = int(robot_y / self.map_resolution)

        R = np.array([[math.cos(robot_theta), -math.sin(robot_theta)],
                      [math.sin(robot_theta),  math.cos(robot_theta)]])
        
        angles = np.linspace(0, 2*np.pi, len(scan_distances), endpoint=False)
        max_dist_mm = 3990

        for angle, dist_mm in zip(angles, scan_distances):
            if dist_mm <= 10:
                continue

            dist_m = dist_mm / 1000.0
            
            # Laser endpoint in robot frame
            laser_x_robot = dist_m * np.cos(angle)
            laser_y_robot = dist_m * np.sin(angle)
            
            # Laser endpoint in world frame
            world_point = (R @ np.array([laser_x_robot, laser_y_robot])) + np.array([robot_x, robot_y])
            px, py = world_point
            px_grid = int(px / self.map_resolution)
            py_grid = int(py / self.map_resolution)

            # Get cells along the ray
            ray_cells = self._bresenham_line(robot_x_grid, robot_y_grid, px_grid, py_grid)

            # Update free space along the ray (excluding the endpoint)
            for cell_x, cell_y in ray_cells[:-1]:
                if 0 <= cell_x < self.map_dim and 0 <= cell_y < self.map_dim:
                    self.log_odds_map[cell_y, cell_x] += self.log_odds_free
            
            # Update endpoint: occupied if hit, free if max range
            if 0 <= px_grid < self.map_dim and 0 <= py_grid < self.map_dim:
                if dist_mm < max_dist_mm:
                    # Hit a wall
                    self.log_odds_map[py_grid, px_grid] += self.log_odds_occ
                else:
                    # Reached max range, this cell is also free
                    self.log_odds_map[py_grid, px_grid] += self.log_odds_free

        # Clip values
        np.clip(self.log_odds_map, self.log_odds_min, self.log_odds_max, out=self.log_odds_map)
        
        # 更新探索百分比 - 改进算法
        # 使用更合理的阈值来判断已知区域
        # 占用区域：log_odds > 2.0 (对应概率 > 0.88)
        # 空闲区域：log_odds < -2.0 (对应概率 < 0.12)
        occupied_cells = np.sum(self.log_odds_map > 2.0)
        free_cells = np.sum(self.log_odds_map < -2.0)
        known_cells = occupied_cells + free_cells
        
        # 计算实际探索区域的比例
        # 方法1：基于整个地图网格（原始方法）
        total_map_cells = self.map_dim * self.map_dim
        grid_percentage = known_cells / total_map_cells
        
        # 方法2：基于实际迷宫大小（更合理）
        if hasattr(self, 'env') and self.env:
            # 计算迷宫的实际面积
            # 注意：MazeEnvironment只有max_x和max_y，没有min_x和min_y
            # 假设迷宫从(0,0)开始到(max_x, max_y)
            maze_width = self.env.max_x - 0  # 从0开始
            maze_height = self.env.max_y - 0  # 从0开始
            maze_area = maze_width * maze_height
            
            # 将网格单元转换为实际面积
            cell_area = self.map_resolution * self.map_resolution
            maze_cells = int(maze_area / cell_area)
            
            if maze_cells > 0:
                maze_percentage = known_cells / maze_cells
                # 使用迷宫百分比，但限制在合理范围内
                self.exploration_percentage = min(maze_percentage, 1.0)
            else:
                self.exploration_percentage = grid_percentage
        else:
            # 如果没有环境信息，使用网格百分比
            self.exploration_percentage = grid_percentage
        
        # 限制在合理范围内，避免异常值
        self.exploration_percentage = min(self.exploration_percentage, 1.0)

    def _create_pathfinding_costmap(self):
        """使用距离变换创建用于A*规划的成本地图"""
        # 1. 创建二值障碍物图
        occ_mask = self.log_odds_map > self.log_odds_occ * 0.8
        
        # 2. 计算到最近障碍物的距离
        # distance_transform_edt计算的是到最近的0的距离, 所以先反转mask
        dist_transform = distance_transform_edt(np.logical_not(occ_mask))

        # 3. 将距离转换为成本
        # 我们希望离障碍物越近, 成本越高
        # 定义一个安全距离, 超过此距离则成本很低
        safe_dist_cells = 0.5 / self.map_resolution # 50cm
        
        # 将距离归一化, 并裁剪
        dist_transform = np.clip(dist_transform, 0, safe_dist_cells)
        
        # 成本与距离成反比, 并用平方增加惩罚力度
        costmap = (safe_dist_cells - dist_transform)**2
        
        # 4. 对未知区域施加一个适中的成本，允许通过但不太鼓励
        unknown_mask = np.abs(self.log_odds_map) < 0.1
        costmap[unknown_mask] = 50  # 降低未知区域成本，从1000降到50
        
        # 5. 障碍物区域成本为无穷大
        costmap[occ_mask] = float('inf')
        
        # --- 新增: 区域惩罚逻辑 ---
        # 如果已找到出口并且仍在探索迷宫，则对外围区域施加高成本
        if self.mission_phase == "EXPLORING_MAZE" and self.exit_pose is not None:
            exit_x, exit_y, exit_theta = self.exit_pose
            
            # 创建坐标网格
            x_coords = (np.arange(self.map_dim) + 0.5) * self.map_resolution
            y_coords = (np.arange(self.map_dim) + 0.5) * self.map_resolution
            grid_x, grid_y = np.meshgrid(x_coords, y_coords)
            
            # 计算出口方向的法向量
            nx, ny = math.cos(exit_theta), math.sin(exit_theta)
            
            # 计算所有点在出口方向上的投影
            # 点(px, py)在出口外侧的条件是: (px-ex, py-ey) 与 (nx, ny)的点积为正
            projection = (grid_x - exit_x) * nx + (grid_y - exit_y) * ny
            
            # 定义一个缓冲区，避免把出口本身也惩罚了
            outside_mask = projection > -0.5  # 0.5米缓冲区
            
            # 施加高昂的成本，但不覆盖已有障碍物
            costmap[outside_mask & (costmap != float('inf'))] += 10000
        
        return costmap

    def _find_frontier_clusters(self, min_cluster_size=5):
        """寻找并聚类前沿点, 返回每个簇的质心和大小"""
        map_free = self.log_odds_map < -0.5
        map_unknown = np.abs(self.log_odds_map) < 0.1
        
        frontiers = set()
        # 找出所有前沿点
        free_indices_rows, free_indices_cols = np.where(map_free)
        for r, c in zip(free_indices_rows, free_indices_cols):
            for dr in [-1, 0, 1]:
                for dc in [-1, 0, 1]:
                    if dr == 0 and dc == 0: continue
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < self.map_dim and 0 <= nc < self.map_dim:
                        if map_unknown[nr, nc]:
                            frontiers.add((c, r)) # (x, y) grid coordinates
                            break
                else: continue
                break
        
        # 聚类
        clusters = []
        visited = set()
        for fx, fy in frontiers:
            if (fx, fy) in visited:
                continue
            
            new_cluster = []
            q = [(fx, fy)]
            visited.add((fx, fy))
            
            head = 0
            while head < len(q):
                cx, cy = q[head]
                head += 1
                new_cluster.append((cx, cy))
                
                for dx in [-1, 0, 1]:
                    for dy in [-1, 0, 1]:
                        if dx == 0 and dy == 0: continue
                        nx, ny = cx + dx, cy + dy
                        if (nx, ny) in frontiers and (nx, ny) not in visited:
                            visited.add((nx, ny))
                            q.append((nx, ny))
            
            if len(new_cluster) >= min_cluster_size:
                centroid = np.mean(new_cluster, axis=0)
                clusters.append({'centroid': tuple(np.int32(centroid)), 'points': new_cluster})

        return clusters

    def _is_exploration_truly_complete(self):
        """智能判断探索是否真正完成 - 基于SLAM和雷达扫描结果"""
        print("执行智能探索完成验证...")
        
        # 1. 检查是否找到了出口
        if self.exit_pose is not None:
            print("已找到出口，探索完成。")
            return True
        
        # 2. 检查雷达扫描是否显示前方被完全堵死
        if self._is_completely_blocked():
            print("雷达显示前方完全被堵死，可能已到达迷宫边界。")
            # 检查是否在迷宫边界附近
            if self._is_near_maze_boundary():
                print("确认在迷宫边界附近，探索完成。")
                return True
            else:
                print("不在边界附近，继续探索。")
                return False
        
        # 3. 检查地图的完整性 - 确保没有大的未知区域
        unknown_ratio = self._calculate_unknown_ratio()
        if unknown_ratio > 0.15:  # 如果未知区域超过15%，认为探索不充分
            print(f"未知区域过多: {unknown_ratio:.1%} > 15%，继续探索。")
            return False
        
        # 4. 检查是否在最近一段时间内没有发现新的前沿点
        if hasattr(self, 'last_frontier_discovery'):
            time_since_last_frontier = time.time() - self.last_frontier_discovery
            if time_since_last_frontier < 30:  # 如果30秒内还在发现前沿点，继续探索
                print(f"最近还在发现前沿点 ({time_since_last_frontier:.1f}秒前)，继续探索。")
                return False
        
        # 5. 检查探索度是否足够高
        if self.exploration_percentage < 0.85:  # 降低阈值到85%
            print(f"探索度不足: {self.exploration_percentage:.1%} < 85%，继续探索。")
            return False
        
        # 6. 新增：检查是否所有前沿点都不可达
        frontier_clusters = self._find_frontier_clusters()
        if frontier_clusters:
            # 检查是否所有前沿点都已被访问或不可达
            accessible_frontiers = 0
            for cluster in frontier_clusters:
                centroid = cluster['centroid']
                if centroid not in self.visited_frontiers:
                    # 简单检查：前沿点是否在合理距离内
                    robot_grid_pos = (int(self.x / self.map_resolution), int(self.y / self.map_resolution))
                    distance = math.sqrt((centroid[0] - robot_grid_pos[0])**2 + (centroid[1] - robot_grid_pos[1])**2)
                    if distance < 50:  # 50格内的前沿点
                        accessible_frontiers += 1
            
            if accessible_frontiers == 0:
                print("所有前沿点都不可达，探索完成。")
                return True
        
        print("探索验证通过！")
        return True
    
    def _is_completely_blocked(self):
        """检查雷达扫描是否显示前方完全被堵死"""
        if not self.recent_scans:
            return False
        
        # 获取最新的扫描数据
        latest_scan = self.recent_scans[-1]
        
        # 检查前方180度范围内的扫描结果
        front_angles = []
        for i, angle in enumerate(np.linspace(-np.pi/2, np.pi/2, 180)):
            scan_idx = int((angle + np.pi) / (2 * np.pi) * len(latest_scan))
            scan_idx = scan_idx % len(latest_scan)
            front_angles.append(latest_scan[scan_idx])
        
        # 如果前方大部分方向都被堵死（距离很近），认为被完全堵死
        blocked_count = sum(1 for dist in front_angles if dist < 0.5)
        blocked_ratio = blocked_count / len(front_angles)
        
        return blocked_ratio > 0.8  # 80%以上的方向被堵死
    
    def _is_near_maze_boundary(self):
        """检查是否在迷宫边界附近"""
        margin = 1.0  # 边界容差
        return (self.x < margin or self.x > self.env.max_x - margin or 
                self.y < margin or self.y > self.env.max_y - margin)
    
    def _calculate_unknown_ratio(self):
        """计算地图中未知区域的比例"""
        # 将log-odds转换为概率
        prob_map = 1 - 1 / (1 + np.exp(self.log_odds_map))
        # 未知区域：概率在0.3-0.7之间
        unknown_mask = (prob_map > 0.3) & (prob_map < 0.7)
        total_cells = prob_map.size
        unknown_cells = np.sum(unknown_mask)
        return unknown_cells / total_cells if total_cells > 0 else 0.0

    def _a_star_pathfinding(self, start_grid, goal_grid, costmap):
        """优化的A* 路径规划算法 (使用预计算的成本地图)"""
        # 添加最大搜索步数限制，防止无限搜索
        MAX_SEARCH_STEPS = 1000
        search_steps = 0
        
        q = queue.PriorityQueue()
        q.put((0, start_grid))
        
        came_from = {start_grid: None}
        cost_so_far = {start_grid: 0}

        while not q.empty() and search_steps < MAX_SEARCH_STEPS:
            _, current = q.get()
            search_steps += 1
            
            if current == goal_grid:
                break
            
            # 探索邻居 (8个方向)
            for dc, dr in [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]:
                neighbor = (current[0] + dc, current[1] + dr)
                
                # 边界检查
                if not (0 <= neighbor[0] < self.map_dim and 0 <= neighbor[1] < self.map_dim):
                    continue

                # 从成本地图中直接获取单元格成本
                cell_cost = costmap[neighbor[1], neighbor[0]]

                if cell_cost == float('inf'):
                    continue

                # 移动成本：直线为1, 对角线为sqrt(2)
                move_cost = 1.0 if dc == 0 or dr == 0 else math.sqrt(2)
                
                # 新的总成本 = 已有成本 + 移动成本 + 目标单元格成本
                new_cost = cost_so_far[current] + move_cost + cell_cost
                
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    # 启发函数：使用曼哈顿距离，计算更快
                    h = abs(goal_grid[0] - neighbor[0]) + abs(goal_grid[1] - neighbor[1])
                    priority = new_cost + h
                    q.put((priority, neighbor))
                    came_from[neighbor] = current
        else: # 如果循环正常结束 (没有break)，说明没找到路径
            return None, float('inf')
            
        # 重建路径
        path = []
        current = goal_grid
        while current is not None:
            path.append(current)
            current = came_from.get(current)
        path.reverse()

        path_cost = cost_so_far.get(goal_grid, float('inf'))
        
        # 确保路径有效
        return (path, path_cost) if path[0] == start_grid else (None, float('inf'))

    def explore_step(self, dt):
        """基于前沿探索的移动决策状态机 - 升级版"""
        # --- 顶层任务状态机 ---
        if self.mission_phase == "EXPLORING_MAZE":
            self._execute_exploration_phase(dt)
        elif self.mission_phase == "RETURNING_TO_START":
            self._execute_return_to_start(dt)
        elif self.mission_phase == "GOING_TO_EXIT":
            self._execute_go_to_exit(dt)
        elif self.mission_phase == "MISSION_COMPLETE":
            # 停止所有移动
            self._apply_movement(dt, 0.0, 0.0)

    def _execute_exploration_phase(self, dt):
        """执行迷宫探索阶段的子状态机"""
        forward_speed = 0.0
        turn_rate = 0.0

        # 触发器: 检查是否需要确认出口
        if self.scan_inefficiency > 0.6 and self.exit_pose is None and self.exploration_state != "CONFIRMING_EXIT":
            print("!!! 扫描无效率 > 60%，可能是出口，正在前往确认... !!!")
            self.exploration_state = "CONFIRMING_EXIT"
            self.exit_confirmation_target = (self.x + 1.0 * math.cos(self.theta), 
                                             self.y + 1.0 * math.sin(self.theta))
            self.current_path = []
            return # 立即返回，等待下一帧执行新状态

        # 子状态机
        try:
            if self.exploration_state == "CONFIRMING_EXIT":
                forward_speed, turn_rate = self._state_confirming_exit()
            
            elif self.exploration_state == "RETURNING_TO_MAZE":
                forward_speed, turn_rate = self._state_returning_to_maze()

            elif self.exploration_state == "FIND_TARGET":
                self._state_find_target() # 此状态负责设置路径，不直接控制移动
            
            elif self.exploration_state == "FOLLOW_PATH":
                forward_speed, turn_rate = self._state_follow_path()
            else:
                print(f"警告：未知的探索状态 {self.exploration_state}，重置为 FIND_TARGET")
                self.exploration_state = "FIND_TARGET"

        except Exception as e:
            print(f"探索阶段错误: {e}")
            print("重置探索状态...")
            self.exploration_state = "FIND_TARGET"
            self.current_path = []
            forward_speed, turn_rate = 0.0, 0.0

        # 将子状态的运动指令应用到主循环
        self._apply_movement(dt, forward_speed, turn_rate)
        
        # 更新移动时间（如果有实际移动）
        if abs(forward_speed) > 0.1 or abs(turn_rate) > 0.1:
            self.last_movement_time = time.time()

    def _state_confirming_exit(self):
        """状态: 确认出口"""
        if self.exit_confirmation_target is None:
            self.exploration_state = "FIND_TARGET"
            return 0.0, 0.0
        
        target_x, target_y = self.exit_confirmation_target
        dist_to_target = math.sqrt((target_x - self.x)**2 + (target_y - self.y)**2)

        if dist_to_target < 0.2:
            print("已到达确认点，进行最终检查...")
            if self.scan_inefficiency > 0.6:
                self.exit_pose = (self.x, self.y, self.theta)
                print(f"*** 出口已确认于 ({self.x:.2f}, {self.y:.2f})! ***")
                self.exploration_state = "RETURNING_TO_MAZE" # 确认后，进入返回状态
            else:
                print("确认失败，恢复正常探索。")
                self.exploration_state = "FIND_TARGET"
            self.exit_confirmation_target = None
            return 0.0, 0.0
        else:
            return self._go_to_goal_controller(target_x, target_y)

    def _state_returning_to_maze(self):
        """状态: 从出口返回迷宫内部"""
        if self.exit_pose is None: # 安全检查
            self.exploration_state = "FIND_TARGET"
            return 0.0, 0.0

        # 设定一个返回目标点，在出口内侧1.5米处
        exit_x, exit_y, exit_theta = self.exit_pose
        return_target_x = exit_x - 1.5 * math.cos(exit_theta)
        return_target_y = exit_y - 1.5 * math.sin(exit_theta)

        dist_to_return_target = math.sqrt((return_target_x - self.x)**2 + (return_target_y - self.y)**2)
        if dist_to_return_target < 0.3:
            print("已返回迷宫内部，继续探索...")
            self.exploration_state = "FIND_TARGET"
            return 0.0, 0.0
        else:
            # 使用Go-to-Goal控制器导航回去
            return self._go_to_goal_controller(return_target_x, return_target_y)

    def _state_find_target(self):
        """状态: 使用A*成本寻找最佳前沿点，避免重复访问"""
        print("寻找最佳前沿点 (基于A*路径成本，避免重复访问)...")
        frontier_clusters = self._find_frontier_clusters()

        if not frontier_clusters:
            # 更智能的探索完成判断
            if self._is_exploration_truly_complete():
                print("探索真正完成：已找到出口或所有区域已充分探索。")
                self.mission_phase = "RETURNING_TO_START"
            else:
                print("暂时找不到前沿点，但探索可能未完成，继续搜索...")
                # 重置一些状态，强制重新搜索
                self.visited_frontiers.clear()
                self.exploration_state = "FIND_TARGET"
            return
        
        # 更新前沿发现时间
        self.last_frontier_discovery = time.time()

        pathfinding_costmap = self._create_pathfinding_costmap()
        robot_grid_pos = (int(self.x / self.map_resolution), int(self.y / self.map_resolution))
        
        best_path = None
        min_cost = float('inf')
        best_cluster = None

        # 过滤掉已访问的前沿点
        available_clusters = []
        for cluster in frontier_clusters:
            centroid = cluster['centroid']
            if centroid not in self.visited_frontiers:
                # 检查是否最近访问过这个区域
                if not self._is_recently_visited(centroid):
                    # 额外检查：确保目标点距离当前位置足够远，避免原地打转
                    robot_grid_pos = (int(self.x / self.map_resolution), int(self.y / self.map_resolution))
                    distance_to_target = math.sqrt((centroid[0] - robot_grid_pos[0])**2 + (centroid[1] - robot_grid_pos[1])**2)
                    if distance_to_target > 3:  # 至少3格距离
                        available_clusters.append(cluster)

        if not available_clusters:
            # 更智能的探索完成判断
            if self._is_exploration_truly_complete():
                print("探索真正完成：已找到出口或所有区域已充分探索。")
                self.mission_phase = "RETURNING_TO_START"
            else:
                print("所有前沿点都已访问过，但探索可能未完成，重置访问历史继续探索...")
                # 清除部分访问历史，允许重新访问一些区域
                self.visited_frontiers.clear()
                # 清除一些旧的访问记录，允许重新探索
                if len(self.visited_positions) > 1000:
                    # 保留最近的一半访问记录
                    recent_positions = list(self.visited_positions)[-500:]
                    self.visited_positions = set(recent_positions)
                self.exploration_state = "FIND_TARGET"
            return

        for cluster in available_clusters:
            target_grid = cluster['centroid']
            path, cost = self._a_star_pathfinding(robot_grid_pos, target_grid, pathfinding_costmap)
            if path and cost < min_cost:
                min_cost = cost
                best_path = path
                best_cluster = cluster
                self.current_target = target_grid
        
        # 如果A*无法找到路径，尝试使用简化的路径规划
        if not best_path and available_clusters:
            print("A*无法找到路径，尝试使用简化路径规划...")
            best_cluster = self._find_closest_accessible_frontier(available_clusters, robot_grid_pos)
            if best_cluster:
                # 使用直线路径作为备用方案
                best_path = self._create_straight_line_path(robot_grid_pos, best_cluster['centroid'])
                min_cost = len(best_path) * 10  # 估算成本
                self.current_target = best_cluster['centroid']
        
        HIGH_COST_THRESHOLD = 5000 # 成本阈值，适应新的成本地图
        if min_cost > HIGH_COST_THRESHOLD:
            # 更智能的探索完成判断
            if self._is_exploration_truly_complete():
                print(f"所有剩余前沿点的最低成本 ({min_cost:.0f}) 过高，且探索真正完成。")
                self.mission_phase = "RETURNING_TO_START"
            else:
                print(f"所有剩余前沿点的最低成本 ({min_cost:.0f}) 过高，但探索可能未完成，尝试降低成本阈值...")
                # 降低成本阈值，允许访问一些高成本但可能重要的区域
                self.exploration_state = "FIND_TARGET"
                # 清除一些访问历史，允许重新探索
                self.visited_frontiers.clear()
        elif best_path and best_cluster:
            print(f"新目标: {self.current_target}, A*成本: {min_cost:.1f}, 路径长度: {len(best_path)}")
            # 标记这个前沿点为已访问
            self.visited_frontiers.add(self.current_target)
            self.current_path = best_path
            self.exploration_state = "FOLLOW_PATH"
            self.path_step = 1
        else:
            print("警告：找不到通往任何有效前沿点的路径。")
            # 更智能的探索完成判断
            if self._is_exploration_truly_complete():
                print("探索真正完成：已找到出口或所有区域已充分探索。")
                self.mission_phase = "RETURNING_TO_START"
            else:
                print("暂时无路可走，但探索可能未完成，尝试重置状态...")
                # 重置一些状态，强制重新搜索
                self.visited_frontiers.clear()
                self.exploration_state = "FIND_TARGET"

    def _is_recently_visited(self, centroid):
        """检查是否最近访问过这个区域"""
        cx, cy = centroid
        recent_radius = 3  # 检查最近3格范围内（减少检查范围）
        
        # 检查最近访问历史
        # 使用list()转换deque以支持切片操作
        recent_history = list(self.exploration_history)[-10:]  # 检查最近10个位置
        for visited_pos in recent_history:
            vx, vy = visited_pos[0], visited_pos[1]
            v_grid_x = int(vx / self.map_resolution)
            v_grid_y = int(vy / self.map_resolution)
            distance = np.sqrt((cx - v_grid_x)**2 + (cy - v_grid_y)**2)
            if distance < recent_radius:
                return True
        return False

    def _state_follow_path(self):
        """状态: 跟随A*规划的路径，带障碍物检测和回退"""
        if not self.current_path or self.path_step >= len(self.current_path):
            # 路径已完成或不存在，停止移动，让上层任务状态机决定下一步
            if self.mission_phase == "EXPLORING_MAZE":
                # 添加短暂延迟，避免频繁重新规划
                if not hasattr(self, '_path_completion_time'):
                    self._path_completion_time = time.time()
                elif time.time() - self._path_completion_time > 0.5:  # 0.5秒延迟
                    self.exploration_state = "FIND_TARGET"
                    delattr(self, '_path_completion_time')
            else:
                # 在任务阶段，清空路径以触发重新规划或完成任务
                self.current_path = []
            return 0.0, 0.0
        
        # 检查前方是否有障碍物
        if self._is_path_blocked():
            print("检测到路径被障碍物阻挡，执行回退...")
            return self._execute_obstacle_avoidance()
        
        next_waypoint_grid = self.current_path[self.path_step]
        target_x = (next_waypoint_grid[0] + 0.5) * self.map_resolution
        target_y = (next_waypoint_grid[1] + 0.5) * self.map_resolution
        
        dist_to_waypoint = math.sqrt((target_x - self.x)**2 + (target_y - self.y)**2)
        if dist_to_waypoint < self.map_resolution * 1.5:
            self.path_step += 1
            if self.path_step >= len(self.current_path):
                # 到达路径终点，返回0速度，等待下一轮决策
                return 0.0, 0.0
        
        return self._go_to_goal_controller(target_x, target_y)

    def _is_path_blocked(self):
        """检查前方路径是否被障碍物阻挡"""
        if not self.recent_scans:
            return False
        
        # 获取最新的扫描数据
        latest_scan = self.recent_scans[-1]
        
        # 检查前方60度范围内的扫描结果
        front_angles = []
        for i, angle in enumerate(np.linspace(-np.pi/3, np.pi/3, 60)):
            scan_idx = int((angle + np.pi) / (2 * np.pi) * len(latest_scan))
            scan_idx = scan_idx % len(latest_scan)
            front_angles.append(latest_scan[scan_idx])
        
        # 如果前方大部分方向都被堵死（距离很近），认为路径被阻挡
        blocked_count = sum(1 for dist in front_angles if dist < 0.3)
        blocked_ratio = blocked_count / len(front_angles)
        
        return blocked_ratio > 0.6  # 60%以上的方向被堵死
    
    def _execute_obstacle_avoidance(self):
        """执行障碍物避让 - 回退并重新规划路径"""
        print("执行障碍物避让：回退并重新规划路径...")
        
        # 1. 立即停止并回退
        forward_speed = -0.5  # 回退速度
        turn_rate = 0.0
        
        # 2. 清空当前路径，强制重新规划
        self.current_path = []
        self.path_step = 0
        
        # 3. 标记当前位置为障碍物区域，避免重复访问
        current_grid = (int(self.x / self.map_resolution), int(self.y / self.map_resolution))
        self.visited_positions.add(current_grid)
        
        # 4. 重置探索状态，寻找新的目标
        self.exploration_state = "FIND_TARGET"
        
        return forward_speed, turn_rate

    def _find_closest_accessible_frontier(self, clusters, robot_pos):
        """找到最近的可访问前沿点"""
        min_distance = float('inf')
        best_cluster = None
        
        for cluster in clusters:
            centroid = cluster['centroid']
            distance = math.sqrt((centroid[0] - robot_pos[0])**2 + (centroid[1] - robot_pos[1])**2)
            if distance < min_distance:
                min_distance = distance
                best_cluster = cluster
        
        return best_cluster
    
    def _create_straight_line_path(self, start_grid, goal_grid):
        """创建直线路径作为备用方案"""
        path = [start_grid]
        
        # 使用Bresenham算法创建直线路径
        x0, y0 = start_grid
        x1, y1 = goal_grid
        
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        
        err = dx - dy
        
        x, y = x0, y0
        
        while x != x1 or y != y1:
            if 2 * err > -dy:
                err -= dy
                x += sx
            if 2 * err < dx:
                err += dx
                y += sy
            
            # 检查边界
            if 0 <= x < self.map_dim and 0 <= y < self.map_dim:
                path.append((x, y))
            else:
                break
        
        return path

    def _go_to_goal_controller(self, target_x, target_y):
        """通用的Go-to-Goal控制器，返回(forward_speed, turn_rate)"""
        angle_to_target = math.atan2(target_y - self.y, target_x - self.x)
        angle_error = math.atan2(math.sin(angle_to_target - self.theta), math.cos(angle_to_target - self.theta))
        
        turn_rate = 0.0
        forward_speed = 0.0
        
        if abs(angle_error) > np.deg2rad(15):
            turn_rate = np.sign(angle_error) * self.angular_speed
        else:
            turn_rate = angle_error * 2.5
            forward_speed = self.linear_speed
            
        return forward_speed, turn_rate

    def _execute_path_following_task(self, goal_world_pos, on_finish_phase):
        """通用的任务路径跟随逻辑"""
        # 1. 检查是否已到达最终目标
        dist_to_goal = math.sqrt((goal_world_pos[0] - self.x)**2 + (goal_world_pos[1] - self.y)**2)
        if dist_to_goal < 0.3:
            print(f"已到达目标: {on_finish_phase}")
            self.mission_phase = on_finish_phase
            self.current_path = [] # 清空路径
            self._apply_movement(0.05, 0.0, 0.0) # 确保在状态切换时停止
            return

        # 2. 如果没有路径，则规划一条新路径
        if not self.current_path:
            print(f"正在规划前往 {on_finish_phase} 的路径...")
            goal_grid_pos = (int(goal_world_pos[0] / self.map_resolution),
                             int(goal_world_pos[1] / self.map_resolution))
            robot_grid_pos = (int(self.x / self.map_resolution), int(self.y / self.map_resolution))
            
            # 使用未惩罚的成本地图进行最终路径规划
            costmap = self._create_pathfinding_costmap()
            
            path, cost = self._a_star_pathfinding(robot_grid_pos, goal_grid_pos, costmap)
            
            if path and len(path) > 1:
                self.current_path = path
                self.path_step = 1
            else:
                print(f"警告: 无法规划到目标 {goal_world_pos} 的路径！任务中止。")
                self.mission_phase = "MISSION_COMPLETE" # 无法规划，任务失败
                self._apply_movement(0.05, 0.0, 0.0)
                return
        
        # 3. 如果有路径，则跟随路径
        forward_speed, turn_rate = self._state_follow_path()
        self._apply_movement(0.05, forward_speed, turn_rate) # 使用固定dt

    def _execute_return_to_start(self, dt):
        """任务: 返回起点"""
        if not hasattr(self, '_start_goal_printed'):
             print("\n--- 任务阶段: 返回起点 ---")
             self._start_goal_printed = True

        goal = self.env.start_point
        self._execute_path_following_task(goal, "GOING_TO_EXIT")
    
    def _execute_go_to_exit(self, dt):
        """任务: 前往出口"""
        if not hasattr(self, '_exit_goal_printed'):
            print("\n--- 任务阶段: 前往出口 ---")
            self._exit_goal_printed = True
        
        # 检查出口位置是否已设置
        if self.exit_pose is None:
            print("警告：出口位置未设置，任务完成。")
            self.mission_phase = "MISSION_COMPLETE"
            return
        
        goal = self.exit_pose[:2]
        self._execute_path_following_task(goal, "MISSION_COMPLETE")

    def _apply_movement(self, dt, forward_speed, turn_rate):
        """将速度和角速度应用到机器人上（无碰撞检测）"""
        turn_rate = np.clip(turn_rate, -self.angular_speed, self.angular_speed)
        dtheta = turn_rate * dt
        next_theta = self.theta + dtheta
        self.theta = math.atan2(math.sin(next_theta), math.cos(next_theta))
        
        dx = forward_speed * math.cos(self.theta) * dt
        dy = forward_speed * math.sin(self.theta) * dt
        self.x += dx
        self.y += dy
        
        # 记录访问位置和历史
        current_grid = (int(self.x / self.map_resolution), int(self.y / self.map_resolution))
        self.visited_positions.add(current_grid)
        self.exploration_history.append((self.x, self.y, self.theta))
        
        self.trajectory_x.append(self.x)
        self.trajectory_y.append(self.y)

def create_visualization():
    """创建可视化界面 - 学术展示风格"""
    # 设置学术风格的matplotlib参数
    plt.style.use('seaborn-v0_8-whitegrid')  # 使用学术风格
    plt.rcParams['font.family'] = 'serif'
    plt.rcParams['font.size'] = 10
    plt.rcParams['axes.grid'] = True
    plt.rcParams['grid.alpha'] = 0.3
    plt.rcParams['figure.facecolor'] = 'white'
    plt.rcParams['axes.facecolor'] = '#fafafa'  # 浅灰色背景
    
    fig = plt.figure(figsize=(16, 12))  # 调整为2x2布局的合适尺寸
    axes = [
        fig.add_subplot(2, 2, 1),                            # ax1: Real World
        fig.add_subplot(2, 2, 2),                            # ax2: SLAM Map  
        fig.add_subplot(2, 2, 3),                            # ax3: Occupancy Grid
        fig.add_subplot(2, 2, 4, projection='polar'),        # ax4: Lidar Scan
    ]
    
    # 使用默认matplotlib后端，避免后端冲突
    # 不强制设置后端，让matplotlib自动选择可用的后端
    
    plt.ion()
    
    # 添加键盘事件监听，支持强制终止
    def on_key_press(event):
        global simulation_running
        if event.key == 'q' or event.key == 'escape':
            print("\n检测到按键 'q' 或 'escape'，正在强制终止程序...")
            simulation_running = False
            plt.close('all')
        elif event.key == 'p':  # 添加暂停功能
            print("\n程序已暂停，按任意键继续...")
            input("按Enter键继续...")
    
    fig.canvas.mpl_connect('key_press_event', on_key_press)
    
    return fig, np.array(axes)

def update_visualization(axes, env, robot, slam, step_count, final=False):
    """更新可视化显示 - 学术展示风格"""
    axes = axes.flatten()
    for ax in axes:
        ax.clear()
        # 设置学术风格的子图背景
        ax.set_facecolor('#fafafa')
    
    # --- 子图1: 真实环境和里程计 ---
    ax1 = axes[0]
    for seg in env.segments:
        ax1.plot([seg["start"][0], seg["end"][0]], [seg["start"][1], seg["end"][1]], 'k-', linewidth=2.5, label='Maze Boundary')
    ax1.plot(robot.trajectory_x, robot.trajectory_y, 'b-', linewidth=2, alpha=0.8, label='Odometry Trajectory')
    ax1.plot(robot.x, robot.y, 'ro', markersize=10, label='True Position', markeredgecolor='darkred', markeredgewidth=1)
    
    # 绘制出口
    if robot.exit_pose:
        ax1.plot(robot.exit_pose[0], robot.exit_pose[1], 'm*', markersize=25, label='Confirmed Exit', markeredgecolor='darkmagenta', markeredgewidth=1)

    ax1.set_xlim(-1, env.max_x + 1)
    ax1.set_ylim(-1, env.max_y + 1)
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.4, linestyle='--')
    ax1.set_title('Real World and Odometry', fontsize=12, fontweight='bold')
    # 移除图例显示，避免复杂地图时破坏美感
    # ax1.legend(loc='upper right', framealpha=0.9)
    ax1.set_xlabel('X (m)', fontsize=10)
    ax1.set_ylabel('Y (m)', fontsize=10)
    
    # --- 子图2: SLAM重建地图 (实时) ---
    ax2 = axes[1]
    if final:
        map_points = slam.get_map_points() # 获取优化后的最终地图
        if map_points.shape[1] > 0:
            ax2.plot(map_points[0, :], map_points[1, :], 'k.', markersize=0.8, label='Final Map', alpha=0.7)
        if len(slam.nodes) > 0:
            optimized_poses = np.array(slam.nodes)
            ax2.plot(optimized_poses[:, 0], optimized_poses[:, 1], 'g-', linewidth=2.5, label='Optimized Trajectory')
    else:
        # 实时显示未优化的地图
        map_points = slam.get_map_points(use_optimized=False)
        if map_points.shape[1] > 0:
            ax2.plot(map_points[0, :], map_points[1, :], 'k.', markersize=0.8, alpha=0.7)
        if len(slam.nodes) > 0:
            unoptimized_poses = np.array([n for n in slam.nodes])
            ax2.plot(unoptimized_poses[:, 0], unoptimized_poses[:, 1], 'b-', alpha=0.8, linewidth=2, label='SLAM Trajectory')

    # 绘制出口
    if robot.exit_pose:
        ax2.plot(robot.exit_pose[0], robot.exit_pose[1], 'm*', markersize=25, label='Confirmed Exit', markeredgecolor='darkmagenta', markeredgewidth=1)

    ax2.set_xlim(-1, env.max_x + 1)
    ax2.set_ylim(-1, env.max_y + 1)
    ax2.set_aspect('equal')
    ax2.grid(True, alpha=0.4, linestyle='--')
    ax2.set_title('SLAM Reconstructed Map and Trajectory', fontsize=12, fontweight='bold')
    ax2.legend(loc='upper right', framealpha=0.9)
    ax2.set_xlabel('X (m)', fontsize=10)
    ax2.set_ylabel('Y (m)', fontsize=10)
    
    # --- 子图3: 机器人感知的地图 ---
    ax3 = axes[2]
    # 将log-odds转换为概率值 (0-1) 以便显示
    prob_map = 1 - 1 / (1 + np.exp(robot.log_odds_map))
    im = ax3.imshow(prob_map, cmap='gray_r', origin='lower', 
               extent=[0, robot.map_size_m, 0, robot.map_size_m], alpha=0.8)
    
    # 在地图上绘制机器人当前位置
    if len(slam.nodes) > 0:
        current_pose = slam.nodes[-1]
        ax3.plot(current_pose[0], current_pose[1], 'ro', markersize=8, label='Current Pose', markeredgecolor='darkred', markeredgewidth=1)

    # 绘制出口
    if robot.exit_pose:
        ax3.plot(robot.exit_pose[0], robot.exit_pose[1], 'm*', markersize=20, label='Confirmed Exit', markeredgecolor='darkmagenta', markeredgewidth=1)

    # 绘制当前规划的路径
    if robot.current_path:
        path_grid = np.array(robot.current_path)
        path_x = (path_grid[:, 0] + 0.5) * robot.map_resolution
        path_y = (path_grid[:, 1] + 0.5) * robot.map_resolution
        ax3.plot(path_x, path_y, 'm-', linewidth=2.5, label='Current Path', alpha=0.9)

    # 绘制前沿簇
    if robot.exploration_state == "FIND_TARGET":
        clusters = robot._find_frontier_clusters()
        for i, cluster in enumerate(clusters):
            points = np.array(cluster['points'])
            ax3.plot(points[:, 0]*robot.map_resolution, points[:, 1]*robot.map_resolution, 'c.', markersize=1.5, alpha=0.6)
            centroid = cluster['centroid']
            ax3.plot(centroid[0]*robot.map_resolution, centroid[1]*robot.map_resolution, 'yx', markersize=8, label='Frontier Center' if i==0 else "", markeredgecolor='orange', markeredgewidth=1)

    ax3.set_xlim(-1, env.max_x + 1)
    ax3.set_ylim(-1, env.max_y + 1)
    ax3.set_aspect('equal')
    ax3.grid(True, alpha=0.4, linestyle='--')
    ax3.set_title(f"Mission: {robot.mission_phase} | State: {robot.exploration_state}\n(Exploration: {robot.exploration_percentage:.1%})", fontsize=12, fontweight='bold')
    ax3.legend(loc='upper right', framealpha=0.9)
    ax3.set_xlabel('X (m)', fontsize=10)
    ax3.set_ylabel('Y (m)', fontsize=10)
    
    # --- 子图4: 实时雷达扫描 ---
    ax4 = axes[3]
    ax4.set_theta_zero_location('N') # 0度在正上方
    ax4.set_theta_direction(-1) # 顺时针
    ax4.set_rlim(0, 4.0) # 雷达最大距离
    ax4.set_title('Real-time Lidar Scan (360°)', fontsize=12, fontweight='bold', pad=20)
    
    if robot.recent_scans:
        num_scans = len(robot.recent_scans)
        for i, scan in enumerate(robot.recent_scans):
            alpha = (i + 1) / (num_scans * 1.5) # 越新的越不透明
            color = '#1f77b4'  # 使用更专业的蓝色
            if i == num_scans - 1: # 最新的一次
                alpha = 1.0
                color = '#d62728'  # 使用红色突出最新扫描
            
            angles = np.linspace(0, 2*np.pi, len(scan), endpoint=False)
            distances_m = np.array(scan) / 1000.0
            ax4.plot(angles, distances_m, '.', markersize=3, alpha=alpha, color=color)
    
    # 添加距离环
    for r in [1, 2, 3, 4]:
        ax4.plot(np.linspace(0, 2*np.pi, 100), [r]*100, 'k--', alpha=0.3, linewidth=0.5)
    
    ax4.grid(True, alpha=0.4)
    ax4.set_xlabel('Distance (m)', fontsize=10)

    plt.tight_layout(pad=2.0)  # 增加子图间距
    plt.pause(0.01)  # 增加暂停时间，减少CPU占用

def main(map_file):
    """主函数"""
    global simulation_running # 允许在函数内修改全局变量
    simulation_running = True # 重置标志，以防多次运行

    print("=== 迷宫SLAM仿真程序 ===")
    print("使用位姿图优化进行实时地图重建")
    print("机器人将自主探索迷宫...")
    
    # 初始化环境和机器人

    env = MazeEnvironment(map_file)
    robot = RobotController(env.start_point[0], env.start_point[1], env=env)
    
    # 初始化SLAM
    slam = PoseGraphSLAM()
    
    # 添加第一个节点 (起始位置)
    initial_pose = (robot.x, robot.y, robot.theta)
    initial_scan = env.simulate_lidar(robot.x, robot.y, robot.theta)
    initial_points = scan_to_points(initial_scan)
    slam.add_node(initial_pose, initial_points)
    
    # 使用初始扫描更新一次地图，以产生第一批前沿点
    print("使用初始扫描更新地图...")
    robot.update_occupancy_grid(initial_pose, initial_scan)
    robot.recent_scans.append(initial_scan) # 保存第一次扫描
    
    # 初始化可视化
    fig, axes = create_visualization()
    fig.canvas.mpl_connect('close_event', on_window_close) # 绑定关闭事件
    
    # 仿真参数 - 优化后的参数
    step_count = 0
    last_time = time.time()
    last_odom_pose = np.array(initial_pose)
    
    max_steps = 25000
    KEYFRAME_INTERVAL_STEPS = 100 # 每100步创建一个关键帧（减少频率）
    LOOP_CLOSURE_SEARCH_RADIUS = 2.0 # 米
    ICP_MAX_ERROR = 0.5
    VISUALIZATION_INTERVAL = 50 # 每50步更新一次可视化（大幅减少频率）

    # 定义约束的信息矩阵 (协方差的逆)
    odom_info = np.linalg.inv(np.diag([0.1**2, 0.1**2, np.deg2rad(5.0)**2]))
    loop_info = np.linalg.inv(np.diag([0.02**2, 0.02**2, np.deg2rad(1.0)**2]))

    # --- 新增：探索完成阈值 ---
    EXPLORATION_FINISH_THRESHOLD = 0.98 # 提高阈值，让探索更充分

    print("开始仿真... 关闭窗口或按 'q'/'escape' 键可停止")
    print("提示：如果程序无响应，请按 Ctrl+C 强制终止")
    
    # 性能监控变量
    last_performance_check = time.time()
    performance_counter = 0
    last_activity_time = time.time()  # 记录最后活动时间
    max_idle_time = 300  # 最大空闲时间（5分钟）
    
    try:
        while step_count < max_steps and robot.mission_phase != "MISSION_COMPLETE" and simulation_running:
            current_time = time.time()
            dt = current_time - last_time
            
            if dt >= 0.02: # 提高决策频率，减少延迟
                # 机器人自主探索
                robot.explore_step(dt)
                
                step_count += 1
                last_time = current_time
                performance_counter += 1
                last_activity_time = current_time  # 更新活动时间

                # 每1000步输出性能信息
                if performance_counter % 1000 == 0:
                    elapsed = current_time - last_performance_check
                    fps = 1000 / elapsed if elapsed > 0 else 0
                    print(f"性能监控: 步数={step_count}, FPS={fps:.1f}, 探索度={robot.exploration_percentage:.1%}")
                    last_performance_check = current_time
                    
                # 检查是否长时间无活动
                if current_time - last_activity_time > max_idle_time:
                    print(f"\n警告：程序已空闲超过{max_idle_time}秒，可能存在死锁。")
                    print("建议按 'q' 键或关闭窗口终止程序。")
                    last_activity_time = current_time  # 重置计时器

                # 手动检查探索是否完成(作为备用方案)
                if robot.mission_phase == "EXPLORING_MAZE" and robot.exploration_percentage > EXPLORATION_FINISH_THRESHOLD:
                    print(f"\n探索度达到 {robot.exploration_percentage:.1%}, 超过阈值 {EXPLORATION_FINISH_THRESHOLD:.1%}.")
                    print("强制进入下一阶段：返回起点。")
                    robot.mission_phase = "RETURNING_TO_START"
                
                # 优化：大幅减少扫描无效率计算频率
                if step_count % 20 == 0:  # 每20步计算一次，大幅减少计算负载
                    try:
                        scan_for_metric = env.simulate_lidar(robot.x, robot.y, robot.theta)
                        inefficient_rays = np.sum(np.array(scan_for_metric) >= 3990)
                        total_rays = len(scan_for_metric)
                        robot.scan_inefficiency = inefficient_rays / total_rays if total_rays > 0 else 0
                        robot.inefficiency_history.append(robot.scan_inefficiency)
                        robot.recent_scans.append(scan_for_metric)
                    except Exception as e:
                        print(f"扫描计算错误: {e}")
                
                if step_count % KEYFRAME_INTERVAL_STEPS == 0:
                    print(f"\n--- 步数: {step_count}, 添加关键帧 ---")
                    try:
                        # 1. 添加新节点 (使用当前的真实位置作为里程计读数)
                        current_pose = np.array([robot.x, robot.y, robot.theta])
                        current_scan = env.simulate_lidar(robot.x, robot.y, robot.theta)
                        current_points = scan_to_points(current_scan)
                        current_node_id = slam.add_node(current_pose, current_points)

                        # 更新机器人的占据栅格地图
                        robot.update_occupancy_grid(current_pose, current_scan)
                        robot.recent_scans.append(current_scan)

                        # 2. 添加里程计约束
                        dx = current_pose[0] - last_odom_pose[0]
                        dy = current_pose[1] - last_odom_pose[1]
                        dtheta = current_pose[2] - last_odom_pose[2]
                        # 将dx, dy转换到上一个关键帧的坐标系下
                        prev_theta = last_odom_pose[2]
                        odom_dx_local = dx * math.cos(-prev_theta) - dy * math.sin(-prev_theta)
                        odom_dy_local = dx * math.sin(-prev_theta) + dy * math.cos(-prev_theta)
                        
                        odom_measurement = np.array([odom_dx_local, odom_dy_local, dtheta])
                        slam.add_edge(current_node_id - 1, current_node_id, odom_measurement, odom_info)
                        last_odom_pose = current_pose
                        
                        # 3. 优化的回环检测 - 大幅限制搜索范围
                        max_loop_closure_nodes = min(10, len(slam.nodes) - 1)  # 最多检查10个最近的节点（减少一半）
                        for old_node_id in range(max(0, len(slam.nodes) - max_loop_closure_nodes), len(slam.nodes) - 1):
                            old_pose = slam.nodes[old_node_id]
                            
                            # 不与最近的几个节点进行匹配，避免错误的短期回环
                            if abs(current_node_id - old_node_id) < 10:
                                continue
                            
                            dist = np.linalg.norm(current_pose[:2] - np.array(old_pose[:2]))
                            
                            if dist < LOOP_CLOSURE_SEARCH_RADIUS:
                                print(f"发现邻近节点: {old_node_id} 和 {current_node_id}, 距离: {dist:.2f}m")
                                old_points = slam.keyframes[old_node_id]
                                
                                R, T, error = _icp_matching(old_points, current_points)
                                
                                print(f"  ICP匹配误差: {error:.4f}")
                                if error < ICP_MAX_ERROR:
                                    print(f"  >>> 回环检测成功！ {old_node_id} <--> {current_node_id} <<<")
                                    loop_measurement = np.array([T[0], T[1], math.atan2(R[1, 0], R[0, 0])])
                                    slam.add_edge(old_node_id, current_node_id, loop_measurement, loop_info)
                    except Exception as e:
                        print(f"关键帧处理错误: {e}")
                    
                if step_count % VISUALIZATION_INTERVAL == 0:
                    try:
                        update_visualization(axes, env, robot, slam, step_count)
                    except Exception as e:
                        print(f"可视化更新错误: {e}")
            
            time.sleep(0.005)  # 减少睡眠时间，提高响应速度
            
    except KeyboardInterrupt:
        print("\n仿真被用户中断")
    
    # 最终检查，如果因为步数耗尽而停止，但有出口，也标记为完成
    if robot.exit_pose and robot.mission_phase != "MISSION_COMPLETE":
        print("\n步数耗尽，但已找到出口。任务结束。")
        robot.mission_phase = "MISSION_COMPLETE"

    print(f"\n任务完成！总步数: {step_count}")
    
    if simulation_running: # 仅在仿真正常完成时执行优化和显示最终结果
        # 最后执行一次全局优化
        print("正在执行最终的全局优化...")
        slam.optimize_graph()
        
        # 显示最终结果
        print("仿真结束。您可以查看最终结果。关闭绘图窗口即可退出。")
        update_visualization(axes, env, robot, slam, step_count, final=True) 

        # 新增：保存最终结果图像
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            script_dir = os.path.dirname(os.path.abspath(__file__))
            results_dir = os.path.join(script_dir, "results")
            os.makedirs(results_dir, exist_ok=True)
            outfile = os.path.join(results_dir, f"slam_result_{timestamp}.png")
            fig.savefig(outfile, dpi=200, bbox_inches="tight")
            print(f"最终结果图已保存: {outfile}")
        except Exception as e:
            print(f"保存结果图失败: {e}")

        plt.ioff()
        plt.show() # 阻塞，直到用户关闭窗口
    else:
        print("仿真被用户提前终止。")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", type=str, default="map_information.json", help="地图文件路径")
    args = parser.parse_args()
    main(args.map)
