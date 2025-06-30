#!/usr/bin/env python3
'''
navigation.py - 导航模块，实现边界检测和路径规划算法

该模块负责实现边界检测算法和路径规划算法，用于自主导航和路径规划。
'''

import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import binary_dilation, generate_binary_structure

class FrontierExploration:
    """边界探索类，实现边界检测算法"""
    
    def __init__(self, map_size_pixels=800, map_size_meters=32):
        """
        初始化边界探索
        
        参数:
            map_size_pixels: 地图大小(像素)
            map_size_meters: 地图大小(米)
        """
        self.map_size_pixels = map_size_pixels
        self.map_size_meters = map_size_meters
        self.cell_size = map_size_meters / map_size_pixels
        
        # 边界检测参数
        self.free_thresh = 50  # 自由空间阈值，小于此值被认为是自由空间
        self.unknown_thresh = 128  # 未知空间阈值，接近此值被认为是未知空间
        self.frontier_min_size = 5  # 最小边界尺寸（像素）
    
    def detect_frontiers(self, occupancy_grid):
        """
        检测地图中的边界
        
        参数:
            occupancy_grid: 占用栅格地图，numpy数组
            
        返回:
            frontiers: 边界点列表，每个边界点是(x, y)坐标，单位为米
        """
        # 确保地图是numpy数组
        if not isinstance(occupancy_grid, np.ndarray):
            occupancy_grid = np.array(occupancy_grid)
        
        # 创建自由空间和未知空间的二值图
        free_space = occupancy_grid < self.free_thresh
        unknown_space = np.abs(occupancy_grid - self.unknown_thresh) < 10
        
        # 对自由空间进行膨胀，以确保边界点在自由空间附近
        struct = generate_binary_structure(2, 2)
        dilated_free_space = binary_dilation(free_space, structure=struct, iterations=1)
        
        # 边界是膨胀后的自由空间与未知空间的交集
        frontiers_map = dilated_free_space & unknown_space
        
        # 对边界进行连通区域分析
        from scipy import ndimage
        labeled_frontiers, num_features = ndimage.label(frontiers_map)
        
        # 过滤小的边界区域
        frontiers = []
        for i in range(1, num_features + 1):
            frontier_cells = np.where(labeled_frontiers == i)
            if len(frontier_cells[0]) >= self.frontier_min_size:
                # 计算边界中心
                center_y = np.mean(frontier_cells[0])
                center_x = np.mean(frontier_cells[1])
                
                # 转换为世界坐标（米）
                center_x_m = center_x * self.cell_size
                center_y_m = center_y * self.cell_size
                
                frontiers.append((center_x_m, center_y_m))
        
        return frontiers
    
    def select_best_frontier(self, frontiers, current_pose):
        """
        从边界列表中选择最佳边界点
        
        参数:
            frontiers: 边界点列表，每个边界点是(x, y)坐标，单位为米
            current_pose: 当前位置，(x, y, theta)，单位为米和度
            
        返回:
            best_frontier: 最佳边界点，(x, y)坐标，单位为米
        """
        if not frontiers:
            return None
        
        # 获取当前位置
        current_x, current_y, _ = current_pose
        
        # 计算到每个边界点的距离
        distances = []
        for frontier in frontiers:
            fx, fy = frontier
            distance = np.sqrt((fx - current_x)**2 + (fy - current_y)**2)
            distances.append(distance)
        
        # 选择最近的边界点
        best_idx = np.argmin(distances)
        return frontiers[best_idx]

class PathPlanner:
    """路径规划类，实现A*和DWA等路径规划算法"""
    
    def __init__(self, map_size_pixels=800, map_size_meters=32):
        """
        初始化路径规划器
        
        参数:
            map_size_pixels: 地图大小(像素)
            map_size_meters: 地图大小(米)
        """
        self.map_size_pixels = map_size_pixels
        self.map_size_meters = map_size_meters
        self.cell_size = map_size_meters / map_size_pixels
        
        # A*算法参数
        self.obstacle_thresh = 80  # 障碍物阈值，大于此值被认为是障碍物
        self.movement_cost = 1.0  # 基本移动成本
        self.diagonal_cost = 1.414  # 对角线移动成本
        
        # 定义8个方向的移动
        self.movements = [
            (-1, 0),  # 上
            (1, 0),   # 下
            (0, -1),  # 左
            (0, 1),   # 右
            (-1, -1), # 左上
            (-1, 1),  # 右上
            (1, -1),  # 左下
            (1, 1)    # 右下
        ]
        
        self.movement_costs = [
            self.movement_cost,  # 上
            self.movement_cost,  # 下
            self.movement_cost,  # 左
            self.movement_cost,  # 右
            self.diagonal_cost,  # 左上
            self.diagonal_cost,  # 右上
            self.diagonal_cost,  # 左下
            self.diagonal_cost   # 右下
        ]
    
    def plan_path(self, occupancy_grid, start_pose, goal_position):
        """
        使用A*算法规划路径
        
        参数:
            occupancy_grid: 占用栅格地图，numpy数组
            start_pose: 起始位置，(x, y, theta)，单位为米和度
            goal_position: 目标位置，(x, y)，单位为米
            
        返回:
            path: 路径点列表，每个路径点是(x, y)坐标，单位为米
        """
        # 确保地图是numpy数组
        if not isinstance(occupancy_grid, np.ndarray):
            occupancy_grid = np.array(occupancy_grid)
        
        # 将起始位置和目标位置转换为栅格坐标
        start_x, start_y, _ = start_pose
        goal_x, goal_y = goal_position
        
        start_cell_x = int(start_x / self.cell_size)
        start_cell_y = int(start_y / self.cell_size)
        goal_cell_x = int(goal_x / self.cell_size)
        goal_cell_y = int(goal_y / self.cell_size)
        
        # 确保坐标在地图范围内
        start_cell_x = max(0, min(start_cell_x, self.map_size_pixels - 1))
        start_cell_y = max(0, min(start_cell_y, self.map_size_pixels - 1))
        goal_cell_x = max(0, min(goal_cell_x, self.map_size_pixels - 1))
        goal_cell_y = max(0, min(goal_cell_y, self.map_size_pixels - 1))
        
        # 如果起点或终点是障碍物，返回空路径
        if (occupancy_grid[start_cell_y, start_cell_x] > self.obstacle_thresh or 
            occupancy_grid[goal_cell_y, goal_cell_x] > self.obstacle_thresh):
            return []
        
        # 使用A*算法规划路径
        path_cells = self._astar(occupancy_grid, (start_cell_y, start_cell_x), (goal_cell_y, goal_cell_x))
        
        if not path_cells:
            return []
        
        # 将路径点转换为世界坐标（米）
        path = []
        for cell_y, cell_x in path_cells:
            x_m = cell_x * self.cell_size
            y_m = cell_y * self.cell_size
            path.append((x_m, y_m))
        
        # 路径平滑
        path = self._smooth_path(path)
        
        return path
    
    def _astar(self, grid, start, goal):
        """
        A*算法实现
        
        参数:
            grid: 占用栅格地图
            start: 起始位置，(y, x)坐标
            goal: 目标位置，(y, x)坐标
            
        返回:
            path: 路径点列表，每个路径点是(y, x)坐标
        """
        import heapq
        
        # 启发式函数：曼哈顿距离
        def heuristic(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])
        
        # 检查位置是否有效
        def is_valid(y, x):
            return (0 <= y < grid.shape[0] and 
                    0 <= x < grid.shape[1] and 
                    grid[y, x] < self.obstacle_thresh)
        
        # 初始化开放列表和关闭列表
        open_set = []
        closed_set = set()
        
        # 使用优先队列实现开放列表
        heapq.heappush(open_set, (0, start))
        
        # 记录从起点到每个点的成本和父节点
        g_score = {start: 0}
        came_from = {}
        
        while open_set:
            # 获取当前成本最低的节点
            _, current = heapq.heappop(open_set)
            
            # 如果到达目标，重建路径
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path
            
            # 将当前节点添加到关闭列表
            closed_set.add(current)
            
            # 检查所有可能的移动
            for i, (dy, dx) in enumerate(self.movements):
                neighbor = (current[0] + dy, current[1] + dx)
                
                # 如果邻居无效或已在关闭列表中，跳过
                if not is_valid(neighbor[0], neighbor[1]) or neighbor in closed_set:
                    continue
                
                # 计算到邻居的成本
                tentative_g_score = g_score[current] + self.movement_costs[i]
                
                # 如果邻居不在开放列表中，或者找到了更好的路径
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    # 更新邻居信息
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score = tentative_g_score + heuristic(neighbor, goal)
                    
                    # 将邻居添加到开放列表
                    heapq.heappush(open_set, (f_score, neighbor))
        
        # 如果无法到达目标，返回空路径
        return []
    
    def _smooth_path(self, path, weight_data=0.5, weight_smooth=0.1, tolerance=0.000001):
        """
        平滑路径
        
        参数:
            path: 路径点列表，每个路径点是(x, y)坐标
            weight_data: 数据权重
            weight_smooth: 平滑权重
            tolerance: 收敛容差
            
        返回:
            smoothed_path: 平滑后的路径点列表
        """
        if len(path) <= 2:
            return path
        
        # 创建路径的副本
        smoothed_path = np.array(path, dtype=np.float64)
        
        # 迭代平滑
        change = tolerance
        while change >= tolerance:
            change = 0.0
            
            # 对每个内部点进行平滑
            for i in range(1, len(smoothed_path) - 1):
                for j in range(2):  # x和y坐标
                    old_val = smoothed_path[i][j]
                    
                    # 应用平滑公式
                    smoothed_path[i][j] += weight_data * (path[i][j] - smoothed_path[i][j])
                    smoothed_path[i][j] += weight_smooth * (smoothed_path[i-1][j] + smoothed_path[i+1][j] - 2.0 * smoothed_path[i][j])
                    
                    change += abs(old_val - smoothed_path[i][j])
        
        return smoothed_path.tolist()
    
    def visualize_path(self, occupancy_grid, path, start_pose, goal_position):
        """
        可视化路径
        
        参数:
            occupancy_grid: 占用栅格地图
            path: 路径点列表，每个路径点是(x, y)坐标，单位为米
            start_pose: 起始位置，(x, y, theta)，单位为米和度
            goal_position: 目标位置，(x, y)，单位为米
        """
        plt.figure(figsize=(10, 10))
        
        # 显示地图
        plt.imshow(occupancy_grid, cmap='gray')
        
        # 将路径点转换为栅格坐标
        path_cells = []
        for x, y in path:
            cell_x = int(x / self.cell_size)
            cell_y = int(y / self.cell_size)
            path_cells.append((cell_x, cell_y))
        
        # 绘制路径
        if path_cells:
            x_vals = [p[0] for p in path_cells]
            y_vals = [p[1] for p in path_cells]
            plt.plot(x_vals, y_vals, 'g-', linewidth=2)
        
        # 标记起点和终点
        start_x, start_y, _ = start_pose
        goal_x, goal_y = goal_position
        
        start_cell_x = int(start_x / self.cell_size)
        start_cell_y = int(start_y / self.cell_size)
        goal_cell_x = int(goal_x / self.cell_size)
        goal_cell_y = int(goal_y / self.cell_size)
        
        plt.plot(start_cell_x, start_cell_y, 'bo', markersize=10)
        plt.plot(goal_cell_x, goal_cell_y, 'ro', markersize=10)
        
        plt.title('路径规划')
        plt.xlabel('X (像素)')
        plt.ylabel('Y (像素)')
        plt.grid(True)
        plt.show() 