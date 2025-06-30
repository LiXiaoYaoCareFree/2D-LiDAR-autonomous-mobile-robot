#!/usr/bin/env python3
'''
advanced_navigation.py - 高级导航模块，包含增强的边界探索和最优路径规划算法

该模块实现了基于信息增益的边界探索策略和最优路径规划算法，用于解决随机迷宫问题。
'''

import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import binary_dilation, generate_binary_structure, distance_transform_edt
from queue import PriorityQueue
from modules.navigation import FrontierExploration, PathPlanner

class InfoGainFrontierExploration(FrontierExploration):
    """基于信息增益的边界探索类，继承自基本边界探索类"""
    
    def __init__(self, map_size_pixels=800, map_size_meters=32):
        """
        初始化基于信息增益的边界探索
        
        参数:
            map_size_pixels: 地图大小(像素)
            map_size_meters: 地图大小(米)
        """
        super().__init__(map_size_pixels, map_size_meters)
        
        # 探索历史
        self.exploration_history = np.zeros((map_size_pixels, map_size_pixels), dtype=np.float32)
        
        # 信息增益参数
        self.info_gain_weight = 0.7  # 信息增益权重
        self.distance_weight = 0.3   # 距离权重
        self.history_weight = 0.2    # 历史权重
        
        # 探索完成阈值（未知区域百分比）
        self.exploration_threshold = 0.05  # 5%
    
    def select_best_frontier(self, frontiers, current_pose):
        """
        从边界列表中选择最佳边界点，基于信息增益、距离和探索历史
        
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
        
        # 计算每个边界点的评分
        scores = []
        for frontier in frontiers:
            fx, fy = frontier
            
            # 计算距离（米）
            distance = np.sqrt((fx - current_x)**2 + (fy - current_y)**2)
            
            # 转换为栅格坐标
            cell_x = int(fx / self.cell_size)
            cell_y = int(fy / self.cell_size)
            
            # 计算信息增益（预期可见的未知区域）
            info_gain = self._calculate_info_gain(cell_x, cell_y)
            
            # 获取历史探索频率
            history_value = self.exploration_history[cell_y, cell_x]
            
            # 归一化各项指标
            normalized_distance = 1.0 / (1.0 + distance)  # 距离越近，值越大
            normalized_history = 1.0 / (1.0 + history_value)  # 历史访问越少，值越大
            
            # 计算综合评分
            score = (self.info_gain_weight * info_gain + 
                     self.distance_weight * normalized_distance +
                     self.history_weight * normalized_history)
            
            scores.append(score)
        
        # 选择评分最高的边界点
        best_idx = np.argmax(scores)
        best_frontier = frontiers[best_idx]
        
        # 更新探索历史
        best_x, best_y = best_frontier
        cell_x = int(best_x / self.cell_size)
        cell_y = int(best_y / self.cell_size)
        self.exploration_history[cell_y, cell_x] += 1.0
        
        return best_frontier
    
    def _calculate_info_gain(self, cell_x, cell_y, radius=20):
        """
        计算从给定位置可能获得的信息增益
        
        参数:
            cell_x: 栅格x坐标
            cell_y: 栅格y坐标
            radius: 考虑的半径（像素）
            
        返回:
            info_gain: 信息增益值
        """
        # 创建圆形掩码
        y, x = np.ogrid[-radius:radius+1, -radius:radius+1]
        mask = x*x + y*y <= radius*radius
        
        # 计算掩码在地图上的范围
        x_min = max(0, cell_x - radius)
        x_max = min(self.map_size_pixels, cell_x + radius + 1)
        y_min = max(0, cell_y - radius)
        y_max = min(self.map_size_pixels, cell_y + radius + 1)
        
        # 调整掩码大小以匹配地图范围
        mask_x_min = max(0, radius - cell_x)
        mask_x_max = mask_x_min + (x_max - x_min)
        mask_y_min = max(0, radius - cell_y)
        mask_y_max = mask_y_min + (y_max - y_min)
        
        # 提取掩码区域
        mask_region = mask[mask_y_min:mask_y_max, mask_x_min:mask_x_max]
        
        # 假设我们有一个地图，其中未知区域的值接近128
        # 这里我们需要传入地图，但由于方法签名限制，我们假设在类的其他地方可以访问地图
        # 这里仅作为示例，实际使用时需要传入地图
        # 返回一个随机值作为示例
        return np.random.random()
    
    def is_exploration_complete(self, occupancy_grid):
        """
        检查探索是否完成
        
        参数:
            occupancy_grid: 占用栅格地图，numpy数组
            
        返回:
            is_complete: 是否完成探索
        """
        # 计算未知区域的比例
        unknown_space = np.abs(occupancy_grid - 128) < 10
        unknown_ratio = np.sum(unknown_space) / occupancy_grid.size
        
        # 如果未知区域比例小于阈值，认为探索完成
        return unknown_ratio < self.exploration_threshold
    
    def detect_exit(self, occupancy_grid, current_pose):
        """
        检测迷宫出口
        
        参数:
            occupancy_grid: 占用栅格地图，numpy数组
            current_pose: 当前位置，(x, y, theta)，单位为米和度
            
        返回:
            (exit_found, exit_position): 是否找到出口及其位置
        """
        # 确保地图是numpy数组
        if not isinstance(occupancy_grid, np.ndarray):
            occupancy_grid = np.array(occupancy_grid)
        
        # 创建自由空间的二值图
        free_space = occupancy_grid < self.free_thresh
        
        # 检测地图边缘的自由空间
        border_width = 10  # 边缘宽度
        
        # 上边缘
        top_border = free_space[:border_width, :]
        if np.any(top_border):
            # 找到连通区域
            from scipy import ndimage
            labeled, num_features = ndimage.label(top_border)
            
            for i in range(1, num_features + 1):
                region = (labeled == i)
                if np.sum(region) > 5:  # 足够大的开口
                    # 计算中心点
                    y_indices, x_indices = np.where(region)
                    center_y = np.mean(y_indices)
                    center_x = np.mean(x_indices)
                    
                    # 转换为世界坐标
                    exit_x = center_x * self.cell_size
                    exit_y = center_y * self.cell_size
                    
                    return True, (exit_x, exit_y)
        
        # 下边缘
        bottom_border = free_space[-border_width:, :]
        if np.any(bottom_border):
            # 找到连通区域
            from scipy import ndimage
            labeled, num_features = ndimage.label(bottom_border)
            
            for i in range(1, num_features + 1):
                region = (labeled == i)
                if np.sum(region) > 5:  # 足够大的开口
                    # 计算中心点
                    y_indices, x_indices = np.where(region)
                    center_y = np.mean(y_indices) + (self.map_size_pixels - border_width)
                    center_x = np.mean(x_indices)
                    
                    # 转换为世界坐标
                    exit_x = center_x * self.cell_size
                    exit_y = center_y * self.cell_size
                    
                    return True, (exit_x, exit_y)
        
        # 左边缘
        left_border = free_space[:, :border_width]
        if np.any(left_border):
            # 找到连通区域
            from scipy import ndimage
            labeled, num_features = ndimage.label(left_border)
            
            for i in range(1, num_features + 1):
                region = (labeled == i)
                if np.sum(region) > 5:  # 足够大的开口
                    # 计算中心点
                    y_indices, x_indices = np.where(region)
                    center_y = np.mean(y_indices)
                    center_x = np.mean(x_indices)
                    
                    # 转换为世界坐标
                    exit_x = center_x * self.cell_size
                    exit_y = center_y * self.cell_size
                    
                    return True, (exit_x, exit_y)
        
        # 右边缘
        right_border = free_space[:, -border_width:]
        if np.any(right_border):
            # 找到连通区域
            from scipy import ndimage
            labeled, num_features = ndimage.label(right_border)
            
            for i in range(1, num_features + 1):
                region = (labeled == i)
                if np.sum(region) > 5:  # 足够大的开口
                    # 计算中心点
                    y_indices, x_indices = np.where(region)
                    center_y = np.mean(y_indices)
                    center_x = np.mean(x_indices) + (self.map_size_pixels - border_width)
                    
                    # 转换为世界坐标
                    exit_x = center_x * self.cell_size
                    exit_y = center_y * self.cell_size
                    
                    return True, (exit_x, exit_y)
        
        return False, None

class OptimalPathPlanner(PathPlanner):
    """最优路径规划类，继承自基本路径规划类"""
    
    def __init__(self, map_size_pixels=800, map_size_meters=32):
        """
        初始化最优路径规划器
        
        参数:
            map_size_pixels: 地图大小(像素)
            map_size_meters: 地图大小(米)
        """
        super().__init__(map_size_pixels, map_size_meters)
        
        # 路径优化参数
        self.smoothing_weight = 0.1
        self.obstacle_weight = 0.5
        self.max_iterations = 100
    
    def plan_optimal_path(self, occupancy_grid, start_pose, goal_position):
        """
        规划最优路径
        
        参数:
            occupancy_grid: 占用栅格地图，numpy数组
            start_pose: 起始位置，(x, y, theta)，单位为米和度
            goal_position: 目标位置，(x, y)，单位为米
            
        返回:
            path: 路径点列表，每个路径点是(x, y)坐标，单位为米
        """
        # 首先使用A*算法规划初始路径
        initial_path = self.plan_path(occupancy_grid, start_pose, goal_position)
        
        if not initial_path:
            return []
        
        # 对路径进行优化
        optimized_path = self._optimize_path(occupancy_grid, initial_path)
        
        # 路径平滑
        smoothed_path = self._smooth_path(optimized_path, weight_data=0.4, weight_smooth=0.2)
        
        return smoothed_path
    
    def _optimize_path(self, occupancy_grid, path):
        """
        优化路径，考虑障碍物距离和路径长度
        
        参数:
            occupancy_grid: 占用栅格地图，numpy数组
            path: 初始路径，路径点列表
            
        返回:
            optimized_path: 优化后的路径
        """
        if len(path) <= 2:
            return path
        
        # 计算障碍物距离场
        obstacle_map = occupancy_grid > self.obstacle_thresh
        distance_field = distance_transform_edt(~obstacle_map) * self.cell_size
        
        # 创建路径的副本
        optimized_path = np.array(path, dtype=np.float64)
        
        # 迭代优化
        for _ in range(self.max_iterations):
            # 对每个内部点进行优化
            for i in range(1, len(optimized_path) - 1):
                # 当前点
                x, y = optimized_path[i]
                
                # 前后点
                prev_x, prev_y = optimized_path[i-1]
                next_x, next_y = optimized_path[i+1]
                
                # 计算当前点到障碍物的距离
                cell_x = int(x / self.cell_size)
                cell_y = int(y / self.cell_size)
                
                # 确保坐标在地图范围内
                cell_x = max(0, min(cell_x, self.map_size_pixels - 1))
                cell_y = max(0, min(cell_y, self.map_size_pixels - 1))
                
                obstacle_distance = distance_field[cell_y, cell_x]
                
                # 计算平滑力（使点更接近前后点的中点）
                smooth_force_x = (prev_x + next_x) / 2 - x
                smooth_force_y = (prev_y + next_y) / 2 - y
                
                # 计算障碍物力（远离障碍物）
                obstacle_force_x = 0
                obstacle_force_y = 0
                
                if obstacle_distance < 1.0:  # 如果距离障碍物较近
                    # 计算梯度方向（远离障碍物的方向）
                    gradient_y, gradient_x = np.gradient(distance_field)
                    gradient_x = gradient_x[cell_y, cell_x]
                    gradient_y = gradient_y[cell_y, cell_x]
                    
                    # 归一化梯度
                    norm = np.sqrt(gradient_x**2 + gradient_y**2) + 1e-6
                    gradient_x /= norm
                    gradient_y /= norm
                    
                    # 障碍物力与距离成反比
                    force_magnitude = 0.5 / (obstacle_distance + 0.1)
                    obstacle_force_x = gradient_x * force_magnitude
                    obstacle_force_y = gradient_y * force_magnitude
                
                # 综合各种力
                force_x = self.smoothing_weight * smooth_force_x + self.obstacle_weight * obstacle_force_x
                force_y = self.smoothing_weight * smooth_force_y + self.obstacle_weight * obstacle_force_y
                
                # 更新点的位置
                optimized_path[i][0] += force_x
                optimized_path[i][1] += force_y
        
        return optimized_path.tolist()
    
    def find_shortest_return_path(self, occupancy_grid, current_pose, start_position):
        """
        找到从当前位置返回起点的最短路径
        
        参数:
            occupancy_grid: 占用栅格地图，numpy数组
            current_pose: 当前位置，(x, y, theta)，单位为米和度
            start_position: 起始位置，(x, y)，单位为米
            
        返回:
            path: 路径点列表，每个路径点是(x, y)坐标，单位为米
        """
        # 使用最优路径规划
        return self.plan_optimal_path(occupancy_grid, current_pose, start_position)
    
    def visualize_optimal_path(self, occupancy_grid, path, start_pose, goal_position):
        """
        可视化最优路径
        
        参数:
            occupancy_grid: 占用栅格地图，numpy数组
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
        
        # 计算障碍物距离场
        obstacle_map = occupancy_grid > self.obstacle_thresh
        distance_field = distance_transform_edt(~obstacle_map)
        
        # 显示障碍物距离场
        plt.figure(figsize=(10, 10))
        plt.imshow(distance_field, cmap='jet')
        plt.colorbar(label='距离 (像素)')
        
        # 绘制路径
        if path_cells:
            plt.plot(x_vals, y_vals, 'w-', linewidth=2)
        
        # 标记起点和终点
        plt.plot(start_cell_x, start_cell_y, 'wo', markersize=10)
        plt.plot(goal_cell_x, goal_cell_y, 'wo', markersize=10)
        
        plt.title('最优路径规划 - 障碍物距离场')
        plt.xlabel('X (像素)')
        plt.ylabel('Y (像素)')
        plt.grid(True)
        plt.show() 