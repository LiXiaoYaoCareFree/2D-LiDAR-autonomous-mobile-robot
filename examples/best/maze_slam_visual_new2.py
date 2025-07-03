

import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as colors
import time
import random
import json
from collections import deque
import heapq

class MazeEnvironment:
    """迷宫环境类（继承原版功能）"""
    
    def __init__(self, map_file="BreezySLAM-master/examples/map1.json"):
        self.size = None
        self.display_size = 18
        self.walls = []
        self.invisible_walls = []
        self.start_pos = None
        self.exits = []
        
        # 入口和出口检测状态
        self.entrance_detected = False
        self.entrance_opening = None
        self.exit_detected = False
        self.exit_position = None
        self.discovered_exits = []  # 动态发现的出口列表
        self.reached_exit_positions = []  # 记录机器人到达出口时的确切位置
        
        self.parse_maze_file(map_file)
        
    def parse_maze_file(self, map_file):
        """解析迷宫文件"""
        try:
            with open(map_file, 'r', encoding='utf-8') as f:
                content = f.read()
            
            if map_file.endswith('.json') or content.strip().startswith('{'):
                self._parse_json_format(content, map_file)
            else:
                self._parse_txt_format(content, map_file)
            
            self.calculate_maze_size()
            # 移除预先出口分析 - 实现真正的SLAM探索模式
            # self.find_exits()  # 注释掉，让机器人通过探索发现出口
            
            if self.start_pos is None:
                self.start_pos = (0.0, 1.0)
                
            # 根据起点是否在边界上，自动调整起点并添加虚拟墙
            self._setup_virtual_entrance()
            
            print(f"Successfully loaded maze file: {map_file}")
            print(f"Found {len(self.walls)} walls")
            print(f"Start position: {self.start_pos}")
            print(f"🔍 True SLAM mode: Robot will discover exits through exploration")
            
        except Exception as e:
            print(f"Error loading {map_file}: {e}")
            self.create_default_maze()
    
    def _parse_json_format(self, content, map_file):
        """解析JSON格式"""
        data = json.loads(content)
        
        if 'segments' in data:
            for segment in data['segments']:
                start = segment['start']
                end = segment['end']
                x1, y1 = float(start[0]), float(start[1])
                x2, y2 = float(end[0]), float(end[1])
                self.walls.append(((x1, y1), (x2, y2)))
        
        if 'start_point' in data:
            start_point = data['start_point']
            x, y = float(start_point[0]), float(start_point[1])
            self.start_pos = (x, y)
    
    def _parse_txt_format(self, content, map_file):
        """解析TXT格式"""
        lines = content.splitlines()
        
        for line in lines:
            line = line.strip()
            if line.startswith('#') or not line:
                continue
            
            if '->' in line:
                line_content = line.split('#')[0].strip()
                parts = line_content.split('->')
                start = parts[0].strip('()')
                end = parts[1].strip('()')
                
                x1, y1 = map(float, start.split(','))
                x2, y2 = map(float, end.split(','))
                self.walls.append(((x1, y1), (x2, y2)))
                
            elif line.startswith('start:'):
                line_content = line.split('#')[0].strip()
                pos_str = line_content.split(':')[1].strip().strip('()')
                try:
                    x, y = map(float, pos_str.split(','))
                    self.start_pos = (x, y)
                except ValueError as e:
                    print(f"Error parsing start position: {e}")
    
    def create_default_maze(self):
        """创建默认迷宫"""
        self.walls = [
            ((0, 0), (1, 0)),
            ((1, 0), (1, 2)),
            ((1, 1), (3, 1)),
            ((1, 2), (2, 2)),
            ((2, 2), (2, 3)),
            ((2, 0), (4, 0)),
            ((4, 0), (4, 4)),
            ((0, 4), (4, 4)),
            ((0, 0), (0, 3)),
            ((1, 3), (1, 4)),
            ((3, 3), (3, 4))
        ]
        self.start_pos = (1.5, 0.5)
        self.exits = [(0.5, 3.5)]
        self.size = 4
    
    def calculate_maze_size(self):
        """计算迷宫大小"""
        if not self.walls:
            self.size = 6
            return
        
        max_coord = 0
        for wall in self.walls:
            (x1, y1), (x2, y2) = wall
            max_coord = max(max_coord, x1, x2, y1, y2)
        
        self.size = max_coord
        
        if self.display_size < self.size + 2:
            self.display_size = int(self.size + 2)
    
    def find_exits(self):
        """寻找出口"""
        self.exits = []
        boundary_walls = self.get_boundary_walls()
        
        # 检查各边界的开口
        for side in ['top', 'right', 'bottom', 'left']:
            gaps = self.find_boundary_gaps(boundary_walls, side)
            for gap in gaps:
                if side == 'top':
                    exit_pos = ((gap[0] + gap[1]) / 2, self.size - 0.1)
                elif side == 'right':
                    exit_pos = (self.size - 0.1, (gap[0] + gap[1]) / 2)
                elif side == 'bottom':
                    exit_pos = ((gap[0] + gap[1]) / 2, 0.1)
                else:  # left
                    exit_pos = (0.1, (gap[0] + gap[1]) / 2)
                self.exits.append(exit_pos)
    
    def get_boundary_walls(self):
        """获取边界墙壁"""
        boundary_walls = {'top': [], 'bottom': [], 'left': [], 'right': []}
        
        for wall in self.walls:
            (x1, y1), (x2, y2) = wall
            
            if y1 == self.size and y2 == self.size:
                boundary_walls['top'].append((min(x1, x2), max(x1, x2)))
            elif y1 == 0 and y2 == 0:
                boundary_walls['bottom'].append((min(x1, x2), max(x1, x2)))
            elif x1 == 0 and x2 == 0:
                boundary_walls['left'].append((min(y1, y2), max(y1, y2)))
            elif x1 == self.size and x2 == self.size:
                boundary_walls['right'].append((min(y1, y2), max(y1, y2)))
        
        return boundary_walls
    
    def find_boundary_gaps(self, boundary_walls, side):
        """查找边界开口"""
        walls = boundary_walls[side]
        if not walls:
            return [(0, self.size)]
        
        walls.sort()
        gaps = []
        
        if side in ['top', 'bottom']:
            if walls[0][0] > 0:
                gaps.append((0, walls[0][0]))
            
            for i in range(len(walls) - 1):
                if walls[i][1] < walls[i+1][0]:
                    gaps.append((walls[i][1], walls[i+1][0]))
            
            if walls[-1][1] < self.size:
                gaps.append((walls[-1][1], self.size))
        else:
            if walls[0][0] > 0:
                gaps.append((0, walls[0][0]))
            
            for i in range(len(walls) - 1):
                if walls[i][1] < walls[i+1][0]:
                    gaps.append((walls[i][1], walls[i+1][0]))
            
            if walls[-1][1] < self.size:
                gaps.append((walls[-1][1], self.size))
        
        return gaps
    
    def add_discovered_exit(self, exit_position):
        """添加动态发现的出口"""
        # 检查是否已经发现过这个出口（避免重复）
        for existing_exit in self.discovered_exits:
            distance = math.sqrt((exit_position[0] - existing_exit[0])**2 + 
                               (exit_position[1] - existing_exit[1])**2)
            if distance < 1.0:  # 如果距离小于1单位，认为是同一个出口
                return False
        
        self.discovered_exits.append(exit_position)
        print(f"🎯 NEW EXIT DISCOVERED at ({exit_position[0]:.2f}, {exit_position[1]:.2f})")
        print(f"📍 Total discovered exits: {len(self.discovered_exits)}")
        return True
    
    def mark_exit_reached(self, robot_position):
        """标记机器人到达出口的确切位置"""
        # 检查是否已经标记过这个位置（避免重复）
        for existing_pos in self.reached_exit_positions:
            distance = math.sqrt((robot_position[0] - existing_pos[0])**2 + 
                               (robot_position[1] - existing_pos[1])**2)
            if distance < 0.5:  # 如果距离小于0.5单位，认为是同一个位置
                return False
        
        self.reached_exit_positions.append(robot_position)
        # 同时将其视为已检测到的出口，便于系统统一处理
        self.exit_detected = True
        self.exit_position = robot_position
        return True
    
    def can_move_to(self, from_pos, to_pos):
        """检查是否可以移动到目标位置（允许在扩展区域 -2 到 size+2 内移动）"""
        # 允许小车在完整扩展区域 (-2, size+2) 范围内移动，包括 -2 到 0 的左下区域
        extended_margin = 2.0
        if not (-extended_margin <= to_pos[0] <= self.size + extended_margin and
                -extended_margin <= to_pos[1] <= self.size + extended_margin):
            return False
        
        # 检查是否与墙壁碰撞
        for wall in self.walls + self.invisible_walls:
            if self._line_intersect_segment(from_pos, to_pos, wall[0], wall[1]):
                return False
        
        return True
    
    def _line_intersect_segment(self, p1, p2, p3, p4):
        """检查线段是否相交"""
        def ccw(A, B, C):
            return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])
        
        return ccw(p1, p3, p4) != ccw(p2, p3, p4) and ccw(p1, p2, p3) != ccw(p1, p2, p4)

    def _setup_virtual_entrance(self):


        # 需要 size 和 start_pos 已经确定
        if self.size is None or self.start_pos is None:
            return

        orig_x, orig_y = self.start_pos

        # 记录是否移动及新增墙体
        moved = False

        # 判断顶边界 (y == size)
        if abs(orig_y - self.size) < 1e-6:
            # 新起点向下移 1
            self.start_pos = (orig_x, orig_y - 1)
            # 在原 y==size 处放置水平隐形墙，长度 2（orig_x-1 到 orig_x+1）
            self.invisible_walls.append(((orig_x - 1, self.size), (orig_x + 1, self.size)))
            moved = True

        # 判断底边界 (y == 0)
        if abs(orig_y - 0) < 1e-6:
            self.start_pos = (orig_x, orig_y + 1)
            self.invisible_walls.append(((orig_x - 1, 0), (orig_x + 1, 0)))
            moved = True

        # 判断左边界 (x == 0)
        if abs(orig_x - 0) < 1e-6:
            self.start_pos = (orig_x + 1, self.start_pos[1])
            self.invisible_walls.append(((0, orig_y - 1), (0, orig_y + 1)))
            moved = True

        # 判断右边界 (x == size)
        if abs(orig_x - self.size) < 1e-6:
            self.start_pos = (orig_x - 1, self.start_pos[1])
            self.invisible_walls.append(((self.size, orig_y - 1), (self.size, orig_y + 1)))
            moved = True

        if moved:
            print(f"🚧 Virtual entrance walls added, new start_pos: {self.start_pos}, total invisible walls: {len(self.invisible_walls)}")

class GlobalSLAMMapper:
    """全局共享SLAM地图"""
    
    def __init__(self, maze_size, display_size=18, resolution=0.1):
        self.size = maze_size
        self.display_size = display_size
        self.resolution = resolution
        self.grid_size = int(display_size / resolution)
        
        # 全局SLAM地图：0=未知，1=自由，2=占用
        self.global_map = np.zeros((self.grid_size, self.grid_size), dtype=int)
        
        # 改进的前沿点管理
        self.frontiers = set()
        self.frontier_info = {}  # 存储每个frontier的详细信息
        self.frontier_last_seen = {}  # 记录frontier最后被确认的时间
        self.frontier_exploration_value = {}  # 记录frontier的探索价值
        self.update_counter = 0  # 更新计数器
        
        # 探索信息
        self.all_explored_cells = set()
        self.robot_paths = {}  # 存储所有机器人的路径
        
        # 保存maze_env引用（稍后设置）
        self.maze_env = None
        
    def set_maze_env(self, maze_env):
        """设置迷宫环境引用"""
        self.maze_env = maze_env
        
    def world_to_grid(self, world_pos):
        """世界坐标转网格坐标"""
        x, y = world_pos
        # 支持负坐标：将 -2 到 display_size-2 的世界坐标映射到 0 到 grid_size-1 的网格
        offset = 2.0  # 偏移量，因为世界坐标可以从 -2 开始
        grid_x = int((x + offset) / self.resolution)
        grid_y = int((y + offset) / self.resolution)
        return (min(max(grid_x, 0), self.grid_size-1), 
                min(max(grid_y, 0), self.grid_size-1))
    
    def grid_to_world(self, grid_pos):
        """网格坐标转世界坐标"""
        grid_x, grid_y = grid_pos
        # 支持负坐标：反向转换
        offset = 2.0
        x = grid_x * self.resolution - offset
        y = grid_y * self.resolution - offset
        return (x, y)
    
    def update_map(self, robot_id, robot_pos, scan_points, obstacle_points):
        """更新全局地图"""
        # 记录机器人路径
        if robot_id not in self.robot_paths:
            self.robot_paths[robot_id] = []
        self.robot_paths[robot_id].append(robot_pos)
        
        # 标记机器人位置为自由空间
        robot_grid = self.world_to_grid(robot_pos)
        rx, ry = robot_grid
        if 0 <= rx < self.grid_size and 0 <= ry < self.grid_size:
            self.global_map[ry, rx] = 1
            self.all_explored_cells.add((rx, ry))
        
        # 处理自由空间扫描点
        for point in scan_points:
            grid_pos = self.world_to_grid(point)
            px, py = grid_pos
            if 0 <= px < self.grid_size and 0 <= py < self.grid_size:
                if self.global_map[py, px] == 0:  # 只更新未知区域
                    self.global_map[py, px] = 1
                    self.all_explored_cells.add((px, py))
        
        # 处理障碍物点（优先级更高）
        for point in obstacle_points:
            grid_pos = self.world_to_grid(point)
            px, py = grid_pos
            if 0 <= px < self.grid_size and 0 <= py < self.grid_size:
                self.global_map[py, px] = 2
                self.all_explored_cells.add((px, py))
        
        # 更新前沿点
        self.update_frontiers()
    
    def update_frontiers(self):
        """改进的前沿点更新（智能管理版）"""
        self.update_counter += 1
        current_valid_frontiers = set()
        
        # 高速模式：只检查最近的探索点，减少计算量
        recent_cells = list(self.all_explored_cells)[-200:] if len(self.all_explored_cells) > 200 else self.all_explored_cells
        
        # 检查最近的自由空间邻居，发现新的potential frontiers
        for x, y in recent_cells:
            if self.global_map[y, x] == 1:  # 自由空间
                # 检查8邻域（包括对角线）
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
                    nx, ny = x + dx, y + dy
                    if (0 <= nx < self.grid_size and 
                        0 <= ny < self.grid_size and
                        self.global_map[ny, nx] == 0):  # 未知区域
                        
                        # 验证前沿点：确保不在墙上且可达
                        world_pos = self.grid_to_world((nx, ny))
                        if self._is_valid_frontier(world_pos, (x, y)):
                            current_valid_frontiers.add(world_pos)
                            
                            # 更新frontier信息
                            if world_pos not in self.frontier_info:
                                # 新发现的frontier
                                self.frontier_info[world_pos] = {
                                    'birth_time': self.update_counter,
                                    'discovery_count': 1,
                                    'nearby_unknown_cells': self._count_nearby_unknown(world_pos)
                                }
                            else:
                                # 已知frontier，更新信息
                                self.frontier_info[world_pos]['discovery_count'] += 1
                                self.frontier_info[world_pos]['nearby_unknown_cells'] = self._count_nearby_unknown(world_pos)
                            
                            # 更新最后确认时间
                            self.frontier_last_seen[world_pos] = self.update_counter
                            
                            # 计算探索价值
                            self._update_exploration_value(world_pos)
        
        # 智能移除策略：不是简单替换，而是基于多个条件
        frontiers_to_remove = set()
        
        for frontier in self.frontiers:
            should_remove = False
            
            # 条件1：如果frontier不再有效（被完全探索）
            if frontier not in current_valid_frontiers:
                # 给予一定的宽容期，避免过早移除
                if (self.update_counter - self.frontier_last_seen.get(frontier, 0)) > 5:
                    # 检查是否真的被完全探索
                    nearby_unknown = self._count_nearby_unknown(frontier)
                    if nearby_unknown == 0:
                        should_remove = True
            
            # 条件2：如果frontier的探索价值太低且存在时间过长
            if frontier in self.frontier_exploration_value:
                age = self.update_counter - self.frontier_info.get(frontier, {}).get('birth_time', 0)
                value = self.frontier_exploration_value[frontier]
                
                # 低价值且"老化"的frontier可以被移除
                if age > 20 and value < 0.3:
                    should_remove = True
                    print(f"🗑️  Removing low-value old frontier at ({frontier[0]:.1f}, {frontier[1]:.1f}) - value: {value:.2f}, age: {age}")
            
            if should_remove:
                frontiers_to_remove.add(frontier)
        
        # 移除标记的frontiers
        for frontier in frontiers_to_remove:
            self.frontiers.discard(frontier)
            self.frontier_info.pop(frontier, None)
            self.frontier_last_seen.pop(frontier, None)
            self.frontier_exploration_value.pop(frontier, None)
        
        # 添加新的有效frontiers
        new_frontiers = current_valid_frontiers - self.frontiers
        self.frontiers.update(new_frontiers)

    
    def _count_nearby_unknown(self, world_pos):
        """计算frontier周围的未知区域数量"""
        frontier_grid = self.world_to_grid(world_pos)
        fx, fy = frontier_grid
        
        unknown_count = 0
        for dx in range(-2, 3):  # 扩大检查范围
            for dy in range(-2, 3):
                nx, ny = fx + dx, fy + dy
                if (0 <= nx < self.grid_size and 0 <= ny < self.grid_size):
                    if self.global_map[ny, nx] == 0:  # 未知区域
                        unknown_count += 1
        
        return unknown_count
    
    def _update_exploration_value(self, world_pos):
        """更新frontier的探索价值"""
        if world_pos not in self.frontier_info:
            return
        
        info = self.frontier_info[world_pos]
        
        # 基础价值：基于周围未知区域数量
        unknown_value = min(info['nearby_unknown_cells'] / 10.0, 1.0)
        
        # 持久性价值：经常被重新发现的frontier价值更高
        persistence_value = min(info['discovery_count'] / 5.0, 1.0)
        
        # 年龄衰减：太老的frontier价值降低
        age = self.update_counter - info['birth_time']
        age_factor = max(0.3, 1.0 - age / 50.0)
        
        # 位置价值：边界附近的frontier价值更高
        boundary_value = self._calculate_boundary_value(world_pos)
        
        # 综合价值
        total_value = (unknown_value * 0.4 + 
                      persistence_value * 0.2 + 
                      boundary_value * 0.3 + 
                      age_factor * 0.1)
        
        self.frontier_exploration_value[world_pos] = total_value
    
    def _calculate_boundary_value(self, world_pos):
        """计算边界价值（边界附近的前沿点价值更高）"""
        x, y = world_pos
        
        # 计算到各边界的距离
        dist_to_left = abs(x - 0)
        dist_to_right = abs(x - self.size)
        dist_to_bottom = abs(y - 0)
        dist_to_top = abs(y - self.size)
        
        min_boundary_dist = min(dist_to_left, dist_to_right, dist_to_bottom, dist_to_top)
        
        # 检查是否在 -2 到 0 的扩展区域（优先级较低）
        in_negative_extension = (x < 0 or y < 0)
        
        # 检查是否在 size 到 size+2 的扩展区域（优先级较低）
        in_positive_extension = (x > self.size or y > self.size)
        
        # 边界附近的前沿点更有价值
        if min_boundary_dist < 1.0:
            # 扩展区域的边界点价值较低
            if in_negative_extension or in_positive_extension:
                return 0.4  # 较低优先级
            else:
                return 0.8  # 正常边界优先级
        elif min_boundary_dist < 2.0:
            if in_negative_extension or in_positive_extension:
                return 0.3
            else:
                return 0.6
        else:
            return 0.4
    
    def _is_valid_frontier(self, world_pos, from_grid):
        """验证前沿点是否有效（不在墙上，可达，在扩展显示区域内）"""
        # 1. 扩展边界检查：允许在扩展区域（-2到max+2）标记边缘点
        if not (-1.8 <= world_pos[0] <= self.display_size + 1.8 and 
                -1.8 <= world_pos[1] <= self.display_size + 1.8):
            return False
        
        # 2. 确保前沿点确实是探索边界（周围有足够的未知区域）
        frontier_grid = self.world_to_grid(world_pos)
        fx, fy = frontier_grid
        
        # 检查周围是否有足够的未知区域，确认这是真正的边界
        unknown_neighbors = 0
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                nx, ny = fx + dx, fy + dy
                if (0 <= nx < self.grid_size and 0 <= ny < self.grid_size):
                    if self.global_map[ny, nx] == 0:  # 未知区域
                        unknown_neighbors += 1
        
        # 至少需要2个未知邻居才算真正的边界（放宽要求）
        if unknown_neighbors < 2:
            return False
        
        # 3. 严格的墙壁检测 - 对所有区域都进行检查
        if self.maze_env:
            # 检查前沿点是否与任何墙壁重叠或太近
            min_distance_to_wall = 0.3  # 增加最小距离到墙壁，防止穿墙
            
            for wall in self.maze_env.walls + self.maze_env.invisible_walls:
                distance = self._point_to_line_distance(world_pos, wall[0], wall[1])
                if distance < min_distance_to_wall:
                    return False
            
            # 4. 连通性检查 - 只对可访问区域进行
            is_in_accessible_area = (0 <= world_pos[0] <= self.maze_env.size and 
                                    0 <= world_pos[1] <= self.maze_env.size)
            
            if is_in_accessible_area:
                # 检查从已知自由空间到前沿点的连通性
                from_world = self.grid_to_world(from_grid)
                if not self.maze_env.can_move_to(from_world, world_pos):
                    return False
            
            # 5. 额外的墙壁穿透检查 - 检查前沿点周围的小区域
            check_radius = 0.2
            for angle in [0, 45, 90, 135, 180, 225, 270, 315]:
                check_x = world_pos[0] + check_radius * math.cos(math.radians(angle))
                check_y = world_pos[1] + check_radius * math.sin(math.radians(angle))
                check_pos = (check_x, check_y)
                
                for wall in self.maze_env.walls + self.maze_env.invisible_walls:
                    distance = self._point_to_line_distance(check_pos, wall[0], wall[1])
                    if distance < 0.1:  # 如果周围点太靠近墙壁，也拒绝
                        return False
        
        return True
    
    def _is_frontier_reachable(self, frontier_world, from_grid):
        """检查前沿点是否可达（不被墙阻挡）"""
        from_world = self.grid_to_world(from_grid)
        
        # 基础边界检查
        if not (0.2 <= frontier_world[0] <= self.display_size - 0.2 and 
                0.2 <= frontier_world[1] <= self.display_size - 0.2):
            return False
        
        # 如果有maze_env，进行更详细的检查
        if self.maze_env:
            # 检查前沿点是否与墙壁太近
            safety_distance = 0.2
            for wall in self.maze_env.walls + self.maze_env.invisible_walls:
                if self._point_to_line_distance(frontier_world, wall[0], wall[1]) < safety_distance:
                    return False
            
            # 检查从已知自由空间到前沿点的路径
            if not self.maze_env.can_move_to(from_world, frontier_world):
                return False
        
        return True
    
    def _point_to_line_distance(self, point, line_start, line_end):
        """计算点到线段的距离"""
        x0, y0 = point
        x1, y1 = line_start
        x2, y2 = line_end
        
        # 线段长度
        line_len = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        if line_len == 0:
            return math.sqrt((x0 - x1)**2 + (y0 - y1)**2)
        
        # 计算投影参数
        t = max(0, min(1, ((x0 - x1) * (x2 - x1) + (y0 - y1) * (y2 - y1)) / (line_len**2)))
        
        # 投影点
        proj_x = x1 + t * (x2 - x1)
        proj_y = y1 + t * (y2 - y1)
        
        # 距离
        return math.sqrt((x0 - proj_x)**2 + (y0 - proj_y)**2)
    
    def get_nearest_frontier(self, robot_pos):
        """获取最优前沿点（优先可访问区域，综合距离和价值）"""
        if not self.frontiers:
            return None
        
        # 分离可访问区域和扩展区域的frontiers
        accessible_frontiers = []
        extended_frontiers = []
        
        for frontier in self.frontiers:
            if (0 <= frontier[0] <= self.maze_env.size and 
                0 <= frontier[1] <= self.maze_env.size):
                accessible_frontiers.append(frontier)
            else:
                extended_frontiers.append(frontier)
        
        # 优先选择可访问区域的frontiers
        target_frontiers = accessible_frontiers if accessible_frontiers else extended_frontiers
        
        if not target_frontiers:
            return None
        
        best_score = float('-inf')
        best_frontier = None
        area_type = "accessible" if accessible_frontiers else "extended"
        
        for frontier in target_frontiers:
            # 计算距离
            dist = math.sqrt((robot_pos[0] - frontier[0])**2 + 
                           (robot_pos[1] - frontier[1])**2)
            
            # 获取探索价值（如果没有则使用默认值）
            exploration_value = self.frontier_exploration_value.get(frontier, 0.5)
            
            # 综合评分：距离越近越好，价值越高越好
            # 归一化距离到0-1范围
            max_possible_dist = math.sqrt(self.display_size**2 + self.display_size**2)
            normalized_dist = dist / max_possible_dist
            distance_score = 1.0 - normalized_dist
            
            # 综合评分：60%看价值，40%看距离
            total_score = exploration_value * 0.6 + distance_score * 0.4
            
            if total_score > best_score:
                best_score = total_score
                best_frontier = frontier
        
        if best_frontier and area_type == "extended":
            print(f"🎯 Selecting frontier from extended area: ({best_frontier[0]:.1f}, {best_frontier[1]:.1f})")
        
        return best_frontier
    
    def assign_frontiers_to_robots(self, robot_positions):
        """为多个机器人分配不同的前沿目标（优先可访问区域）"""
        if not self.frontiers:
            return {}
        
        # 分离可访问区域和扩展区域的frontiers
        accessible_frontiers = []
        extended_frontiers = []
        
        for frontier in self.frontiers:
            if (0 <= frontier[0] <= self.maze_env.size and 
                0 <= frontier[1] <= self.maze_env.size):
                accessible_frontiers.append(frontier)
            else:
                extended_frontiers.append(frontier)
        
        assignments = {}
        
        # 优先分配可访问区域的frontiers
        available_accessible = accessible_frontiers.copy()
        
        for robot_id, pos in robot_positions.items():
            if not available_accessible:
                break
            
            best_score = float('-inf')
            best_frontier = None
            best_idx = -1
            
            for i, frontier in enumerate(available_accessible):
                # 计算距离
                dist = math.sqrt((pos[0] - frontier[0])**2 + (pos[1] - frontier[1])**2)
                
                # 获取探索价值
                exploration_value = self.frontier_exploration_value.get(frontier, 0.5)
                
                # 综合评分
                max_possible_dist = math.sqrt(self.display_size**2 + self.display_size**2)
                normalized_dist = dist / max_possible_dist
                distance_score = 1.0 - normalized_dist
                total_score = exploration_value * 0.6 + distance_score * 0.4
                
                if total_score > best_score:
                    best_score = total_score
                    best_frontier = frontier
                    best_idx = i
            
            if best_frontier:
                assignments[robot_id] = best_frontier
                available_accessible.pop(best_idx)
        
        # 如果还有机器人没有分配到可访问区域的frontier，再分配扩展区域的
        unassigned_robots = [robot_id for robot_id in robot_positions.keys() if robot_id not in assignments]
        available_extended = extended_frontiers.copy()
        
        for robot_id in unassigned_robots:
            if not available_extended:
                break
            
            pos = robot_positions[robot_id]
            best_score = float('-inf')
            best_frontier = None
            best_idx = -1
            
            for i, frontier in enumerate(available_extended):
                # 计算距离
                dist = math.sqrt((pos[0] - frontier[0])**2 + (pos[1] - frontier[1])**2)
                
                # 获取探索价值
                exploration_value = self.frontier_exploration_value.get(frontier, 0.5)
                
                # 综合评分
                max_possible_dist = math.sqrt(self.display_size**2 + self.display_size**2)
                normalized_dist = dist / max_possible_dist
                distance_score = 1.0 - normalized_dist
                total_score = exploration_value * 0.6 + distance_score * 0.4
                
                if total_score > best_score:
                    best_score = total_score
                    best_frontier = frontier
                    best_idx = i
            
            if best_frontier:
                assignments[robot_id] = best_frontier
                available_extended.pop(best_idx)
                print(f"🎯 Robot {robot_id} assigned extended area frontier: ({best_frontier[0]:.1f}, {best_frontier[1]:.1f})")
        
        return assignments

class AStarPathPlanner:
    """A*路径规划器（防穿墙版本）"""
    
    def __init__(self, global_mapper, maze_env):
        self.mapper = global_mapper
        self.maze_env = maze_env
    
    def plan_path(self, start_pos, goal_pos):
        """使用A*算法规划路径（严格防穿墙）"""
        start_grid = self.mapper.world_to_grid(start_pos)
        goal_grid = self.mapper.world_to_grid(goal_pos)
        
        if start_grid == goal_grid:
            return [start_pos, goal_pos]
        
        # 验证目标点是否可达
        goal_world = self.mapper.grid_to_world(goal_grid)
        if not self._is_position_safe(goal_world):
            # 寻找最近的安全目标点
            goal_grid = self._find_nearest_safe_position(goal_grid)
            if not goal_grid:
                return [start_pos]  # 无法找到安全路径
        
        # A*算法实现
        open_set = []
        heapq.heappush(open_set, (0, start_grid))
        
        came_from = {}
        g_score = {start_grid: 0}
        f_score = {start_grid: self._heuristic(start_grid, goal_grid)}
        
        while open_set:
            current = heapq.heappop(open_set)[1]
            
            if current == goal_grid:
                # 重构路径并验证
                path = self._reconstruct_path(came_from, current, start_pos)
                return self._smooth_and_validate_path(path)
            
            # 检查邻居（4方向，避免对角线穿墙）
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                neighbor = (current[0] + dx, current[1] + dy)
                
                if self._is_grid_position_valid(neighbor):
                    # 验证移动是否安全（不穿墙）
                    current_world = self.mapper.grid_to_world(current)
                    neighbor_world = self.mapper.grid_to_world(neighbor)
                    
                    if self._is_movement_safe(current_world, neighbor_world):
                        tentative_g = g_score[current] + 1
                        
                        if neighbor not in g_score or tentative_g < g_score[neighbor]:
                            came_from[neighbor] = current
                            g_score[neighbor] = tentative_g
                            f_score[neighbor] = tentative_g + self._heuristic(neighbor, goal_grid)
                            heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        # 如果没找到路径，返回当前位置
        return [start_pos]
    
    def _is_grid_position_valid(self, grid_pos):
        """检查网格位置是否有效"""
        x, y = grid_pos
        if not (0 <= x < self.mapper.grid_size and 0 <= y < self.mapper.grid_size):
            return False
        
        # 检查是否是障碍物
        if self.mapper.global_map[y, x] == 2:
            return False
        
        return True
    
    def _is_position_safe(self, world_pos):
        """检查位置是否安全（不撞墙且在允许区域内）"""
        x, y = world_pos
        
        # 检查是否在完整扩展允许区域内，包括 -2 到 0 区域（探索时需要）
        if not (-1.9 <= x <= self.maze_env.size + 1.9 and -1.9 <= y <= self.maze_env.size + 1.9):
            return False
        
        # 检查是否与墙壁冲突
        safety_radius = 0.15  # 安全半径
        
        for wall in self.maze_env.walls + self.maze_env.invisible_walls:
            if self._point_to_line_distance(world_pos, wall[0], wall[1]) < safety_radius:
                return False
        
        return True
    
    def _is_movement_safe(self, from_pos, to_pos):
        """检查从一个位置移动到另一个位置是否安全"""
        # 使用maze_env的can_move_to方法
        return self.maze_env.can_move_to(from_pos, to_pos) and self._is_position_safe(to_pos)
    
    def _find_nearest_safe_position(self, target_grid):
        """寻找最近的安全位置"""
        # 在目标周围搜索安全位置
        for radius in range(1, 10):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if abs(dx) == radius or abs(dy) == radius:  # 只检查边界
                        candidate = (target_grid[0] + dx, target_grid[1] + dy)
                        if self._is_grid_position_valid(candidate):
                            candidate_world = self.mapper.grid_to_world(candidate)
                            if self._is_position_safe(candidate_world):
                                return candidate
        return None
    
    def _reconstruct_path(self, came_from, current, start_pos):
        """重构路径"""
        path = []
        while current in came_from:
            world_pos = self.mapper.grid_to_world(current)
            path.append(world_pos)
            current = came_from[current]
        path.append(start_pos)
        path.reverse()
        return path
    
    def _smooth_and_validate_path(self, path):
        """平滑并验证路径"""
        if len(path) <= 2:
            return path
        
        smoothed_path = [path[0]]
        
        for i in range(1, len(path)):
            # 验证每一步移动
            if self._is_movement_safe(smoothed_path[-1], path[i]):
                smoothed_path.append(path[i])
            else:
                # 如果直接移动不安全，添加中间点
                mid_point = ((smoothed_path[-1][0] + path[i][0]) / 2,
                           (smoothed_path[-1][1] + path[i][1]) / 2)
                if self._is_position_safe(mid_point):
                    smoothed_path.append(mid_point)
                smoothed_path.append(path[i])
        
        return smoothed_path
    
    def _point_to_line_distance(self, point, line_start, line_end):
        """计算点到线段的距离"""
        x0, y0 = point
        x1, y1 = line_start
        x2, y2 = line_end
        
        # 线段长度
        line_len = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        if line_len == 0:
            return math.sqrt((x0 - x1)**2 + (y0 - y1)**2)
        
        # 计算投影参数
        t = max(0, min(1, ((x0 - x1) * (x2 - x1) + (y0 - y1) * (y2 - y1)) / (line_len**2)))
        
        # 投影点
        proj_x = x1 + t * (x2 - x1)
        proj_y = y1 + t * (y2 - y1)
        
        # 距离
        return math.sqrt((x0 - proj_x)**2 + (y0 - proj_y)**2)
    
    def _heuristic(self, a, b):
        """A*启发式函数（欧几里得距离）"""
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

class OptimalPathPlanner:
    """专用于最短路径计算的八方向A*路径规划器"""
    
    def __init__(self, global_mapper, maze_env):
        self.mapper = global_mapper
        self.maze_env = maze_env
        # 八方向移动：4个基本方向 + 4个对角线方向
        self.directions = [
            (-1, 0, 1.0),    # 左
            (1, 0, 1.0),     # 右
            (0, -1, 1.0),    # 下
            (0, 1, 1.0),     # 上
            (-1, -1, 1.414), # 左下（对角线）
            (-1, 1, 1.414),  # 左上（对角线）
            (1, -1, 1.414),  # 右下（对角线）
            (1, 1, 1.414)    # 右上（对角线）
        ]
    
    def plan_optimal_path(self, start_pos, goal_pos):
        """计算八方向最优路径"""
        print(f"🎯 Planning optimal 8-direction path from {start_pos} to {goal_pos}")
        
        start_grid = self.mapper.world_to_grid(start_pos)
        goal_grid = self.mapper.world_to_grid(goal_pos)
        
        if start_grid == goal_grid:
            return [start_pos, goal_pos]
        
        # 验证起点和终点
        if not self._is_position_accessible(start_pos):
            print(f"❌ Start position not accessible: {start_pos}")
            return [start_pos]
        
        if not self._is_position_accessible(goal_pos):
            print(f"❌ Goal position not accessible: {goal_pos}")
            # 寻找最近的可达目标点
            goal_grid = self._find_nearest_accessible_position(goal_grid)
            if not goal_grid:
                return [start_pos]
            goal_pos = self.mapper.grid_to_world(goal_grid)
            print(f"✅ Adjusted goal to: {goal_pos}")
        
        # A*算法实现（八方向版本）
        open_set = []
        heapq.heappush(open_set, (0, start_grid))
        
        came_from = {}
        g_score = {start_grid: 0}
        f_score = {start_grid: self._heuristic(start_grid, goal_grid)}
        
        explored_nodes = 0
        
        while open_set:
            current = heapq.heappop(open_set)[1]
            explored_nodes += 1
            
            if current == goal_grid:
                print(f"✅ Path found! Explored {explored_nodes} nodes")
                path = self._reconstruct_optimal_path(came_from, current, start_pos, goal_pos)
                return self._smooth_optimal_path(path)
            
            # 检查八个方向的邻居
            for dx, dy, cost in self.directions:
                neighbor = (current[0] + dx, current[1] + dy)
                
                if self._is_grid_position_accessible(neighbor):
                    # 检查移动是否安全（包括对角线移动）
                    current_world = self.mapper.grid_to_world(current)
                    neighbor_world = self.mapper.grid_to_world(neighbor)
                    
                    if self._is_movement_safe(current_world, neighbor_world, is_diagonal=(cost > 1.0)):
                        tentative_g = g_score[current] + cost
                        
                        if neighbor not in g_score or tentative_g < g_score[neighbor]:
                            came_from[neighbor] = current
                            g_score[neighbor] = tentative_g
                            f_score[neighbor] = tentative_g + self._heuristic(neighbor, goal_grid)
                            heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        print(f"❌ No path found after exploring {explored_nodes} nodes")
        return [start_pos]
    
    def _is_position_accessible(self, world_pos):
        """检查位置是否可达（严格限制在迷宫内部，远离墙壁）"""
        x, y = world_pos
        
        # 严格限制在迷宫内部区域（0到size）
        margin = 0.3  # 大幅增加边距确保安全
        if not (margin <= x <= self.maze_env.size - margin and 
                margin <= y <= self.maze_env.size - margin):
            return False
        
        # 检查是否与墙壁冲突
        safety_radius = 0.35  # 大幅增加安全半径
        
        for wall in self.maze_env.walls + self.maze_env.invisible_walls:
            wall_dist = self._point_to_line_distance(world_pos, wall[0], wall[1])
            if wall_dist < safety_radius:
                return False
        
        return True
    
    def _is_grid_position_accessible(self, grid_pos):
        """检查网格位置是否可达"""
        x, y = grid_pos
        if not (0 <= x < self.mapper.grid_size and 0 <= y < self.mapper.grid_size):
            return False
        
        # 检查是否是障碍物
        if self.mapper.global_map[y, x] == 2:
            return False
        
        # 检查对应的世界坐标是否安全
        world_pos = self.mapper.grid_to_world(grid_pos)
        return self._is_position_accessible(world_pos)
    
    def _is_movement_safe(self, from_pos, to_pos, is_diagonal=False):
        """检查移动是否安全（包括对角线移动的严格检查）"""
        # 基础位置检查
        if not self._is_position_accessible(to_pos):
            return False
        
        # 使用maze_env的can_move_to方法进行基础验证
        if not self.maze_env.can_move_to(from_pos, to_pos):
            return False
        
        # 对角线移动需要额外的严格检查
        if is_diagonal:
            # 检查对角线移动的多个中间点
            steps = 5  # 增加检查点数量
            for i in range(1, steps):
                t = i / steps
                mid_x = from_pos[0] + t * (to_pos[0] - from_pos[0])
                mid_y = from_pos[1] + t * (to_pos[1] - from_pos[1])
                mid_pos = (mid_x, mid_y)
                
                if not self._is_position_accessible(mid_pos):
                    return False
            
            # 检查对角线移动不会"切角"穿墙
            # 必须至少有一个直角路径可行
            corner1 = (from_pos[0], to_pos[1])
            corner2 = (to_pos[0], from_pos[1])
            
            corner1_safe = (self._is_position_accessible(corner1) and 
                           self.maze_env.can_move_to(from_pos, corner1) and
                           self.maze_env.can_move_to(corner1, to_pos))
            corner2_safe = (self._is_position_accessible(corner2) and 
                           self.maze_env.can_move_to(from_pos, corner2) and
                           self.maze_env.can_move_to(corner2, to_pos))
            
            if not (corner1_safe or corner2_safe):
                return False
            
            # 额外的墙壁距离检查 - 对角线路径上每个点都要检查
            for i in range(steps + 1):
                t = i / steps
                check_x = from_pos[0] + t * (to_pos[0] - from_pos[0])
                check_y = from_pos[1] + t * (to_pos[1] - from_pos[1])
                check_pos = (check_x, check_y)
                
                # 检查与所有墙壁的距离
                min_wall_distance = 0.4  # 大幅增加安全距离，防止穿墙
                for wall in self.maze_env.walls + self.maze_env.invisible_walls:
                    wall_dist = self._point_to_line_distance(check_pos, wall[0], wall[1])
                    if wall_dist < min_wall_distance:
                        return False
        else:
            # 直线移动也要检查中间点
            steps = 3
            for i in range(1, steps):
                t = i / steps
                mid_x = from_pos[0] + t * (to_pos[0] - from_pos[0])
                mid_y = from_pos[1] + t * (to_pos[1] - from_pos[1])
                mid_pos = (mid_x, mid_y)
                
                if not self._is_position_accessible(mid_pos):
                    return False
        
        return True
    
    def _find_nearest_accessible_position(self, target_grid):
        """寻找最近的可达位置"""
        for radius in range(1, 20):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if abs(dx) == radius or abs(dy) == radius:
                        candidate = (target_grid[0] + dx, target_grid[1] + dy)
                        if self._is_grid_position_accessible(candidate):
                            candidate_world = self.mapper.grid_to_world(candidate)
                            if (0.3 <= candidate_world[0] <= self.maze_env.size - 0.3 and
                                0.3 <= candidate_world[1] <= self.maze_env.size - 0.3):
                                return candidate
        return None
    
    def _reconstruct_optimal_path(self, came_from, current, start_pos, goal_pos):
        """重构最优路径"""
        path = [goal_pos]  # 从目标点开始
        
        while current in came_from:
            world_pos = self.mapper.grid_to_world(current)
            path.append(world_pos)
            current = came_from[current]
        
        path.append(start_pos)
        path.reverse()
        return path
    
    def _smooth_optimal_path(self, path):
        """优化路径（移除不必要的中间点）- 严格防穿墙版本"""
        if len(path) <= 2:
            return path
        
        smoothed_path = [path[0]]
        i = 0
        
        while i < len(path) - 1:
            # 尝试直线连接到尽可能远的点，但要严格验证
            j = len(path) - 1
            found_direct = False
            
            while j > i + 1:
                is_diagonal = self._is_diagonal_movement(smoothed_path[-1], path[j])
                
                # 对于较长的跳跃，进行额外验证
                if self._is_movement_safe_detailed(smoothed_path[-1], path[j], is_diagonal):
                    smoothed_path.append(path[j])
                    i = j
                    found_direct = True
                    break
                j -= 1
            
            if not found_direct:
                smoothed_path.append(path[i + 1])
                i += 1
        
        return smoothed_path
    
    def _is_movement_safe_detailed(self, from_pos, to_pos, is_diagonal):
        """详细的移动安全检查（用于路径平滑化）"""
        # 计算距离
        distance = math.sqrt((to_pos[0] - from_pos[0])**2 + (to_pos[1] - from_pos[1])**2)
        
        # 对于较长的移动，使用更多检查点
        if distance > 1.0:
            steps = max(10, int(distance * 10))  # 至少10个检查点
        else:
            steps = 5
        
        # 检查路径上的每个点
        for i in range(steps + 1):
            t = i / steps
            check_x = from_pos[0] + t * (to_pos[0] - from_pos[0])
            check_y = from_pos[1] + t * (to_pos[1] - from_pos[1])
            check_pos = (check_x, check_y)
            
            # 检查位置是否可达
            if not self._is_position_accessible(check_pos):
                return False
            
            # 检查与墙壁的距离
            min_wall_distance = 0.4  # 更大的安全距离，与其他检查一致
            for wall in self.maze_env.walls + self.maze_env.invisible_walls:
                wall_dist = self._point_to_line_distance(check_pos, wall[0], wall[1])
                if wall_dist < min_wall_distance:
                    return False
        
        # 如果是对角线移动，还要检查直角路径
        if is_diagonal:
            corner1 = (from_pos[0], to_pos[1])
            corner2 = (to_pos[0], from_pos[1])
            
            corner1_safe = (self._is_position_accessible(corner1) and 
                           self.maze_env.can_move_to(from_pos, corner1) and
                           self.maze_env.can_move_to(corner1, to_pos))
            corner2_safe = (self._is_position_accessible(corner2) and 
                           self.maze_env.can_move_to(from_pos, corner2) and
                           self.maze_env.can_move_to(corner2, to_pos))
            
            if not (corner1_safe or corner2_safe):
                return False
        
        # 最终使用maze_env验证
        return self.maze_env.can_move_to(from_pos, to_pos)
    
    def _is_diagonal_movement(self, from_pos, to_pos):
        """判断是否为对角线移动"""
        dx = abs(to_pos[0] - from_pos[0])
        dy = abs(to_pos[1] - from_pos[1])
        return dx > 0.01 and dy > 0.01
    
    def _point_to_line_distance(self, point, line_start, line_end):
        """计算点到线段的距离"""
        x0, y0 = point
        x1, y1 = line_start
        x2, y2 = line_end
        
        line_len = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        if line_len == 0:
            return math.sqrt((x0 - x1)**2 + (y0 - y1)**2)
        
        t = max(0, min(1, ((x0 - x1) * (x2 - x1) + (y0 - y1) * (y2 - y1)) / (line_len**2)))
        
        proj_x = x1 + t * (x2 - x1)
        proj_y = y1 + t * (y2 - y1)
        
        return math.sqrt((x0 - proj_x)**2 + (y0 - proj_y)**2)
    
    def _heuristic(self, a, b):
        """A*启发式函数（八方向欧几里得距离）"""
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        
        # 对角线距离优化的启发式
        # 对角线步数 = min(dx, dy)，直线步数 = |dx - dy|
        diagonal_steps = min(dx, dy)
        straight_steps = abs(dx - dy)
        
        return diagonal_steps * 1.414 + straight_steps * 1.0

class LaserSimulator:
    """激光雷达模拟器（优化版）"""
    
    def __init__(self, maze_env, max_range=8.0):
        self.maze_env = maze_env
        self.max_range = max_range
    
    def scan(self, robot_pos):
        """执行360度激光扫描"""
        x, y = robot_pos
        scan_points = []
        obstacle_points = []
        scan_ranges = []
        scan_angles = []
        
        # 高速模式：360度扫描，每4度一个射线（减少计算量）
        for angle_deg in range(0, 360, 4):
            angle_rad = math.radians(angle_deg)
            ray_direction = (math.cos(angle_rad), math.sin(angle_rad))
            
            # 找到最近的墙碰撞点
            closest_collision = None
            min_distance = self.max_range
            
            # 检查内部墙壁
            for wall in self.maze_env.walls:
                collision = self.ray_wall_intersection(robot_pos, ray_direction, wall[0], wall[1])
                if collision and collision[2] < min_distance:
                    closest_collision = collision
                    min_distance = collision[2]
            
            # 检查隐形墙
            for wall in self.maze_env.invisible_walls:
                collision = self.ray_wall_intersection(robot_pos, ray_direction, wall[0], wall[1])
                if collision and collision[2] < min_distance:
                    closest_collision = collision
                    min_distance = collision[2]
            
            # 检查边界（只有接近时才检测）
            boundary_collision = self.ray_boundary_intersection(robot_pos, ray_direction)
            is_boundary_hit = False
            if boundary_collision and boundary_collision[2] < min_distance:
                closest_collision = boundary_collision
                min_distance = boundary_collision[2]
                # 标记这是外框碰撞，不应写入障碍点
                is_boundary_hit = True
            
            if closest_collision:
                collision_x, collision_y, distance = closest_collision
                
                # 只有碰到真实墙体或隐形墙时才记录为障碍点，外框碰撞不记录
                if not is_boundary_hit:
                    obstacle_points.append((collision_x, collision_y))
                
                scan_ranges.append(distance)
                scan_angles.append(angle_deg)
                
                # 在射线路径上添加自由空间点
                for d in np.arange(0.2, distance, 0.2):
                    free_x = x + d * math.cos(angle_rad)
                    free_y = y + d * math.sin(angle_rad)
                    scan_points.append((free_x, free_y))
            else:
                scan_ranges.append(self.max_range)
                scan_angles.append(angle_deg)
                for d in np.arange(0.2, self.max_range, 0.2):
                    free_x = x + d * math.cos(angle_rad)
                    free_y = y + d * math.sin(angle_rad)
                    scan_points.append((free_x, free_y))
        
        # 检查出口（180度连续开放区域）
        exit_found = self.detect_exit_from_scan(robot_pos, scan_ranges, scan_angles)
        
        return scan_points, obstacle_points, scan_ranges, scan_angles, exit_found
    
    def ray_wall_intersection(self, ray_start, ray_direction, wall_start, wall_end):
        """计算射线与墙壁的交点"""
        x1, y1 = ray_start
        dx, dy = ray_direction
        x3, y3 = wall_start
        x4, y4 = wall_end
        
        denom = (x4 - x3) * dy - (y4 - y3) * dx
        if abs(denom) < 1e-10:
            return None
        
        t = ((y3 - y1) * dx - (x3 - x1) * dy) / denom
        u = ((y3 - y1) * (x4 - x3) - (x3 - x1) * (y4 - y3)) / denom
        
        if 0 <= t <= 1 and u >= 0:
            intersection_x = x3 + t * (x4 - x3)
            intersection_y = y3 + t * (y4 - y3)
            distance = math.sqrt((intersection_x - x1)**2 + (intersection_y - y1)**2)
            return (intersection_x, intersection_y, distance)
        
        return None
    
    def ray_boundary_intersection(self, ray_start, ray_direction):
        """计算射线与扩展边界的交点（可扫描区域边界在-2和max+2）"""
        robot_x, robot_y = ray_start
        
        # 扩展边界（激光可以扫描到的边界）
        extended_boundaries = [
            # 外边界（-2和max+2处的边界）
            ((-2, -2), (-2, self.maze_env.size + 2)),  # 左外边界
            ((self.maze_env.size + 2, -2), (self.maze_env.size + 2, self.maze_env.size + 2)),  # 右外边界
            ((-2, -2), (self.maze_env.size + 2, -2)),  # 下外边界
            ((-2, self.maze_env.size + 2), (self.maze_env.size + 2, self.maze_env.size + 2)),  # 上外边界
        ]
        
        # **注意**：不再把迷宫本身的 0/size 边界加入碰撞检测，只保留 -2 与 size+2 的外框，
        # 避免在 SLAM 地图中出现靠近真实边界 (x=0 或 y=0 等) 的伪障碍条纹。
        
        closest_intersection = None
        min_distance = self.max_range
        
        for boundary in extended_boundaries:
            intersection = self.ray_wall_intersection(ray_start, ray_direction, boundary[0], boundary[1])
            if intersection and intersection[2] < min_distance:
                closest_intersection = intersection
                min_distance = intersection[2]
        
        return closest_intersection
    
    def detect_exit_from_scan(self, robot_pos, scan_ranges, scan_angles):
        """简化版出口检测：基于边界方向的大量激光射线未命中内壁而直接打到外框。"""

        # 最少需要一定数量扫描数据
        if len(scan_ranges) < 45:
            return False

        x, y = robot_pos
        size = self.maze_env.size

        # 距离四条主边界的距离
        dists = {
            'left': x,
            'right': size - x,
            'bottom': y,
            'top': size - y
        }

        # 找到最近边界，并要求 <1m，否则不认为在出口附近
        side, d_to_boundary = min(dists.items(), key=lambda kv: kv[1])

        # 需远离入口（避免把入口误判为出口）
        sx, sy = self.maze_env.start_pos
        if math.hypot(x - sx, y - sy) < 3.0:  # 距入口 <3m 直接返回
            return False

        # 必须非常靠近边界 <0.5m
        if d_to_boundary > 0.5:
            return False

        # 该边界对应的中心角（度）
        side_angle = {
            'right': 0,
            'top': 90,
            'left': 180,
            'bottom': 270
        }[side]

        # 收集 ±25° 内的射线
        candidate = []
        for rng, ang in zip(scan_ranges, scan_angles):
            diff = abs((ang - side_angle + 180) % 360 - 180)
            if diff <= 25:
                candidate.append((rng, ang))

        if len(candidate) < 8:
            return False

        # 理论上在该方向应碰到边界的距离
        expected = d_to_boundary

        # 判定多少射线直接命中外框（range 与外框交点距离几乎一致）
        boundary_hits = 0
        for rng, ang in candidate:
            ang_rad = math.radians(ang)
            ray_dir = (math.cos(ang_rad), math.sin(ang_rad))
            bc = self.ray_boundary_intersection(robot_pos, ray_dir)
            if bc and abs(bc[2] - rng) < 0.05:
                boundary_hits += 1

        # 若 ≥80% 的射线直接到外框，则认为有出口缺口
        if boundary_hits / len(candidate) < 0.8:
            return False

        # 记录并返回
        if self.maze_env.add_discovered_exit(robot_pos):
            print(f"🎯 NEW EXIT discovered at {side} boundary (simple gap)!" )

        self.maze_env.mark_exit_reached(robot_pos)
        self.maze_env.exit_detected = True
        self.maze_env.exit_position = robot_pos

        print(f"   Robot position: ({x:.2f}, {y:.2f})")
        print(f"   Boundary hits in ±25° sector: {boundary_hits}/{len(candidate)} (expected={expected:.2f}m)")
        return True

class SmartMazeSLAMVisualizer:
    """智能可视化器"""
    
    def __init__(self, maze_env, global_mapper, num_robots):
        self.maze_env = maze_env
        self.global_mapper = global_mapper
        self.num_robots = num_robots
        
        # 创建4面板显示
        self.fig, self.axes = plt.subplots(2, 2, figsize=(20, 12))
        self.ax_true = self.axes[0, 0]      # 真实迷宫
        self.ax_global = self.axes[0, 1]    # 全局SLAM地图
        self.ax_frontiers = self.axes[1, 0] # 前沿探索图
        self.ax_status = self.axes[1, 1]    # 状态信息
        
        # 机器人颜色
        self.robot_colors = ['blue', 'red', 'green', 'orange', 'purple', 'brown']
        
        # 当前扫描数据
        self.current_scan_data = {}
        
        plt.ion()
    
    def update_scan_data(self, robot_id, robot_pos, scan_points, obstacle_points, scan_ranges, scan_angles):
        """更新扫描数据"""
        self.current_scan_data[robot_id] = {
            'pos': robot_pos,
            'scan_points': scan_points,
            'obstacle_points': obstacle_points,
            'scan_ranges': scan_ranges,
            'scan_angles': scan_angles
        }
    
    def draw_true_maze(self, robot_positions, robot_targets):
        """绘制真实迷宫"""
        ax = self.ax_true
        ax.clear()
        
        # 绘制扩展区域的灰色背景（不可通行但可扫描的区域）
        # 左边区域 (-2到0)
        ax.fill_between([-2, 0], -2, self.maze_env.size + 2, color='lightgray', alpha=0.3, zorder=0)
        # 右边区域 (max到max+2)
        ax.fill_between([self.maze_env.size, self.maze_env.size + 2], -2, self.maze_env.size + 2, color='lightgray', alpha=0.3, zorder=0)
        # 下边区域 (0到max, -2到0)
        ax.fill_between([0, self.maze_env.size], -2, 0, color='lightgray', alpha=0.3, zorder=0)
        # 上边区域 (0到max, max到max+2)
        ax.fill_between([0, self.maze_env.size], self.maze_env.size, self.maze_env.size + 2, color='lightgray', alpha=0.3, zorder=0)
        
        # 绘制墙壁
        for wall in self.maze_env.walls:
            (x1, y1), (x2, y2) = wall
            ax.plot([x1, x2], [y1, y2], 'k-', linewidth=3)
        
        # 绘制起始位置
        start_x, start_y = self.maze_env.start_pos
        ax.plot(start_x, start_y, 'go', markersize=10, label='Start')
        
        # 绘制已发现的出口
        for i, exit_pos in enumerate(self.maze_env.discovered_exits):
            exit_x, exit_y = exit_pos
            ax.plot(exit_x, exit_y, 'rs', markersize=10, label='Discovered Exit' if i == 0 else '')
            # 添加出口标识
            ax.text(exit_x + 0.1, exit_y + 0.1, f'Exit{i+1}', fontsize=8, color='red')
        
        # 绘制机器人到达出口时的位置标记（红色小圈）
        for i, reached_pos in enumerate(self.maze_env.reached_exit_positions):
            reached_x, reached_y = reached_pos
            ax.plot(reached_x, reached_y, 'ro', markersize=12, markerfacecolor='red', 
                   markeredgecolor='darkred', markeredgewidth=2, alpha=0.8,
                   label='Exit Reached' if i == 0 else '')
            # 添加到达标识
            ax.text(reached_x + 0.15, reached_y + 0.15, f'REACHED', fontsize=7, 
                   color='darkred', fontweight='bold')
        
        # 绘制机器人和路径
        for i, (robot_id, pos) in enumerate(robot_positions.items()):
            color = self.robot_colors[i % len(self.robot_colors)]
            
            # 绘制机器人
            ax.plot(pos[0], pos[1], 'o', color=color, markersize=8, label=robot_id)
            
            # 绘制路径
            if robot_id in self.global_mapper.robot_paths:
                path = self.global_mapper.robot_paths[robot_id]
                if len(path) > 1:
                    path_x = [p[0] for p in path]
                    path_y = [p[1] for p in path]
                    ax.plot(path_x, path_y, '--', color=color, alpha=0.5, linewidth=1)
            
            # 绘制目标点
            if robot_id in robot_targets and robot_targets[robot_id]:
                target = robot_targets[robot_id]
                ax.plot(target[0], target[1], 'x', color=color, markersize=12, markeredgewidth=3)
                # 绘制目标连线
                ax.plot([pos[0], target[0]], [pos[1], target[1]], ':', color=color, alpha=0.7)
        
        ax.set_xlim(-2, self.maze_env.size + 2)
        ax.set_ylim(-2, self.maze_env.size + 2)
        ax.set_aspect('equal')
        ax.legend(loc='upper right', fontsize=8, ncol=2)
        ax.set_title("True Maze + Robot Targets", fontsize=14)
    
    def draw_global_slam_map(self):
        """绘制全局SLAM地图"""
        ax = self.ax_global
        ax.clear()
        
        # 所有扩展区域都不绘制背景，让其保持与未扫描区域相同的颜色
        
        # 创建颜色地图
        cmap = colors.ListedColormap(['lightgray', 'white', 'black'])
        ax.imshow(self.global_mapper.global_map, cmap=cmap, origin='lower', 
                 extent=[-2, self.global_mapper.display_size-2, -2, self.global_mapper.display_size-2], alpha=0.8)
        
        # 绘制所有机器人位置
        for i, (robot_id, path) in enumerate(self.global_mapper.robot_paths.items()):
            if path:
                color = self.robot_colors[i % len(self.robot_colors)]
                current_pos = path[-1]
                ax.plot(current_pos[0], current_pos[1], 'o', color=color, markersize=6)
        
        ax.set_xlim(-2, self.maze_env.size + 2)
        ax.set_ylim(-2, self.maze_env.size + 2)
        ax.set_aspect('equal')
        ax.set_title("Global SLAM Map", fontsize=14)
    
    def draw_frontier_exploration(self):
        """绘制前沿探索图"""
        ax = self.ax_frontiers
        ax.clear()
        
        # 所有扩展区域都不绘制背景，让其保持与未扫描区域相同的颜色
        
        # 绘制全局地图背景
        cmap = colors.ListedColormap(['lightgray', 'white', 'black'])
        ax.imshow(self.global_mapper.global_map, cmap=cmap, origin='lower', 
                 extent=[-2, self.global_mapper.display_size-2, -2, self.global_mapper.display_size-2], alpha=0.7)
        
        # 绘制前沿点
        if self.global_mapper.frontiers:
            frontier_x = [f[0] for f in self.global_mapper.frontiers]
            frontier_y = [f[1] for f in self.global_mapper.frontiers]
            ax.plot(frontier_x, frontier_y, 'r*', markersize=8, alpha=0.8, label=f'Frontiers ({len(self.global_mapper.frontiers)})')
        
        # 绘制机器人位置
        for i, (robot_id, path) in enumerate(self.global_mapper.robot_paths.items()):
            if path:
                color = self.robot_colors[i % len(self.robot_colors)]
                current_pos = path[-1]
                ax.plot(current_pos[0], current_pos[1], 'o', color=color, markersize=8)
        
        ax.set_xlim(-2, self.maze_env.size + 2)
        ax.set_ylim(-2, self.maze_env.size + 2)
        ax.set_aspect('equal')
        ax.legend()
        ax.set_title("Frontier Exploration", fontsize=14)
    
    def draw_status_info(self, robots_info, exploration_stats):
        """绘制状态信息"""
        ax = self.ax_status
        ax.clear()
        ax.set_xlim(0, 10)
        ax.set_ylim(0, 12)
        
        # 显示机器人状态（分两列）
        left_robots = list(robots_info.items())[:3]
        right_robots = list(robots_info.items())[3:]
        
        # 左列
        y_pos = 11
        for robot_id, info in left_robots:
            color = self.robot_colors[list(robots_info.keys()).index(robot_id) % len(self.robot_colors)]
            
            status_text = f"{robot_id}:\n"
            status_text += f"  Pos: ({info['pos'][0]:.1f}, {info['pos'][1]:.1f})\n"
            status_text += f"  Steps: {info['steps']}\n"
            status_text += f"  Status: {info['status']}\n"
            if info['target']:
                status_text += f"  Target: ({info['target'][0]:.1f}, {info['target'][1]:.1f})"
            
            ax.text(0.5, y_pos, status_text, fontsize=9, color=color, 
                   verticalalignment='top', fontweight='bold')
            y_pos -= 3.5
        
        # 右列
        y_pos = 11
        for robot_id, info in right_robots:
            color = self.robot_colors[list(robots_info.keys()).index(robot_id) % len(self.robot_colors)]
            
            status_text = f"{robot_id}:\n"
            status_text += f"  Pos: ({info['pos'][0]:.1f}, {info['pos'][1]:.1f})\n"
            status_text += f"  Steps: {info['steps']}\n"
            status_text += f"  Status: {info['status']}\n"
            if info['target']:
                status_text += f"  Target: ({info['target'][0]:.1f}, {info['target'][1]:.1f})"
            
            ax.text(5.5, y_pos, status_text, fontsize=9, color=color, 
                   verticalalignment='top', fontweight='bold')
            y_pos -= 3.5
        
        # 全局统计
        stats_text = f"Exploration Progress:\n"
        stats_text += f"Frontiers: {exploration_stats['frontiers']}\n"
        stats_text += f"Coverage: {exploration_stats['coverage']:.1f}%\n"
        stats_text += f"Active Robots: {exploration_stats['active_robots']}/{self.num_robots}"
        
        ax.text(5, 1.5, stats_text, fontsize=11, fontweight='bold', 
               horizontalalignment='center', verticalalignment='center',
               bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
        
        ax.set_title("Robot Status & Progress", fontsize=14)
        ax.axis('off')
    
    def update_display(self, robot_positions, robot_targets, robots_info, exploration_stats):
        """更新显示"""
        self.draw_true_maze(robot_positions, robot_targets)
        self.draw_global_slam_map()
        self.draw_frontier_exploration()
        self.draw_status_info(robots_info, exploration_stats)
        plt.tight_layout()
        plt.subplots_adjust(top=0.95, bottom=0.05, left=0.05, right=0.97, hspace=0.2, wspace=0.2)
        plt.pause(0.001)

class SingleRobotVisualizer:
    """单机器人可视化器（简化版）"""
    
    def __init__(self, maze_env, global_mapper, robot):
        self.maze_env = maze_env
        self.global_mapper = global_mapper
        self.robot = robot
        
        # 创建2x2面板显示
        self.fig, self.axes = plt.subplots(2, 2, figsize=(16, 12))
        self.ax_true = self.axes[0, 0]      # 真实迷宫 + 机器人
        self.ax_slam = self.axes[0, 1]      # SLAM地图
        self.ax_frontiers = self.axes[1, 0] # 前沿探索图
        self.ax_radar = self.axes[1, 1]     # 雷达模拟图
        
        # 存储最新的扫描数据
        self.latest_scan_points = []
        self.latest_obstacle_points = []
        self.latest_scan_ranges = []
        self.latest_scan_angles = []
        
        # 最短路径
        self.shortest_path = None
        
        plt.ion()
    
    def update_scan_data(self, scan_points, obstacle_points, scan_ranges, scan_angles):
        """更新扫描数据"""
        self.latest_scan_points = scan_points
        self.latest_obstacle_points = obstacle_points  
        self.latest_scan_ranges = scan_ranges
        self.latest_scan_angles = scan_angles
    
    def set_shortest_path(self, shortest_path):
        """设置最短路径"""
        self.shortest_path = shortest_path
    
    def update_display(self):
        """更新显示"""
        self.draw_true_maze()
        self.draw_slam_map()
        self.draw_frontiers()
        self.draw_radar_simulation()
        
        # 显示状态信息
        status_text = f"Robot: {self.robot.robot_id}\n"
        status_text += f"Position: ({self.robot.position[0]:.2f}, {self.robot.position[1]:.2f})\n"
        status_text += f"Steps: {self.robot.steps}\n"
        status_text += f"Status: {self.robot.status}\n"
        status_text += f"No Move Count: {self.robot.no_move_counter}/3\n"
        status_text += f"Frontiers: {len(self.global_mapper.frontiers)}\n"
        status_text += f"Coverage: {self._calculate_coverage():.1f}%\n"
        status_text += f"Discovered Exits: {len(self.maze_env.discovered_exits)}\n"
        status_text += f"Exit Reached: {len(self.maze_env.reached_exit_positions)}\n"
        
        # 显示最短路径状态
        if self.shortest_path:
            path_length = 0
            diagonal_count = 0
            for i in range(len(self.shortest_path) - 1):
                p1 = self.shortest_path[i]
                p2 = self.shortest_path[i + 1]
                segment_length = math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
                path_length += segment_length
                # 统计对角线段
                dx = abs(p2[0] - p1[0])
                dy = abs(p2[1] - p1[1])
                if dx > 0.01 and dy > 0.01:
                    diagonal_count += 1
            status_text += f"Optimal Path: {path_length:.3f} units\n"
            status_text += f"Diagonal Segments: {diagonal_count}\n"
        
        # 显示探索状态
        if len(self.maze_env.reached_exit_positions) > 0:
            status_text += f"🔴 EXIT REACHED! MARKED!"
            if self.shortest_path:
                status_text += f"\n💚 SHORTEST PATH SHOWN!"
        elif len(self.maze_env.discovered_exits) == 0:
            status_text += f"🔍 EXPLORING FOR EXITS..."
        else:
            status_text += f"🎯 EXITS FOUND!"
        
        self.fig.suptitle(status_text, fontsize=11, y=0.98)
        
        plt.tight_layout()
        plt.subplots_adjust(top=0.92, bottom=0.08, left=0.08, right=0.95, hspace=0.25, wspace=0.25)
        plt.pause(0.001)  # 极快的更新频率
    
    def draw_true_maze(self):
        """绘制真实迷宫"""
        ax = self.ax_true
        ax.clear()
        
        # 绘制扩展区域的灰色背景（不可通行但可扫描的区域）
        # 左边区域 (-2到0)
        ax.fill_between([-2, 0], -2, self.maze_env.size + 2, color='lightgray', alpha=0.3, zorder=0)
        # 右边区域 (max到max+2)
        ax.fill_between([self.maze_env.size, self.maze_env.size + 2], -2, self.maze_env.size + 2, color='lightgray', alpha=0.3, zorder=0)
        # 下边区域 (0到max, -2到0)
        ax.fill_between([0, self.maze_env.size], -2, 0, color='lightgray', alpha=0.3, zorder=0)
        # 上边区域 (0到max, max到max+2)
        ax.fill_between([0, self.maze_env.size], self.maze_env.size, self.maze_env.size + 2, color='lightgray', alpha=0.3, zorder=0)
        
        # 绘制墙壁
        for wall in self.maze_env.walls:
            (x1, y1), (x2, y2) = wall
            ax.plot([x1, x2], [y1, y2], 'k-', linewidth=3)
        
        # 绘制起始位置
        start_x, start_y = self.maze_env.start_pos
        ax.plot(start_x, start_y, 'go', markersize=8, label='Start')
        
        # 绘制已发现的出口
        for i, exit_pos in enumerate(self.maze_env.discovered_exits):
            exit_x, exit_y = exit_pos
            ax.plot(exit_x, exit_y, 'rs', markersize=8, label='Discovered Exit' if i == 0 else '')
            # 添加出口标识
            ax.text(exit_x + 0.1, exit_y + 0.1, f'Exit{i+1}', fontsize=8, color='red')
        
        # 绘制机器人到达出口时的位置标记（红色小圈）
        for i, reached_pos in enumerate(self.maze_env.reached_exit_positions):
            reached_x, reached_y = reached_pos
            # 使用空心圆圈，让蓝色机器人点保持可见
            ax.plot(reached_x, reached_y, 'o', markersize=20, markerfacecolor='none',
                    markeredgecolor='red', markeredgewidth=3, alpha=0.9, zorder=10,
                    label='🔴 EXIT REACHED' if i == 0 else '')
            # 添加文本标签（放在圆圈外侧）
            ax.text(reached_x + 0.25, reached_y + 0.25, 'EXIT', fontsize=9,
                    color='darkred', fontweight='bold', zorder=11)
        
        # 绘制机器人路径
        if self.robot.robot_id in self.global_mapper.robot_paths:
            path = self.global_mapper.robot_paths[self.robot.robot_id]
            if len(path) > 1:
                path_x = [p[0] for p in path]
                path_y = [p[1] for p in path]
                ax.plot(path_x, path_y, 'b--', alpha=0.6, linewidth=2, label='Path')
        
        # 绘制最短路径（如果存在）
        if self.shortest_path and len(self.shortest_path) > 1:
            path_x = [p[0] for p in self.shortest_path]
            path_y = [p[1] for p in self.shortest_path]
            
            # 绘制主要路径线
            ax.plot(path_x, path_y, 'g-', linewidth=4, alpha=0.8, label='Optimal Path (8-dir)', zorder=5)
            
            # 绘制路径段标记：直线段和对角线段用不同样式
            for i in range(len(self.shortest_path) - 1):
                p1 = self.shortest_path[i]
                p2 = self.shortest_path[i + 1]
                
                # 判断是否为对角线移动
                dx = abs(p2[0] - p1[0])
                dy = abs(p2[1] - p1[1])
                is_diagonal = dx > 0.01 and dy > 0.01
                
                if is_diagonal:
                    # 对角线段用虚线标记
                    ax.plot([p1[0], p2[0]], [p1[1], p2[1]], 'g--', 
                           linewidth=2, alpha=0.6, zorder=4)
                    # 在对角线段中点添加小圆点
                    mid_x = (p1[0] + p2[0]) / 2
                    mid_y = (p1[1] + p2[1]) / 2
                    ax.plot(mid_x, mid_y, 'go', markersize=4, alpha=0.7, zorder=4)
            
            # 标记起点和终点
            ax.plot(path_x[0], path_y[0], 'go', markersize=12, markeredgewidth=2, 
                   markeredgecolor='darkgreen', label='Start', zorder=6)
            ax.plot(path_x[-1], path_y[-1], 'gs', markersize=12, markeredgewidth=2, 
                   markeredgecolor='darkgreen', label='End', zorder=6)
        
        # 绘制机器人当前位置
        ax.plot(self.robot.position[0], self.robot.position[1], 'bo', markersize=10, label='Robot')
        
        # 绘制目标点
        if self.robot.current_target:
            target = self.robot.current_target
            ax.plot(target[0], target[1], 'rx', markersize=12, markeredgewidth=3, label='Target')
            # 绘制目标连线
            ax.plot([self.robot.position[0], target[0]], 
                   [self.robot.position[1], target[1]], 'r:', alpha=0.7)
        
        ax.set_xlim(-2, self.maze_env.size + 2)
        ax.set_ylim(-2, self.maze_env.size + 2)
        ax.set_aspect('equal')
        ax.legend(loc='upper right', fontsize=8)
        ax.set_title("True Maze + Robot", fontsize=12)
    
    def draw_slam_map(self):
        """绘制SLAM地图"""
        ax = self.ax_slam
        ax.clear()
        
        # 所有扩展区域都不绘制背景，让其保持与未扫描区域相同的颜色
        
        # 创建颜色地图
        cmap = colors.ListedColormap(['lightgray', 'white', 'black'])
        ax.imshow(self.global_mapper.global_map, cmap=cmap, origin='lower', 
                 extent=[-2, self.global_mapper.display_size-2, -2, self.global_mapper.display_size-2], alpha=0.8)
        
        # 绘制机器人到达出口时的位置标记（红色小圈）
        for i, reached_pos in enumerate(self.maze_env.reached_exit_positions):
            reached_x, reached_y = reached_pos
            ax.plot(reached_x, reached_y, 'o', markersize=20, markerfacecolor='none',
                    markeredgecolor='red', markeredgewidth=3, alpha=0.9,
                    label='🔴 EXIT REACHED' if i == 0 else '')
        
        # 绘制最短路径（如果存在）
        if self.shortest_path and len(self.shortest_path) > 1:
            path_x = [p[0] for p in self.shortest_path]
            path_y = [p[1] for p in self.shortest_path]
            ax.plot(path_x, path_y, 'g-', linewidth=3, alpha=0.8, label='Optimal Path (8-dir)')
            
            # 标记对角线段
            for i in range(len(self.shortest_path) - 1):
                p1 = self.shortest_path[i]
                p2 = self.shortest_path[i + 1]
                dx = abs(p2[0] - p1[0])
                dy = abs(p2[1] - p1[1])
                if dx > 0.01 and dy > 0.01:  # 对角线移动
                    ax.plot([p1[0], p2[0]], [p1[1], p2[1]], 'g--', 
                           linewidth=2, alpha=0.5, zorder=3)
            
            # 标记起点和终点
            ax.plot(path_x[0], path_y[0], 'go', markersize=10, markeredgewidth=2, 
                   markeredgecolor='darkgreen')
            ax.plot(path_x[-1], path_y[-1], 'gs', markersize=10, markeredgewidth=2, 
                   markeredgecolor='darkgreen')
        
        # 绘制机器人位置
        ax.plot(self.robot.position[0], self.robot.position[1], 'bo', markersize=8)
        
        ax.set_xlim(-2, self.maze_env.size + 2)
        ax.set_ylim(-2, self.maze_env.size + 2)
        ax.set_aspect('equal')
        ax.set_title("SLAM Map", fontsize=12)
    
    def draw_frontiers(self):
        """绘制前沿探索图"""
        ax = self.ax_frontiers
        ax.clear()
        
        # 所有扩展区域都不绘制背景，让其保持与未扫描区域相同的颜色
        
        # 绘制全局地图背景
        cmap = colors.ListedColormap(['lightgray', 'white', 'black'])
        ax.imshow(self.global_mapper.global_map, cmap=cmap, origin='lower', 
                 extent=[-2, self.global_mapper.display_size-2, -2, self.global_mapper.display_size-2], alpha=0.7)
        
        # 绘制机器人到达出口时的位置标记（红色小圈）
        for i, reached_pos in enumerate(self.maze_env.reached_exit_positions):
            reached_x, reached_y = reached_pos
            ax.plot(reached_x, reached_y, 'o', markersize=20, markerfacecolor='none',
                    markeredgecolor='red', markeredgewidth=3, alpha=0.9,
                    label='🔴 EXIT REACHED' if i == 0 else '')
        
        # 绘制前沿点
        if self.global_mapper.frontiers:
            frontier_x = [f[0] for f in self.global_mapper.frontiers]
            frontier_y = [f[1] for f in self.global_mapper.frontiers]
            ax.plot(frontier_x, frontier_y, 'r*', markersize=6, alpha=0.8, 
                   label=f'Frontiers ({len(self.global_mapper.frontiers)})')
        
        # 绘制最短路径（如果存在）
        if self.shortest_path and len(self.shortest_path) > 1:
            path_x = [p[0] for p in self.shortest_path]
            path_y = [p[1] for p in self.shortest_path]
            ax.plot(path_x, path_y, 'g-', linewidth=3, alpha=0.8, label='Optimal Path (8-dir)')
            
            # 标记对角线段
            for i in range(len(self.shortest_path) - 1):
                p1 = self.shortest_path[i]
                p2 = self.shortest_path[i + 1]
                dx = abs(p2[0] - p1[0])
                dy = abs(p2[1] - p1[1])
                if dx > 0.01 and dy > 0.01:  # 对角线移动
                    ax.plot([p1[0], p2[0]], [p1[1], p2[1]], 'g--', 
                           linewidth=2, alpha=0.5, zorder=3)
            
            # 标记起点和终点
            ax.plot(path_x[0], path_y[0], 'go', markersize=10, markeredgewidth=2, 
                   markeredgecolor='darkgreen')
            ax.plot(path_x[-1], path_y[-1], 'gs', markersize=10, markeredgewidth=2, 
                   markeredgecolor='darkgreen')
        
        # 绘制机器人位置
        ax.plot(self.robot.position[0], self.robot.position[1], 'bo', markersize=8)
        
        ax.set_xlim(-2, self.maze_env.size + 2)
        ax.set_ylim(-2, self.maze_env.size + 2)
        ax.set_aspect('equal')
        ax.legend()
        ax.set_title("Frontier Exploration", fontsize=12)
    
    def draw_radar_simulation(self):
        """绘制雷达模拟图"""
        ax = self.ax_radar
        ax.clear()
        
        # 所有扩展区域都不绘制背景，让其保持与未扫描区域相同的颜色
        
        # 绘制迷宫墙壁
        for wall in self.maze_env.walls:
            (x1, y1), (x2, y2) = wall
            ax.plot([x1, x2], [y1, y2], 'k-', linewidth=2)
        
        # 绘制机器人位置
        robot_x, robot_y = self.robot.position
        ax.plot(robot_x, robot_y, 'bo', markersize=10, label='Robot')
        
        # 绘制雷达射线
        if self.latest_scan_ranges and self.latest_scan_angles:
            for i, (distance, angle_deg) in enumerate(zip(self.latest_scan_ranges, self.latest_scan_angles)):
                # 计算射线终点
                angle_rad = math.radians(angle_deg)
                end_x = robot_x + distance * math.cos(angle_rad)
                end_y = robot_y + distance * math.sin(angle_rad)
                
                # 绘制射线（从机器人到命中点）
                ax.plot([robot_x, end_x], [robot_y, end_y], 'r-', alpha=0.3, linewidth=0.5)
        
        # 绘制自由空间点（扫描到的点）
        if self.latest_scan_points:
            scan_x = [p[0] for p in self.latest_scan_points]
            scan_y = [p[1] for p in self.latest_scan_points]
            ax.plot(scan_x, scan_y, 'g.', markersize=1, alpha=0.5, label='Free Space')
        
        # 绘制障碍点
        if self.latest_obstacle_points:
            obs_x = [p[0] for p in self.latest_obstacle_points]
            obs_y = [p[1] for p in self.latest_obstacle_points]
            ax.plot(obs_x, obs_y, 'ro', markersize=3, alpha=0.8, label='Obstacles')
         
        ax.set_xlim(-2, self.maze_env.size + 2)
        ax.set_ylim(-2, self.maze_env.size + 2)
        ax.set_aspect('equal')
        ax.legend()
        ax.set_title("Radar Simulation", fontsize=12)
    
    def _calculate_coverage(self):
        """计算探索覆盖率"""
        total_cells = self.global_mapper.grid_size * self.global_mapper.grid_size
        explored_cells = len(self.global_mapper.all_explored_cells)
        return (explored_cells / total_cells) * 100

class SmartMazeExplorer:
    """智能迷宫探索器 - 前沿探索算法"""
    
    def __init__(self, robot_id, maze_env, global_mapper, laser_sim, path_planner):
        self.robot_id = robot_id
        self.maze_env = maze_env
        self.global_mapper = global_mapper
        self.laser_sim = laser_sim
        self.path_planner = path_planner
        
        # 机器人状态
        self.position = self.maze_env.start_pos
        self.steps = 0
        self.max_steps = float('inf')  # 取消步数上限，直到发现出口或外部条件终止
        self.status = "Exploring"
        
        # 探索目标
        self.current_target = None
        self.target_path = []
        self.path_index = 0
        
        # 移动步长（更小以确保平滑移动）
        self.step_size = 0.1
        
        # 探索策略参数
        self.exploration_priority = random.random()  # 随机探索优先级
        
        # 移动状态追踪
        self.stuck_counter = 0  # 卡住计数器
        self.no_move_counter = 0  # 不移动计数器
        self.last_position = self.position  # 上一次位置
        
        # 存储最新的扫描数据（用于雷达可视化）
        self.latest_scan_points = []
        self.latest_obstacle_points = []
        self.latest_scan_ranges = []
        self.latest_scan_angles = []
        
    def update(self):
        """更新机器人状态（优化版）"""
        if self.status in ["Exit Found", "Max Steps", "Stopped", "Stuck"]:
            return False
        
        # 不再因步数达到上限而终止
        
        # 记录移动前位置
        old_position = self.position
        
        # 执行激光扫描
        scan_points, obstacle_points, scan_ranges, scan_angles, exit_found = self.laser_sim.scan(self.position)
        
        # 存储扫描数据用于可视化
        self.latest_scan_points = scan_points
        self.latest_obstacle_points = obstacle_points
        self.latest_scan_ranges = scan_ranges
        self.latest_scan_angles = scan_angles
        
        # 更新全局地图
        self.global_mapper.update_map(self.robot_id, self.position, scan_points, obstacle_points)
        
        # 高速模式：每3步更新一次前沿点，提高速度
        if self.steps % 3 == 0:
            self.global_mapper.update_frontiers()
        
        # 检查是否找到出口
        if exit_found:
            self.status = "Exit Found"
            print(f"🎯 {self.robot_id} found exit at {self.position}!")
            return False
        
        # 检查是否超过3轮不动，如果是则强制随机移动
        if self.no_move_counter >= 3:
            moved = self._force_safe_random_move()
            if moved:
                self.no_move_counter = 0
                self.last_position = self.position
                self.status = "Forced Move"
            else:
                self.no_move_counter += 1
        else:
            # 选择探索目标
            if not self.current_target or self._reached_target():
                self._select_new_target()
            
            # 执行移动
            moved = False
            if self.current_target:
                moved = self._move_towards_target()
            else:
                # 没有目标时，随机探索
                moved = self._random_exploration()
            
            # 检查是否实际移动了位置
            position_changed = (abs(self.position[0] - self.last_position[0]) > 0.01 or 
                              abs(self.position[1] - self.last_position[1]) > 0.01)
            
            if position_changed:
                self.no_move_counter = 0  # 重置不移动计数器
                self.last_position = self.position
            else:
                self.no_move_counter += 1  # 增加不移动计数器
            
            # 检查是否移动成功
            if not moved:
                self.stuck_counter += 1
                if self.stuck_counter > 10:  # 连续10次无法移动
                    self.status = "Stuck"
                    return False
            else:
                self.stuck_counter = 0  # 重置卡住计数器
        
        self.steps += 1
        return True
    
    def _select_new_target(self):
        """选择新的探索目标"""
        # 获取所有机器人位置（用于分配前沿点）
        robot_positions = {}
        for rid, path in self.global_mapper.robot_paths.items():
            if path:
                robot_positions[rid] = path[-1]
        
        # 分配前沿目标
        frontier_assignments = self.global_mapper.assign_frontiers_to_robots(robot_positions)
        
        if self.robot_id in frontier_assignments:
            # 分配到前沿目标
            target = frontier_assignments[self.robot_id]
            self.current_target = target
            self.status = "To Frontier"
            
            # 规划路径
            self.target_path = self.path_planner.plan_path(self.position, target)
            self.path_index = 0

        else:
            # 没有前沿点，尝试探索边界区域
            self._explore_boundary_areas()
    
    def _explore_boundary_areas(self):
        """探索边界区域寻找出口"""
        # 优先探索迷宫边界
        boundary_targets = []
        
        # 生成边界探索点
        for i in range(0, int(self.maze_env.size), 2):
            # 上边界
            boundary_targets.append((i + 0.5, self.maze_env.size - 0.5))
            # 下边界
            boundary_targets.append((i + 0.5, 0.5))
            # 左边界
            boundary_targets.append((0.5, i + 0.5))
            # 右边界
            boundary_targets.append((self.maze_env.size - 0.5, i + 0.5))
        
        # 找到最近的未探索边界点
        min_dist = float('inf')
        best_target = None
        
        for target in boundary_targets:
            # 检查是否已经被探索
            grid_pos = self.global_mapper.world_to_grid(target)
            if self.global_mapper.global_map[grid_pos[1], grid_pos[0]] == 0:  # 未探索
                dist = math.sqrt((self.position[0] - target[0])**2 + (self.position[1] - target[1])**2)
                if dist < min_dist:
                    min_dist = dist
                    best_target = target
        
        if best_target:
            self.current_target = best_target
            self.status = "To Boundary"
            self.target_path = self.path_planner.plan_path(self.position, best_target)
            self.path_index = 0
        else:
            # 所有区域都已探索
            self.status = "Complete"
            self.current_target = None
    
    def _reached_target(self):
        """检查是否到达目标"""
        if not self.current_target:
            return True
        
        dist = math.sqrt((self.position[0] - self.current_target[0])**2 + 
                        (self.position[1] - self.current_target[1])**2)
        return dist < 0.5
    
    def _move_towards_target(self):
        """向目标移动（增强碰撞检测）"""
        if not self.target_path or self.path_index >= len(self.target_path):
            return False
        
        # 获取路径中的下一个点
        next_waypoint = self.target_path[self.path_index]
        
        # 计算移动方向
        dx = next_waypoint[0] - self.position[0]
        dy = next_waypoint[1] - self.position[1]
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < self.step_size:
            # 到达当前路径点，但要验证是否安全
            if self._is_movement_safe(self.position, next_waypoint):
                self.position = next_waypoint
                self.path_index += 1
                return True
            else:
                # 路径点不安全，重新规划
                self._replan_path()
                return False
        else:
            # 向当前路径点移动
            move_x = self.position[0] + (dx / distance) * self.step_size
            move_y = self.position[1] + (dy / distance) * self.step_size
            new_position = (move_x, move_y)
            
            # 严格检查移动安全性
            if self._is_movement_safe(self.position, new_position):
                self.position = new_position
                return True
            else:
                # 路径被阻挡，尝试绕行或重新规划
                if self._try_avoid_obstacle():
                    return True
                else:
                    self._replan_path()
                    return False
    
    def _is_movement_safe(self, from_pos, to_pos):
        """检查移动是否安全（多重验证，限制在0到size区域）"""
        # 基础碰撞检测
        if not self.maze_env.can_move_to(from_pos, to_pos):
            return False
        
        # 边界检查到有效移动区域（允许扩展区域 -2 到 size+2，保留小的安全边距）
        margin = 0.05
        extended_margin = 2.0 - margin
        if not (-extended_margin <= to_pos[0] <= self.maze_env.size + extended_margin and 
                -extended_margin <= to_pos[1] <= self.maze_env.size + extended_margin):
            return False
        
        # 墙壁距离检查
        safety_distance = 0.15
        for wall in self.maze_env.walls + self.maze_env.invisible_walls:
            if self._point_to_line_distance(to_pos, wall[0], wall[1]) < safety_distance:
                return False
        
        return True
    
    def _point_to_line_distance(self, point, line_start, line_end):
        """计算点到线段的距离"""
        x0, y0 = point
        x1, y1 = line_start
        x2, y2 = line_end
        
        # 线段长度
        line_len = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        if line_len == 0:
            return math.sqrt((x0 - x1)**2 + (y0 - y1)**2)
        
        # 计算投影参数
        t = max(0, min(1, ((x0 - x1) * (x2 - x1) + (y0 - y1) * (y2 - y1)) / (line_len**2)))
        
        # 投影点
        proj_x = x1 + t * (x2 - x1)
        proj_y = y1 + t * (y2 - y1)
        
        # 距离
        return math.sqrt((x0 - proj_x)**2 + (y0 - proj_y)**2)
    
    def _try_avoid_obstacle(self):
        """尝试避开障碍物"""
        # 尝试左右绕行
        angles = [math.pi/4, -math.pi/4, math.pi/2, -math.pi/2]  # 45度、90度左右
        
        for angle in angles:
            # 计算绕行方向
            cos_a, sin_a = math.cos(angle), math.sin(angle)
            new_x = self.position[0] + self.step_size * cos_a
            new_y = self.position[1] + self.step_size * sin_a
            new_pos = (new_x, new_y)
            
            if self._is_movement_safe(self.position, new_pos):
                self.position = new_pos
                return True
        
        return False
    
    def _replan_path(self):
        """重新规划路径"""
        if self.current_target:
            self.target_path = self.path_planner.plan_path(self.position, self.current_target)
            self.path_index = 0
            
            # 如果重新规划后路径太短，说明目标不可达
            if len(self.target_path) <= 1:
                self.current_target = None
                self.status = "Target Unreachable"
    
    def _random_exploration(self):
        """随机探索（当没有目标时）（增强安全性）"""
        # 尝试多个随机方向
        max_attempts = 8
        
        for _ in range(max_attempts):
            # 随机选择方向
            angle = random.uniform(0, 2 * math.pi)
            move_x = self.position[0] + self.step_size * math.cos(angle)
            move_y = self.position[1] + self.step_size * math.sin(angle)
            new_position = (move_x, move_y)
            
            # 使用增强的安全检查
            if self._is_movement_safe(self.position, new_position):
                self.position = new_position
                self.status = "Random"
                return True
        
        # 如果所有随机方向都被阻挡，尝试固定方向
        fixed_directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        
        for dx, dy in fixed_directions:
            move_x = self.position[0] + self.step_size * dx
            move_y = self.position[1] + self.step_size * dy
            new_position = (move_x, move_y)
            
            if self._is_movement_safe(self.position, new_position):
                self.position = new_position
                self.status = "Random"
                return True
        
        # 如果完全无法移动，保持原位
        self.status = "No Move"
        return False
    
    def _force_safe_random_move(self):
        """强制安全随机移动（当小车超过2轮不动时）"""
        print(f"🚨 Forcing safe random move for {self.robot_id}...")
        
        # 尝试更多方向和更小的步长
        step_sizes = [self.step_size, self.step_size * 0.7, self.step_size * 0.5]  # 尝试不同步长
        
        for step_size in step_sizes:
            # 尝试12个方向（每30度一个）
            for angle_deg in range(0, 360, 30):
                angle_rad = math.radians(angle_deg)
                move_x = self.position[0] + step_size * math.cos(angle_rad)
                move_y = self.position[1] + step_size * math.sin(angle_rad)
                new_position = (move_x, move_y)
                
                # 使用增强的安全检查
                if self._is_movement_safe(self.position, new_position):
                    print(f"✅ Found safe direction: {angle_deg}° with step size {step_size:.2f}")
                    self.position = new_position
                    self.status = "Forced Random"
                    return True
            
            # 如果12个方向都不行，尝试更精细的方向
            for angle_deg in range(15, 360, 15):  # 每15度一个
                angle_rad = math.radians(angle_deg)
                move_x = self.position[0] + step_size * math.cos(angle_rad)
                move_y = self.position[1] + step_size * math.sin(angle_rad)
                new_position = (move_x, move_y)
                
                if self._is_movement_safe(self.position, new_position):
                    print(f"✅ Found safe direction: {angle_deg}° with step size {step_size:.2f}")
                    self.position = new_position
                    self.status = "Forced Random"
                    return True
        
        # 如果所有方向都被阻挡，尝试非常小的移动
        tiny_step = self.step_size * 0.2
        tiny_directions = [(1, 0), (-1, 0), (0, 1), (0, -1), 
                          (0.7, 0.7), (-0.7, 0.7), (0.7, -0.7), (-0.7, -0.7)]
        
        for dx, dy in tiny_directions:
            # 标准化方向向量
            length = math.sqrt(dx*dx + dy*dy)
            dx, dy = dx/length, dy/length
            
            move_x = self.position[0] + tiny_step * dx
            move_y = self.position[1] + tiny_step * dy
            new_position = (move_x, move_y)
            
            if self._is_movement_safe(self.position, new_position):
                print(f"✅ Found tiny safe move: ({dx:.2f}, {dy:.2f}) with step size {tiny_step:.2f}")
                self.position = new_position
                self.status = "Forced Tiny"
                return True
        
        print(f"❌ Cannot find any safe random move for {self.robot_id}")
        return False

class GlobalMazeSLAMSystem:
    """单机器人迷宫SLAM系统主控制器"""
    
    def __init__(self, map_file="BreezySLAM-master/examples/map1.json"):

        # 初始化环境和组件
        self.maze_env = MazeEnvironment(map_file)
        self.global_mapper = GlobalSLAMMapper(self.maze_env.size, self.maze_env.display_size)
        self.global_mapper.set_maze_env(self.maze_env)  # 设置迷宫环境引用
        self.path_planner = AStarPathPlanner(self.global_mapper, self.maze_env)
        
        # 创建单个机器人
        robot_id = "Robot-1"
        laser_sim = LaserSimulator(self.maze_env)
        
        self.robot = SmartMazeExplorer(robot_id, self.maze_env, self.global_mapper, laser_sim, self.path_planner)
        self.robot.position = self.maze_env.start_pos
        self.robot.step_size = 0.15  # 高速模式：增大步长，提高移动速度
        
        # 创建可视化器
        self.visualizer = SingleRobotVisualizer(self.maze_env, self.global_mapper, self.robot)
        
        # 最短路径相关
        self.shortest_path = None
        self.exploration_completed = False

    def run_exploration(self):
        import time
        start_time = time.time()
        target_time = 300  # 5分钟 = 300秒
        
        iteration = 0  # 统计已执行迭代次数（不再设固定上限）
        
        while True:  # 仅受时间限制或退出条件控制
            iteration += 1
            
            # 更新机器人
            robot_active = self.robot.update()
            
            # 检查时间限制（5分钟）
            elapsed_time = time.time() - start_time
            if elapsed_time > target_time:
                print(f"⏰ Time limit reached: {elapsed_time:.1f}s")
                break
            
            # 检查终止条件
            if self._check_termination_conditions(robot_active):
                break
            
            # 更新可视化（高速模式：每5次迭代更新一次）
            if iteration % 5 == 0:
                self._update_visualization()
            
            # 进度报告（高速模式：每50次报告一次）
            if iteration % 50 == 0:
                frontiers_count = len(self.global_mapper.frontiers)
                coverage = self._calculate_coverage()
                current_time = time.time() - start_time
                iterations_per_sec = iteration / current_time if current_time > 0 else 0
                estimated_total = iterations_per_sec * target_time if iterations_per_sec > 0 else 0
                print(f"⏱️  Step {iteration}: {current_time:.1f}s, "
                      f"{iterations_per_sec:.1f} iter/s, Est.Total(5m): {estimated_total:.0f}, "
                      f"Robot: ({self.robot.position[0]:.2f}, {self.robot.position[1]:.2f}), "
                      f"Status: {self.robot.status}, Coverage: {coverage:.1f}%")
        
        # 性能统计
        total_time = time.time() - start_time
        avg_speed = iteration / total_time if total_time > 0 else 0

        # 确保最短路径被传递给可视化器
        if self.exploration_completed and self.shortest_path:
            self.visualizer.set_shortest_path(self.shortest_path)
        
        # 刷新一次可视化，确保终点红圈和最短路径显示
        self._update_visualization()
        
        self._print_final_results()
        
        # 保持显示
        plt.ioff()
        plt.show()
    
    def _check_termination_conditions(self, robot_active):
        """检查终止条件"""
        # 新的终止条件：发现出口 AND 所有0-max区域的前沿点都已遍历
        if self.maze_env.exit_detected:
            # 检查是否还有0-max区域内的前沿点未遍历
            remaining_core_frontiers = 0
            for frontier in self.global_mapper.frontiers:
                if (0 <= frontier[0] <= self.maze_env.size and 
                    0 <= frontier[1] <= self.maze_env.size):
                    remaining_core_frontiers += 1
            
            if remaining_core_frontiers > 0:
                print(f"🎯 Exit found, but {remaining_core_frontiers} core frontiers remain. Continuing exploration...")
                return False
            
            if self.maze_env.exit_position and len(self.maze_env.reached_exit_positions) == 0:
                self.maze_env.mark_exit_reached(self.maze_env.exit_position)
            print("🎯 Exit found and all core frontiers explored! Exploration complete.")
            
            # 计算最短路径
            self._calculate_shortest_path()
            self.exploration_completed = True
            return True
        
        # 若机器人物理位置已经越过主迷宫边界（含扩展区），同样视为成功逃出
        robot_pos = self.robot.position
        if robot_pos[0] < -1.9 or robot_pos[0] > self.maze_env.size + 1.9 or \
           robot_pos[1] < -1.9 or robot_pos[1] > self.maze_env.size + 1.9:
            # 在极端越界位置直接标记出口
            self.maze_env.mark_exit_reached(robot_pos)
            print("🎉 Robot physically left maze boundary — exit assumed.")
            
            # 计算最短路径
            self._calculate_shortest_path()
            self.exploration_completed = True
            return True
        
        return False
    
    def _calculate_coverage(self):
        """计算探索覆盖率"""
        total_cells = self.global_mapper.grid_size * self.global_mapper.grid_size
        explored_cells = len(self.global_mapper.all_explored_cells)
        return (explored_cells / total_cells) * 100
    
    def _calculate_shortest_path(self):
        """计算从起点到终点的最短路径（使用八方向A*算法）"""
        print("🛤️  Calculating optimal shortest path from start to end...")
        
        # 获取起点和终点
        start_pos = self.maze_env.start_pos
        
        # 确定终点：优先使用已到达的出口位置，否则使用预设出口
        if self.maze_env.reached_exit_positions:
            end_pos = self.maze_env.reached_exit_positions[0]  # 使用第一个到达的出口
        elif self.maze_env.discovered_exits:
            end_pos = self.maze_env.discovered_exits[0]  # 使用第一个发现的出口
        elif self.maze_env.exits:
            end_pos = self.maze_env.exits[0]  # 使用预设出口
        else:
            print("❌ No exit found, cannot calculate shortest path")
            return
        
        # 创建专用于最短路径计算的八方向A*规划器
        optimal_planner = OptimalPathPlanner(self.global_mapper, self.maze_env)
        
        # 使用八方向A*算法计算最优路径
        self.shortest_path = optimal_planner.plan_optimal_path(start_pos, end_pos)
        
        if self.shortest_path and len(self.shortest_path) > 1:
            # 计算路径长度（精确计算，包括对角线距离）
            path_length = 0
            diagonal_segments = 0
            straight_segments = 0
            
            for i in range(len(self.shortest_path) - 1):
                p1 = self.shortest_path[i]
                p2 = self.shortest_path[i + 1]
                segment_length = math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
                path_length += segment_length
                
                # 统计直线和对角线段数
                dx = abs(p2[0] - p1[0])
                dy = abs(p2[1] - p1[1])
                if dx > 0.01 and dy > 0.01:
                    diagonal_segments += 1
                else:
                    straight_segments += 1
            
        else:
            print("❌ Failed to calculate optimal shortest path")
    
    def _update_visualization(self):
        """更新可视化"""
        # 更新扫描数据
        self.visualizer.update_scan_data(
            self.robot.latest_scan_points,
            self.robot.latest_obstacle_points,
            self.robot.latest_scan_ranges,
            self.robot.latest_scan_angles
        )
        
        # 如果探索完成，传递最短路径
        if self.exploration_completed and self.shortest_path:
            self.visualizer.set_shortest_path(self.shortest_path)
        
        # 更新显示
        self.visualizer.update_display()
    
    def _print_final_results(self):

        # 显示到已发现出口的距离
        if self.maze_env.discovered_exits:
            print(f"   Distances to discovered exits:")
            for i, exit_pos in enumerate(self.maze_env.discovered_exits):
                distance = math.sqrt((self.robot.position[0] - exit_pos[0])**2 +
                                   (self.robot.position[1] - exit_pos[1])**2)
                print(f"     Discovered Exit{i+1} at ({exit_pos[0]:.1f}, {exit_pos[1]:.1f}): {distance:.2f} units")
        else:
            print(f"   No exits discovered yet")


        if self.maze_env.exit_detected:
            print(f"   Exit detection position: {self.maze_env.exit_position}")

        # 显示边界距离
        robot_pos = self.robot.position
        distance_to_boundary = min(
            robot_pos[0],  # 距左边界
            robot_pos[1],  # 距下边界
            self.maze_env.size - robot_pos[0],  # 距右边界
            self.maze_env.size - robot_pos[1]   # 距上边界
        )
        print(f"   Distance to boundary: {distance_to_boundary:.2f} units")
        
        # 显示最短路径信息
        if self.shortest_path and len(self.shortest_path) > 1:
            path_length = 0
            diagonal_segments = 0
            straight_segments = 0
            
            for i in range(len(self.shortest_path) - 1):
                p1 = self.shortest_path[i]
                p2 = self.shortest_path[i + 1]
                segment_length = math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
                path_length += segment_length
                
                # 统计直线和对角线段数
                dx = abs(p2[0] - p1[0])
                dy = abs(p2[1] - p1[1])
                if dx > 0.01 and dy > 0.01:
                    diagonal_segments += 1
                else:
                    straight_segments += 1
            
            print(f"\n🛤️  Optimal Path Information (8-Direction A*):")
            print(f"   📍 Start: ({self.maze_env.start_pos[0]:.2f}, {self.maze_env.start_pos[1]:.2f})")
            if self.maze_env.reached_exit_positions:
                end_pos = self.maze_env.reached_exit_positions[0]
            elif self.maze_env.discovered_exits:
                end_pos = self.maze_env.discovered_exits[0]
            elif self.maze_env.exits:
                end_pos = self.maze_env.exits[0]
            else:
                end_pos = None
            
        print("="*60)

def main():
    """主程序"""
    print("🔍 Single Robot Maze SLAM Explorer - High Speed Mode")
    print("="*56)
    
    # 可以更改地图文件
    map_file = "BreezySLAM-master/examples/map1.json"
    
    try:
        # 创建并运行单机器人系统
        system = GlobalMazeSLAMSystem(map_file)
        system.run_exploration()
        
    except KeyboardInterrupt:
        print("\n⏹️  Exploration interrupted by user")
    except Exception as e:
        print(f"❌ Error during exploration: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main() 