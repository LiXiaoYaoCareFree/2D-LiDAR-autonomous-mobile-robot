"""
@file: maze_exploration.py
@breif: 基于二维激光雷达的自主移动机器人迷宫测绘与导航
"""
import sys, os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, Wedge
from matplotlib.gridspec import GridSpec
import time
import math
from matplotlib.font_manager import FontProperties
from matplotlib.animation import FuncAnimation

# 尝试设置中文字体
try:
    plt.rcParams['font.sans-serif'] = ['SimHei']  # 指定默认字体为黑体
    plt.rcParams['axes.unicode_minus'] = False  # 解决保存图像是负号'-'显示为方块的问题
except:
    print("警告: 无法设置中文字体，将使用默认字体")

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from python_motion_planning import *

class LaserSensor:
    """模拟2D激光雷达传感器"""
    def __init__(self, env, range_max=10, angle_min=-np.pi, angle_max=np.pi, angle_increment=np.pi/45):
        self.env = env
        self.range_max = range_max
        self.angle_min = angle_min
        self.angle_max = angle_max
        self.angle_increment = angle_increment
        
    def scan(self, robot_pos):
        """模拟激光雷达扫描"""
        x, y, theta = robot_pos
        angles = np.arange(self.angle_min, self.angle_max, self.angle_increment)
        ranges = []
        
        for angle in angles:
            # 计算激光方向
            beam_angle = theta + angle
            # 检测障碍物
            beam_range = self._ray_casting(x, y, beam_angle)
            ranges.append(beam_range)
            
        return angles, np.array(ranges)
    
    def _ray_casting(self, x, y, angle):
        """光线投射算法检测障碍物"""
        range_found = self.range_max
        step_size = 0.5  # 增大步长以提高性能
        
        for dist in np.arange(0, self.range_max, step_size):
            # 计算激光束终点
            point_x = x + dist * np.cos(angle)
            point_y = y + dist * np.sin(angle)
            
            # 检查是否为整数坐标，便于网格地图检查
            grid_x, grid_y = int(round(point_x)), int(round(point_y))
            
            # 检查是否超出地图边界
            if grid_x < 0 or grid_x >= self.env.x_range or grid_y < 0 or grid_y >= self.env.y_range:
                range_found = dist
                break
                
            # 检查是否碰到障碍物
            if (grid_x, grid_y) in self.env.obstacles:
                range_found = dist
                break
                
        return range_found

class Robot:
    """自主移动机器人"""
    def __init__(self, start_pos, env, goal_pos=None, animation_speed=0.05):
        self.x, self.y, self.theta = start_pos
        self.env = env
        self.goal_pos = goal_pos
        self.path = []  # 机器人走过的路径
        self.laser_sensor = LaserSensor(env)
        self.animation_speed = animation_speed  # 动画速度控制参数
        
        # 为栅格地图创建探索地图
        self.explored_map = set()  # 已经探索的区域
        self.frontier = set()  # 边界区域
        
        # 添加起始位置
        self.path.append((self.x, self.y))
        self.explored_map.add((int(self.x), int(self.y)))
        
        # DFS探索需要的数据结构
        self.dfs_stack = []
        self.visited = set()
        self.grid_visited = set()  # 栅格DFS已访问节点
        self.grid_stack = []       # 栅格DFS栈
        
        # 初始化起始位置
        start_grid = (int(round(self.x)), int(round(self.y)))
        self.grid_stack.append(start_grid)
        self.grid_visited.add(start_grid)
        
        # 激光数据可视化
        self.laser_points = []
        
        # 为A*算法创建路径
        self.optimal_path = []
        
        # 移动方向 (上, 右, 下, 左)
        self.directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        
    def update_position(self, new_pos):
        """更新机器人位置"""
        # 计算动画插值帧
        start_x, start_y, start_theta = self.x, self.y, self.theta
        end_x, end_y, end_theta = new_pos
        
        # 归一化角度差，避免大于π的角度差
        angle_diff = end_theta - start_theta
        if angle_diff > np.pi:
            angle_diff -= 2 * np.pi
        elif angle_diff < -np.pi:
            angle_diff += 2 * np.pi
            
        # 更新位置（无动画）
        self.x, self.y, self.theta = new_pos
        self.path.append((self.x, self.y))
        
        # 更新探索地图
        grid_x, grid_y = int(round(self.x)), int(round(self.y))
        self.explored_map.add((grid_x, grid_y))
        self.grid_visited.add((grid_x, grid_y))
        
        # 更新周围的探索区域（模拟视野）
        view_range = 2
        for dx in range(-view_range, view_range + 1):
            for dy in range(-view_range, view_range + 1):
                nx, ny = grid_x + dx, grid_y + dy
                if 0 <= nx < self.env.x_range and 0 <= ny < self.env.y_range:
                    # 只添加非障碍物区域
                    if (nx, ny) not in self.env.obstacles:
                        self.explored_map.add((nx, ny))
        
    def scan_environment(self):
        """扫描周围环境并更新探索地图"""
        angles, ranges = self.laser_sensor.scan((self.x, self.y, self.theta))
        
        # 存储激光点用于可视化
        self.laser_points = []
        
        # 更新探索的区域
        for angle, range_val in zip(angles, ranges):
            scan_angle = self.theta + angle
            # 计算激光束终点
            end_x = self.x + range_val * np.cos(scan_angle)
            end_y = self.y + range_val * np.sin(scan_angle)
            
            # 存储激光点
            self.laser_points.append((end_x, end_y))
            
            if range_val < self.laser_sensor.range_max:
                # 检测到障碍物
                grid_x, grid_y = int(round(end_x)), int(round(end_y))
                
                # 添加障碍物周围区域到探索区域
                detect_range = 1
                for dx in range(-detect_range, detect_range + 1):
                    for dy in range(-detect_range, detect_range + 1):
                        check_x, check_y = grid_x + dx, grid_y + dy
                        # 确保在地图范围内
                        if 0 <= check_x < self.env.x_range and 0 <= check_y < self.env.y_range:
                            self.explored_map.add((check_x, check_y))
            
            # 更新沿着激光束的探索区域
            step_size = 0.5
            for dist in np.arange(0, range_val, step_size):
                # 计算激光束上的点
                point_x = self.x + dist * np.cos(scan_angle)
                point_y = self.y + dist * np.sin(scan_angle)
                grid_x, grid_y = int(round(point_x)), int(round(point_y))
                
                # 确保在地图范围内
                if 0 <= grid_x < self.env.x_range and 0 <= grid_y < self.env.y_range:
                    if (grid_x, grid_y) not in self.env.obstacles:
                        self.explored_map.add((grid_x, grid_y))
        
        return angles, ranges
    
    def grid_dfs_explore(self):
        """基于栅格的DFS探索算法"""
        # 如果栈为空，探索结束
        if not self.grid_stack:
            return False
        
        # 从栈中弹出当前位置
        current = self.grid_stack.pop()
        
        # 计算当前位置到机器人位置的方向
        current_x, current_y = current
        robot_x, robot_y = int(round(self.x)), int(round(self.y))
        
        # 如果当前位置不是机器人位置，需要先移动到该位置
        if current != (robot_x, robot_y):
            # 计算移动方向
            dx = current_x - robot_x
            dy = current_y - robot_y
            
            # 设置朝向角度
            target_theta = math.atan2(dy, dx)
            
            # 更新位置
            self.update_position((current_x, current_y, target_theta))
            return True
        
        # 探索周围的四个方向
        for dx, dy in self.directions:
            next_x, next_y = current_x + dx, current_y + dy
            next_pos = (next_x, next_y)
            
            # 检查是否是有效位置（在地图内，不是障碍物，且未访问过）
            if (0 <= next_x < self.env.x_range and 
                0 <= next_y < self.env.y_range and 
                next_pos not in self.env.obstacles and
                next_pos not in self.grid_visited):
                
                # 将新位置加入栈和已访问集合
                self.grid_stack.append(next_pos)
                self.grid_visited.add(next_pos)
                
                # 计算朝向
                target_theta = math.atan2(dy, dx)
                
                # 更新位置
                self.update_position((next_x, next_y, target_theta))
                return True
        
        # 如果四个方向都探索过，但栈不为空，则继续
        if self.grid_stack:
            return True
        
        return False
    
    def find_optimal_path(self, start, goal):
        """使用A*算法找到最优路径"""
        a_star = AStar(start=start, goal=goal, env=self.env)
        cost, path, _ = a_star.plan()
        
        if path:
            self.optimal_path = path
            return True
        return False

class MazeExploration:
    """迷宫测绘与导航系统"""
    def __init__(self):
        # 创建环境
        self.grid_env = self.create_environment()
        
        # 定义起点和目标点
        self.start_pos = (5, 5, 0)  # (x, y, theta)
        self.goal_pos = (45, 25)    # 目标位置，初始不知道
        
        # 创建机器人
        self.robot = Robot(self.start_pos, self.grid_env)
        
        # 设置可视化
        self.setup_visualization()
        
        # 设置状态
        self.exploration_complete = False
        self.path_planning_complete = False
        self.goal_found = False
        self.animation_running = True
        self.exploration_paused = False
        self.current_state = "exploration"  # 当前状态：exploration, path_planning, complete
        
        # 添加目标检测距离
        self.goal_detection_distance = 3  # 当机器人靠近目标3个单位时，视为找到目标
        
        # 添加帧率控制
        self.last_update_time = time.time()
        self.update_interval = 0.05  # 更新间隔，单位秒
        
        # 探索完成度阈值
        self.exploration_threshold = 0.70  # 探索70%的区域认为完成
        
        # 计数器
        self.step_count = 0
        self.exploration_start_time = time.time()
        
    def create_environment(self):
        """创建迷宫环境"""
        grid_env = Grid(51, 31)
        obstacles = grid_env.obstacles
        for i in range(10, 21):
            obstacles.add((i, 15))
        for i in range(30, 35):
            obstacles.add((i, 15))
        for i in range(15):
            obstacles.add((20, i))
        for i in range(15, 30):
            obstacles.add((30, i))
        for i in range(16):
            obstacles.add((40, i))
        grid_env.update(obstacles)
        return grid_env
        
    def setup_visualization(self):
        """设置可视化界面"""
        plt.ion()  # 交互模式
        self.fig = plt.figure(figsize=(16, 8))
        gs = GridSpec(1, 2, width_ratios=[1, 1])
        
        # 左侧图：实时探索
        self.ax1 = self.fig.add_subplot(gs[0])
        self.ax1.set_xlim(0, self.grid_env.x_range)
        self.ax1.set_ylim(0, self.grid_env.y_range)
        self.ax1.set_title("Robot Exploration")
        self.ax1.set_aspect('equal')
        
        # 右侧图：栅格地图
        self.ax2 = self.fig.add_subplot(gs[1])
        self.ax2.set_xlim(0, self.grid_env.x_range)
        self.ax2.set_ylim(0, self.grid_env.y_range)
        self.ax2.set_title("Grid Map")
        self.ax2.set_aspect('equal')
        
        # 绘制障碍物
        for obs in self.grid_env.obstacles:
            self.ax1.add_patch(Rectangle((obs[0]-0.5, obs[1]-0.5), 1, 1, color='black'))
            self.ax2.add_patch(Rectangle((obs[0]-0.5, obs[1]-0.5), 1, 1, color='black'))
        
        # 绘制起点和终点
        start_x, start_y, _ = self.start_pos
        self.ax1.scatter(start_x, start_y, c='green', s=100, marker='o')
        self.ax2.scatter(start_x, start_y, c='green', s=100, marker='o')
        
        # 绘制目标点
        goal_x, goal_y = self.goal_pos
        self.ax1.scatter(goal_x, goal_y, c='red', s=100, marker='*')
        self.ax2.scatter(goal_x, goal_y, c='red', s=100, marker='*')
        
        # 初始化机器人位置
        self.robot_marker1, = self.ax1.plot([start_x], [start_y], 'bo', markersize=10)
        self.robot_marker2, = self.ax2.plot([start_x], [start_y], 'bo', markersize=10)
        
        # 机器人方向指示
        length = 1.0
        dx = length * np.cos(self.robot.theta)
        dy = length * np.sin(self.robot.theta)
        self.direction1, = self.ax1.plot([start_x, start_x + dx], [start_y, start_y + dy], 'b-', linewidth=2)
        self.direction2, = self.ax2.plot([start_x, start_x + dx], [start_y, start_y + dy], 'b-', linewidth=2)
        
        # 初始化路径
        self.path_line1, = self.ax1.plot([], [], 'g-', linewidth=2, alpha=0.7)
        self.path_line2, = self.ax2.plot([], [], 'g-', linewidth=2, alpha=0.7)
        
        # 初始化最优路径
        self.optimal_path_line1, = self.ax1.plot([], [], 'r-', linewidth=2, alpha=0.7)
        self.optimal_path_line2, = self.ax2.plot([], [], 'r-', linewidth=2, alpha=0.7)
        
        # 激光雷达可视化
        self.laser_lines = []
        
        # 初始化阴影覆盖（未探索区域）
        self.unexplored = np.ones((self.grid_env.x_range, self.grid_env.y_range))
        for obs in self.grid_env.obstacles:
            if 0 <= obs[0] < self.grid_env.x_range and 0 <= obs[1] < self.grid_env.y_range:
                self.unexplored[obs[0], obs[1]] = 0
        self.shadow = self.ax2.imshow(self.unexplored.T, origin='lower', 
                                      extent=(0, self.grid_env.x_range, 0, self.grid_env.y_range),
                                      cmap='gray', alpha=0.5, vmin=0, vmax=1)
                                      
        # 信息显示
        self.info_text = self.ax1.text(1, 29, "", fontsize=10, bbox=dict(facecolor='white', alpha=0.5))
        self.status_text = self.ax1.text(1, 27, "", fontsize=10, color='red', bbox=dict(facecolor='white', alpha=0.5))
        
        # 添加控制按钮
        button_axes = plt.axes([0.45, 0.01, 0.1, 0.04])
        self.pause_button = plt.Button(button_axes, 'Pause', color='lightgoldenrodyellow')
        self.pause_button.on_clicked(self.toggle_pause)
        
        # 为动画设置
        plt.tight_layout()
        
    def toggle_pause(self, event):
        """暂停/继续动画"""
        self.exploration_paused = not self.exploration_paused
        if self.exploration_paused:
            self.pause_button.label.set_text('Continue')
        else:
            self.pause_button.label.set_text('Pause')
        
    def update_visualization(self):
        """更新可视化"""
        # 控制更新频率
        current_time = time.time()
        if current_time - self.last_update_time < self.update_interval:
            return
        self.last_update_time = current_time
        
        # 更新机器人位置
        self.robot_marker1.set_data([self.robot.x], [self.robot.y])
        self.robot_marker2.set_data([self.robot.x], [self.robot.y])
        
        # 更新方向指示
        length = 1.0
        dx = length * np.cos(self.robot.theta)
        dy = length * np.sin(self.robot.theta)
        self.direction1.set_data([self.robot.x, self.robot.x + dx], [self.robot.y, self.robot.y + dy])
        self.direction2.set_data([self.robot.x, self.robot.x + dx], [self.robot.y, self.robot.y + dy])
        
        # 更新路径
        if self.robot.path:
            path_x, path_y = zip(*self.robot.path)
            self.path_line1.set_data(path_x, path_y)
            self.path_line2.set_data(path_x, path_y)
        
        # 更新最优路径
        if self.robot.optimal_path:
            opt_path_x, opt_path_y = zip(*self.robot.optimal_path)
            self.optimal_path_line1.set_data(opt_path_x, opt_path_y)
            self.optimal_path_line2.set_data(opt_path_x, opt_path_y)
        
        # 更新激光雷达可视化
        # 清除旧的激光线
        for line in self.laser_lines:
            line.remove() if hasattr(line, 'remove') else None
        self.laser_lines = []
        
        # 添加新的激光线
        for point in self.robot.laser_points:
            line, = self.ax1.plot([self.robot.x, point[0]], [self.robot.y, point[1]], 'r-', alpha=0.3, linewidth=0.5)
            self.laser_lines.append(line)
        
        # 更新探索区域（移除阴影）
        for pos in self.robot.explored_map:
            if 0 <= pos[0] < self.grid_env.x_range and 0 <= pos[1] < self.grid_env.y_range:
                self.unexplored[pos[0], pos[1]] = 0
        
        # 更新阴影
        self.shadow.set_data(self.unexplored.T)
        
        # 计算探索完成度
        total_explorable = self.grid_env.x_range * self.grid_env.y_range - len(self.grid_env.obstacles)
        explored_count = np.sum(self.unexplored == 0) - len(self.grid_env.obstacles)
        explored_percent = explored_count / total_explorable * 100
        
        # 检查是否达到探索阈值
        if explored_percent >= self.exploration_threshold * 100 and not self.exploration_complete:
            self.exploration_complete = True
            print(f"Exploration threshold reached: {explored_percent:.2f}% of map explored")
        
        # 更新信息显示
        dist_to_goal = math.sqrt((self.robot.x - self.goal_pos[0])**2 + (self.robot.y - self.goal_pos[1])**2)
        elapsed_time = time.time() - self.exploration_start_time
        info_str = f"Position: ({self.robot.x:.1f}, {self.robot.y:.1f})\nDistance to goal: {dist_to_goal:.1f}\nExplored: {explored_percent:.1f}%\nSteps: {self.step_count}\nTime: {elapsed_time:.1f}s"
        self.info_text.set_text(info_str)
        
        # 更新状态显示
        status_str = f"Status: {self.current_state.capitalize()}"
        if self.exploration_paused:
            status_str += " (Paused)"
        self.status_text.set_text(status_str)
        
        # 检查是否找到目标
        if not self.goal_found and dist_to_goal <= self.goal_detection_distance:
            self.goal_found = True
            print("Goal found! Distance:", dist_to_goal)
        
        # 刷新图形
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
    
    def animate(self, frame):
        """动画更新函数"""
        if self.exploration_paused:
            return
            
        if self.current_state == "exploration":
            # 扫描环境
            self.robot.scan_environment()
            
            # 使用基于栅格的DFS探索
            if not self.robot.grid_dfs_explore():
                print("Grid DFS exploration completed!")
                self.exploration_complete = True
                self.current_state = "path_planning"
            
            self.step_count += 1
            
            # 打印当前位置和距离目标的距离
            if self.step_count % 100 == 0:
                dist_to_goal = math.sqrt((self.robot.x - self.goal_pos[0])**2 + (self.robot.y - self.goal_pos[1])**2)
                print(f"Step {self.step_count}: Robot at ({self.robot.x:.1f}, {self.robot.y:.1f}), Distance to goal: {dist_to_goal:.1f}")
            
            # 检查是否找到目标
            dist_to_goal = math.sqrt((self.robot.x - self.goal_pos[0])**2 + (self.robot.y - self.goal_pos[1])**2)
            if not self.goal_found and dist_to_goal <= self.goal_detection_distance:
                self.goal_found = True
                print("Goal found during exploration!")
                self.current_state = "path_planning"
        
        elif self.current_state == "path_planning":
            # 探索完成后的路径规划
            print("Planning optimal path...")
            
            # 如果找到了目标，则规划从目标到起点的路径
            if self.goal_found:
                print("Planning path from goal to start...")
                
                # 创建机器人当前位置到目标的路径
                if self.robot.find_optimal_path((int(round(self.robot.x)), int(round(self.robot.y))), self.goal_pos):
                    print("Path to goal planned.")
                    self.update_visualization()
                    time.sleep(1)  # 暂停1秒展示路径
                
                # 使用A*算法找到从目标到起点的最优路径
                if self.robot.find_optimal_path(self.goal_pos, (self.start_pos[0], self.start_pos[1])):
                    self.path_planning_complete = True
                    print("Path planning completed!")
                    
                    # 更新可视化
                    self.update_visualization()
                    self.current_state = "complete"
                else:
                    print("Cannot find path from goal to start!")
                    # 尝试从机器人当前位置返回起点
                    if self.robot.find_optimal_path((int(round(self.robot.x)), int(round(self.robot.y))), (self.start_pos[0], self.start_pos[1])):
                        print("Found path from current position to start.")
                        self.path_planning_complete = True
                        self.update_visualization()
                        self.current_state = "complete"
            else:
                # 尝试直接找到目标位置
                print("Goal not found during exploration. Trying to find path to goal...")
                if self.robot.find_optimal_path((int(round(self.robot.x)), int(round(self.robot.y))), self.goal_pos):
                    self.goal_found = True
                    print("Path to goal planned.")
                    self.update_visualization()
                    
                    # 然后规划从目标到起点的路径
                    if self.robot.find_optimal_path(self.goal_pos, (self.start_pos[0], self.start_pos[1])):
                        self.path_planning_complete = True
                        print("Path planning from goal to start completed!")
                        self.update_visualization()
                        self.current_state = "complete"
        
        # 更新可视化
        self.update_visualization()
        
        # 如果完成，停止动画
        if self.current_state == "complete":
            return False
            
        return True
        
    def run(self):
        """运行迷宫测绘与导航"""
        print("Starting maze exploration with real-time animation...")
        self.exploration_start_time = time.time()
        
        # 使用FuncAnimation实现动画效果
        self.anim = FuncAnimation(
            self.fig, 
            self.animate,
            frames=None,  # 无限帧
            interval=50,  # 每50毫秒更新一次
            repeat=False,
            save_count=0
        )
        
        # 显示图形并等待用户关闭
        plt.show(block=True)

if __name__ == '__main__':
    maze_exploration = MazeExploration()
    maze_exploration.run() 