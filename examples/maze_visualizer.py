'''
maze_visualizer.py - 用于可视化迷宫探索过程
'''

import os
import time
import pickle
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as colormap
import matplotlib.lines as mlines
from matplotlib.patches import Rectangle

# 这有助于在各种平台上运行
import matplotlib
matplotlib.use('TkAgg')

class MazeVisualizer:
    """迷宫可视化器，用于可视化机器人探索迷宫过程"""
    
    # 机器人显示参数
    ROBOT_HEIGHT_M = 0.5
    ROBOT_WIDTH_M = 0.3
    
    def __init__(self, title="迷宫探索可视化", map_size_pixels=500, map_size_meters=30, show_trajectory=True):
        """初始化可视化器
        
        参数:
            title: 窗口标题
            map_size_pixels: 地图像素大小
            map_size_meters: 地图实际大小（米）
            show_trajectory: 是否显示轨迹
        """
        self.title = title
        self.map_size_pixels = map_size_pixels
        self.map_size_meters = map_size_meters
        self.show_trajectory = show_trajectory
        self.map_scale_meters_per_pixel = map_size_meters / float(map_size_pixels)
        
        # 创建图形
        self.fig = plt.figure(figsize=(10, 10))
        
        # 存储Python ID以检测窗口关闭
        self.figid = id(self.fig)
        
        # 设置窗口标题
        try:
            self.fig.canvas.set_window_title(title)
        except:
            try:
                self.fig.canvas.manager.set_window_title(title)
            except:
                pass
                
        plt.title(title)
        
        # 创建坐标轴
        self.ax = self.fig.gca()
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.grid(False)
        
        # 设置坐标轴范围
        self.ax.set_xlim([0, map_size_pixels])
        self.ax.set_ylim([0, map_size_pixels])
        
        # 重新标记坐标轴刻度以显示米
        ticks = np.arange(0, map_size_pixels + 100, 100)
        labels = [str(self.map_scale_meters_per_pixel * tick) for tick in ticks]
        self.ax.set_xticklabels(labels)
        self.ax.set_yticklabels(labels)
        
        # 地图图像
        self.map_img = None
        
        # 机器人图像
        self.robot = None
        
        # 轨迹
        self.trajectory = []
        self.trajectory_line = None
        
        # 目标点
        self.goal_point = None
        self.start_point = None
        
        # 规划路径
        self.path_line = None
        
        # 上次更新时间
        self.last_update_time = 0
        self.update_interval = 0.1  # 100ms
        
    def update(self, robot, env):
        """更新可视化
        
        参数:
            robot: 机器人对象
            env: 环境对象
            
        返回:
            是否成功更新
        """
        # 检查是否需要更新（限制更新频率）
        current_time = time.time()
        if current_time - self.last_update_time < self.update_interval:
            return True
            
        self.last_update_time = current_time
        
        # 更新机器人位置
        self._update_robot(robot.x, robot.y, robot.theta)
        
        # 更新轨迹
        if self.show_trajectory:
            self._update_trajectory(robot.path)
            
        # 更新地图
        self._update_map(env)
        
        # 更新起点和终点
        self._update_points(env)
        
        # 更新规划路径
        if hasattr(robot, 'goal_path') and robot.goal_path:
            self._update_path([(robot.x, robot.y)] + robot.goal_path)
        
        # 刷新显示
        return self._refresh()
        
    def _update_robot(self, x, y, theta):
        """更新机器人位置
        
        参数:
            x: X坐标（米）
            y: Y坐标（米）
            theta: 方向角（弧度）
        """
        # 移除之前的机器人图像
        if self.robot is not None:
            self.robot.remove()
            
        # 计算像素坐标
        x_pix = x / self.map_scale_meters_per_pixel
        y_pix = y / self.map_scale_meters_per_pixel
        
        # 使用箭头表示机器人
        dx = 0.1 * np.cos(theta)
        dy = 0.1 * np.sin(theta)
        
        self.robot = self.ax.arrow(
            x_pix, y_pix, dx, dy,
            head_width=self.ROBOT_WIDTH_M / self.map_scale_meters_per_pixel,
            head_length=self.ROBOT_HEIGHT_M / self.map_scale_meters_per_pixel,
            fc='r', ec='r'
        )
        
    def _update_trajectory(self, path):
        """更新轨迹
        
        参数:
            path: 路径点列表 [(x1,y1), (x2,y2), ...]
        """
        if not path:
            return
            
        # 转换为像素坐标
        trajectory_pix = [(x / self.map_scale_meters_per_pixel, y / self.map_scale_meters_per_pixel) 
                          for x, y in path]
        
        # 移除之前的轨迹线
        if self.trajectory_line is not None:
            self.trajectory_line.remove()
            
        # 绘制新轨迹
        x_values = [p[0] for p in trajectory_pix]
        y_values = [p[1] for p in trajectory_pix]
        self.trajectory_line = self.ax.plot(x_values, y_values, 'b-', linewidth=1)[0]
        
    def _update_map(self, env):
        """更新地图
        
        参数:
            env: 环境对象
        """
        # 创建空白地图
        map_img = np.zeros((self.map_size_pixels, self.map_size_pixels), dtype=np.uint8)
        map_img.fill(128)  # 灰色背景
        
        # 处理不同类型的环境对象
        if hasattr(env, 'grid_env'):
            grid = env.grid_env
        elif hasattr(env, 'obstacles'):
            # 如果是列表形式的障碍物
            if isinstance(env.obstacles, list) or isinstance(env.obstacles, set):
                # 确定环境大小
                if hasattr(env, 'x_range') and hasattr(env, 'y_range'):
                    width, height = env.x_range, env.y_range
                else:
                    # 从障碍物坐标推断环境大小
                    max_x = max([x for x, y in env.obstacles]) if env.obstacles else 15
                    max_y = max([y for x, y in env.obstacles]) if env.obstacles else 15
                    width, height = max_x + 1, max_y + 1
                
                # 创建网格
                grid = np.zeros((width, height), dtype=np.uint8)
                for x, y in env.obstacles:
                    if 0 <= x < width and 0 <= y < height:
                        grid[x, y] = 1
            else:
                # 如果obstacles不是列表或集合，创建默认网格
                grid = np.zeros((15, 15), dtype=np.uint8)
        else:
            # 如果没有grid_env和obstacles属性，创建默认网格
            grid = np.zeros((15, 15), dtype=np.uint8)
        
        # 缩放网格到地图大小
        scale_x = self.map_size_pixels / grid.shape[0]
        scale_y = self.map_size_pixels / grid.shape[1]
        
        # 填充地图
        for x in range(grid.shape[0]):
            for y in range(grid.shape[1]):
                # 计算像素坐标
                x_pix_start = int(x * scale_x)
                y_pix_start = int(y * scale_y)
                x_pix_end = int((x + 1) * scale_x)
                y_pix_end = int((y + 1) * scale_y)
                
                # 根据网格值设置颜色
                if grid[x, y] == 1:  # 障碍物
                    map_img[y_pix_start:y_pix_end, x_pix_start:x_pix_end] = 0  # 黑色
                else:  # 自由空间
                    map_img[y_pix_start:y_pix_end, x_pix_start:x_pix_end] = 255  # 白色
        
        # 更新或创建地图图像
        if self.map_img is None:
            self.map_img = self.ax.imshow(map_img, cmap=colormap.gray, origin='lower')
        else:
            self.map_img.set_data(map_img)
            
    def _update_points(self, env):
        """更新起点和终点
        
        参数:
            env: 环境对象
        """
        # 移除之前的点
        if self.start_point is not None:
            self.start_point.remove()
        if self.goal_point is not None:
            self.goal_point.remove()
            
        # 添加起点（如果有）
        if hasattr(env, 'start') and env.start is not None:
            x_pix = env.start[0] / self.map_scale_meters_per_pixel
            y_pix = env.start[1] / self.map_scale_meters_per_pixel
            self.start_point = self.ax.plot(x_pix, y_pix, 'go', markersize=10)[0]
            
        # 添加终点（如果有）
        if hasattr(env, 'goal') and env.goal is not None:
            x_pix = env.goal[0] / self.map_scale_meters_per_pixel
            y_pix = env.goal[1] / self.map_scale_meters_per_pixel
            self.goal_point = self.ax.plot(x_pix, y_pix, 'mo', markersize=10)[0]
            
    def _update_path(self, path):
        """更新规划路径
        
        参数:
            path: 路径点列表 [(x1,y1), (x2,y2), ...]
        """
        if not path:
            return
            
        # 移除之前的路径线
        if self.path_line is not None:
            self.path_line.remove()
            
        # 转换为像素坐标
        path_pix = [(x / self.map_scale_meters_per_pixel, y / self.map_scale_meters_per_pixel) 
                    for x, y in path]
        
        # 绘制新路径
        x_values = [p[0] for p in path_pix]
        y_values = [p[1] for p in path_pix]
        self.path_line = self.ax.plot(x_values, y_values, 'g-', linewidth=2)[0]
        
    def _refresh(self):
        """刷新显示
        
        返回:
            是否成功刷新
        """
        # 如果我们有一个新的图形，说明出了问题（关闭图形失败）
        if self.figid != id(plt.gcf()):
            return False
            
        # 不阻塞地重绘当前对象
        plt.draw()
        
        # 刷新显示，在窗口关闭或键盘中断时设置标志
        try:
            plt.pause(0.01)  # 任意暂停以强制重绘
            return True
        except:
            return False
            
    def close(self):
        """关闭可视化窗口"""
        plt.close(self.fig)


class MazeReplay:
    """迷宫回放器，用于离线回放迷宫探索过程"""
    
    def __init__(self, data_file, speed=1.0):
        """初始化回放器
        
        参数:
            data_file: 数据文件路径
            speed: 回放速度倍数
        """
        self.data_file = data_file
        self.speed = speed
        self.data = None
        self.visualizer = None
        
    def load_data(self):
        """加载数据文件"""
        print(f"加载数据文件: {self.data_file}")
        
        # 检查文件扩展名
        if self.data_file.endswith('.pkl'):
            # 加载pickle文件
            with open(self.data_file, 'rb') as f:
                self.data = pickle.load(f)
        else:
            raise ValueError(f"不支持的文件格式: {self.data_file}")
            
        # 打印数据摘要
        print(f"加载了 {len(self.data['timestamps'])} 帧数据")
        if 'metadata' in self.data:
            for key, value in self.data['metadata'].items():
                print(f"{key}: {value}")
                
        return True
        
    def replay(self):
        """回放迷宫探索过程"""
        if self.data is None:
            if not self.load_data():
                print("加载数据失败")
                return False
                
        # 创建可视化器
        self.visualizer = MazeVisualizer(
            title=f"迷宫探索回放 - {os.path.basename(self.data_file)}",
            show_trajectory=True
        )
        
        # 获取帧数
        num_frames = len(self.data['timestamps'])
        
        # 计算帧间时间间隔
        if num_frames > 1:
            # 计算原始帧率
            total_time_ms = self.data['timestamps'][-1] - self.data['timestamps'][0]
            avg_frame_time = total_time_ms / (num_frames - 1) / 1000.0  # 转换为秒
            
            # 应用速度倍数
            frame_time = avg_frame_time / self.speed
        else:
            frame_time = 0.1  # 默认帧时间
            
        print(f"回放帧率: {1.0/frame_time:.2f} 帧/秒 (速度: {self.speed}x)")
        
        # 创建简单的机器人和环境模拟类，用于回放
        class ReplayRobot:
            def __init__(self):
                self.x = 0
                self.y = 0
                self.theta = 0
                self.path = []
                self.goal_path = []
                
        class ReplayEnvironment:
            def __init__(self):
                self.obstacles = set()
                self.start = None
                self.goal = None
                self.x_range = 15  # 默认大小
                self.y_range = 15  # 默认大小
                
        # 创建模拟对象
        robot = ReplayRobot()
        env = ReplayEnvironment()
        
        # 回放每一帧
        for i in range(num_frames):
            # 更新机器人位置
            if i < len(self.data['positions']):
                robot.x, robot.y, robot.theta = self.data['positions'][i]
                
            # 更新路径
            if i > 0:
                robot.path = [self.data['positions'][j][:2] for j in range(i+1)]
                
            # 更新规划路径
            if i < len(self.data['paths']) and self.data['paths'][i] is not None:
                robot.goal_path = self.data['paths'][i]
                
            # 更新环境
            if i < len(self.data['maps']) and self.data['maps'][i] is not None:
                # 处理地图数据
                map_data = self.data['maps'][i]
                if isinstance(map_data, list):
                    # 如果地图是障碍物列表
                    env.obstacles = set(map_data)
                    
                    # 计算环境大小
                    if env.obstacles:
                        max_x = max([x for x, y in env.obstacles]) + 1
                        max_y = max([y for x, y in env.obstacles]) + 1
                        env.x_range = max(env.x_range, max_x)
                        env.y_range = max(env.y_range, max_y)
                else:
                    # 如果是其他类型的地图数据，尝试转换为障碍物集合
                    try:
                        env.obstacles = set()
                        if hasattr(map_data, 'obstacles'):
                            env.obstacles = set(map_data.obstacles)
                    except (TypeError, AttributeError):
                        pass
                        
            # 更新目标点
            if i < len(self.data['goals']) and self.data['goals'][i] is not None:
                env.goal = self.data['goals'][i]
                
            # 更新可视化
            if not self.visualizer.update(robot, env):
                print("可视化窗口已关闭")
                return False
                
            # 等待下一帧
            time.sleep(frame_time)
            
        print("回放完成")
        
        # 等待用户关闭窗口
        while self.visualizer._refresh():
            time.sleep(0.1)
            
        return True
        

def main():
    """主函数"""
    import argparse
    
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='迷宫探索可视化')
    parser.add_argument('data_file', help='数据文件路径')
    parser.add_argument('--speed', type=float, default=1.0, help='回放速度倍数，默认1.0')
    args = parser.parse_args()
    
    # 创建回放器
    replay = MazeReplay(args.data_file, args.speed)
    
    # 开始回放
    replay.replay()
    
    # 等待用户关闭窗口
    plt.show()
    
if __name__ == '__main__':
    main() 