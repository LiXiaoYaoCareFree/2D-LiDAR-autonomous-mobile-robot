'''
maze_data_recorder.py - 用于记录迷宫探索过程中的数据
'''

import os
import time
import pickle
import numpy as np

class MazeDataRecorder:
    """迷宫数据记录器，用于记录机器人探索迷宫过程中的数据"""
    
    def __init__(self, filename="maze_exploration", save_dir="data"):
        """初始化数据记录器
        
        参数:
            filename: 保存文件名前缀
            save_dir: 保存目录
        """
        self.filename = filename
        self.save_dir = save_dir
        
        # 确保保存目录存在
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
            
        # 初始化数据存储
        self.timestamps = []  # 时间戳
        self.positions = []   # 机器人位置
        self.scans = []       # 激光扫描数据
        self.maps = []        # 地图状态
        self.paths = []       # 规划路径
        self.goals = []       # 目标点
        self.start_time = time.time()
        
    def record(self, robot, grid_env):
        """记录一帧数据
        
        参数:
            robot: 机器人对象
            grid_env: 网格环境对象
        """
        # 记录时间戳（毫秒）
        timestamp = int((time.time() - self.start_time) * 1000)
        self.timestamps.append(timestamp)
        
        # 记录机器人位置
        position = (robot.x, robot.y, robot.theta)
        self.positions.append(position)
        
        # 记录激光扫描数据
        scan_data = robot.get_scan_data() if hasattr(robot, 'get_scan_data') else None
        self.scans.append(scan_data)
        
        # 记录地图状态（深拷贝）
        map_data = np.array(grid_env) if hasattr(grid_env, '__array__') else grid_env
        self.maps.append(map_data)
        
        # 记录规划路径
        path = robot.goal_path if hasattr(robot, 'goal_path') else None
        self.paths.append(path)
        
        # 记录目标点
        goal = grid_env.goal if hasattr(grid_env, 'goal') else None
        self.goals.append(goal)
        
    def save(self):
        """保存记录的数据到文件
        
        返回:
            保存的文件路径
        """
        # 创建数据字典，确保所有数据都是可序列化的
        data = {
            'timestamps': self.timestamps,
            'positions': self.positions,
            'scans': self.scans,
            'paths': self.paths,
            'goals': self.goals,
            'metadata': {
                'record_time': time.strftime('%Y-%m-%d %H:%M:%S'),
                'frames': len(self.timestamps)
            }
        }
        
        # 尝试保存地图数据，如果地图包含不可序列化的对象，则跳过
        try:
            # 将地图数据转换为简单的二维数组
            simplified_maps = []
            for map_data in self.maps:
                if hasattr(map_data, 'obstacles'):
                    # 如果地图有obstacles属性，保存障碍物坐标
                    simplified_maps.append(list(map_data.obstacles))
                else:
                    # 否则保存原始地图数据
                    simplified_maps.append(map_data)
            data['maps'] = simplified_maps
        except (TypeError, AttributeError):
            print("警告：无法序列化地图数据，跳过保存地图")
            data['maps'] = []
        
        # 生成文件名
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        filepath = os.path.join(self.save_dir, f"{self.filename}_{timestamp}")
        
        # 保存为pickle文件
        with open(f"{filepath}.pkl", 'wb') as f:
            pickle.dump(data, f)
            
        # 同时保存为dat文件格式（兼容BreezySLAM）
        self._save_as_dat(f"{filepath}.dat")
            
        return filepath
    
    def _save_as_dat(self, filepath):
        """将数据保存为BreezySLAM兼容的dat格式
        
        参数:
            filepath: 保存路径
        """
        with open(filepath, 'w') as f:
            for i in range(len(self.timestamps)):
                # 时间戳（微秒）
                timestamp = self.timestamps[i] * 1000  # 毫秒转微秒
                
                # 位置信息（模拟里程计）
                x, y, theta = self.positions[i]
                # 将位置转换为模拟里程计值
                left_wheel = int(x * 100)   # 简单地将x坐标乘以100作为左轮计数
                right_wheel = int(y * 100)  # 简单地将y坐标乘以100作为右轮计数
                
                # 激光扫描数据
                scan_data = self.scans[i]
                
                # URG04LX类的scan_size是682，detection_angle_degrees是240度
                # 我们需要生成682个点，覆盖240度范围
                urg_scan_data = [5000] * 682  # 默认最大距离为5000mm
                
                if scan_data is not None:
                    # 如果有扫描数据，将其映射到URG04LX的格式
                    # 我们的传感器可能有36个点（每10度一个），需要映射到URG04LX的682个点（240度）
                    if len(scan_data) == 36:
                        # 计算每个URG04LX点对应的角度
                        for j in range(682):
                            urg_angle = -120 + (j * 240.0 / 681.0)  # URG04LX的角度范围是[-120, 120]
                            
                            # 将URG04LX角度映射到我们的传感器角度
                            # 我们的传感器角度范围是[0, 360)，每10度一个点
                            our_angle = (urg_angle + 180) % 360  # 转换为[0, 360)
                            our_idx = int(our_angle / 10)
                            
                            # 如果索引有效，使用我们的扫描数据
                            if 0 <= our_idx < len(scan_data):
                                urg_scan_data[j] = scan_data[our_idx]
                
                # 格式化为BreezySLAM的dat格式
                # 时间戳 0 0 左轮 右轮 0 0 ... 0 扫描数据(从第24列开始)
                line = f"{timestamp} 0 0 {left_wheel} {right_wheel} " + "0 " * 19
                
                # 添加扫描数据
                for value in urg_scan_data:
                    line += f"{int(value)} "
                    
                f.write(line.strip() + "\n") 