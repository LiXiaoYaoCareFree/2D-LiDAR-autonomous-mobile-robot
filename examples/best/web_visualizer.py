#!/usr/bin/env python3

"""
基于Web的迷宫SLAM实时可视化系统
使用Flask + SocketIO + Plotly.js实现科技感界面
"""

import json
import time
import threading
from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit
import numpy as np
import base64
import io
from maze_slam_visual_new2 import *

class WebMazeSLAMVisualizer:
    """Web版迷宫SLAM可视化器"""
    
    def __init__(self):
        self.app = Flask(__name__)
        self.app.config['SECRET_KEY'] = 'maze_slam_secret_key'
        self.socketio = SocketIO(self.app, cors_allowed_origins="*")
        
        # 系统状态
        self.simulation_running = False
        self.simulation_thread = None
        self.system = None
        
        # 设置路由
        self._setup_routes()
        self._setup_socketio_events()
        
    def _setup_routes(self):
        """设置Flask路由"""
        
        @self.app.route('/')
        def index():
            return render_template('index.html')
        
        @self.app.route('/api/start_simulation', methods=['POST'])
        def start_simulation():
            try:
                data = request.get_json()
                
                # 解析用户输入
                maze_size = float(data.get('maze_size', 8))
                walls_data = data.get('walls', [])
                start_pos = data.get('start_pos', [1.0, 1.0])
                
                # 创建临时地图文件
                self._create_custom_map(maze_size, walls_data, start_pos)
                
                # 启动仿真
                if not self.simulation_running:
                    self.simulation_running = True
                    self.simulation_thread = threading.Thread(
                        target=self._run_simulation_thread,
                        args=("custom_map.json",)
                    )
                    self.simulation_thread.daemon = True
                    self.simulation_thread.start()
                    
                    return jsonify({"status": "success", "message": "仿真已启动"})
                else:
                    return jsonify({"status": "error", "message": "仿真已在运行中"})
                    
            except Exception as e:
                return jsonify({"status": "error", "message": f"启动失败: {str(e)}"})
        
        @self.app.route('/api/stop_simulation', methods=['POST'])
        def stop_simulation():
            try:
                self.simulation_running = False
                if self.simulation_thread:
                    self.simulation_thread.join(timeout=2)
                return jsonify({"status": "success", "message": "仿真已停止"})
            except Exception as e:
                return jsonify({"status": "error", "message": f"停止失败: {str(e)}"})
    
    def _setup_socketio_events(self):
        """设置SocketIO事件"""
        
        @self.socketio.on('connect')
        def handle_connect():
            print('客户端连接成功')
            emit('status', {'message': '连接成功，准备开始仿真'})
        
        @self.socketio.on('disconnect')
        def handle_disconnect():
            print('客户端断开连接')
        
        @self.socketio.on('request_update')
        def handle_request_update():
            if self.system:
                self._emit_current_state()
    
    def _create_custom_map(self, maze_size, walls_data, start_pos):
        """根据用户输入创建自定义地图文件"""
        map_data = {
            "maze_size": maze_size,
            "start_point": start_pos,
            "segments": []
        }
        
        # 添加边界墙（如果用户没有指定）
        if not walls_data:
            # 创建默认的边界墙
            walls_data = [
                {"start": [0, 0], "end": [maze_size, 0]},          # 底边
                {"start": [maze_size, 0], "end": [maze_size, maze_size]},  # 右边
                {"start": [maze_size, maze_size], "end": [0, maze_size]},  # 顶边
                {"start": [0, maze_size], "end": [0, 0]},          # 左边
                # 添加一些内部墙壁示例
                {"start": [2, 2], "end": [6, 2]},
                {"start": [6, 2], "end": [6, 6]},
                {"start": [2, 4], "end": [4, 4]},
            ]
        
        # 转换墙壁数据
        for wall in walls_data:
            map_data["segments"].append({
                "start": wall["start"],
                "end": wall["end"]
            })
        
        # 保存到文件
        with open("custom_map.json", "w", encoding="utf-8") as f:
            json.dump(map_data, f, indent=2)
    
    def _run_simulation_thread(self, map_file):
        """在单独线程中运行仿真"""
        try:
            print(f"🚀 启动Web仿真系统，地图文件: {map_file}")
            
            # 创建系统（修改版本，支持Web回调）
            self.system = WebGlobalMazeSLAMSystem(map_file, self._emit_update_callback)
            
            # 运行探索
            self.system.run_exploration()
            
        except Exception as e:
            print(f"仿真线程错误: {e}")
            self.socketio.emit('error', {'message': f'仿真错误: {str(e)}'})
        finally:
            self.simulation_running = False
            self.socketio.emit('simulation_ended', {'message': '仿真结束'})
    
    def _emit_update_callback(self, data):
        """仿真更新回调函数"""
        if self.simulation_running:
            try:
                # 确保在主线程中发送WebSocket消息
                import threading
                def emit_data():
                    self.socketio.emit('simulation_update', data)
                    print(f"📡 WebSocket数据已发送: 机器人位置 {data.get('status', {}).get('position', 'unknown')}")
                
                # 如果是在后台线程，使用start_background_task
                if threading.current_thread() != threading.main_thread():
                    self.socketio.start_background_task(emit_data)
                else:
                    emit_data()
                    
            except Exception as e:
                print(f"WebSocket发送失败: {e}")
                # 尝试直接发送
                try:
                    self.socketio.emit('simulation_update', data)
                except Exception as e2:
                    print(f"备用发送也失败: {e2}")
    
    def _emit_current_state(self):
        """发送当前状态"""
        if self.system and self.system.robot:
            try:
                data = self._prepare_visualization_data()
                self.socketio.emit('simulation_update', data)
                print("📡 发送当前状态数据")
            except Exception as e:
                print(f"发送当前状态失败: {e}")
    
    def _prepare_visualization_data(self):
        """准备可视化数据"""
        if not self.system:
            return {}
        
        # 获取机器人和环境数据
        robot = self.system.robot
        maze_env = self.system.maze_env
        global_mapper = self.system.global_mapper
        
        # 准备真实迷宫数据
        true_maze_data = self._get_true_maze_data(robot, maze_env, global_mapper)
        
        # 准备SLAM地图数据
        slam_map_data = self._get_slam_map_data(global_mapper, maze_env)
        
        # 准备前沿探索数据
        frontier_data = self._get_frontier_data(global_mapper, robot)
        
        # 准备雷达数据
        radar_data = self._get_radar_data(robot, maze_env)
        
        # 准备状态信息
        status_info = self._get_status_info(robot, global_mapper, maze_env)
        
        return {
            'true_maze': true_maze_data,
            'slam_map': slam_map_data,
            'frontier': frontier_data,
            'radar': radar_data,
            'status': status_info,
            'timestamp': time.time()
        }
    
    def _get_true_maze_data(self, robot, maze_env, global_mapper):
        """获取真实迷宫数据"""
        data = {
            'walls': [],
            'robot_pos': [robot.position[0], robot.position[1]],
            'robot_path': [],
            'start_pos': list(maze_env.start_pos),
            'discovered_exits': [],
            'reached_exits': [],
            'target_pos': None,
            'shortest_path': [],
            'maze_size': maze_env.size
        }
        
        # 墙壁数据
        for wall in maze_env.walls:
            data['walls'].append({
                'x1': wall[0][0], 'y1': wall[0][1],
                'x2': wall[1][0], 'y2': wall[1][1]
            })
        
        # 机器人路径
        if robot.robot_id in global_mapper.robot_paths:
            path = global_mapper.robot_paths[robot.robot_id]
            data['robot_path'] = [[p[0], p[1]] for p in path]
        
        # 发现的出口
        data['discovered_exits'] = [[e[0], e[1]] for e in maze_env.discovered_exits]
        
        # 到达的出口
        data['reached_exits'] = [[e[0], e[1]] for e in maze_env.reached_exit_positions]
        
        # 目标位置
        if robot.current_target:
            data['target_pos'] = [robot.current_target[0], robot.current_target[1]]
        
        # 最短路径
        if hasattr(self.system, 'shortest_path') and self.system.shortest_path:
            data['shortest_path'] = [[p[0], p[1]] for p in self.system.shortest_path]
        
        return data
    
    def _get_slam_map_data(self, global_mapper, maze_env):
        """获取SLAM地图数据"""
        # 将地图数据转换为适合Web显示的格式
        map_array = global_mapper.global_map.tolist()
        
        data = {
            'map_array': map_array,
            'resolution': global_mapper.resolution,
            'grid_size': global_mapper.grid_size,
            'display_size': global_mapper.display_size,
            'maze_size': maze_env.size,
            'reached_exits': [[e[0], e[1]] for e in maze_env.reached_exit_positions]
        }
        
        return data
    
    def _get_frontier_data(self, global_mapper, robot):
        """获取前沿探索数据"""
        data = {
            'frontiers': [[f[0], f[1]] for f in global_mapper.frontiers],
            'robot_pos': [robot.position[0], robot.position[1]],
            'map_array': global_mapper.global_map.tolist(),
            'resolution': global_mapper.resolution,
            'grid_size': global_mapper.grid_size,
            'display_size': global_mapper.display_size
        }
        
        return data
    
    def _get_radar_data(self, robot, maze_env):
        """获取雷达数据"""
        data = {
            'robot_pos': [robot.position[0], robot.position[1]],
            'walls': [],
            'scan_points': [[p[0], p[1]] for p in robot.latest_scan_points],
            'obstacle_points': [[p[0], p[1]] for p in robot.latest_obstacle_points],
            'scan_ranges': robot.latest_scan_ranges,
            'scan_angles': robot.latest_scan_angles,
            'maze_size': maze_env.size
        }
        
        # 墙壁数据
        for wall in maze_env.walls:
            data['walls'].append({
                'x1': wall[0][0], 'y1': wall[0][1],
                'x2': wall[1][0], 'y2': wall[1][1]
            })
        
        return data
    
    def _get_status_info(self, robot, global_mapper, maze_env):
        """获取状态信息"""
        # 计算覆盖率
        total_cells = global_mapper.grid_size * global_mapper.grid_size
        explored_cells = len(global_mapper.all_explored_cells)
        coverage = (explored_cells / total_cells) * 100
        
        data = {
            'robot_id': robot.robot_id,
            'position': [robot.position[0], robot.position[1]],
            'steps': robot.steps,
            'status': robot.status,
            'no_move_count': robot.no_move_counter,
            'frontiers_count': len(global_mapper.frontiers),
            'coverage': coverage,
            'discovered_exits': len(maze_env.discovered_exits),
            'reached_exits': len(maze_env.reached_exit_positions),
            'path_length': 0,
            'diagonal_segments': 0
        }
        
        # 计算路径长度
        if hasattr(self.system, 'shortest_path') and self.system.shortest_path:
            path_length = 0
            diagonal_segments = 0
            path = self.system.shortest_path
            
            for i in range(len(path) - 1):
                p1 = path[i]
                p2 = path[i + 1]
                segment_length = ((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)**0.5
                path_length += segment_length
                
                # 检查对角线移动
                dx = abs(p2[0] - p1[0])
                dy = abs(p2[1] - p1[1])
                if dx > 0.01 and dy > 0.01:
                    diagonal_segments += 1
            
            data['path_length'] = path_length
            data['diagonal_segments'] = diagonal_segments
        
        return data
    
    def run(self, host='127.0.0.1', port=5000, debug=False):
        """运行Web服务器"""
        print(f"🌐 启动Web可视化服务器...")
        print(f"📍 访问地址: http://{host}:{port}")
        print(f"🎮 在浏览器中打开上述地址开始使用")
        
        # 修复DNS解析问题
        try:
            import eventlet
            # 禁用eventlet的DNS解析，避免超时问题
            eventlet.monkey_patch(socket=False)
        except:
            pass
        
        # 使用IP地址而不是hostname避免DNS解析问题
        if host == 'localhost':
            host = '127.0.0.1'
        elif host == '0.0.0.0':
            # 在Windows上0.0.0.0可能有问题，改用127.0.0.1
            host = '127.0.0.1'
            print(f"🔧 调整host为: {host} (避免DNS问题)")
        
        try:
            # 使用更稳定的启动方式
            self.socketio.run(
                self.app, 
                host=host, 
                port=port, 
                debug=debug,
                use_reloader=False,  # 禁用重载器避免重复启动
                log_output=debug
            )
        except Exception as e:
            print(f"❌ 启动失败: {e}")
            print("🔧 尝试备用启动方式...")
            # 备用启动方式
            try:
                self.app.run(host=host, port=port, debug=debug, use_reloader=False)
            except Exception as e2:
                print(f"❌ 备用启动也失败: {e2}")
                raise

class WebGlobalMazeSLAMSystem(GlobalMazeSLAMSystem):
    """Web版迷宫SLAM系统"""
    
    def __init__(self, map_file, update_callback=None):
        # 调用父类初始化，但不创建matplotlib可视化器
        print("🚀 Initializing Web-based Maze SLAM System...")
        
        # 初始化环境和组件
        self.maze_env = MazeEnvironment(map_file)
        self.global_mapper = GlobalSLAMMapper(self.maze_env.size, self.maze_env.display_size)
        self.global_mapper.set_maze_env(self.maze_env)
        self.path_planner = AStarPathPlanner(self.global_mapper, self.maze_env)
        
        # 创建单个机器人
        robot_id = "Robot-1"
        laser_sim = LaserSimulator(self.maze_env)
        
        self.robot = SmartMazeExplorer(robot_id, self.maze_env, self.global_mapper, laser_sim, self.path_planner)
        self.robot.position = self.maze_env.start_pos
        self.robot.step_size = 0.15
        
        # 设置True SLAM模式：机器人通过探索发现出口
        self.maze_env.exits = []  # 清空预设出口
        self.maze_env.discovered_exits = []  # 初始化发现的出口
        self.maze_env.reached_exit_positions = []  # 初始化到达的出口位置
        self.maze_env.exit_detected = False  # 初始化出口检测状态
        self.maze_env.exit_position = None  # 初始化出口位置
        print("🔍 True SLAM mode: Robot will discover exits through exploration")
        
        # Web相关
        self.update_callback = update_callback
        self.shortest_path = None
        self.exploration_completed = False
        
        print(f"✅ Web系统初始化完成")
    
    def run_exploration(self):
        """运行Web版探索"""
        print("🔍 开始Web版迷宫探索...")
        
        import time
        start_time = time.time()
        target_time = 300  # 5分钟
        
        iteration = 0
        
        while True:
            iteration += 1
            
            # 更新机器人（包含激光扫描、地图更新、前沿点更新、目标选择等）
            robot_active = self.robot.update()
            
            # 检查时间限制
            elapsed_time = time.time() - start_time
            if elapsed_time > target_time:
                print(f"⏰ 时间限制到达: {elapsed_time:.1f}s")
                break
            
            # 检查终止条件 - 使用父类方法
            if self._check_termination_conditions(robot_active):
                self.exploration_completed = True
                break
            
            # 发送更新（降低频率以提高性能）
            if iteration % 5 == 0 and self.update_callback:  # 改为每5次更新一次，提高频率
                try:
                    data = self._prepare_web_data()
                    # 使用线程安全的方式发送数据
                    import threading
                    if threading.current_thread() != threading.main_thread():
                        # 如果在后台线程，排队发送
                        self._queue_update(data)
                    else:
                        self.update_callback(data)
                except Exception as e:
                    print(f"Web更新错误: {e}")
            
            # 进度报告
            if iteration % 50 == 0:  # 减少日志频率
                frontiers_count = len(self.global_mapper.frontiers)
                coverage = self._calculate_coverage()
                current_time = time.time() - start_time
                print(f"⏱️  步骤 {iteration}: {current_time:.1f}s, "
                      f"机器人: ({self.robot.position[0]:.2f}, {self.robot.position[1]:.2f}), "
                      f"状态: {self.robot.status}, 覆盖率: {coverage:.1f}%")
            
            # 短暂停顿，让Web界面有时间更新
            time.sleep(0.02)  # 增加停顿时间，确保稳定性
        
        # 计算最短路径 - 使用父类方法
        if self.exploration_completed:
            print("🛤️  计算最优路径...")
            self._calculate_shortest_path()
        
        # 发送最终更新
        if self.update_callback:
            try:
                final_data = self._prepare_web_data()
                final_data['simulation_completed'] = True
                self.update_callback(final_data)
                print("📡 发送最终数据更新")
            except Exception as e:
                print(f"最终Web更新错误: {e}")
        
        print(f"🏁 Web探索完成，共 {iteration} 次迭代")
    
    def _queue_update(self, data):
        """线程安全的数据更新排队"""
        # 简单的实现：直接调用回调，依赖SocketIO的线程安全机制
        try:
            self.update_callback(data)
        except Exception as e:
            print(f"排队更新失败: {e}")
    
    def _calculate_coverage(self):
        """计算探索覆盖率"""
        total_cells = self.global_mapper.grid_size * self.global_mapper.grid_size
        explored_cells = len(self.global_mapper.all_explored_cells)
        return (explored_cells / total_cells) * 100
    
    def _check_termination_conditions(self, robot_active):
        """检查终止条件 - 重写父类方法以支持Web回调"""
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
            
            return True
        
        # 若机器人物理位置已经越过主迷宫边界（含扩展区），同样视为成功逃出
        robot_pos = self.robot.position
        if robot_pos[0] < -1.9 or robot_pos[0] > self.maze_env.size + 1.9 or \
           robot_pos[1] < -1.9 or robot_pos[1] > self.maze_env.size + 1.9:
            # 在极端越界位置直接标记出口
            self.maze_env.mark_exit_reached(robot_pos)
            print("🎉 Robot physically left maze boundary — exit assumed.")
            return True
        
        return False
    
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
        from maze_slam_visual_new2 import OptimalPathPlanner
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
            
            print(f"✅ Optimal shortest path calculated!")
            print(f"   🎯 Algorithm: 8-direction A* with diagonal movement")
            print(f"   📍 Start: ({start_pos[0]:.2f}, {start_pos[1]:.2f})")
            print(f"   🎯 End: ({end_pos[0]:.2f}, {end_pos[1]:.2f})")
            print(f"   📏 Path length: {path_length:.3f} units")
            print(f"   🔢 Waypoints: {len(self.shortest_path)}")
            print(f"   ➡️  Straight segments: {straight_segments}")
            print(f"   ↗️  Diagonal segments: {diagonal_segments}")
            print(f"   🏠 Path optimized within maze boundaries")
        else:
            print("❌ Failed to calculate optimal shortest path")

    def _prepare_web_data(self):
        """为Web准备数据"""
        try:
            # 获取真实迷宫数据
            true_maze_data = {
                'walls': [],
                'robot_pos': [self.robot.position[0], self.robot.position[1]],
                'robot_path': [],
                'start_pos': list(self.maze_env.start_pos),
                'discovered_exits': [[e[0], e[1]] for e in self.maze_env.discovered_exits],
                'reached_exits': [[e[0], e[1]] for e in self.maze_env.reached_exit_positions],
                'target_pos': [self.robot.current_target[0], self.robot.current_target[1]] if self.robot.current_target else None,
                'shortest_path': [[p[0], p[1]] for p in self.shortest_path] if self.shortest_path else [],
                'maze_size': self.maze_env.size
            }
            
            # 墙壁数据
            for wall in self.maze_env.walls:
                true_maze_data['walls'].append({
                    'x1': wall[0][0], 'y1': wall[0][1],
                    'x2': wall[1][0], 'y2': wall[1][1]
                })
            
            # 机器人路径
            if self.robot.robot_id in self.global_mapper.robot_paths:
                path = self.global_mapper.robot_paths[self.robot.robot_id]
                true_maze_data['robot_path'] = [[p[0], p[1]] for p in path]
            
            # SLAM地图数据
            slam_map_data = {
                'map_array': self.global_mapper.global_map.tolist(),
                'resolution': self.global_mapper.resolution,
                'grid_size': self.global_mapper.grid_size,
                'display_size': self.global_mapper.display_size,
                'maze_size': self.maze_env.size
            }
            
            # 前沿数据
            frontier_data = {
                'frontiers': [[f[0], f[1]] for f in self.global_mapper.frontiers],
                'robot_pos': [self.robot.position[0], self.robot.position[1]],
                'map_array': self.global_mapper.global_map.tolist()
            }
            
            # 雷达数据
            radar_data = {
                'robot_pos': [self.robot.position[0], self.robot.position[1]],
                'walls': true_maze_data['walls'],
                'scan_points': [[p[0], p[1]] for p in self.robot.latest_scan_points],
                'obstacle_points': [[p[0], p[1]] for p in self.robot.latest_obstacle_points],
                'maze_size': self.maze_env.size
            }
            
            # 状态信息
            total_cells = self.global_mapper.grid_size * self.global_mapper.grid_size
            explored_cells = len(self.global_mapper.all_explored_cells)
            coverage = (explored_cells / total_cells) * 100
            
            status_info = {
                'robot_id': self.robot.robot_id,
                'position': [self.robot.position[0], self.robot.position[1]],
                'steps': self.robot.steps,
                'status': self.robot.status,
                'frontiers_count': len(self.global_mapper.frontiers),
                'coverage': coverage,
                'discovered_exits': len(self.maze_env.discovered_exits),
                'reached_exits': len(self.maze_env.reached_exit_positions)
            }
            
            return {
                'true_maze': true_maze_data,
                'slam_map': slam_map_data,
                'frontier': frontier_data,
                'radar': radar_data,
                'status': status_info,
                'timestamp': time.time()
            }
            
        except Exception as e:
            print(f"准备Web数据时出错: {e}")
            return {
                'error': str(e),
                'timestamp': time.time()
            }

if __name__ == "__main__":
    # 创建并运行Web可视化系统
    web_viz = WebMazeSLAMVisualizer()
    web_viz.run(host='127.0.0.1', port=5000, debug=False) 