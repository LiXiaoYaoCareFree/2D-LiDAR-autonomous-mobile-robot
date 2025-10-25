#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Python SLAM桥接脚本 - 接收C#数据并调用PoseGraph_Slam-Simulation
"""

import sys
import json
import time
import threading
import queue
from datetime import datetime
import numpy as np
import os

# 添加PoseGraph_Slam-Simulation路径
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'PoseGraph_Slam-Simulation'))

try:
    from new import PoseGraphSLAM
    from maze_slam_simulation import Robot, Environment, SLAMSimulation
except ImportError as e:
    print(f"ERROR: 无法导入PoseGraph_Slam-Simulation模块: {e}")
    sys.exit(1)

class PythonSLAMBridge:
    """Python SLAM桥接类"""
    
    def __init__(self):
        self.slam = PoseGraphSLAM()
        self.robot_data_queue = queue.Queue()
        self.running = False
        self.lock = threading.Lock()
        
        # 初始化SLAM相关数据
        self.current_pose = [0.0, 0.0, 0.0]  # [x, y, theta]
        self.trajectory = []
        self.map_points = []
        self.laser_data_history = []
        
        # 统计信息
        self.stats = {
            'total_nodes': 0,
            'total_edges': 0,
            'optimization_count': 0,
            'last_update': None
        }
    
    def start(self):
        """启动SLAM桥接"""
        self.running = True
        print("Python SLAM桥接已启动")
        
        # 启动数据处理线程
        self.data_thread = threading.Thread(target=self._process_data_loop)
        self.data_thread.daemon = True
        self.data_thread.start()
    
    def stop(self):
        """停止SLAM桥接"""
        self.running = False
        print("Python SLAM桥接已停止")
    
    def add_sensor_data(self, sensor_data):
        """添加传感器数据"""
        try:
            with self.lock:
                self.robot_data_queue.put(sensor_data)
            return True
        except Exception as e:
            print(f"ERROR: 添加传感器数据失败: {e}")
            return False
    
    def _process_data_loop(self):
        """数据处理循环"""
        while self.running:
            try:
                # 获取传感器数据
                if not self.robot_data_queue.empty():
                    sensor_data = self.robot_data_queue.get(timeout=0.1)
                    self._process_sensor_data(sensor_data)
                else:
                    time.sleep(0.01)  # 短暂休眠
            except queue.Empty:
                continue
            except Exception as e:
                print(f"ERROR: 数据处理循环错误: {e}")
                time.sleep(0.1)
    
    def _process_sensor_data(self, sensor_data):
        """处理传感器数据"""
        try:
            # 更新机器人位姿
            self.current_pose = [
                sensor_data['robot_pose']['x'],
                sensor_data['robot_pose']['y'],
                sensor_data['robot_pose']['theta']
            ]
            
            # 添加到轨迹
            self.trajectory.append(self.current_pose.copy())
            
            # 处理激光数据
            if 'laser_data' in sensor_data and sensor_data['laser_data']:
                laser_points = []
                for point in sensor_data['laser_data']:
                    if point['distance'] > 0 and point['distance'] < 10:  # 有效距离范围
                        laser_points.append([
                            point['x'],
                            point['y'],
                            point['quality']
                        ])
                
                if laser_points:
                    # 添加到位姿图SLAM
                    node_id = self.slam.add_node(self.current_pose, laser_points)
                    self.stats['total_nodes'] += 1
                    
                    # 添加里程计边
                    if len(self.trajectory) > 1:
                        prev_pose = self.trajectory[-2]
                        dx = self.current_pose[0] - prev_pose[0]
                        dy = self.current_pose[1] - prev_pose[1]
                        dtheta = self.current_pose[2] - prev_pose[2]
                        
                        # 归一化角度
                        dtheta = self._normalize_angle(dtheta)
                        
                        # 添加边
                        self.slam.add_edge(
                            node_id - 1, node_id,
                            [dx, dy, dtheta],
                            np.eye(3)  # 信息矩阵
                        )
                        self.stats['total_edges'] += 1
                    
                    # 执行图优化（每10个节点优化一次）
                    if self.stats['total_nodes'] % 10 == 0:
                        self.slam.optimize()
                        self.stats['optimization_count'] += 1
                    
                    # 更新地图点
                    self._update_map_points()
                    
                    # 发送结果
                    self._send_slam_result()
            
            self.stats['last_update'] = datetime.now()
            
        except Exception as e:
            print(f"ERROR: 处理传感器数据失败: {e}")
    
    def _normalize_angle(self, angle):
        """归一化角度到[-pi, pi]"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
    
    def _update_map_points(self):
        """更新地图点"""
        try:
            # 获取优化后的地图点
            map_points = self.slam.get_map_points(use_optimized=True)
            if map_points.shape[1] > 0:
                self.map_points = map_points.T.tolist()  # 转换为列表格式
            else:
                self.map_points = []
        except Exception as e:
            print(f"ERROR: 更新地图点失败: {e}")
    
    def _send_slam_result(self):
        """发送SLAM结果"""
        try:
            result = {
                'map_points': self.map_points,
                'trajectory': self.trajectory,
                'optimized_poses': self.slam.nodes,
                'statistics': self.stats,
                'timestamp': datetime.now().isoformat()
            }
            
            # 输出结果到标准输出
            print(f"SLAM_RESULT:{json.dumps(result)}")
            sys.stdout.flush()
            
        except Exception as e:
            print(f"ERROR: 发送SLAM结果失败: {e}")
    
    def get_slam_result(self):
        """获取当前SLAM结果"""
        with self.lock:
            return {
                'map_points': self.map_points,
                'trajectory': self.trajectory,
                'optimized_poses': self.slam.nodes,
                'statistics': self.stats,
                'timestamp': datetime.now().isoformat()
            }

def main():
    """主函数"""
    bridge = PythonSLAMBridge()
    bridge.start()
    
    try:
        # 读取标准输入
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
                
            if line == "GET_SLAM_RESULT":
                # 请求SLAM结果
                result = bridge.get_slam_result()
                print(f"SLAM_RESULT:{json.dumps(result)}")
                sys.stdout.flush()
            elif line == "STOP":
                # 停止SLAM
                bridge.stop()
                break
            else:
                try:
                    # 解析传感器数据
                    sensor_data = json.loads(line)
                    bridge.add_sensor_data(sensor_data)
                except json.JSONDecodeError as e:
                    print(f"ERROR: JSON解析失败: {e}")
                except Exception as e:
                    print(f"ERROR: 处理输入失败: {e}")
    
    except KeyboardInterrupt:
        print("收到中断信号，正在停止...")
    except Exception as e:
        print(f"ERROR: 主循环错误: {e}")
    finally:
        bridge.stop()

if __name__ == "__main__":
    main()
