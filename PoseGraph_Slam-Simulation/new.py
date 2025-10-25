# -*- coding: utf-8 -*-
"""
独立的位姿图优化SLAM (Pose-Graph SLAM) 模块
"""

import numpy as np
import math

def _pi_2_pi(angle):
    """将角度归一化到[-pi, pi]"""
    return (angle + math.pi) % (2 * math.pi) - math.pi

class Edge:
    def __init__(self, from_id, to_id, measurement, information):
        self.from_id = from_id
        self.to_id = to_id
        self.measurement = measurement
        self.information = information

class PoseGraphSLAM:
    """使用图优化的SLAM实现"""
    def __init__(self):
        self.nodes = []
        self.edges = []
        self.keyframes = []
        self.optimized_nodes = None

    def add_node(self, pose, points):
        """添加一个节点(位姿)和一个关键帧(点云)"""
        self.nodes.append(pose)
        self.keyframes.append(points)
        return len(self.nodes) - 1

    def add_edge(self, from_id, to_id, measurement, information):
        """向图中添加一条边 (约束)"""
        edge = Edge(from_id, to_id, measurement, information)
        self.edges.append(edge)

    def get_map_points(self, use_optimized=True):
        """
        获取全局点云地图.
        :param use_optimized: True则使用优化后的位姿, False使用优化前的里程计位姿
        """
        all_points = []
        
        poses_to_use = self.optimized_nodes if use_optimized and self.optimized_nodes else self.nodes
        if not poses_to_use:
            return np.array([[], []])

        for i, pose in enumerate(poses_to_use):
            x, y, theta = pose
            points = self.keyframes[i]
            if points.shape[1] == 0:
                continue

            R = np.array([[math.cos(theta), -math.sin(theta)],
                          [math.sin(theta), math.cos(theta)]])
            
            transformed_points = (R @ points) + np.array([x, y])[:, np.newaxis]
            all_points.append(transformed_points)
        
        if not all_points:
            return np.array([[], []])
            
        return np.hstack(all_points)

    def optimize_graph(self, num_iterations=20, verbose=True):
        """执行位姿图优化"""
        if not self.edges:
            if verbose:
                print("图中没有边，无需优化。")
            return

        x = np.array(self.nodes).flatten()

        for i in range(num_iterations):
            H = np.zeros((len(x), len(x)))
            b = np.zeros(len(x))

            # 锚定第一个节点
            H[0:3, 0:3] += np.identity(3)

            for edge in self.edges:
                from_idx = edge.from_id
                to_idx = edge.to_id  # 修复：使用正确的属性名

                v_i = x[3 * from_idx : 3 * from_idx + 3]
                v_j = x[3 * to_idx : 3 * to_idx + 3]
                z_ij = edge.measurement

                # 计算误差
                t_i = v_i[:2]
                theta_i = v_i[2]
                
                R_i = np.array([[math.cos(theta_i), -math.sin(theta_i)],
                              [math.sin(theta_i), math.cos(theta_i)]])
                
                t_j = v_j[:2]
                theta_j = v_j[2]

                delta_t = t_j - t_i
                
                e_t = R_i.T @ delta_t - z_ij[:2]
                e_theta = self._normalize_angle(theta_j - theta_i - z_ij[2])
                
                error = np.concatenate([e_t, [e_theta]])

                # 计算雅可比矩阵
                J_i = np.zeros((3, 3))
                J_i[:2, :2] = -R_i.T
                J_i[0, 2] = R_i[0, 1] * delta_t[0] + R_i[1, 1] * delta_t[1]
                J_i[1, 2] = -R_i[0, 0] * delta_t[0] - R_i[1, 0] * delta_t[1]
                J_i[2, 2] = -1

                J_j = np.zeros((3, 3))
                J_j[:2, :2] = R_i.T
                J_j[2, 2] = 1

                # 更新H矩阵和b向量
                p1_indices = np.arange(3 * from_idx, 3 * from_idx + 3)
                p2_indices = np.arange(3 * to_idx, 3 * to_idx + 3)

                H[p1_indices, p1_indices] += J_i.T @ edge.information @ J_i
                H[p1_indices, p2_indices] += J_i.T @ edge.information @ J_j
                H[p2_indices, p1_indices] += J_j.T @ edge.information @ J_i
                H[p2_indices, p2_indices] += J_j.T @ edge.information @ J_j

                b[p1_indices] += (J_i.T @ edge.information @ error)
                b[p2_indices] += (J_j.T @ edge.information @ error)
                
            # 求解线性方程 H * dx = -b
            # 使用伪逆以处理H可能为奇异矩阵的情况
            try:
                dx = -np.linalg.pinv(H) @ b
            except np.linalg.LinAlgError:
                print("警告: H矩阵奇异, 跳过本次迭代")
                continue

            # 更新节点位姿
            x += dx
        
        # 将更新后的位姿存入 optimized_nodes
        self.optimized_nodes = x.reshape((-1, 3)).tolist()
        
        if verbose:
            print("--- 图优化完成 ---")

    def _normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

