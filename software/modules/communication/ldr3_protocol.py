"""
LDR3 通信协议模块
用于解析和构建 STM32 下位机的 LDR3 二进制帧格式

协议格式：
- 帧头：20字节 (magic + 帧信息 + 里程计 + 速度)
- 节点：N × 6字节 (每个 LiDAR 扫描点)
"""

import struct
import math
import logging
from dataclasses import dataclass
from typing import List, Tuple, Optional

logger = logging.getLogger(__name__)


# ==================== 协议常量定义 ====================

class ProtocolConstants:
    """LDR3 协议常量"""
    # 魔数标识 (小端序 'LDR3')
    FRAME_MAGIC = 0x3352444C
    
    # 帧头格式: <IHHiihh (20字节)
    # I: magic (4B)
    # H: node_count (2B) 
    # H: yaw_cdeg (2B) - IMU航向角(厘度)
    # i: odometry_left_mm (4B) - 左轮累计里程(毫米)
    # i: odometry_right_mm (4B) - 右轮累计里程(毫米)
    # h: velocity_left_mmps (2B) - 左轮速度(毫米/秒)
    # h: velocity_right_mmps (2B) - 右轮速度(毫米/秒)
    HEADER_FORMAT = "<IHHiihh"
    HEADER_SIZE = struct.calcsize(HEADER_FORMAT)  # 20字节
    
    # 节点格式: <HHBB (6字节)
    # H: distance_mm (2B) - 距离(毫米)
    # H: angle_cdeg (2B) - 角度(厘度)
    # B: quality (1B) - 质量
    # B: flags (1B) - 标志位
    NODE_FORMAT = "<HHBB"
    NODE_SIZE = struct.calcsize(NODE_FORMAT)  # 6字节
    
    # 速度命令门槛 (单位: m/s)
    # 用于克服电机启动摩擦力的最小速度
    MIN_WHEEL_SPEED_FORWARD = 0.13  # 前进最小轮速
    MIN_WHEEL_SPEED_ROTATION = 0.13  # 原地旋转最小轮速
    
    # 雷达传感器偏心补偿 (相对机器人中心)
    # 雷达安装在车身中心后方 5cm (车头朝向 +x)
    LIDAR_OFFSET_X = -0.05  # 米
    LIDAR_OFFSET_Y = 0.0    # 米


# ==================== 数据结构 ====================

@dataclass
class LidarNode:
    """单个雷达扫描点数据"""
    distance_m: float      # 距离 (米)
    angle_rad: float       # 角度 (弧度)
    quality: int           # 质量值
    flags: int             # 标志位


@dataclass
class FrameHeader:
    """LDR3 帧头数据"""
    magic: int                    # 魔数
    node_count: int               # 节点数量
    yaw_cdeg: int                 # IMU航向角(厘度)
    odometry_left_mm: int         # 左轮累计里程(毫米)
    odometry_right_mm: int        # 右轮累计里程(毫米)
    velocity_left_mmps: int       # 左轮速度(毫米/秒)
    velocity_right_mmps: int      # 右轮速度(毫米/秒)


@dataclass
class LDR3Frame:
    """完整的 LDR3 帧数据"""
    header: FrameHeader
    nodes: List[LidarNode]


# ==================== 协议解析器 ====================

class LDR3Parser:
    """LDR3 协议帧解析器"""
    
    def __init__(self, 
                 lidar_angle_clockwise: bool = True,
                 imu_yaw_clockwise: bool = True,
                 angle_offset_deg: float = 0.0):
        """
        初始化解析器
        
        Args:
            lidar_angle_clockwise: 雷达角度是否顺时针增加
            imu_yaw_clockwise: IMU航向角是否顺时针增加
            angle_offset_deg: 角度偏移校准 (度)
        """
        self.lidar_angle_clockwise = lidar_angle_clockwise
        self.imu_yaw_clockwise = imu_yaw_clockwise
        self.angle_offset_deg = angle_offset_deg
        
        self._buffer = bytearray()
        self._parsed_frame_count = 0
    
    def find_frame_start(self, data: bytes) -> int:
        """
        在数据中查找帧起始位置
        
        Args:
            data: 待搜索的字节数据
            
        Returns:
            帧起始位置索引，-1 表示未找到
        """
        magic_bytes = struct.pack("<I", ProtocolConstants.FRAME_MAGIC)
        return data.find(magic_bytes)
    
    def parse_header(self, data: bytes) -> Optional[FrameHeader]:
        """
        解析帧头
        
        Args:
            data: 至少包含 HEADER_SIZE 字节的数据
            
        Returns:
            解析成功返回 FrameHeader，否则返回 None
        """
        if len(data) < ProtocolConstants.HEADER_SIZE:
            return None
        
        try:
            unpacked = struct.unpack(
                ProtocolConstants.HEADER_FORMAT, 
                data[:ProtocolConstants.HEADER_SIZE]
            )
            
            magic, node_count, yaw_cdeg, odom_left, odom_right, vel_left, vel_right = unpacked
            
            if magic != ProtocolConstants.FRAME_MAGIC:
                return None
            
            return FrameHeader(
                magic=magic,
                node_count=node_count,
                yaw_cdeg=yaw_cdeg,
                odometry_left_mm=odom_left,
                odometry_right_mm=odom_right,
                velocity_left_mmps=vel_left,
                velocity_right_mmps=vel_right
            )
        except struct.error as e:
            logger.debug(f"帧头解析失败: {e}")
            return None
    
    def parse_nodes(self, data: bytes, count: int) -> Optional[List[LidarNode]]:
        """
        解析雷达节点数据
        
        Args:
            data: 节点数据字节
            count: 节点数量
            
        Returns:
            解析成功返回节点列表，否则返回 None
        """
        expected_size = count * ProtocolConstants.NODE_SIZE
        if len(data) < expected_size:
            return None
        
        nodes = []
        try:
            for i in range(count):
                offset = i * ProtocolConstants.NODE_SIZE
                node_data = data[offset:offset + ProtocolConstants.NODE_SIZE]
                
                dist_mm, ang_cdeg, quality, flags = struct.unpack(
                    ProtocolConstants.NODE_FORMAT, 
                    node_data
                )
                
                # 单位转换: 毫米 → 米
                distance_m = max(0.0, dist_mm / 1000.0)
                
                # 角度转换: 厘度 → 度 → 弧度
                angle_deg = ang_cdeg / 100.0
                
                # 雷达角度方向转换 (如果需要)
                if self.lidar_angle_clockwise:
                    angle_deg = -angle_deg
                
                # 应用角度偏移
                angle_deg += self.angle_offset_deg
                
                # 转换为弧度
                angle_rad = math.radians(angle_deg)
                
                nodes.append(LidarNode(
                    distance_m=distance_m,
                    angle_rad=angle_rad,
                    quality=quality,
                    flags=flags
                ))
        except struct.error as e:
            logger.debug(f"节点解析失败: {e}")
            return None
        
        return nodes
    
    def parse_frame(self, data: bytes) -> Optional[Tuple[LDR3Frame, int]]:
        """
        从数据中解析完整帧
        
        Args:
            data: 字节数据
            
        Returns:
            成功返回 (LDR3Frame, 帧总字节数)，失败返回 None
        """
        # 查找帧起始
        start_idx = self.find_frame_start(data)
        if start_idx < 0:
            return None
        
        # 跳过无效数据
        if start_idx > 0:
            data = data[start_idx:]
        
        # 解析帧头
        header = self.parse_header(data)
        if header is None:
            return None
        
        # 计算完整帧大小
        frame_size = ProtocolConstants.HEADER_SIZE + header.node_count * ProtocolConstants.NODE_SIZE
        
        # 检查数据是否足够
        if len(data) < frame_size:
            return None
        
        # 解析节点
        node_data = data[ProtocolConstants.HEADER_SIZE:frame_size]
        nodes = self.parse_nodes(node_data, header.node_count)
        
        if nodes is None:
            return None
        
        self._parsed_frame_count += 1
        
        return LDR3Frame(header=header, nodes=nodes), frame_size
    
    def feed_data(self, data: bytes) -> Optional[LDR3Frame]:
        """
        向解析器馈送数据并尝试解析帧
        
        Args:
            data: 新接收的字节数据
            
        Returns:
            成功解析返回 LDR3Frame，否则返回 None
        """
        self._buffer.extend(data)
        
        # 尝试解析
        result = self.parse_frame(bytes(self._buffer))
        
        if result is not None:
            frame, frame_size = result
            # 移除已解析的数据
            del self._buffer[:frame_size]
            return frame
        
        # 限制缓冲区大小，防止内存溢出
        if len(self._buffer) > 1024 * 1024:  # 1MB
            # 只保留最后 4 字节 (可能是部分魔数)
            del self._buffer[:-4]
        
        return None


# ==================== 协议构建器 ====================

class LDR3CommandBuilder:
    """LDR3 速度命令构建器"""
    
    @staticmethod
    def apply_velocity_threshold(vL: float, vR: float) -> Tuple[float, float]:
        """
        应用速度门槛逻辑，确保轮速满足电机启动要求
        
        策略：
        1. 如果两轮同向（前进/后退）：只对平均速度应用门槛
        2. 如果两轮反向（原地旋转）：对差速应用门槛
        
        Args:
            vL: 左轮目标速度 (m/s)
            vR: 右轮目标速度 (m/s)
            
        Returns:
            调整后的 (vL, vR)
        """
        FLOOR_FWD = ProtocolConstants.MIN_WHEEL_SPEED_FORWARD
        FLOOR_ROT = ProtocolConstants.MIN_WHEEL_SPEED_ROTATION
        
        # 零速度
        if vL == 0.0 and vR == 0.0:
            return 0.0, 0.0
        
        # 计算平均速度(前进分量)和差速(旋转分量)
        avg_velocity = 0.5 * (vL + vR)
        diff_velocity = 0.5 * (vR - vL)
        
        # 判断运动模式
        same_direction = (vL >= 0.0 and vR >= 0.0) or (vL <= 0.0 and vR <= 0.0)
        
        if same_direction:
            # 同向运动：应用前进门槛到平均速度
            if abs(avg_velocity) > 0.0:
                new_avg = math.copysign(
                    max(abs(avg_velocity), FLOOR_FWD),
                    avg_velocity
                )
            else:
                new_avg = 0.0
            
            new_diff = diff_velocity
        else:
            # 反向运动(原地旋转)：应用旋转门槛到差速
            if abs(diff_velocity) > 0.0:
                new_diff = math.copysign(
                    max(abs(diff_velocity), FLOOR_ROT),
                    diff_velocity
                )
            else:
                new_diff = 0.0
            
            new_avg = avg_velocity
        
        # 重构左右轮速度
        vL_adjusted = new_avg - new_diff
        vR_adjusted = new_avg + new_diff
        
        return vL_adjusted, vR_adjusted
    
    @staticmethod
    def build_velocity_command(vL_mps: float, vR_mps: float, 
                               apply_threshold: bool = True) -> bytes:
        """
        构建速度控制命令
        
        命令格式: "A&B\r\n"
        其中 A = int(vL_mps * 100), B = int(vR_mps * 100)
        
        Args:
            vL_mps: 左轮速度 (m/s)
            vR_mps: 右轮速度 (m/s)
            apply_threshold: 是否应用速度门槛
            
        Returns:
            ASCII 格式的命令字节串
        """
        # 应用速度门槛
        if apply_threshold:
            vL_mps, vR_mps = LDR3CommandBuilder.apply_velocity_threshold(vL_mps, vR_mps)
        
        # 转换为整数 (cm/s)
        vL_cmps = int(round(vL_mps * 100.0))
        vR_cmps = int(round(vR_mps * 100.0))
        
        # 限幅
        vL_cmps = max(-10000, min(10000, vL_cmps))
        vR_cmps = max(-10000, min(10000, vR_cmps))
        
        # 构建命令字符串
        command_str = f"{vL_cmps}&{vR_cmps}\r\n"
        
        return command_str.encode('ascii')


# ==================== 坐标转换工具 ====================

class CoordinateTransform:
    """坐标转换工具类"""
    
    @staticmethod
    def lidar_to_robot_frame(nodes: List[LidarNode]) -> List[Tuple[float, float]]:
        """
        将雷达扫描点从传感器坐标系转换到机器人坐标系
        (应用雷达偏心补偿)
        
        Args:
            nodes: 雷达节点列表
            
        Returns:
            [(x, y), ...] 在机器人坐标系中的点坐标 (米)
        """
        points = []
        offset_x = ProtocolConstants.LIDAR_OFFSET_X
        offset_y = ProtocolConstants.LIDAR_OFFSET_Y
        
        for node in nodes:
            # 极坐标 → 笛卡尔坐标 (传感器坐标系)
            x_sensor = node.distance_m * math.cos(node.angle_rad)
            y_sensor = node.distance_m * math.sin(node.angle_rad)
            
            # 平移到机器人坐标系
            x_robot = x_sensor + offset_x
            y_robot = y_sensor + offset_y
            
            points.append((x_robot, y_robot))
        
        return points
    
    @staticmethod
    def lidar_to_world_frame(nodes: List[LidarNode], 
                            robot_x: float, robot_y: float, robot_theta: float) -> List[Tuple[float, float]]:
        """
        将雷达扫描点从传感器坐标系转换到世界坐标系
        
        Args:
            nodes: 雷达节点列表
            robot_x: 机器人世界坐标 x (米)
            robot_y: 机器人世界坐标 y (米)
            robot_theta: 机器人航向角 (弧度)
            
        Returns:
            [(x, y), ...] 在世界坐标系中的点坐标 (米)
        """
        points = []
        offset_x = ProtocolConstants.LIDAR_OFFSET_X
        offset_y = ProtocolConstants.LIDAR_OFFSET_Y
        
        cos_theta = math.cos(robot_theta)
        sin_theta = math.sin(robot_theta)
        
        for node in nodes:
            # 雷达传感器在机器人坐标系中的位置
            lidar_x_robot = offset_x
            lidar_y_robot = offset_y
            
            # 传感器位置在世界坐标系
            lidar_x_world = robot_x + cos_theta * lidar_x_robot - sin_theta * lidar_y_robot
            lidar_y_world = robot_y + sin_theta * lidar_x_robot + cos_theta * lidar_y_robot
            
            # 扫描点相对传感器的位置
            dx_sensor = node.distance_m * math.cos(node.angle_rad)
            dy_sensor = node.distance_m * math.sin(node.angle_rad)
            
            # 旋转到世界坐标系
            dx_world = cos_theta * dx_sensor - sin_theta * dy_sensor
            dy_world = sin_theta * dx_sensor + cos_theta * dy_sensor
            
            # 最终世界坐标
            x_world = lidar_x_world + dx_world
            y_world = lidar_y_world + dy_world
            
            points.append((x_world, y_world))
        
        return points

