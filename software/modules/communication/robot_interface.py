"""
机器人通信接口模块
支持 LDR3 二进制协议和传统文本协议
"""

import serial
import threading
import time
import math
from typing import Tuple, Optional, Callable, List
import logging

from .ldr3_protocol import (
    LDR3Parser, 
    LDR3CommandBuilder, 
    LDR3Frame,
    ProtocolConstants
)

logger = logging.getLogger(__name__)


class RobotPose:
    """机器人位姿"""
    
    def __init__(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        self.x = x
        self.y = y
        self.theta = theta
        
        # 里程计状态
        self.s_left_m = 0.0
        self.s_right_m = 0.0
        self.last_update_time = time.time()
        
        # 速度状态
        self.v_left_mps = 0.0
        self.v_right_mps = 0.0
        
        self._lock = threading.Lock()
    
    def update_from_odometry(self, delta_left_m: float, delta_right_m: float, 
                            yaw_rad: float, wheel_base_m: float):
        """
        使用差分里程计更新位姿
        
        Args:
            delta_left_m: 左轮增量距离 (米)
            delta_right_m: 右轮增量距离 (米)
            yaw_rad: IMU 航向角 (弧度)
            wheel_base_m: 轮距 (米)
        """
        with self._lock:
            # 计算前进距离
            delta_s = 0.5 * (delta_left_m + delta_right_m)
            
            # 使用 IMU 航向角作为主要方向来源
            self.theta = yaw_rad
            
            # 更新位置 (在当前航向角方向上移动)
            self.x += delta_s * math.cos(self.theta)
            self.y += delta_s * math.sin(self.theta)
            
            # 更新里程计累计
            self.s_left_m += delta_left_m
            self.s_right_m += delta_right_m
            
            self.last_update_time = time.time()
    
    def update_velocity(self, v_left_mps: float, v_right_mps: float):
        """
        更新轮速
        
        Args:
            v_left_mps: 左轮速度 (m/s)
            v_right_mps: 右轮速度 (m/s)
        """
        with self._lock:
            self.v_left_mps = v_left_mps
            self.v_right_mps = v_right_mps
    
    def get(self) -> Tuple[float, float, float]:
        """获取当前位姿 (x, y, theta)"""
        with self._lock:
            return self.x, self.y, self.theta
    
    def get_velocity(self) -> Tuple[float, float]:
        """获取当前轮速 (v_left, v_right)"""
        with self._lock:
            return self.v_left_mps, self.v_right_mps


class RobotInterface:
    """机器人通信接口 (支持 LDR3 协议)"""
    
    def __init__(self, 
                 port: str, 
                 baudrate: int = 921600,
                 wheel_base: float = 0.165,
                 init_pose: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                 use_ldr3_protocol: bool = True,
                 data_callback: Optional[Callable] = None):
        """
        初始化机器人接口
        
        Args:
            port: 串口设备路径
            baudrate: 波特率 (LDR3 推荐 921600)
            wheel_base: 轮距 (米)
            init_pose: 初始位姿 (x, y, theta)
            use_ldr3_protocol: 是否使用 LDR3 二进制协议
            data_callback: 数据回调函数
        """
        self.port = port
        self.baudrate = baudrate
        self.wheel_base = wheel_base
        self.use_ldr3_protocol = use_ldr3_protocol
        self.data_callback = data_callback
        
        # 打开串口
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1,
                write_timeout=0.1
            )
        except Exception as e:
            logger.error(f"无法打开串口 {port}: {e}")
            raise
        
        # 初始化位姿
        x0, y0, theta0 = init_pose
        self.pose = RobotPose(x0, y0, theta0)
        
        # 雷达扫描数据 (angles_rad, ranges_m)
        self._last_scan = ([], [])
        self._scan_lock = threading.Lock()
        
        # LDR3 协议解析器
        if self.use_ldr3_protocol:
            self.ldr3_parser = LDR3Parser(
                lidar_angle_clockwise=True,
                imu_yaw_clockwise=True,
                angle_offset_deg=0.0
            )
        
        # 里程计跟踪 (用于计算增量)
        self._last_odom_left_mm = None
        self._last_odom_right_mm = None
        
        # IMU 航向角偏移 (用于对齐初始角度)
        self._yaw_offset_rad = None
        
        # 速度命令状态 (用于 keepalive)
        self._cmd_vL = 0.0
        self._cmd_vR = 0.0
        self._cmd_lock = threading.Lock()
        
        # 线程控制
        self.is_running = True
        self.rx_thread = threading.Thread(
            target=self._serial_reader_thread, 
            name="ldr3-rx",
            daemon=True
        )
        self.tx_thread = threading.Thread(
            target=self._keepalive_thread,
            name="ldr3-tx",
            daemon=True
        )
        
        # 启动线程
        self.rx_thread.start()
        self.tx_thread.start()
        
        logger.info(f"机器人接口已启动: {port} @ {baudrate} bps, LDR3={use_ldr3_protocol}")
    
    def _serial_reader_thread(self):
        """串口接收线程"""
        while self.is_running:
            try:
                if self.use_ldr3_protocol:
                    self._read_ldr3_protocol()
                else:
                    self._read_text_protocol()
            except Exception as e:
                logger.error(f"串口读取错误: {e}")
                time.sleep(0.01)
    
    def _read_ldr3_protocol(self):
        """读取并解析 LDR3 二进制帧"""
        # 读取数据
        data = self.ser.read(4096)
        if not data:
            time.sleep(0.005)
            return
        
        # 馈送到解析器
        frame = self.ldr3_parser.feed_data(data)
        
        if frame is not None:
            self._process_ldr3_frame(frame)
    
    def _process_ldr3_frame(self, frame: LDR3Frame):
        """
        处理解析完成的 LDR3 帧
        
        Args:
            frame: 解析后的帧数据
        """
        header = frame.header
        
        # 1. 处理 IMU 航向角
        yaw_deg_raw = header.yaw_cdeg / 100.0
        
        # IMU 顺时针转数学逆时针
        yaw_deg = -yaw_deg_raw
        yaw_rad = math.radians(yaw_deg)
        
        # 首次接收时对齐初始航向
        if self._yaw_offset_rad is None:
            current_theta = self.pose.theta
            self._yaw_offset_rad = current_theta - yaw_rad
        
        # 应用偏移
        yaw_rad_aligned = yaw_rad + self._yaw_offset_rad
        yaw_rad_aligned = (yaw_rad_aligned + math.pi) % (2 * math.pi) - math.pi
        
        # 2. 处理里程计
        odom_left_mm = header.odometry_left_mm
        odom_right_mm = header.odometry_right_mm
        
        if self._last_odom_left_mm is not None:
            # 计算增量 (单位: 毫米)
            delta_left_mm = odom_left_mm - self._last_odom_left_mm
            delta_right_mm = odom_right_mm - self._last_odom_right_mm
            
            # 转换为米
            delta_left_m = delta_left_mm / 1000.0
            delta_right_m = delta_right_mm / 1000.0
            
            # 更新位姿
            self.pose.update_from_odometry(
                delta_left_m, delta_right_m,
                yaw_rad_aligned, self.wheel_base
            )
        
        # 保存当前里程计值
        self._last_odom_left_mm = odom_left_mm
        self._last_odom_right_mm = odom_right_mm
        
        # 3. 处理速度
        vel_left_mmps = header.velocity_left_mmps
        vel_right_mmps = header.velocity_right_mmps
        
        vel_left_mps = vel_left_mmps / 1000.0
        vel_right_mps = vel_right_mmps / 1000.0
        
        self.pose.update_velocity(vel_left_mps, vel_right_mps)
        
        # 4. 处理雷达扫描
        if frame.nodes:
            angles = []
            ranges = []
            
            for node in frame.nodes:
                # 可以添加质量过滤
                if node.quality > 10 and node.distance_m > 0.0:
                    angles.append(node.angle_rad)
                    ranges.append(node.distance_m)
            
            with self._scan_lock:
                self._last_scan = (angles, ranges)
        
        # 回调通知
        if self.data_callback:
            self.data_callback('ldr3_frame', {
                'pose': self.pose.get(),
                'velocity': self.pose.get_velocity(),
                'scan_points': len(frame.nodes)
            })
    
    def _read_text_protocol(self):
        """读取并解析文本协议 (兼容旧版)"""
        line_bytes = self.ser.readline()
        if not line_bytes:
            time.sleep(0.01)
            return
        
        try:
            line = line_bytes.decode('utf-8', errors='ignore').strip()
            self._parse_text_line(line)
        except Exception as e:
            logger.debug(f"文本协议解析错误: {e}")
    
    def _parse_text_line(self, line: str):
        """解析文本协议行 (保留兼容性)"""
        # TODO: 实现文本协议解析 (如果需要)
        pass
    
    def _keepalive_thread(self):
        """速度命令 keepalive 线程"""
        keepalive_period = 0.15  # 发送周期 (秒)
        
        while self.is_running:
            try:
                # 获取当前命令
                with self._cmd_lock:
                    vL = self._cmd_vL
                    vR = self._cmd_vR
                
                # 构建并发送命令
                command = LDR3CommandBuilder.build_velocity_command(vL, vR)
                self.ser.write(command)
                self.ser.flush()
                
            except Exception as e:
                logger.debug(f"Keepalive 发送失败: {e}")
            
            time.sleep(keepalive_period)
    
    def get_pose(self) -> Tuple[float, float, float]:
        """
        获取当前位姿
        
        Returns:
            (x, y, theta) 其中 x, y 单位为米，theta 单位为弧度
        """
        return self.pose.get()
    
    def get_lidar_scan(self) -> Tuple[List[float], List[float]]:
        """
        获取最新的雷达扫描数据
        
        Returns:
            (angles_rad, ranges_m) 角度和距离列表
        """
        with self._scan_lock:
            return self._last_scan
    
    def get_velocity(self) -> Tuple[float, float]:
        """
        获取当前轮速
        
        Returns:
            (v_left_mps, v_right_mps) 左右轮速度 (m/s)
        """
        return self.pose.get_velocity()
    
    def send_wheel_speeds(self, v_left_mps: float, v_right_mps: float):
        """
        发送轮速控制命令
        
        Args:
            v_left_mps: 左轮速度 (m/s)
            v_right_mps: 右轮速度 (m/s)
        """
        with self._cmd_lock:
            self._cmd_vL = float(v_left_mps)
            self._cmd_vR = float(v_right_mps)
        
        # 立即发送一次 (keepalive 线程会持续发送)
        try:
            command = LDR3CommandBuilder.build_velocity_command(
                v_left_mps, v_right_mps
            )
            self.ser.write(command)
            self.ser.flush()
            logger.debug(f"发送速度命令: vL={v_left_mps:.3f}, vR={v_right_mps:.3f}")
        except Exception as e:
            logger.error(f"发送速度命令失败: {e}")
    
    def send_binary_command(self, command_byte: int):
        """
        发送单字节命令 (兼容旧接口)
        
        Args:
            command_byte: 命令字节
        """
        try:
            import struct
            self.ser.write(struct.pack('B', command_byte))
            logger.debug(f"发送二进制命令: 0x{command_byte:02X}")
        except Exception as e:
            logger.error(f"发送命令失败: {e}")
    
    def stop(self):
        """停止机器人并关闭接口"""
        logger.info("停止机器人...")
        
        # 发送停止命令
        self.send_wheel_speeds(0.0, 0.0)
        time.sleep(0.2)
        
        # 停止线程
        self.is_running = False
        
        if self.rx_thread.is_alive():
            self.rx_thread.join(timeout=1.0)
        if self.tx_thread.is_alive():
            self.tx_thread.join(timeout=1.0)
        
        # 关闭串口
        try:
            self.ser.close()
        except Exception as e:
            logger.error(f"关闭串口失败: {e}")
        
        logger.info("机器人接口已关闭")
