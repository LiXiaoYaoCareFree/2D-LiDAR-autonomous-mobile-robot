"""
Windows优化的机器人通信接口
基于原有RobotInterface，针对Windows系统进行优化
"""

import serial
import threading
import time
import math
import logging
from typing import Tuple, Optional, Callable, List
import platform

from .robot_interface import RobotInterface, RobotPose
from .ldr3_protocol import (
    LDR3Parser, 
    LDR3CommandBuilder, 
    LDR3Frame,
    ProtocolConstants
)
from .port_detector import create_port_detector
from .permission_handler import check_and_fix_permissions

logger = logging.getLogger(__name__)


class WindowsRobotInterface(RobotInterface):
    """Windows优化的机器人通信接口"""
    
    def __init__(self, 
                 port: str = None, 
                 baudrate: int = 921600,
                 wheel_base: float = 0.165,
                 init_pose: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                 use_ldr3_protocol: bool = True,
                 data_callback: Optional[Callable] = None,
                 auto_detect_port: bool = True):
        """
        初始化Windows机器人接口
        
        Args:
            port: 串口设备路径，为None时自动检测
            baudrate: 波特率 (LDR3 推荐 921600)
            wheel_base: 轮距 (米)
            init_pose: 初始位姿 (x, y, theta)
            use_ldr3_protocol: 是否使用 LDR3 二进制协议
            data_callback: 数据回调函数
            auto_detect_port: 是否自动检测端口
        """
        self.auto_detect_port = auto_detect_port
        
        # 自动检测端口
        if port is None and auto_detect_port:
            port = self._auto_detect_robot_port()
            if port is None:
                raise RuntimeError("无法自动检测机器人串口，请手动指定端口")
        
        # 验证端口格式
        if port and not self._validate_port_format(port):
            logger.warning(f"端口格式可能不正确: {port}")
        
        # 检查并处理权限问题
        if port:
            logger.info(f"检查端口 {port} 的权限...")
            if not check_and_fix_permissions(port):
                logger.warning(f"端口 {port} 权限检查失败，但继续尝试连接")
        
        # 调用父类初始化
        super().__init__(
            port=port,
            baudrate=baudrate,
            wheel_base=wheel_base,
            init_pose=init_pose,
            use_ldr3_protocol=use_ldr3_protocol,
            data_callback=data_callback
        )
        
        # Windows特定的优化设置
        self._apply_windows_optimizations()
    
    def _auto_detect_robot_port(self) -> Optional[str]:
        """自动检测机器人端口"""
        try:
            detector = create_port_detector()
            port = detector.find_robot_port()
            
            if port:
                logger.info(f"自动检测到机器人端口: {port}")
                return port
            else:
                logger.warning("自动检测失败，请手动指定端口")
                return None
                
        except Exception as e:
            logger.error(f"自动检测端口失败: {e}")
            return None
    
    def _validate_port_format(self, port: str) -> bool:
        """验证端口格式"""
        if not port:
            return False
        
        system = platform.system()
        if system == "Windows":
            # Windows COM端口格式
            return port.upper().startswith("COM") and port[3:].isdigit()
        elif system == "Darwin":  # macOS
            # macOS设备格式
            return port.startswith("/dev/tty.")
        else:  # Linux
            # Linux设备格式
            return port.startswith("/dev/tty")
    
    def _apply_windows_optimizations(self):
        """应用Windows特定的优化设置"""
        try:
            # 设置Windows特定的串口参数
            if hasattr(self.ser, 'set_buffer_size'):
                # 如果支持，设置缓冲区大小
                self.ser.set_buffer_size(rx_size=8192, tx_size=8192)
            
            # 设置Windows特定的超时参数
            self.ser.timeout = 0.1
            self.ser.write_timeout = 0.1
            
            # 刷新缓冲区
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            
            logger.info("已应用Windows串口优化设置")
            
        except Exception as e:
            logger.debug(f"Windows优化设置失败: {e}")
    
    def reconnect(self, new_port: str = None, new_baudrate: int = None):
        """
        重新连接串口
        
        Args:
            new_port: 新的端口，为None时使用当前端口
            new_baudrate: 新的波特率，为None时使用当前波特率
        """
        logger.info("正在重新连接串口...")
        
        # 停止当前连接
        self.stop()
        
        # 更新参数
        if new_port is not None:
            self.port = new_port
        if new_baudrate is not None:
            self.baudrate = new_baudrate
        
        # 等待一下
        time.sleep(1.0)
        
        # 重新初始化
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1,
                write_timeout=0.1
            )
            
            # 重新启动线程
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
            
            self.rx_thread.start()
            self.tx_thread.start()
            
            logger.info(f"串口重新连接成功: {self.port} @ {self.baudrate} bps")
            
        except Exception as e:
            logger.error(f"串口重新连接失败: {e}")
            raise
    
    def get_connection_status(self) -> dict:
        """获取连接状态信息"""
        status = {
            'port': self.port,
            'baudrate': self.baudrate,
            'is_open': self.ser.is_open if hasattr(self, 'ser') else False,
            'is_running': self.is_running,
            'rx_thread_alive': self.rx_thread.is_alive() if hasattr(self, 'rx_thread') else False,
            'tx_thread_alive': self.tx_thread.is_alive() if hasattr(self, 'tx_thread') else False,
            'protocol': 'LDR3' if self.use_ldr3_protocol else 'Text'
        }
        
        return status
    
    def scan_available_ports(self) -> List[dict]:
        """扫描可用的串口设备"""
        try:
            detector = create_port_detector()
            return detector.get_available_ports()
        except Exception as e:
            logger.error(f"扫描串口失败: {e}")
            return []
    
    def test_connection(self) -> bool:
        """测试连接是否正常"""
        try:
            if not self.ser.is_open:
                return False
            
            # 发送测试命令
            test_command = LDR3CommandBuilder.build_velocity_command(0.0, 0.0)
            self.ser.write(test_command)
            self.ser.flush()
            
            # 等待响应
            time.sleep(0.1)
            response = self.ser.read(100)
            
            # 检查是否有数据返回
            return len(response) > 0
            
        except Exception as e:
            logger.debug(f"连接测试失败: {e}")
            return False


def create_windows_robot_interface(config: dict = None) -> WindowsRobotInterface:
    """
    便捷函数：创建Windows机器人接口
    
    Args:
        config: 配置字典，包含port, baudrate等参数
        
    Returns:
        WindowsRobotInterface实例
    """
    if config is None:
        config = {}
    
    return WindowsRobotInterface(
        port=config.get('port'),
        baudrate=config.get('baudrate', 921600),
        wheel_base=config.get('wheel_base', 0.165),
        init_pose=config.get('init_pose', (0.0, 0.0, 0.0)),
        use_ldr3_protocol=config.get('use_ldr3_protocol', True),
        auto_detect_port=config.get('auto_detect_port', True)
    )


if __name__ == "__main__":
    # 测试Windows机器人接口
    logging.basicConfig(level=logging.INFO)
    
    print("=== Windows机器人接口测试 ===")
    
    try:
        # 创建接口（自动检测端口）
        robot = create_windows_robot_interface()
        
        print("连接状态:")
        status = robot.get_connection_status()
        for key, value in status.items():
            print(f"  {key}: {value}")
        
        print("\n可用端口:")
        ports = robot.scan_available_ports()
        for port in ports:
            print(f"  {port['port']}: {port['description']}")
        
        print("\n连接测试:")
        if robot.test_connection():
            print("  连接正常")
        else:
            print("  连接异常")
        
        # 运行一段时间
        print("\n运行测试 (5秒)...")
        time.sleep(5)
        
        # 获取位姿
        pose = robot.get_pose()
        print(f"当前位姿: x={pose[0]:.3f}, y={pose[1]:.3f}, theta={pose[2]:.3f}")
        
        # 停止
        robot.stop()
        print("测试完成")
        
    except Exception as e:
        print(f"测试失败: {e}")
