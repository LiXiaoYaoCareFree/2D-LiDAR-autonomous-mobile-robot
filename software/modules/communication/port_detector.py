"""
Windows/Mac 串口自动检测模块
支持跨平台的串口设备自动发现和连接
"""

import serial
import serial.tools.list_ports
import platform
import logging
from typing import List, Optional, Dict, Tuple
import time

logger = logging.getLogger(__name__)


class PortDetector:
    """跨平台串口检测器"""
    
    def __init__(self):
        self.system = platform.system()
        self.known_vid_pids = {
            # 常见的USB转串口芯片VID:PID
            (0x0403, 0x6001): "FTDI FT232",  # FTDI
            (0x10C4, 0xEA60): "Silicon Labs CP210x",  # Silicon Labs
            (0x1A86, 0x7523): "CH340",  # CH340
            (0x067B, 0x2303): "Prolific PL2303",  # Prolific
            (0x04D8, 0x000A): "Microchip CDC",  # Microchip
            (0x2341, 0x0043): "Arduino Uno",  # Arduino
            (0x2341, 0x0001): "Arduino Mega",  # Arduino
        }
    
    def get_available_ports(self) -> List[Dict[str, str]]:
        """
        获取所有可用的串口设备
        
        Returns:
            串口信息列表，每个元素包含 {'port', 'description', 'hwid', 'vid_pid'}
        """
        ports = []
        
        try:
            # 使用pyserial的list_ports工具
            available_ports = serial.tools.list_ports.comports()
            
            for port in available_ports:
                port_info = {
                    'port': port.device,
                    'description': port.description or 'Unknown',
                    'hwid': port.hwid or '',
                    'vid_pid': self._extract_vid_pid(port.hwid)
                }
                ports.append(port_info)
                
        except Exception as e:
            logger.error(f"获取串口列表失败: {e}")
        
        return ports
    
    def _extract_vid_pid(self, hwid: str) -> Optional[Tuple[int, int]]:
        """从硬件ID中提取VID:PID"""
        if not hwid:
            return None
        
        try:
            # 查找VID:PID格式 (如 VID:0403 PID:6001)
            import re
            match = re.search(r'VID:([0-9A-F]{4})\s+PID:([0-9A-F]{4})', hwid, re.IGNORECASE)
            if match:
                vid = int(match.group(1), 16)
                pid = int(match.group(2), 16)
                return (vid, pid)
        except Exception:
            pass
        
        return None
    
    def find_robot_port(self, 
                        preferred_ports: List[str] = None,
                        test_baudrates: List[int] = None) -> Optional[str]:
        """
        智能查找机器人串口
        
        Args:
            preferred_ports: 优先尝试的端口列表
            test_baudrates: 测试的波特率列表
            
        Returns:
            找到的端口名称，未找到返回None
        """
        if preferred_ports is None:
            preferred_ports = self._get_default_preferred_ports()
        
        if test_baudrates is None:
            test_baudrates = [921600, 115200, 9600, 38400]
        
        # 1. 首先尝试优先端口
        for port_name in preferred_ports:
            if self._test_port_connection(port_name, test_baudrates):
                logger.info(f"在优先端口找到机器人: {port_name}")
                return port_name
        
        # 2. 扫描所有可用端口
        available_ports = self.get_available_ports()
        logger.info(f"发现 {len(available_ports)} 个串口设备")
        
        for port_info in available_ports:
            port_name = port_info['port']
            logger.info(f"测试端口: {port_name} ({port_info['description']})")
            
            if self._test_port_connection(port_name, test_baudrates):
                logger.info(f"找到机器人端口: {port_name}")
                return port_name
        
        logger.warning("未找到机器人串口")
        return None
    
    def _get_default_preferred_ports(self) -> List[str]:
        """获取默认的优先端口列表"""
        if self.system == "Windows":
            return ["COM3", "COM4", "COM5", "COM6", "COM7", "COM8", "COM9", "COM10"]
        elif self.system == "Darwin":  # macOS
            return ["/dev/tty.usbserial-*", "/dev/tty.usbmodem*", "/dev/tty.SLAB_USBtoUART"]
        else:  # Linux
            return ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyACM0", "/dev/ttyACM1"]
    
    def _test_port_connection(self, port_name: str, baudrates: List[int]) -> bool:
        """
        测试端口连接
        
        Args:
            port_name: 端口名称
            baudrates: 测试的波特率列表
            
        Returns:
            连接成功返回True
        """
        for baudrate in baudrates:
            try:
                # 尝试打开串口
                ser = serial.Serial(
                    port=port_name,
                    baudrate=baudrate,
                    timeout=0.5,
                    write_timeout=0.5
                )
                
                # 发送测试命令
                test_command = b"AT\r\n"  # 简单的AT命令测试
                ser.write(test_command)
                ser.flush()
                
                # 等待响应
                time.sleep(0.1)
                response = ser.read(100)
                
                # 检查是否有LDR3协议特征
                if self._detect_ldr3_protocol(response):
                    ser.close()
                    logger.info(f"在 {port_name} @ {baudrate} 检测到LDR3协议")
                    return True
                
                # 检查是否有其他机器人协议特征
                if self._detect_robot_protocol(response):
                    ser.close()
                    logger.info(f"在 {port_name} @ {baudrate} 检测到机器人协议")
                    return True
                
                ser.close()
                
            except Exception as e:
                logger.debug(f"测试端口 {port_name} @ {baudrate} 失败: {e}")
                continue
        
        return False
    
    def _detect_ldr3_protocol(self, data: bytes) -> bool:
        """检测LDR3协议特征"""
        if len(data) < 4:
            return False
        
        # 检查LDR3魔数 (小端序 'LDR3' = 0x3352444C)
        import struct
        try:
            magic = struct.unpack('<I', data[:4])[0]
            return magic == 0x3352444C
        except:
            return False
    
    def _detect_robot_protocol(self, data: bytes) -> bool:
        """检测其他机器人协议特征"""
        if not data:
            return False
        
        # 检查常见的机器人协议标识
        robot_indicators = [
            b"ROBOT",
            b"LIDAR", 
            b"ODOM",
            b"IMU",
            b"OK",
            b"READY"
        ]
        
        data_str = data.decode('utf-8', errors='ignore').upper()
        for indicator in robot_indicators:
            if indicator.decode('utf-8').upper() in data_str:
                return True
        
        return False
    
    def get_port_info(self, port_name: str) -> Dict[str, str]:
        """获取指定端口的详细信息"""
        ports = self.get_available_ports()
        for port_info in ports:
            if port_info['port'] == port_name:
                return port_info
        return {}


class WindowsPortDetector(PortDetector):
    """Windows专用端口检测器"""
    
    def __init__(self):
        super().__init__()
        self._wmic_available = self._check_wmic_availability()
    
    def _check_wmic_availability(self) -> bool:
        """检查WMIC是否可用"""
        try:
            import subprocess
            result = subprocess.run(['wmic', '/?'], 
                                  capture_output=True, 
                                  timeout=5)
            return result.returncode == 0
        except:
            return False
    
    def get_detailed_port_info(self) -> List[Dict[str, str]]:
        """获取详细的端口信息（Windows专用）"""
        ports = self.get_available_ports()
        
        if self._wmic_available:
            try:
                # 使用WMIC获取更详细的信息
                import subprocess
                result = subprocess.run([
                    'wmic', 'path', 'win32_serialport', 'get', 
                    'deviceid,description,name,status'
                ], capture_output=True, text=True, timeout=10)
                
                if result.returncode == 0:
                    lines = result.stdout.strip().split('\n')
                    for line in lines[1:]:  # 跳过标题行
                        if line.strip():
                            parts = line.strip().split()
                            if len(parts) >= 2:
                                port_name = parts[0]
                                description = ' '.join(parts[1:])
                                
                                # 更新端口信息
                                for port_info in ports:
                                    if port_info['port'] == port_name:
                                        port_info['description'] = description
                                        break
            except Exception as e:
                logger.debug(f"WMIC查询失败: {e}")
        
        return ports


def create_port_detector() -> PortDetector:
    """创建适合当前系统的端口检测器"""
    system = platform.system()
    
    if system == "Windows":
        return WindowsPortDetector()
    else:
        return PortDetector()


def auto_detect_robot_port() -> Optional[str]:
    """自动检测机器人端口（便捷函数）"""
    detector = create_port_detector()
    return detector.find_robot_port()


if __name__ == "__main__":
    # 测试端口检测
    logging.basicConfig(level=logging.INFO)
    
    detector = create_port_detector()
    
    print("=== 可用串口设备 ===")
    ports = detector.get_available_ports()
    for port in ports:
        print(f"端口: {port['port']}")
        print(f"  描述: {port['description']}")
        print(f"  硬件ID: {port['hwid']}")
        if port['vid_pid']:
            vid, pid = port['vid_pid']
            print(f"  VID:PID: 0x{vid:04X}:0x{pid:04X}")
        print()
    
    print("=== 自动检测机器人端口 ===")
    robot_port = detector.find_robot_port()
    if robot_port:
        print(f"找到机器人端口: {robot_port}")
    else:
        print("未找到机器人端口")
