"""
Windows串口权限处理模块
自动处理串口权限问题
"""

import os
import sys
import platform
import subprocess
import logging
from typing import List, Optional

logger = logging.getLogger(__name__)


class PermissionHandler:
    """Windows串口权限处理器"""
    
    def __init__(self):
        self.system = platform.system()
        self.is_admin = self._check_admin_privileges()
    
    def _check_admin_privileges(self) -> bool:
        """检查是否有管理员权限"""
        try:
            if self.system == "Windows":
                # Windows下检查管理员权限
                import ctypes
                return ctypes.windll.shell32.IsUserAnAdmin()
            else:
                # Unix系统检查root权限
                return os.geteuid() == 0
        except Exception:
            return False
    
    def request_admin_privileges(self) -> bool:
        """请求管理员权限（Windows）"""
        if self.system != "Windows":
            return True
        
        try:
            import ctypes
            # 尝试以管理员权限重新启动程序
            ctypes.windll.shell32.ShellExecuteW(
                None, 
                "runas", 
                sys.executable, 
                " ".join(sys.argv), 
                None, 
                1
            )
            return True
        except Exception as e:
            logger.error(f"请求管理员权限失败: {e}")
            return False
    
    def check_port_availability(self, port_name: str) -> dict:
        """检查端口可用性"""
        result = {
            'available': False,
            'error': None,
            'suggestions': []
        }
        
        try:
            import serial
            ser = serial.Serial(
                port=port_name,
                baudrate=9600,
                timeout=1.0
            )
            ser.close()
            result['available'] = True
        except serial.SerialException as e:
            error_msg = str(e)
            result['error'] = error_msg
            
            if "PermissionError" in error_msg or "拒绝访问" in error_msg:
                result['suggestions'].extend([
                    "以管理员身份运行程序",
                    "检查端口是否被其他程序占用",
                    "重新插拔USB设备"
                ])
            elif "could not open port" in error_msg:
                result['suggestions'].extend([
                    "检查端口名称是否正确",
                    "确认设备已连接",
                    "尝试其他COM端口"
                ])
            else:
                result['suggestions'].append("检查设备连接和驱动")
        
        return result
    
    def find_alternative_ports(self) -> List[str]:
        """查找可用的替代端口"""
        try:
            import serial.tools.list_ports
            ports = serial.tools.list_ports.comports()
            return [port.device for port in ports]
        except Exception as e:
            logger.error(f"查找端口失败: {e}")
            return []
    
    def suggest_solutions(self, port_name: str) -> List[str]:
        """提供解决方案建议"""
        suggestions = []
        
        # 检查权限
        if not self.is_admin:
            suggestions.append("以管理员身份运行程序")
        
        # 检查端口占用
        suggestions.extend([
            "关闭其他可能使用串口的程序（如PuTTY、TeraTerm、Arduino IDE）",
            "重新插拔USB转串口设备",
            "尝试不同的USB端口",
            "更新USB转串口驱动程序"
        ])
        
        # 检查替代端口
        alt_ports = self.find_alternative_ports()
        if alt_ports:
            suggestions.append(f"尝试其他端口: {', '.join(alt_ports)}")
        
        return suggestions
    
    def handle_permission_error(self, port_name: str) -> bool:
        """处理权限错误"""
        logger.warning(f"端口 {port_name} 权限被拒绝")
        
        # 检查端口状态
        port_status = self.check_port_availability(port_name)
        
        if not port_status['available']:
            logger.error(f"端口 {port_name} 不可用: {port_status['error']}")
            
            # 提供解决方案
            suggestions = self.suggest_solutions(port_name)
            logger.info("建议的解决方案:")
            for i, suggestion in enumerate(suggestions, 1):
                logger.info(f"  {i}. {suggestion}")
            
            # 如果是权限问题且不是管理员，尝试请求管理员权限
            if not self.is_admin and "PermissionError" in str(port_status['error']):
                logger.info("尝试请求管理员权限...")
                return self.request_admin_privileges()
        
        return False


def create_permission_handler() -> PermissionHandler:
    """创建权限处理器"""
    return PermissionHandler()


def check_and_fix_permissions(port_name: str) -> bool:
    """检查并修复权限问题"""
    handler = create_permission_handler()
    
    # 检查权限
    if not handler.is_admin:
        logger.warning("当前没有管理员权限，可能无法访问串口")
        return handler.handle_permission_error(port_name)
    
    # 检查端口可用性
    port_status = handler.check_port_availability(port_name)
    if not port_status['available']:
        return handler.handle_permission_error(port_name)
    
    return True


if __name__ == "__main__":
    # 测试权限处理
    logging.basicConfig(level=logging.INFO)
    
    handler = create_permission_handler()
    
    print("=== 权限检查 ===")
    print(f"管理员权限: {handler.is_admin}")
    print(f"操作系统: {platform.system()}")
    
    # 测试端口
    test_ports = ["COM3", "COM4", "COM5"]
    for port in test_ports:
        print(f"\n测试端口 {port}:")
        status = handler.check_port_availability(port)
        print(f"  可用: {status['available']}")
        if status['error']:
            print(f"  错误: {status['error']}")
        if status['suggestions']:
            print("  建议:")
            for suggestion in status['suggestions']:
                print(f"    - {suggestion}")
